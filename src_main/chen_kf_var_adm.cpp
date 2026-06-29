/*
Indirect variable admittance controller for Schunk Powerball pHRI.

Paper:
    Chen et al., "Variable Admittance Control Using Velocity-Curvature
    Patterns to Enhance Physical Human-Robot Interaction", IEEE RA-L,
    2024.

Inputs:
    Force sensor stream over TCP: external force [N].
    Robot state: joint position [rad] and joint velocity [rad/s].

Outputs:
    Robot command: joint velocity [rad/s].
    CSV log: force, velocity, estimated path parameters, intended
    force, damping matrix, and joint data for analysis.

Controller:
    1. Direct intention: force magnitude sets the basic damping.
    2. Indirect intention: recent motion updates a Kalman filter that
       estimates the velocity-curvature power-law parameters K and beta.
    3. Guidance: intended acceleration produces intended force.
       Damping is low along intended force and high perpendicular
       to intended force.
*/

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>

#include <chrono>
#include <atomic>
#include <mutex>
#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>

#include "powerball/schunk_powerball.h"
#include "vrep/v_repClass.h"
#include "powerball/schunk_kinematics.h"

#include <TooN/LU.h>
#include <TooN/SVD.h>

using namespace std;
using boost::asio::ip::tcp;
using namespace TooN;

// -------------------- Globals (shared between threads) --------------------
// FT is defined in powerball_utils.cpp and shared across the program.
extern Vector<6,float> FT;
simxFloat newPos[3] = {0.0f, 0.439f, 0.275f};

// -------------------- User Settings --------------------
static const float dt = 0.005f;  // TODO controller period [s]

float M_inertia = 5.0f;  // TODO paper md, range: 1 to 10 kg

float cmin_pos = 90.0f;   // TODO low damping, paper: 90 Ns/m
float cmax_pos = 150.0f;  // TODO high damping, paper: 150 Ns/m
float c_rate_max = 300.0f;  // TODO damping slew rate [Ns/m/s]
float dc_max = c_rate_max * dt;

float F_min = 1.0f;   // TODO paper fmin, range: 0.5 to 3 N
float F_max = 11.0f;  // TODO paper fmax, range: 5 to 15 N

int power_law_window = 50;  // TODO only used by old LS estimator
int power_law_min_points = 10;  // TODO only used by old LS estimator

float kf_process_logK = 1e-4f;  // TODO log(K) process noise
float kf_process_beta = 1e-5f;  // TODO beta process noise
float kf_measurement_noise = 2.5e-3f;  // TODO log-speed noise
float kf_initial_covariance = 1.0f;  // TODO initial uncertainty
float kf_min_covariance = 1e-6f;  // TODO covariance lower limit
float kf_logK_min = -10.0f;  // TODO lower log(K) bound
float kf_logK_max = 3.0f;  // TODO upper log(K) bound
float kf_fit_alpha = 0.02f;  // TODO running fit-quality update rate

float min_speed_for_guidance = 0.005f;  // TODO [m/s]
float min_curvature = 0.001f;  // TODO [1/m]
float max_curvature = 200.0f;  // TODO safety limit [1/m]

float beta_min = 0.0f;       // paper spectrum lower bound
float beta_max = 2.0f / 3.0f;  // paper spectrum upper bound
float beta_default = 1.0f / 3.0f;

float tangential_acc_limit = 1.0f;  // TODO [m/s^2]
float max_intended_acc = 2.0f;  // TODO [m/s^2]
int tangential_search_points = 21;  // TODO odd number is easiest

float intended_force_lpf_alpha = 0.14f;  // about 5 Hz at dt=0.005 s

float ft_scale_x = 1.0f;  // TODO sensor x scale/sign
float ft_scale_y = 1.0f;  // TODO sensor y scale/sign

float ft_lpf_alpha = 0.5f;  // smaller = smoother alpha=[0 1]
float ft_deadband = 0.05f;  // TODO force deadband [N]

float lambda_dls = 0.12f;  // TODO damped least-squares IK
float v_meas_lpf_alpha = 0.20f;  // TODO Cartesian velocity smoothing
float qdot_lpf_alpha = 0.20f;  // TODO joint velocity smoothing
float qdot_limit = 30.0f * (float)M_PI / 180.0f;  // TODO [rad/s]

float cart_vel_limit = 0.20f;  // TODO Cartesian speed limit [m/s]
float adm_time = 0.0f;

// -------------------- Controller State --------------------
float Fx_filt = 0.0f;
float Fy_filt = 0.0f;

float bd_desired = cmax_pos;
float bd_applied = cmax_pos;
float bd_prev = cmax_pos;

float beta_hat = beta_default;
float K_hat = 0.0f;
float power_law_R2 = 0.0f;
float power_law_RMSE = 0.0f;
float curvature_hat = 0.0f;
float target_curvature = 0.0f;

float gamma1 = 1.0f;
float gamma2 = 1.0f;

bool indirect_ready = false;
float last_turn_sign = 1.0f;

Vector<6,float> Q = Zeros;
Vector<6,float> Qe = Zeros;
Vector<6,float> Qdot = Zeros;
Vector<6,float> Qdot_a = Zeros;
Vector<6,float> Qdot_cmd = Zeros;
Vector<6,float> Qdot_prev = Zeros;

Vector<3,float> X = Zeros;
Vector<6,float> F_modified = Zeros;
Vector<6,float> F_cmd = Zeros;
Vector<6,float> v_meas = Zeros;
Vector<6,float> v_meas_lpf = Zeros;
Vector<6,float> v_meas_lpf_prev = Zeros;
Vector<6,float> vel = Zeros;
Vector<3,float> euler_zyx = Zeros;

Vector<2,float> intended_acc = makeVector(0.0f, 0.0f);
Vector<2,float> intended_acc_prev = makeVector(0.0f, 0.0f);
Vector<2,float> intended_force = makeVector(0.0f, 0.0f);
Vector<2,float> intended_force_prev = makeVector(0.0f, 0.0f);
Vector<2,float> intended_direction = makeVector(1.0f, 0.0f);

Matrix<2,2,float> B_planar = Data(cmax_pos, 0.0f,
                                  0.0f, cmax_pos);

Vector<6,float> Cd_diag = makeVector(cmax_pos, cmax_pos, cmax_pos,
                                     cmax_pos, cmax_pos, cmax_pos);
Vector<6,float> Md_diag = Ones * M_inertia;

std::deque< Vector<2,float> > velocity_history;
std::mutex ft_mutex;

// Kalman filter state: [log(K), beta].
bool kf_initialized = false;
bool kf_fit_stats_ready = false;
float kf_state_logK = 0.0f;
float kf_state_beta = beta_default;
float kf_P00 = 1.0f;
float kf_P01 = 0.0f;
float kf_P11 = 1.0f;
float kf_log_speed_avg = 0.0f;
float kf_log_speed_sq_avg = 0.0f;
float kf_residual_sq_avg = 0.0f;


Matrix<6,6,float> R_F_offset = Data(
    cos(M_PI/2), -sin(M_PI/2), 0, 0, 0, 0,
    sin(M_PI/2),  cos(M_PI/2), 0, 0, 0, 0,
    0,            0,           1, 0, 0, 0,
    0, 0, 0, cos(M_PI/2), -sin(M_PI/2), 0,
    0, 0, 0, sin(M_PI/2),  cos(M_PI/2), 0,
    0, 0, 0, 0,            0,           1
);

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float deadbandf(float x, float db)
{
    if (fabs(x) < db) return 0.0f;
    return x;
}

static float vec2_dot(Vector<2,float> a, Vector<2,float> b)
{
    return a[0] * b[0] + a[1] * b[1];
}

static float vec2_cross(Vector<2,float> a, Vector<2,float> b)
{
    return a[0] * b[1] - a[1] * b[0];
}

static float vec2_norm(Vector<2,float> a)
{
    return std::sqrt(vec2_dot(a, a));
}

static Vector<2,float> vec2_unit(Vector<2,float> a,
                                 Vector<2,float> fallback)
{
    float a_norm = vec2_norm(a);
    if (a_norm < 1e-6f)
        return fallback;

    return a / a_norm;
}

static Vector<2,float> vec2_left_normal(Vector<2,float> a)
{
    return makeVector(-a[1], a[0]);
}

static Vector<3,float> rotation_matrix_to_euler_zyx(
    Matrix<3,3,float> R)
{
    /*
    Convert rotation matrix to ZYX Euler angles [yaw pitch roll].

    Inputs:
        R: end-effector rotation matrix in the base frame.

    Outputs:
        euler_zyx: [yaw pitch roll] in radians.
    */
    float yaw = std::atan2(R[1][0], R[0][0]);
    float pitch = std::atan2(
        -R[2][0],
        std::sqrt(R[2][1] * R[2][1] + R[2][2] * R[2][2]));
    float roll = std::atan2(R[2][1], R[2][2]);

    return makeVector(yaw, pitch, roll);
}

static long long now_us()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(
        system_clock::now().time_since_epoch()).count();
}

static void sleep_to_keep_dt(std::chrono::time_point
                             <std::chrono::system_clock> t0)
{
    using namespace std::chrono;
    duration<float> elapsed = system_clock::now() - t0;
    float sleep_s = dt - elapsed.count();
    if (sleep_s > 0)
        usleep((useconds_t)(sleep_s * 1e6));
}

void stop(std::atomic<bool>* stopFlag)
{
    char in;
    cin.get(in);
    stopFlag->store(true);
}

int vrep_draw(std::atomic<bool>* stopFlag)
{
    V_rep vrep;
    int res = vrep.connect();
    if (res == -1)
    {
        cout << "V-REP Connection Error!" << endl;
        return 0;
    }

    while (!stopFlag->load())
    {
        vrep.setSphere(&newPos[0]);
        usleep(40 * 1000);
    }
    return 0;
}

static float compute_planar_force_magnitude()
{
    return std::sqrt(F_cmd[0] * F_cmd[0] + F_cmd[1] * F_cmd[1]);
}

static float compute_direct_damping(float force_magnitude)
{
    /*
    Direct intention from Chen et al. 2024, Eq. (7).

    Inputs:
        force_magnitude: planar human force norm [N].

    Outputs:
        damping: scalar virtual damping [Ns/m].

    Larger force means faster intended motion, so damping decreases.
    */

    if (force_magnitude <= F_min)
        return cmax_pos;

    if (force_magnitude >= F_max)
        return cmin_pos;

    float force_ratio = (force_magnitude - F_min) / (F_max - F_min);
    float damping = cmax_pos - (cmax_pos - cmin_pos) * force_ratio;

    return clampf(damping, cmin_pos, cmax_pos);
}

static float rate_limit_damping(float desired_damping,
                                float previous_damping)
{
    float damping_step = desired_damping - previous_damping;

    if (damping_step > dc_max) damping_step = dc_max;
    if (damping_step < -dc_max) damping_step = -dc_max;

    return previous_damping + damping_step;
}

static void push_power_law_history()
{
    Vector<2,float> velocity_xy = makeVector(v_meas_lpf[0], v_meas_lpf[1]);

    velocity_history.push_back(velocity_xy);

    int max_history_size = power_law_window + 2;
    while ((int)velocity_history.size() > max_history_size)
        velocity_history.pop_front();
}


static void update_power_law_fit_quality(float log_speed,
                                         float fitted_log_speed)
{
    float residual = log_speed - fitted_log_speed;
    float residual_squared = residual * residual;
    float log_speed_squared = log_speed * log_speed;

    if (!kf_fit_stats_ready)
    {
        kf_log_speed_avg = log_speed;
        kf_log_speed_sq_avg = log_speed_squared;
        kf_residual_sq_avg = residual_squared;
        kf_fit_stats_ready = true;
    }
    else
    {
        kf_log_speed_avg =
            (1.0f - kf_fit_alpha) * kf_log_speed_avg +
            kf_fit_alpha * log_speed;
        kf_log_speed_sq_avg =
            (1.0f - kf_fit_alpha) * kf_log_speed_sq_avg +
            kf_fit_alpha * log_speed_squared;
        kf_residual_sq_avg =
            (1.0f - kf_fit_alpha) * kf_residual_sq_avg +
            kf_fit_alpha * residual_squared;
    }

    float log_speed_variance =
        kf_log_speed_sq_avg - kf_log_speed_avg * kf_log_speed_avg;

    power_law_RMSE = std::sqrt(kf_residual_sq_avg);

    if (log_speed_variance > 1e-6f)
    {
        power_law_R2 =
            1.0f - kf_residual_sq_avg / log_speed_variance;
        power_law_R2 = clampf(power_law_R2, 0.0f, 1.0f);
    }
    else
    {
        power_law_R2 = 0.0f;
    }
}

static void estimate_power_law_kf()
{
    /*
    Online Kalman filter for Chen's velocity-curvature power law:
        v = K * kappa^(-beta)

    Log form used as the measurement equation:
        log(v) = log(K) - beta * log(kappa)

    State:
        x = [log(K), beta]

    Measurement:
        z = log(v)
        H = [1, -log(kappa)]

    Inputs:
        velocity_history: recent measured Cartesian velocities [m/s].
        The latest three samples are used to estimate local curvature.

    Outputs:
        K_hat: estimated velocity gain.
        beta_hat: estimated power-law exponent.
        curvature_hat: most recent valid curvature estimate [1/m].
    */

    int history_size = (int)velocity_history.size();
    if (history_size < 3)
    {
        indirect_ready = false;
        return;
    }

    Vector<2,float> v_prev = velocity_history[history_size - 3];
    Vector<2,float> v = velocity_history[history_size - 2];
    Vector<2,float> v_next = velocity_history[history_size - 1];

    Vector<2,float> acceleration = (v_next - v_prev) / (2.0f * dt);

    float speed = vec2_norm(v);
    if (speed < min_speed_for_guidance)
    {
        indirect_ready = false;
        return;
    }

    float curvature =
        fabs(vec2_cross(v, acceleration)) / (speed * speed * speed);

    if (curvature < min_curvature)
    {
        indirect_ready = false;
        return;
    }

    curvature = clampf(curvature, min_curvature, max_curvature);
    curvature_hat = curvature;

    float log_curvature = std::log(curvature);
    float log_speed = std::log(speed);

    if (!kf_initialized)
    {
        kf_state_beta = beta_default;
        kf_state_logK = log_speed + kf_state_beta * log_curvature;
        kf_state_logK = clampf(kf_state_logK, kf_logK_min, kf_logK_max);
        kf_P00 = kf_initial_covariance;
        kf_P01 = 0.0f;
        kf_P11 = kf_initial_covariance;
        kf_initialized = true;
    }

    kf_P00 += kf_process_logK;
    kf_P11 += kf_process_beta;

    float measurement_h0 = 1.0f;
    float measurement_h1 = -log_curvature;
    float predicted_log_speed =
        measurement_h0 * kf_state_logK +
        measurement_h1 * kf_state_beta;
    float innovation = log_speed - predicted_log_speed;

    float innovation_variance =
        measurement_h0 * measurement_h0 * kf_P00 +
        2.0f * measurement_h0 * measurement_h1 * kf_P01 +
        measurement_h1 * measurement_h1 * kf_P11 +
        kf_measurement_noise;

    if (innovation_variance < 1e-9f)
    {
        indirect_ready = false;
        return;
    }

    float gain_logK =
        (kf_P00 * measurement_h0 + kf_P01 * measurement_h1) /
        innovation_variance;
    float gain_beta =
        (kf_P01 * measurement_h0 + kf_P11 * measurement_h1) /
        innovation_variance;

    kf_state_logK += gain_logK * innovation;
    kf_state_beta += gain_beta * innovation;

    kf_state_logK = clampf(kf_state_logK, kf_logK_min, kf_logK_max);
    kf_state_beta = clampf(kf_state_beta, beta_min, beta_max);

    float covariance_projection0 =
        measurement_h0 * kf_P00 + measurement_h1 * kf_P01;
    float covariance_projection1 =
        measurement_h0 * kf_P01 + measurement_h1 * kf_P11;

    float P00_new = kf_P00 - gain_logK * covariance_projection0;
    float P01_new = kf_P01 - gain_logK * covariance_projection1;
    float P10_new = kf_P01 - gain_beta * covariance_projection0;
    float P11_new = kf_P11 - gain_beta * covariance_projection1;

    kf_P00 = std::max(P00_new, kf_min_covariance);
    kf_P01 = 0.5f * (P01_new + P10_new);
    kf_P11 = std::max(P11_new, kf_min_covariance);

    beta_hat = kf_state_beta;
    K_hat = std::exp(kf_state_logK);

    float fitted_log_speed =
        measurement_h0 * kf_state_logK +
        measurement_h1 * kf_state_beta;
    update_power_law_fit_quality(log_speed, fitted_log_speed);

    indirect_ready = true;
}


static float compute_target_curvature(float speed)
{
    /*
    From v = K * kappa^(-beta):
        kappa = (K / v)^(1 / beta)

    For beta near zero, the paper's power-law constraint becomes
    poorly conditioned, so the most recent measured curvature is used.
    */

    if (speed < min_speed_for_guidance)
        return 0.0f;

    if (beta_hat < 0.02f || K_hat <= 0.0f)
        return clampf(curvature_hat, 0.0f, max_curvature);

    float curvature = std::pow(K_hat / speed, 1.0f / beta_hat);
    return clampf(curvature, 0.0f, max_curvature);
}

static bool candidate_respects_direction(Vector<2,float> v,
                                         Vector<2,float> force,
                                         Vector<2,float> acc)
{
    /*
    Simplified implementation of the two direction constraints in
    Chen et al. 2024, Eq. (10).

    The next velocity should turn in the same direction indicated by
    the force and stay between the current velocity and force direction.
    */

    Vector<2,float> v_next = v + acc * dt;

    float speed = vec2_norm(v);
    float next_speed = vec2_norm(v_next);
    float force_norm = vec2_norm(force);

    if (speed < min_speed_for_guidance ||
        next_speed < min_speed_for_guidance ||
        force_norm < 1e-6f)
    {
        return true;
    }

    float force_cross = vec2_cross(v, force);
    float next_cross = vec2_cross(v, v_next);

    if (fabs(force_cross) > 1e-6f && force_cross * next_cross <= 0.0f)
        return false;

    float cos_next = vec2_dot(v, v_next) / (speed * next_speed);
    float cos_force = vec2_dot(v, force) / (speed * force_norm);

    if (cos_force < 0.98f && cos_next <= cos_force)
        return false;

    return true;
}

static void compute_intended_acceleration()
{
    /*
    Local minimum-jerk approximation of Chen et al. 2024, Eq. (10).

    The paper states a constrained optimization but does not provide a
    closed-form solver. This implementation keeps it explicit:
        - normal acceleration magnitude comes from target curvature
        - turn direction comes from force direction
        - tangential acceleration is chosen by a small search that stays
          closest to the previous intended acceleration
    */

    Vector<2,float> v = makeVector(v_meas_lpf[0], v_meas_lpf[1]);
    Vector<2,float> force = makeVector(F_cmd[0], F_cmd[1]);

    float speed = vec2_norm(v);
    if (!indirect_ready || speed < min_speed_for_guidance)
    {
        intended_acc = makeVector(0.0f, 0.0f);
        target_curvature = 0.0f;
        return;
    }

    Vector<2,float> tangent = vec2_unit(v, makeVector(1.0f, 0.0f));
    Vector<2,float> normal_left = vec2_left_normal(tangent);

    float force_cross = vec2_cross(v, force);
    if (fabs(force_cross) > 1e-6f)
        last_turn_sign = (force_cross > 0.0f) ? 1.0f : -1.0f;

    target_curvature = compute_target_curvature(speed);
    float normal_acc = speed * speed * target_curvature;
    normal_acc = clampf(normal_acc, 0.0f, max_intended_acc);

    float center_tangent_acc = vec2_dot(intended_acc_prev, tangent);
    float best_cost = 1e30f;
    Vector<2,float> best_acc = makeVector(0.0f, 0.0f);
    bool found_candidate = false;

    for (int i = 0; i < tangential_search_points; i++)
    {
        float ratio = 0.0f;
        if (tangential_search_points > 1)
            ratio = (float)i / (float)(tangential_search_points - 1);

        float tangent_acc =
            -tangential_acc_limit +
            2.0f * tangential_acc_limit * ratio;

        tangent_acc += center_tangent_acc;
        tangent_acc = clampf(tangent_acc,
                             -tangential_acc_limit,
                             tangential_acc_limit);

        Vector<2,float> candidate =
            tangent_acc * tangent +
            last_turn_sign * normal_acc * normal_left;

        if (!candidate_respects_direction(v, force, candidate))
            continue;

        Vector<2,float> diff = candidate - intended_acc_prev;
        float cost = vec2_dot(diff, diff);

        if (cost < best_cost)
        {
            best_cost = cost;
            best_acc = candidate;
            found_candidate = true;
        }
    }

    if (found_candidate)
        intended_acc = best_acc;
    else
        intended_acc =
            center_tangent_acc * tangent +
            last_turn_sign * normal_acc * normal_left;

    intended_acc_prev = intended_acc;
}

static void update_guided_damping()
{
    /*
    Chen et al. 2024, Eq. (11)-(14).

    Intended force:
        F_intended = md * a_intended + (bd / gamma1) * v

    Local damping:
        B_local = diag(bd / gamma1, gamma2 * bd)

    Then rotate B_local into the robot base frame using the intended
    force direction as the local x-axis.
    */

    Vector<2,float> v = makeVector(v_meas_lpf[0], v_meas_lpf[1]);
    float speed = vec2_norm(v);
    float force_magnitude = compute_planar_force_magnitude();

    bd_desired = compute_direct_damping(force_magnitude);
    bd_applied = rate_limit_damping(bd_desired, bd_prev);
    bd_prev = bd_applied;

    gamma1 = 1.0f + 2.0f * speed * speed;
    gamma2 = 1.0f + 4.0f * speed * speed;

    Vector<2,float> intended_force_raw =
        M_inertia * intended_acc + (bd_applied / gamma1) * v;

    intended_force =
        (1.0f - intended_force_lpf_alpha) * intended_force_prev +
        intended_force_lpf_alpha * intended_force_raw;
    intended_force_prev = intended_force;

    Vector<2,float> force_fallback =
        vec2_unit(makeVector(F_cmd[0], F_cmd[1]), intended_direction);

    if (!indirect_ready || vec2_norm(intended_force) < 1e-6f)
        intended_direction = force_fallback;
    else
        intended_direction = vec2_unit(intended_force, force_fallback);

    Vector<2,float> y_direction = vec2_left_normal(intended_direction);

    Matrix<2,2,float> R = Zeros;
    R[0][0] = intended_direction[0];
    R[1][0] = intended_direction[1];
    R[0][1] = y_direction[0];
    R[1][1] = y_direction[1];

    Matrix<2,2,float> B_local = Data(
        bd_applied / gamma1, 0.0f,
        0.0f, gamma2 * bd_applied
    );

    if (indirect_ready)
        B_planar = R * B_local * R.T();
    else
        B_planar = Data(bd_applied, 0.0f,
                        0.0f, bd_applied);

    Cd_diag[0] = B_planar[0][0];
    Cd_diag[1] = B_planar[1][1];
    Cd_diag[2] = cmax_pos;
    Cd_diag[3] = cmax_pos;
    Cd_diag[4] = cmax_pos;
    Cd_diag[5] = cmax_pos;
}

static void integrate_planar_admittance()
{
    /*
    Mass-damper admittance:
        M_d * x_ddot + B_d * x_dot = F_ext

    Inputs:
        F_cmd: external force in robot base frame [N].
        B_planar: 2D virtual damping matrix [Ns/m].
        M_inertia: scalar virtual mass [kg].

    Outputs:
        vel: Cartesian velocity command [m/s, rad/s].
    */

    Vector<2,float> force_xy = makeVector(F_cmd[0], F_cmd[1]);
    Vector<2,float> vel_xy = makeVector(vel[0], vel[1]);

    Vector<2,float> acc_xy =
        (force_xy - B_planar * vel_xy) / M_inertia;

    vel_xy = vel_xy + acc_xy * dt;

    float planar_speed = vec2_norm(vel_xy);
    if (planar_speed > cart_vel_limit)
        vel_xy = vel_xy * (cart_vel_limit / planar_speed);

    vel[0] = vel_xy[0];
    vel[1] = vel_xy[1];
    vel[2] = 0.0f;
    vel[3] = 0.0f;
    vel[4] = 0.0f;
    vel[5] = 0.0f;
}

static void apply_workspace_limits()
{
    // TODO range: adjust to safe robot table workspace [m].
    float xmin = 0.18f, xmax = 0.55f;
    float ymin = -0.4f, ymax = 0.4f;
    float zmin = 0.11f, zmax = 0.12f;

    if (X[0] <= xmin && vel[0] < 0.0f) vel[0] = 0.0f;
    if (X[0] >= xmax && vel[0] > 0.0f) vel[0] = 0.0f;

    if (X[1] <= ymin && vel[1] < 0.0f) vel[1] = 0.0f;
    if (X[1] >= ymax && vel[1] > 0.0f) vel[1] = 0.0f;

    if (X[2] <= zmin && vel[2] < 0.0f) vel[2] = 0.0f;
    if (X[2] >= zmax && vel[2] > 0.0f) vel[2] = 0.0f;
}

static void write_run_hyperparams_json(const std::string& json_path,
                                       const std::string& subject,
                                       const std::string& data_csv_path)
{
    std::ofstream f(json_path);
    if (!f.is_open())
    {
        std::cerr << "WARN: unable to open hyperparams file: "
                  << json_path << std::endl;
        return;
    }

    f << "{\n";
    f << "  \"subject\": \"" << subject << "\",\n";
    f << "  \"data_csv_path\": \"" << data_csv_path << "\",\n";
    f << "  \"hyperparams\": {\n";
    f << "    \"dt\": " << dt << ",\n";
    f << "    \"M_inertia\": " << M_inertia << ",\n";
    f << "    \"cmin_pos\": " << cmin_pos << ",\n";
    f << "    \"cmax_pos\": " << cmax_pos << ",\n";
    f << "    \"F_min\": " << F_min << ",\n";
    f << "    \"F_max\": " << F_max << ",\n";
    f << "    \"ft_lpf_alpha\": " << ft_lpf_alpha << ",\n";
    f << "    \"v_meas_lpf_alpha\": " << v_meas_lpf_alpha << ",\n";
    f << "    \"intended_force_lpf_alpha\": "
      << intended_force_lpf_alpha << ",\n";
    f << "    \"power_law_estimator\": \"kalman_filter\",\n";
    f << "    \"kf_process_logK\": " << kf_process_logK << ",\n";
    f << "    \"kf_process_beta\": " << kf_process_beta << ",\n";
    f << "    \"kf_measurement_noise\": "
      << kf_measurement_noise << ",\n";
    f << "    \"kf_initial_covariance\": "
      << kf_initial_covariance << ",\n";
    f << "    \"kf_min_covariance\": " << kf_min_covariance << ",\n";
    f << "    \"kf_logK_min\": " << kf_logK_min << ",\n";
    f << "    \"kf_logK_max\": " << kf_logK_max << ",\n";
    f << "    \"kf_fit_alpha\": " << kf_fit_alpha << ",\n";
    f << "    \"min_speed_for_guidance\": "
      << min_speed_for_guidance << ",\n";
    f << "    \"min_curvature\": " << min_curvature << ",\n";
    f << "    \"max_curvature\": " << max_curvature << ",\n";
    f << "    \"tangential_acc_limit\": "
      << tangential_acc_limit << ",\n";
    f << "    \"max_intended_acc\": " << max_intended_acc << ",\n";
    f << "    \"cart_vel_limit\": " << cart_vel_limit << "\n";
    f << "  }\n";
    f << "}\n";
}

void computations()
{
    Kin kin;

    Matrix<6,6,float> J = Zeros;
    Matrix<3,3,float> R_ee = Zeros;
    Matrix<6,6,float> Rmat = Zeros;
    Matrix<6,6,float> I6 = Zeros;
    Matrix<6,6,float> A = Zeros;

    for (int i = 0; i < 6; i++)
        I6[i][i] = 1.0f;

    kin.Jacob(Q, &J);
    kin.FK_R(Q, &R_ee);
    kin.FK_pos(Q, &X);
    euler_zyx = rotation_matrix_to_euler_zyx(R_ee);

    newPos[0] = -X[1];
    newPos[1] =  X[0];
    newPos[2] =  X[2];

    Rmat.slice<0,0,3,3>() = R_ee;
    Rmat.slice<3,3,3,3>() = R_ee;

    Vector<6,float> FT_local = Zeros;
    {
        std::lock_guard<std::mutex> lock(ft_mutex);
        FT_local = FT;
    }

    float Fx_raw =  FT_local[1] * ft_scale_x;
    float Fy_raw = -FT_local[0] * ft_scale_y;

    Fx_filt = (1.0f - ft_lpf_alpha) * Fx_filt + ft_lpf_alpha * Fx_raw;
    Fy_filt = (1.0f - ft_lpf_alpha) * Fy_filt + ft_lpf_alpha * Fy_raw;

    Fx_filt = deadbandf(Fx_filt, ft_deadband);
    Fy_filt = deadbandf(Fy_filt, ft_deadband);

    F_modified = Zeros;
    F_modified[0] = Fx_filt;
    F_modified[1] = Fy_filt;

    F_cmd = Rmat * (R_F_offset * F_modified);

    v_meas = J * Qdot_a;
    v_meas_lpf =
        (1.0f - v_meas_lpf_alpha) * v_meas_lpf_prev +
        v_meas_lpf_alpha * v_meas;
    v_meas_lpf_prev = v_meas_lpf;

    push_power_law_history();
    estimate_power_law_kf();
    compute_intended_acceleration();
    update_guided_damping();
    integrate_planar_admittance();
    apply_workspace_limits();

    A = J * J.T() + I6 * (lambda_dls * lambda_dls);

    SVD<6,6,float> svdA(A);
    Vector<6,float> y = svdA.backsub(vel);
    Qdot_cmd = J.T() * y;

    Qdot = (1.0f - qdot_lpf_alpha) * Qdot_prev +
           qdot_lpf_alpha * Qdot_cmd;
    Qdot_prev = Qdot;

    float qn = norm_inf(Qdot);
    if (qn > qdot_limit)
        Qdot = Qdot * (qdot_limit / qn);
}

void TCP_receive(std::atomic<bool>* stopFlag)
{
    boost::asio::io_service io_service;
    tcp::endpoint sender_endpoint(
        boost::asio::ip::address::from_string("192.168.1.30"),
        boost::lexical_cast<int>("1000")
    );

    tcp::socket socket(io_service);
    socket.connect(sender_endpoint);
    socket.non_blocking(true);

    boost::system::error_code ignored_error;
    char recv_buf[128];

    {
        std::string msg = "TARE(1)\n";
        socket.write_some(boost::asio::buffer(msg, msg.size()),
                          ignored_error);
        socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    }

    {
        std::string msg = "L1()\n";
        socket.write_some(boost::asio::buffer(msg, msg.size()),
                          ignored_error);
        socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    }

    while (!stopFlag->load())
    {
        int len = socket.read_some(boost::asio::buffer(recv_buf),
                                   ignored_error);

        if (ignored_error == boost::asio::error::would_block ||
            ignored_error == boost::asio::error::try_again)
        {
            usleep(1000);
            continue;
        }

        if (ignored_error || len <= 0)
            continue;

        if (len >= (int)sizeof(recv_buf))
            len = (int)sizeof(recv_buf) - 1;

        recv_buf[len] = '\0';

        int timeStamp = 0;
        Vector<6,float> FT_tmp = Zeros;
        int matched = sscanf(recv_buf, "F={%f,%f,%f,%f,%f,%f},%d",
                             &FT_tmp[0], &FT_tmp[1], &FT_tmp[2],
                             &FT_tmp[3], &FT_tmp[4], &FT_tmp[5],
                             &timeStamp);

        if (matched == 7)
        {
            std::lock_guard<std::mutex> lock(ft_mutex);
            FT = FT_tmp;
        }
    }
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    string SubName;
    cout << "What is subject name? ";
    cin >> SubName;
    cout << "Please wait " << SubName << endl;
    cin.get();

    std::atomic<bool> stop_flag(false);

    boost::thread vrep_thread(vrep_draw, &stop_flag);

    std::string write_data_csv_path =
        "/home/srisadha/hamid_powerball/src_main/data/admittance/"
        "chen_var_adm_data/" +
        SubName + "_chen_var_adm_schunk.csv";
    std::ofstream dataFile(write_data_csv_path);

    dataFile << "Time_us,"
             << "Q1,Q2,Q3,Q4,Q5,Q6,"
             << "Qdot_a1,Qdot_a2,Qdot_a3,Qdot_a4,Qdot_a5,Qdot_a6,"
             << "Qdot1,Qdot2,Qdot3,Qdot4,Qdot5,Qdot6,"
             << "X,Y,Z,"
             << "Rx,Ry,Rz,"
             << "FT1,FT2,FT3,FT4,FT5,FT6,"
             << "F_cmd1,F_cmd2,F_cmd3,F_cmd4,F_cmd5,F_cmd6,"
             << "v_meas1,v_meas2,v_meas3,v_meas4,v_meas5,v_meas6,"
             << "v_lpf1,v_lpf2,v_lpf3,v_lpf4,v_lpf5,v_lpf6,"
             << "vel1,vel2,vel3,vel4,vel5,vel6,"
             << "bd_desired,bd_applied,beta_hat,K_hat,"
             << "power_law_R2,power_law_RMSE,"
             << "curvature_hat,target_curvature,gamma1,gamma2,"
             << "intended_acc_x,intended_acc_y,"
             << "intended_force_x,intended_force_y,"
             << "intended_dir_x,intended_dir_y,"
             << "B11,B12,B21,B22,indirect_ready\n";

    write_run_hyperparams_json(
        "/home/srisadha/hamid_powerball/src_main/data/admittance/json/" +
        SubName + "_chen_var_adm.hyperparams.json",
        SubName,
        write_data_csv_path
    );

    SchunkPowerball pb;

    pb.set_sdo_controlword(NODE_ALL, STATUS_OPERATION_ENABLED);
    for (int k = NODE_1; k <= NODE_6; ++k)
        pb.unbrake(k);

    pb.update();
    Q = pb.get_pos();

    boost::thread stop_thread(stop, &stop_flag);

    Qe = Data(0.0078f, -0.354f, 1.82f, 0.0f, 0.968f, 0.0f);  // TODO start pose

    Vector<6,float> dQ = Qe - Q;

    float maxq = 0.0f;
    for (int i = 0; i < 6; i++)
        maxq = std::max(maxq, (float)fabs(dQ[i]));

    float Ttravel = std::max(1.0f, maxq * 3.0f);
    int itNum = (int)(Ttravel / dt);

    Vector<6,float> Qhold = Q;
    for (int k = 1; k <= itNum && !stop_flag.load(); k++)
    {
        float s = (1.0f - cos((float)k / (float)itNum *
                              (float)M_PI)) * 0.5f;
        Vector<6,float> Qt = s * dQ + Qhold;

        pb.set_pos(Qt);
        pb.update();
        Q = pb.get_pos();
        usleep((useconds_t)(dt * 1e6));
    }

    pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);
    pb.update();

    boost::thread FT_thread(TCP_receive, &stop_flag);

    cout << "Indirect admittance loop started! "
         << "(press Enter to stop)\n";

    const int max_steps = (int)(270.0f / dt);

    for (int cnt = 0; cnt < max_steps && !stop_flag.load(); cnt++)
    {
        auto t0 = std::chrono::system_clock::now();

        adm_time = cnt * dt;

        pb.update();
        Q = pb.get_pos();
        Qdot_a = pb.get_vel();

        computations();

        pb.set_vel(Qdot);

        long long t_us = now_us();
        Vector<6,float> FT_log = Zeros;
        {
            std::lock_guard<std::mutex> lock(ft_mutex);
            FT_log = FT;
        }

        dataFile << t_us << ","
                 << Q[0] << "," << Q[1] << "," << Q[2] << ","
                 << Q[3] << "," << Q[4] << "," << Q[5] << ","
                 << Qdot_a[0] << "," << Qdot_a[1] << ","
                 << Qdot_a[2] << "," << Qdot_a[3] << ","
                 << Qdot_a[4] << "," << Qdot_a[5] << ","
                 << Qdot[0] << "," << Qdot[1] << ","
                 << Qdot[2] << "," << Qdot[3] << ","
                 << Qdot[4] << "," << Qdot[5] << ","
                 << X[0] << "," << X[1] << "," << X[2] << ","
                 << euler_zyx[0] << "," << euler_zyx[1] << ","
                 << euler_zyx[2] << ","
                 << FT_log[0] << "," << FT_log[1] << ","
                 << FT_log[2] << "," << FT_log[3] << ","
                 << FT_log[4] << "," << FT_log[5] << ","
                 << F_cmd[0] << "," << F_cmd[1] << ","
                 << F_cmd[2] << "," << F_cmd[3] << ","
                 << F_cmd[4] << "," << F_cmd[5] << ","
                 << v_meas[0] << "," << v_meas[1] << ","
                 << v_meas[2] << "," << v_meas[3] << ","
                 << v_meas[4] << "," << v_meas[5] << ","
                 << v_meas_lpf[0] << "," << v_meas_lpf[1] << ","
                 << v_meas_lpf[2] << "," << v_meas_lpf[3] << ","
                 << v_meas_lpf[4] << "," << v_meas_lpf[5] << ","
                 << vel[0] << "," << vel[1] << ","
                 << vel[2] << "," << vel[3] << ","
                 << vel[4] << "," << vel[5] << ","
                 << bd_desired << "," << bd_applied << ","
                 << beta_hat << "," << K_hat << ","
                 << power_law_R2 << "," << power_law_RMSE << ","
                 << curvature_hat << "," << target_curvature << ","
                 << gamma1 << "," << gamma2 << ","
                 << intended_acc[0] << "," << intended_acc[1] << ","
                 << intended_force[0] << "," << intended_force[1] << ","
                 << intended_direction[0] << ","
                 << intended_direction[1] << ","
                 << B_planar[0][0] << "," << B_planar[0][1] << ","
                 << B_planar[1][0] << "," << B_planar[1][1] << ","
                 << (indirect_ready ? 1 : 0) << "\n";

        sleep_to_keep_dt(t0);
    }

    dataFile.close();

    stop_flag.store(true);

    if (FT_thread.joinable())
        FT_thread.join();
    if (vrep_thread.joinable())
        vrep_thread.join();
    if (stop_thread.joinable())
        stop_thread.detach();

    pb.update();

    usleep(500 * 1000);
    cout << "Exiting ...\n";
    return 0;
}
