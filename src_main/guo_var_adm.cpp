/*
Variable admittance controller with 3-D human intention estimation.

Paper:
    Guo et al., "Variable Admittance Control With Human Intention
    Estimation for Physical Human-Robot Interaction", IEEE TIE, 2026.

Inputs:
    Force sensor stream over TCP: external force [N].
    Robot state: joint position [rad] and joint velocity [rad/s].

Outputs:
    Robot command: joint velocity [rad/s].
    CSV log: force, Cartesian motion, fitted speed-curvature-torsion
    parameters, expected force, damping matrix, and joint data.

Controller:
    1. Direct intention: force magnitude changes scalar damping b0
       through the paper's sigmoid rule.
    2. Indirect intention: recent 3-D Cartesian motion estimates
       speed, curvature, torsion, and Frenet-Serret frame.
    3. Guidance: an expected force direction defines low damping;
       orthogonal directions receive higher damping.
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

float M_inertia = 10.0f;  // TODO paper m, range: 5 to 15 kg

float bmin = 90.0f;   // TODO paper lower damping [Ns/m]
float bmax = 150.0f;  // TODO paper upper damping [Ns/m]
float damping_lambda = 8.0f;  // TODO paper sigmoid lambda
float fmin = 1.0f;   // TODO paper minimum force threshold [N]
float fmax = 11.0f;  // TODO paper maximum force threshold [N]

int power_law_window = 50;  // TODO paper N, history samples
int power_law_min_points = 10;  // TODO least-square minimum samples

float min_speed_for_intention = 0.005f;  // TODO [m/s]
float min_curvature = 0.001f;  // TODO [1/m]
float max_curvature = 200.0f;  // TODO [1/m]
float min_torsion = 0.001f;  // TODO [1/m]
float max_torsion = 200.0f;  // TODO [1/m]

float beta_min = -2.0f;  // TODO fitted curvature exponent lower bound
float beta_max = 2.0f;   // TODO fitted curvature exponent upper bound
float gamma_min = -2.0f;  // TODO fitted torsion exponent lower bound
float gamma_max = 2.0f;   // TODO fitted torsion exponent upper bound

float mu_speed_gain = 10.0f;  // TODO paper: mu = 1 + 10 |v|

float ft_scale_x = 1.0f;  // TODO sensor x scale/sign
float ft_scale_y = 1.0f;  // TODO sensor y scale/sign
float ft_scale_z = 1.0f;  // TODO sensor z scale/sign

float ft_lpf_alpha = 0.5f;  // smaller = smoother alpha=[0 1]
float ft_deadband = 0.05f;  // TODO force deadband [N]

float lambda_dls = 0.12f;  // TODO damped least-squares IK
float qdot_lpf_alpha = 0.20f;  // TODO joint velocity smoothing
float qdot_limit = 30.0f * (float)M_PI / 180.0f;  // TODO [rad/s]

float cart_vel_limit = 0.20f;  // TODO Cartesian speed limit [m/s]
float adm_time = 0.0f;

// TODO range: adjust to safe robot workspace [m].
float workspace_xmin = 0.18f, workspace_xmax = 0.55f;
float workspace_ymin = -0.40f, workspace_ymax = 0.40f;
float workspace_zmin = 0.08f, workspace_zmax = 0.35f;

// -------------------- Controller State --------------------
float Fx_filt = 0.0f;
float Fy_filt = 0.0f;
float Fz_filt = 0.0f;

float b0 = bmax;
float mu_guidance = 1.0f;

float alpha_hat = 0.0f;
float beta_hat = 0.0f;
float gamma_hat = 0.0f;
float power_law_R2 = 0.0f;
float power_law_RMSE = 0.0f;

float curvature_meas = 0.0f;
float torsion_meas = 0.0f;
float curvature_hat = 0.0f;
float torsion_hat = 0.0f;
bool indirect_ready = false;

Vector<6,float> Q = Zeros;
Vector<6,float> Qe = Zeros;
Vector<6,float> Qdot = Zeros;
Vector<6,float> Qdot_a = Zeros;
Vector<6,float> Qdot_cmd = Zeros;
Vector<6,float> Qdot_prev = Zeros;

Vector<3,float> X = Zeros;
Vector<3,float> X_prev = Zeros;
Vector<3,float> path_vel = Zeros;
Vector<3,float> path_vel_prev = Zeros;
Vector<3,float> path_acc = Zeros;
Vector<3,float> path_acc_prev = Zeros;
Vector<3,float> path_jerk = Zeros;

bool have_position_prev = false;
bool have_velocity_prev = false;
bool have_acceleration_prev = false;

Vector<6,float> F_modified = Zeros;
Vector<6,float> F_cmd = Zeros;
Vector<6,float> v_meas = Zeros;
Vector<6,float> vel = Zeros;
Vector<3,float> euler_zyx = Zeros;

Vector<3,float> T_vec = makeVector(1.0f, 0.0f, 0.0f);
Vector<3,float> N_vec = makeVector(0.0f, 1.0f, 0.0f);
Vector<3,float> B_vec = makeVector(0.0f, 0.0f, 1.0f);
Vector<3,float> N_vec_prev = makeVector(0.0f, 1.0f, 0.0f);
bool have_frame_prev = false;

Vector<3,float> T_hat_next = makeVector(1.0f, 0.0f, 0.0f);
Vector<3,float> N_hat_next = makeVector(0.0f, 1.0f, 0.0f);
Vector<3,float> B_hat_next = makeVector(0.0f, 0.0f, 1.0f);

Vector<3,float> F_expected = Zeros;
Matrix<3,3,float> B_cart = Zeros;
Matrix<3,3,float> B_local = Zeros;
Matrix<3,3,float> R_guidance = Zeros;

std::deque<float> speed_history;
std::deque<float> curvature_history;
std::deque<float> torsion_abs_history;
std::mutex ft_mutex;

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

static float vec3_dot(Vector<3,float> a, Vector<3,float> b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static Vector<3,float> vec3_cross(Vector<3,float> a,
                                  Vector<3,float> b)
{
    return a ^ b;
}

static float vec3_norm(Vector<3,float> a)
{
    return std::sqrt(vec3_dot(a, a));
}

static Vector<3,float> vec3_unit(Vector<3,float> a,
                                 Vector<3,float> fallback)
{
    float a_norm = vec3_norm(a);
    if (a_norm < 1e-6f)
        return fallback;

    return a / a_norm;
}

static bool finite_float(float x)
{
    return std::isfinite((double)x);
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

static float compute_force_magnitude()
{
    Vector<3,float> force = F_cmd.slice<0,3>();
    return vec3_norm(force);
}

static float compute_direct_damping_sigmoid(float force_magnitude)
{
    /*
    Direct intention from Guo et al. 2026, Eq. (4).

    Inputs:
        force_magnitude: human interaction force norm [N].

    Outputs:
        b0: scalar damping [Ns/m].

    Small force means precision mode, so damping approaches bmax.
    Large force means fast motion mode, so damping approaches bmin.
    */

    float ratio = (force_magnitude - fmin) / (fmax - fmin);
    ratio = clampf(ratio, 0.0f, 1.0f);

    float exponent = damping_lambda * (2.0f * ratio - 1.0f);
    float sigmoid_part = 1.0f / (1.0f + std::exp(exponent));
    float damping = bmin + (bmax - bmin) * sigmoid_part;

    return clampf(damping, bmin, bmax);
}

static bool solve_3_by_3(float A[3][3], float b[3], float x[3])
{
    /*
    Solve a small 3-by-3 linear system with Gaussian elimination.

    Inputs:
        A: normal-equation matrix.
        b: normal-equation right-hand side.

    Outputs:
        x: least-square parameters.
    */

    float M[3][4];
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
            M[r][c] = A[r][c];
        M[r][3] = b[r];
    }

    for (int col = 0; col < 3; col++)
    {
        int pivot = col;
        for (int r = col + 1; r < 3; r++)
        {
            if (fabs(M[r][col]) > fabs(M[pivot][col]))
                pivot = r;
        }

        if (fabs(M[pivot][col]) < 1e-9f)
            return false;

        if (pivot != col)
        {
            for (int c = col; c < 4; c++)
                std::swap(M[col][c], M[pivot][c]);
        }

        float pivot_value = M[col][col];
        for (int c = col; c < 4; c++)
            M[col][c] /= pivot_value;

        for (int r = 0; r < 3; r++)
        {
            if (r == col)
                continue;

            float scale = M[r][col];
            for (int c = col; c < 4; c++)
                M[r][c] -= scale * M[col][c];
        }
    }

    for (int r = 0; r < 3; r++)
        x[r] = M[r][3];

    return true;
}

static void push_power_law_sample(float speed,
                                  float curvature,
                                  float torsion_abs)
{
    speed_history.push_back(speed);
    curvature_history.push_back(curvature);
    torsion_abs_history.push_back(torsion_abs);

    while ((int)speed_history.size() > power_law_window)
        speed_history.pop_front();
    while ((int)curvature_history.size() > power_law_window)
        curvature_history.pop_front();
    while ((int)torsion_abs_history.size() > power_law_window)
        torsion_abs_history.pop_front();
}

static void estimate_speed_curvature_torsion_power_law()
{
    /*
    Fit the 3-D speed-curvature-torsion power law:
        v = alpha * kappa^beta * |tau|^gamma

    Log least-square form:
        log(v) = log(alpha) + beta log(kappa)
                 + gamma log(|tau|)

    Outputs:
        alpha_hat, beta_hat, gamma_hat, power_law_R2, power_law_RMSE.
    */

    int count = (int)speed_history.size();
    if (count < power_law_min_points)
    {
        indirect_ready = false;
        power_law_R2 = 0.0f;
        power_law_RMSE = 0.0f;
        return;
    }

    float A[3][3] = {{0.0f, 0.0f, 0.0f},
                     {0.0f, 0.0f, 0.0f},
                     {0.0f, 0.0f, 0.0f}};
    float b[3] = {0.0f, 0.0f, 0.0f};
    std::vector<float> log_speed_values;
    std::vector<float> log_kappa_values;
    std::vector<float> log_tau_values;

    log_speed_values.reserve((size_t)count);
    log_kappa_values.reserve((size_t)count);
    log_tau_values.reserve((size_t)count);

    for (int i = 0; i < count; i++)
    {
        float x0 = 1.0f;
        float x1 = std::log(curvature_history[i]);
        float x2 = std::log(torsion_abs_history[i]);
        float y = std::log(speed_history[i]);

        A[0][0] += x0 * x0;
        A[0][1] += x0 * x1;
        A[0][2] += x0 * x2;
        A[1][0] += x1 * x0;
        A[1][1] += x1 * x1;
        A[1][2] += x1 * x2;
        A[2][0] += x2 * x0;
        A[2][1] += x2 * x1;
        A[2][2] += x2 * x2;

        b[0] += x0 * y;
        b[1] += x1 * y;
        b[2] += x2 * y;

        log_speed_values.push_back(y);
        log_kappa_values.push_back(x1);
        log_tau_values.push_back(x2);
    }

    float p[3] = {0.0f, 0.0f, 0.0f};
    bool solved = solve_3_by_3(A, b, p);
    if (!solved)
    {
        indirect_ready = false;
        power_law_R2 = 0.0f;
        power_law_RMSE = 0.0f;
        return;
    }

    float log_alpha = p[0];
    float beta = clampf(p[1], beta_min, beta_max);
    float gamma = clampf(p[2], gamma_min, gamma_max);

    float mean_y = 0.0f;
    for (int i = 0; i < count; i++)
        mean_y += log_speed_values[i];
    mean_y /= (float)count;

    float sum_squared_error = 0.0f;
    float total_squared_error = 0.0f;

    for (int i = 0; i < count; i++)
    {
        float y_fit = log_alpha +
                      beta * log_kappa_values[i] +
                      gamma * log_tau_values[i];
        float residual = log_speed_values[i] - y_fit;
        float total_error = log_speed_values[i] - mean_y;

        sum_squared_error += residual * residual;
        total_squared_error += total_error * total_error;
    }

    if (total_squared_error > 1e-9f)
        power_law_R2 = 1.0f - sum_squared_error / total_squared_error;
    else
        power_law_R2 = 0.0f;

    power_law_R2 = clampf(power_law_R2, 0.0f, 1.0f);
    power_law_RMSE = std::sqrt(sum_squared_error / (float)count);

    alpha_hat = std::exp(log_alpha);
    beta_hat = beta;
    gamma_hat = gamma;
    indirect_ready = true;
}

static void update_motion_geometry()
{
    /*
    Estimate 3-D velocity, acceleration, jerk, curvature, and torsion.

    Paper equations:
        v = (r(k) - r(k-1)) / T
        a = (v(k) - v(k-1)) / T
        j = (a(k) - a(k-1)) / T
        kappa = |v x a| / |v|^3
        tau = ((v x a) dot j) / |v x a|^2
    */

    if (!have_position_prev)
    {
        X_prev = X;
        have_position_prev = true;
        return;
    }

    path_vel = (X - X_prev) / dt;
    X_prev = X;

    if (!have_velocity_prev)
    {
        path_vel_prev = path_vel;
        have_velocity_prev = true;
        return;
    }

    path_acc = (path_vel - path_vel_prev) / dt;
    path_vel_prev = path_vel;

    if (!have_acceleration_prev)
    {
        path_acc_prev = path_acc;
        have_acceleration_prev = true;
        return;
    }

    path_jerk = (path_acc - path_acc_prev) / dt;
    path_acc_prev = path_acc;

    float speed = vec3_norm(path_vel);
    if (speed < min_speed_for_intention)
        return;

    Vector<3,float> v_cross_a = vec3_cross(path_vel, path_acc);
    float cross_norm = vec3_norm(v_cross_a);
    if (cross_norm < 1e-8f)
        return;

    curvature_meas = cross_norm / (speed * speed * speed);
    curvature_meas = clampf(curvature_meas, min_curvature, max_curvature);

    torsion_meas = vec3_dot(v_cross_a, path_jerk) /
                   (cross_norm * cross_norm);
    torsion_meas = clampf(torsion_meas, -max_torsion, max_torsion);

    float torsion_abs = fabs(torsion_meas);
    if (torsion_abs < min_torsion)
        torsion_abs = min_torsion;

    T_vec = vec3_unit(path_vel, T_vec);
    B_vec = vec3_unit(v_cross_a, B_vec);
    N_vec = vec3_unit(vec3_cross(B_vec, T_vec), N_vec);

    push_power_law_sample(speed, curvature_meas, torsion_abs);
    estimate_speed_curvature_torsion_power_law();
}

static void estimate_expected_curvature_and_torsion()
{
    /*
    Practical version of Guo et al. 2026, Eq. (8)-(10).

    Eq. (10) gives local curvature and torsion from dN/ds. Eq. (8)
    constrains their product through the fitted power law. This code
    keeps the Frenet direction from dN/ds, then scales kappa/tau to
    better satisfy the fitted speed-curvature-torsion relation.
    */

    float speed = vec3_norm(path_vel);
    if (!indirect_ready || speed < min_speed_for_intention)
    {
        curvature_hat = curvature_meas;
        torsion_hat = torsion_meas;
        return;
    }

    float kappa_raw = curvature_meas;
    float tau_raw = torsion_meas;

    if (have_frame_prev)
    {
        Vector<3,float> dNds = (N_vec - N_vec_prev) / (speed * dt);
        float kappa_fs = -vec3_dot(dNds, T_vec);
        float tau_fs = vec3_dot(dNds, B_vec);

        if (finite_float(kappa_fs) && fabs(kappa_fs) > min_curvature)
            kappa_raw = fabs(kappa_fs);
        if (finite_float(tau_fs) && fabs(tau_fs) > min_torsion)
            tau_raw = tau_fs;
    }

    N_vec_prev = N_vec;
    have_frame_prev = true;

    kappa_raw = clampf(fabs(kappa_raw), min_curvature, max_curvature);
    float tau_sign = (tau_raw >= 0.0f) ? 1.0f : -1.0f;
    float tau_abs = clampf(fabs(tau_raw), min_torsion, max_torsion);

    float target_product = speed / std::max(alpha_hat, 1e-6f);
    float raw_product =
        std::pow(kappa_raw, beta_hat) *
        std::pow(tau_abs, gamma_hat);
    float exponent_sum = beta_hat + gamma_hat;

    float scale = 1.0f;
    if (fabs(exponent_sum) > 0.05f &&
        target_product > 1e-9f &&
        raw_product > 1e-9f)
    {
        scale = std::pow(target_product / raw_product,
                         1.0f / exponent_sum);
        if (!finite_float(scale))
            scale = 1.0f;
    }

    curvature_hat = clampf(kappa_raw * scale,
                           min_curvature,
                           max_curvature);
    torsion_hat = tau_sign * clampf(tau_abs * scale,
                                    min_torsion,
                                    max_torsion);
}

static void update_predicted_frame_and_expected_force()
{
    /*
    Predict the next Frenet-Serret frame and expected force.

    Paper equations:
        T_temp = T + |v| kappa_hat N T_s
        B_temp = B - |v| tau_hat N T_s
        F_expected = m |v|^2 tau_hat B
                   + m |v|^2 kappa_hat N
                   + b0 |v| T
    */

    float speed = vec3_norm(path_vel);
    if (speed < min_speed_for_intention)
    {
        F_expected = Zeros;
        return;
    }

    Vector<3,float> T_temp =
        T_vec + speed * curvature_hat * N_vec * dt;
    Vector<3,float> B_temp =
        B_vec - speed * torsion_hat * N_vec * dt;

    T_hat_next = vec3_unit(T_temp, T_vec);

    Vector<3,float> B_orth =
        B_temp - vec3_dot(B_temp, T_hat_next) * T_hat_next;
    B_hat_next = vec3_unit(B_orth, B_vec);
    N_hat_next = vec3_unit(vec3_cross(B_hat_next, T_hat_next), N_vec);

    Vector<3,float> torsion_force =
        M_inertia * speed * speed * torsion_hat * B_hat_next;
    Vector<3,float> normal_force =
        M_inertia * speed * speed * curvature_hat * N_hat_next;
    Vector<3,float> drive_force = b0 * speed * T_hat_next;

    F_expected = torsion_force + normal_force + drive_force;
}

static void update_directional_damping()
{
    /*
    Guo et al. 2026, Eq. (14)-(15).

    Local damping:
        Bd = diag(b0, mu b0, mu b0),  mu = 1 + 10 |v|

    The local x-axis is aligned with expected force. This creates low
    damping along the estimated intention direction and higher damping
    in the two orthogonal directions.
    */

    float speed = vec3_norm(path_vel);
    mu_guidance = 1.0f + mu_speed_gain * speed;

    B_local = Data(
        b0, 0.0f, 0.0f,
        0.0f, mu_guidance * b0, 0.0f,
        0.0f, 0.0f, mu_guidance * b0
    );

    Vector<3,float> xg = vec3_unit(F_expected, T_hat_next);
    Vector<3,float> zg = B_hat_next;
    Vector<3,float> yg = vec3_cross(zg, xg);

    if (vec3_norm(yg) < 1e-6f)
    {
        yg = makeVector(0.0f, 1.0f, 0.0f);
        if (fabs(vec3_dot(xg, yg)) > 0.9f)
            yg = makeVector(1.0f, 0.0f, 0.0f);

        yg = yg - vec3_dot(yg, xg) * xg;
    }

    yg = vec3_unit(yg, makeVector(0.0f, 1.0f, 0.0f));
    zg = vec3_unit(vec3_cross(xg, yg), makeVector(0.0f, 0.0f, 1.0f));

    R_guidance.T()[0] = xg;
    R_guidance.T()[1] = yg;
    R_guidance.T()[2] = zg;

    B_cart = R_guidance * B_local * R_guidance.T();
}

static void integrate_admittance()
{
    /*
    Mass-damper admittance:
        M Xddot + B Xdot = Fh

    Inputs:
        F_cmd: external force in robot base frame [N].
        B_cart: 3-D virtual damping matrix [Ns/m].
        M_inertia: scalar virtual mass [kg].

    Outputs:
        vel: Cartesian velocity command [m/s, rad/s].
    */

    Vector<3,float> force = F_cmd.slice<0,3>();
    Vector<3,float> vel3 = vel.slice<0,3>();

    Vector<3,float> acc3 = (force - B_cart * vel3) / M_inertia;
    vel3 = vel3 + acc3 * dt;

    float speed = vec3_norm(vel3);
    if (speed > cart_vel_limit)
        vel3 = vel3 * (cart_vel_limit / speed);

    vel[0] = vel3[0];
    vel[1] = vel3[1];
    vel[2] = vel3[2];
    vel[3] = 0.0f;
    vel[4] = 0.0f;
    vel[5] = 0.0f;
}

static void apply_workspace_limits()
{
    if (X[0] <= workspace_xmin && vel[0] < 0.0f) vel[0] = 0.0f;
    if (X[0] >= workspace_xmax && vel[0] > 0.0f) vel[0] = 0.0f;

    if (X[1] <= workspace_ymin && vel[1] < 0.0f) vel[1] = 0.0f;
    if (X[1] >= workspace_ymax && vel[1] > 0.0f) vel[1] = 0.0f;

    if (X[2] <= workspace_zmin && vel[2] < 0.0f) vel[2] = 0.0f;
    if (X[2] >= workspace_zmax && vel[2] > 0.0f) vel[2] = 0.0f;
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
    f << "    \"bmin\": " << bmin << ",\n";
    f << "    \"bmax\": " << bmax << ",\n";
    f << "    \"damping_lambda\": " << damping_lambda << ",\n";
    f << "    \"fmin\": " << fmin << ",\n";
    f << "    \"fmax\": " << fmax << ",\n";
    f << "    \"power_law_window\": " << power_law_window << ",\n";
    f << "    \"power_law_min_points\": "
      << power_law_min_points << ",\n";
    f << "    \"min_speed_for_intention\": "
      << min_speed_for_intention << ",\n";
    f << "    \"min_curvature\": " << min_curvature << ",\n";
    f << "    \"max_curvature\": " << max_curvature << ",\n";
    f << "    \"min_torsion\": " << min_torsion << ",\n";
    f << "    \"max_torsion\": " << max_torsion << ",\n";
    f << "    \"mu_speed_gain\": " << mu_speed_gain << ",\n";
    f << "    \"ft_lpf_alpha\": " << ft_lpf_alpha << ",\n";
    f << "    \"ft_deadband\": " << ft_deadband << ",\n";
    f << "    \"lambda_dls\": " << lambda_dls << ",\n";
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
    float Fz_raw =  FT_local[2] * ft_scale_z;

    Fx_filt = (1.0f - ft_lpf_alpha) * Fx_filt + ft_lpf_alpha * Fx_raw;
    Fy_filt = (1.0f - ft_lpf_alpha) * Fy_filt + ft_lpf_alpha * Fy_raw;
    Fz_filt = (1.0f - ft_lpf_alpha) * Fz_filt + ft_lpf_alpha * Fz_raw;

    Fx_filt = deadbandf(Fx_filt, ft_deadband);
    Fy_filt = deadbandf(Fy_filt, ft_deadband);
    Fz_filt = deadbandf(Fz_filt, ft_deadband);

    F_modified = Zeros;
    F_modified[0] = Fx_filt;
    F_modified[1] = Fy_filt;
    F_modified[2] = Fz_filt;

    F_cmd = Rmat * (R_F_offset * F_modified);

    v_meas = J * Qdot_a;

    b0 = compute_direct_damping_sigmoid(compute_force_magnitude());
    update_motion_geometry();
    estimate_expected_curvature_and_torsion();
    update_predicted_frame_and_expected_force();
    update_directional_damping();
    integrate_admittance();
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
        SubName + "_guo_var_adm_schunk.csv";
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
             << "path_vx,path_vy,path_vz,"
             << "path_ax,path_ay,path_az,"
             << "path_jx,path_jy,path_jz,"
             << "vel1,vel2,vel3,vel4,vel5,vel6,"
             << "b0,mu_guidance,"
             << "alpha_hat,beta_hat,gamma_hat,"
             << "power_law_R2,power_law_RMSE,"
             << "curvature_meas,torsion_meas,"
             << "curvature_hat,torsion_hat,"
             << "T_x,T_y,T_z,N_x,N_y,N_z,B_x,B_y,B_z,"
             << "T_hat_x,T_hat_y,T_hat_z,"
             << "N_hat_x,N_hat_y,N_hat_z,"
             << "B_hat_x,B_hat_y,B_hat_z,"
             << "F_expected_x,F_expected_y,F_expected_z,"
             << "B11,B12,B13,B21,B22,B23,B31,B32,B33,"
             << "indirect_ready\n";

    write_run_hyperparams_json(
        "/home/srisadha/hamid_powerball/src_main/data/admittance/json/" +
        SubName + "_guo_var_adm.hyperparams.json",
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

    Qe = Data(0.0078f, -0.354f, 1.82f, 0.0f, 0.968f, 0.0f);

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

    cout << "Guo 3-D intention admittance loop started! "
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
                 << path_vel[0] << "," << path_vel[1] << ","
                 << path_vel[2] << ","
                 << path_acc[0] << "," << path_acc[1] << ","
                 << path_acc[2] << ","
                 << path_jerk[0] << "," << path_jerk[1] << ","
                 << path_jerk[2] << ","
                 << vel[0] << "," << vel[1] << ","
                 << vel[2] << "," << vel[3] << ","
                 << vel[4] << "," << vel[5] << ","
                 << b0 << "," << mu_guidance << ","
                 << alpha_hat << "," << beta_hat << ","
                 << gamma_hat << ","
                 << power_law_R2 << "," << power_law_RMSE << ","
                 << curvature_meas << "," << torsion_meas << ","
                 << curvature_hat << "," << torsion_hat << ","
                 << T_vec[0] << "," << T_vec[1] << ","
                 << T_vec[2] << ","
                 << N_vec[0] << "," << N_vec[1] << ","
                 << N_vec[2] << ","
                 << B_vec[0] << "," << B_vec[1] << ","
                 << B_vec[2] << ","
                 << T_hat_next[0] << "," << T_hat_next[1] << ","
                 << T_hat_next[2] << ","
                 << N_hat_next[0] << "," << N_hat_next[1] << ","
                 << N_hat_next[2] << ","
                 << B_hat_next[0] << "," << B_hat_next[1] << ","
                 << B_hat_next[2] << ","
                 << F_expected[0] << "," << F_expected[1] << ","
                 << F_expected[2] << ","
                 << B_cart[0][0] << "," << B_cart[0][1] << ","
                 << B_cart[0][2] << ","
                 << B_cart[1][0] << "," << B_cart[1][1] << ","
                 << B_cart[1][2] << ","
                 << B_cart[2][0] << "," << B_cart[2][1] << ","
                 << B_cart[2][2] << ","
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
