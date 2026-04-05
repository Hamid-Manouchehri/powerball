#include <iostream>
#include <cmath>
#include <unistd.h>

#include "powerball/schunk_powerball.h"
#include "powerball/schunk_kinematics.h"

#include <TooN/TooN.h>
#include <TooN/SVD.h>

using namespace std;
using namespace TooN;

static const float dt = 0.005f;   // 200 Hz

enum ControlMode
{
    JOINT_POSITION = 0,
    JOINT_VELOCITY = 1,
    TASK_POSITION  = 2,
    TASK_VELOCITY  = 3
};

static void sleep_dt()
{
    usleep((useconds_t)(dt * 1e6));
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static Vector<6,float> saturate_inf_norm(const Vector<6,float>& qdot, float limit)
{
    Vector<6,float> out = qdot;
    float n = norm_inf(out);
    if (n > limit && n > 1e-8f)
        out *= (limit / n);
    return out;
}

// DLS IK: qdot = J^T (J J^T + lambda^2 I)^-1 v
static Vector<6,float> dls_ik(const Matrix<6,6,float>& J,
                              const Vector<6,float>& v,
                              float lambda_dls)
{
    Matrix<6,6,float> I = Zeros;
    for (int i = 0; i < 6; i++) I[i][i] = 1.0f;

    Matrix<6,6,float> A = J * J.T() + I * (lambda_dls * lambda_dls);
    SVD<6,6,float> svdA(A);
    Vector<6,float> y = svdA.backsub(v);
    return J.T() * y;
}

// Smooth joint-space position move using set_pos()
static void move_joint_position(SchunkPowerball& pb,
                                Kin& kin,
                                const Vector<6,float>& q_target,
                                float move_time_sec)
{
    pb.update();
    Vector<6,float> q0 = pb.get_pos();
    Matrix<3,3,float> R = Zeros;


    int N = (int)(move_time_sec / dt);
    if (N < 1) N = 1;

    for (int k = 1; k <= N; k++)
    {
        float s = (1.0f - cosf((float)M_PI * (float)k / (float)N)) * 0.5f; // cosine blend
        Vector<6,float> q_cmd = q0 + s * (q_target - q0);

        Vector <6,float> q = pb.get_pos();
        kin.FK_R(q, &R);
        cout << R << "\n";


        pb.set_pos(q_cmd);
        pb.update();
        sleep_dt();
    }
}

// Joint velocity mode: constant qdot for duration
static void move_joint_velocity(SchunkPowerball& pb,
                                Vector<6,float> qdot_cmd,
                                float duration_sec,
                                float qdot_limit)
{
    qdot_cmd = saturate_inf_norm(qdot_cmd, qdot_limit);

    pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);
    pb.update();

    int N = (int)(duration_sec / dt);
    for (int k = 0; k < N; k++)
    {
        pb.update();
        pb.set_vel(qdot_cmd);
        sleep_dt();
    }

    pb.set_vel(Zeros);
    pb.update();
}

// Task velocity mode: constant Cartesian velocity for duration
static void move_task_velocity(SchunkPowerball& pb,
                               Kin& kin,
                               Vector<6,float> v_cmd,
                               float duration_sec,
                               float lambda_dls,
                               float qdot_limit)
{
    pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);
    pb.update();

    int N = (int)(duration_sec / dt);
    for (int k = 0; k < N; k++)
    {
        pb.update();
        Vector<6,float> q = pb.get_pos();

        Matrix<6,6,float> J = Zeros;
        kin.Jacob(q, &J);

        Vector<6,float> qdot_cmd = dls_ik(J, v_cmd, lambda_dls);
        qdot_cmd = saturate_inf_norm(qdot_cmd, qdot_limit);

        pb.set_vel(qdot_cmd);
        sleep_dt();
    }

    pb.set_vel(Zeros);
    pb.update();
}

// Task position mode: proportional Cartesian position controller + DLS IK
static void move_task_position(SchunkPowerball& pb,
                               Kin& kin,
                               const Vector<3,float>& x_target,
                               const Matrix<3,3,float>& Rd,
                               float duration_sec,
                               float kp_pos,
                               float kp_rot,
                               float vmax_xyz,
                               float vmax_rot,
                               float lambda_dls,
                               float qdot_limit)
{
    pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);
    pb.update();

    int N = (int)(duration_sec / dt);
    for (int k = 0; k < N; k++)
    {
        pb.update();
        Vector<6,float> q = pb.get_pos();

        Vector<3,float> x = Zeros;
        Matrix<3,3,float> R = Zeros;
        
        kin.FK_pos(q, &x);
        kin.FK_R(q, &R);
        
        cout << R << "\n";

        Vector<3,float> ex = x_target - x;
        
        Vector<3,float> ne = R.T()[0];
        Vector<3,float> se = R.T()[1];
        Vector<3,float> ae = R.T()[2];
        
        Vector<3,float> nd = Rd.T()[0];
        Vector<3,float> sd = Rd.T()[1];
        Vector<3,float> ad = Rd.T()[2];
        
        Vector<3,float> eo = 0.5f*((ne ^ nd) + (se ^ sd) + (ae ^ ad));

        Vector<6,float> v_cmd = Zeros;
        v_cmd[0] = clampf(kp_pos * ex[0], -vmax_xyz, vmax_xyz);
        v_cmd[1] = clampf(kp_pos * ex[1], -vmax_xyz, vmax_xyz);
        v_cmd[2] = clampf(kp_pos * ex[2], -vmax_xyz, vmax_xyz);
        
        // v_cmd[3] = clampf(kp_rot * eo[0], -vmax_rot, vmax_rot);
        // v_cmd[4] = clampf(kp_rot * eo[1], -vmax_rot, vmax_rot);
        // v_cmd[5] = clampf(kp_rot * eo[2], -vmax_rot, vmax_rot);
        
        v_cmd[3] = kp_rot * eo[0];
        v_cmd[4] = kp_rot * eo[1];
        v_cmd[5] = kp_rot * eo[2];

        Matrix<6,6,float> J = Zeros;
        kin.Jacob(q, &J);

        Vector<6,float> qdot_cmd = dls_ik(J, v_cmd, lambda_dls);
        qdot_cmd = saturate_inf_norm(qdot_cmd, qdot_limit);

        pb.set_vel(qdot_cmd);
        sleep_dt();
    }

    pb.set_vel(Zeros);
    pb.update();
}

int main()
{
    SchunkPowerball pb;
    Kin kin;
    
    // ---------- choose one ---------
    ControlMode mode = TASK_POSITION;  // TODO: [JOINT_POSITION, JOINT_VELOCITY, TASK_POSITION, TASK_VELOCITY]
    // -------------------------------

    const float qdot_limit = 30.0f * (float)M_PI / 180.0f; // rad/s
    const float lambda_dls = 0.10f;

    pb.update();
    Vector<6,float> q = pb.get_pos();

    // Example targets
    // Vector<6,float> q_target = Data(0.0f, -M_PI/6.0f, M_PI/2.0f, 0.0f, M_PI/3.0f, 0.0f);  // TODO
    Vector<6,float> q_target = Data(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);  // TODO
    Vector<6,float> qdot_cmd = makeVector(5, 0, 0, 0, 0, 0) * ((float)M_PI / 180.0f); // TODO; rad/s
    Vector<6,float> v_cmd = Zeros;  // TODO
    v_cmd[0] = 0.03f; // 3 cm/s in x

    Vector<3,float> x_now = Zeros;
    kin.FK_pos(q, &x_now);
    // Vector<3,float> x_target = x_now + makeVector(0.05f, 0.00f, 0.00f); // move +5 cm in x
    Vector<3,float> x_target = Data(0.44, -0.4, 0.1171);
    Matrix<3,3,float> Rd = Data(-1.0f, -0.0f, -0.0f,
                                -0.0f, 1.0f, 0.0f,
                                0.0f, 0.0f, -1.0f
    );

    cout << "Starting..." << endl;

    switch (mode)
    {
        case JOINT_POSITION:
            move_joint_position(pb, kin, q_target, 3.0f);
            break;

        case JOINT_VELOCITY:
            move_joint_velocity(pb, qdot_cmd, 2.0f, qdot_limit);
            break;

        case TASK_VELOCITY:
            move_task_velocity(pb, kin, v_cmd, 2.0f, lambda_dls, qdot_limit);
            break;

        case TASK_POSITION:
            move_task_position(pb, kin, x_target, Rd, 10.0f, 2.0f, 1.0f, 0.04f, 0.3f, lambda_dls, qdot_limit);
            break;

        default:
            break;
    }

    pb.set_vel(Zeros);
    pb.update();
    pb.shutdown_motors();
    pb.update();

    cout << "Done." << endl;
    return 0;
}