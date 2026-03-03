// Clean version: Myo + FT + Schunk Powerball admittance + (optional) V-REP visualization
// Removed: ALL VSM/dynamixel stuff
// Added: NO new libraries (only removed unused ones)

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <ostream>
#include <typeinfo>

// boost headers (keep, because you already use boost::thread)
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>

// schunk powerball + vrep + utils
#include "powerball/schunk_powerball.h"
#include "vrep/v_repClass.h"
#include "powerball/schunk_kinematics.h"
#include "utils/utils.h"
#include "utils/powerball_utils.h"

// Toon headers
#include <TooN/LU.h>
#include <TooN/SVD.h>

// myo band headers
#include "myolinux/myoclient.h"
#include "myolinux/serial.h"

using namespace std;
using namespace TooN;
using namespace myolinux;

// -------------------- User settings --------------------
static const float dt = 0.005f;                 // control loop time step [s]
static const char* MYO_PORT = "/dev/ttyACM0";   // myo serial device
static const int   MYO_BAUD = 115200;

static const std::string DATA_DIR = "../data/no_vsm/peg-hole"; // make this folder manually
// -------------------------------------------------------

// -------------------- Global shared states --------------------

// Myo client (one instance only)
myo::Client client(Serial{MYO_PORT, MYO_BAUD});

// Myo data
Vector<8, int>   EMG = Zeros;
Vector<4, float> ORI = Zeros;
Vector<3, float> ACC = Zeros;
Vector<3, float> GYR = Zeros;

// Robot state
Vector<6, float> Q     = Zeros;    // joint position
Vector<6, float> Qs    = Zeros;
Vector<6, float> Qe    = Zeros;
Vector<6, float> Qdot  = Zeros;    // commanded joint velocity
Vector<6, float> Qdot_a= Zeros;    // measured joint velocity
Vector<3, float> X     = Zeros;    // end-effector position
Vector<3, float> X_init= Zeros;

// V-REP visualization point (sphere)
simxFloat newPos[3] = {0.0f, 0.439f, 0.275f};

// Logging control (when true, Myo thread writes)
bool Start_record = false;

// Admittance parameters (simple diagonal)
Vector<6, float> Md_diag = makeVector(1,1,1,1,1,1) * 0.05f;
Matrix<6,6, double> Md_inv = Md_diag.as_diagonal();

Vector<6, float> Cd_diag = makeVector(200,200,200,200,200,200); // default high damping
Matrix<6,6, double> Cd;

Vector<6, float> vel = Zeros; // end-effector velocity state for admittance

// FT sensor vector is assumed to be provided by your existing codebase.
// (It was used in your original code too.)
extern Vector<6, float> FT;

// -------------------- Small helpers --------------------

// returns current time in microseconds since epoch
static long long now_us()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

// Stop thread: press Enter to stop program
void stop(bool* stopFlag)
{
    char in;
    cin.get(in);
    *stopFlag = true;
    Start_record = false;
}

// Optional V-REP thread: draws a sphere at newPos
int vrep_draw(bool* stopFlag)
{
    if (*stopFlag) return 0;

    V_rep vrep;
    int res = vrep.connect();
    if (res == -1)
    {
        cout << "V-REP Connection Error!" << endl;
        return 0;
    }

    while (!(*stopFlag))
    {
        vrep.setSphere(&newPos[0]);
        usleep(40 * 1000);
    }
    return 0;
}

// -------------------- Myo --------------------

void Myo_init()
{
    client.connect();
    if (!client.connected())
    {
        cout << "Unable to connect to Myo band" << endl;
        exit(1);
    }
    cout << "Myo connected" << endl;

    client.setSleepMode(myo::SleepMode::NeverSleep);
    client.setMode(myo::EmgMode::SendEmg,
                   myo::ImuMode::SendData,
                   myo::ClassifierMode::Disabled);

    client.onEmg([](myo::EmgSample sample)
    {
        for (size_t i = 0; i < 8; i++)
            EMG[i] = static_cast<int>(sample[i]);
    });

    client.onImu([](myo::OrientationSample ori,
                    myo::AccelerometerSample acc,
                    myo::GyroscopeSample gyr)
    {
        for (size_t i = 0; i < 4; i++)
        {
            ORI[i] = ori[i];
            if (i < 3)
            {
                ACC[i] = acc[i];
                GYR[i] = gyr[i];
            }
        }
    });
}

// Myo logger thread: writes EMG + IMU continuously when Start_record=true
void Myo_receive(bool* stopFlag)
{
    std::ofstream EMGFile(DATA_DIR + "/EMG.csv");
    std::ofstream IMUFile(DATA_DIR + "/IMU.csv");

    EMGFile << "Time_us,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8\n";
    IMUFile << "Time_us,ORI1,ORI2,ORI3,ORI4,ACC1,ACC2,ACC3,GYR1,GYR2,GYR3\n";

    while (!(*stopFlag))
    {
        try
        {
            client.listen(); // blocks until a Myo packet arrives (calls callbacks)

            if (!Start_record) continue;

            long long t = now_us();

            EMGFile << t
                    << "," << EMG[0] << "," << EMG[1] << "," << EMG[2] << "," << EMG[3]
                    << "," << EMG[4] << "," << EMG[5] << "," << EMG[6] << "," << EMG[7] << "\n";

            IMUFile << t
                    << "," << ORI[0] << "," << ORI[1] << "," << ORI[2] << "," << ORI[3]
                    << "," << ACC[0] << "," << ACC[1] << "," << ACC[2]
                    << "," << GYR[0] << "," << GYR[1] << "," << GYR[2] << "\n";
        }
        catch (myo::DisconnectedException&)
        {
            cout << "MYO Disconnected" << endl;
        }
    }

    EMGFile.close();
    IMUFile.close();
}

// 10s EMG calibration logger
void EMG_calib(bool* calibFlag)
{
    std::ofstream EMGFile(DATA_DIR + "/emg_calib.csv");
    EMGFile << "Time_us,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8\n";

    auto t0 = std::chrono::steady_clock::now();

    while (*calibFlag)
    {
        client.listen();

        long long t = now_us();
        EMGFile << t
                << "," << EMG[0] << "," << EMG[1] << "," << EMG[2] << "," << EMG[3]
                << "," << EMG[4] << "," << EMG[5] << "," << EMG[6] << "," << EMG[7] << "\n";

        float elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count();
        if (elapsed > 10.0f) *calibFlag = false;
    }

    EMGFile.close();
}

// -------------------- Admittance computation --------------------
// Reads current Q, FT -> computes vel -> computes Qdot using Jacobian pseudoinverse
void admittance_compute()
{
    Kin kin;

    Matrix<6,6,float> J = Zeros;
    Matrix<3,3,float> Rb_e = Zeros;

    // 1) Kinematics
    kin.Jacob(Q, &J);
    kin.FK_R(Q, &Rb_e);
    kin.FK_pos(Q, &X);

    // 2) Update V-REP sphere (simple mapping)
    newPos[0] = -(X[1] - X_init[1]) + 0.06f;
    newPos[1] =  (X[0] - X_init[0]) + 0.439f;
    newPos[2] = 0.01f;

    // 3) Force transform (FT frame -> base frame). Keep your original fixed rotation.
    Matrix<3,3,float> Re_f = Data(cos(-M_PI/4), -sin(-M_PI/4), 0,
                                  sin(-M_PI/4),  cos(-M_PI/4), 0,
                                  0,            0,           1);

    Matrix<6,6,float> Rb_e_mat = Zeros;
    Matrix<6,6,float> Re_f_mat = Zeros;
    Rb_e_mat.slice<0,0,3,3>() = Rb_e;
    Rb_e_mat.slice<3,3,3,3>() = Rb_e;
    Re_f_mat.slice<0,0,3,3>() = Re_f;
    Re_f_mat.slice<3,3,3,3>() = Re_f;

    Vector<6,float> FT_local = FT;
    FT_local.slice<3,3>() = Zeros; // ignore moments -> no rotation

    Vector<6,float> F_modified = Rb_e_mat * Re_f_mat * FT_local;

    // 4) Admittance dynamics: vel += dt * (M^-1(F - C vel))
    vel = vel + dt * (Md_inv * F_modified - Md_inv * Cd * vel);
    vel.slice<3,3>() = Zeros; // make sure angular vel is zero

    // 5) Convert Cartesian vel to joint vel: J * Qdot = vel
    SVD<6,6,float> svdJ(J);
    Qdot = svdJ.backsub(vel);
}

// ---------------------------- MAIN ----------------------------
int main(int argc, char** argv)
{
    bool use_vrep = true;
    bool calib_mode = false;
    bool stopFlag = false;
    bool FT_calibrated = false;

    // CLI: -h, -calib, ld, hd
    if (argc > 1)
    {
        if (strcmp(argv[1], "-h") == 0)
        {
            cout << "Usage:\n";
            cout << "  ./admittance -calib\n";
            cout << "  ./admittance ld\n";
            cout << "  ./admittance hd\n";
            return 0;
        }
        if (strcmp(argv[1], "-calib") == 0)
        {
            calib_mode = true;
        }
        if (strcmp(argv[1], "ld") == 0)
        {
            Cd_diag = makeVector(80.0f,80.0f,200.0f,200.0f,200.0f,200.0f);
            cout << "Damping: LOW\n";
        }
        if (strcmp(argv[1], "hd") == 0)
        {
            Cd_diag = makeVector(200.0f,200.0f,200.0f,200.0f,200.0f,200.0f);
            cout << "Damping: HIGH\n";
        }
    }
    Cd = Cd_diag.as_diagonal();

    // 1) Init Myo
    Myo_init();

    // Calibration only
    if (calib_mode)
    {
        cout << "EMG calibration for 10 seconds...\n";
        bool calibFlag = true;
        boost::thread Calib_thread(EMG_calib, &calibFlag);
        Calib_thread.join();
        cout << "Done.\n";
        return 0;
    }

    // 2) Start Myo logger thread
    boost::thread Myo_thread(Myo_receive, &stopFlag);

    // 3) Optional V-REP thread
    boost::thread vrep_thread;
    if (use_vrep)
        vrep_thread = boost::thread(vrep_draw, &stopFlag);

    // 4) Stop thread (press Enter)
    boost::thread stop_thread(stop, &stopFlag);

    // 5) FT sensor thread (your existing function)
    // IMPORTANT: Robotiq_ft_receive expects (float, bool, bool*, bool*)
    boost::thread FT_thread(Robotiq_ft_receive, dt, true, &stopFlag, &FT_calibrated);
    while (!FT_calibrated) {} // wait until FT is calibrated

    // 6) Robot init + move to start pose
    SchunkPowerball pb;
    Kin kin;
    pb.update();

    Qs = pb.get_pos();
    Qe = makeVector(-30.0f,-10.0f,95.0f,0.0f,74.0f,105.0f) * (float)(M_PI/180.0);

    vector<vector<double>> matrix;
    float T_travel = calculate_travel_time(Qs, Qe, 5);
    cout << "Travel time to start pose: " << T_travel << " sec\n";

    kin.HerInter(Qs, Zeros, Qe, dt, T_travel, &matrix);

    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;

    Vector<6,float> Q_interm = Zeros;

    for (int i = 0; i < (int)matrix.size(); i++)
    {
        timeLoop = std::chrono::system_clock::now();

        for (int j = 0; j < 6; j++)
            Q_interm[j] = (float)matrix[i][j];

        pb.set_pos(Q_interm);
        pb.update();

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        float sleep_s = dt - elaps_loop.count();
        if (sleep_s > 0) usleep((useconds_t)(sleep_s * 1e6));
    }

    pb.update();
    Q = pb.get_pos();
    kin.FK_pos(Q, &X_init);

    // Velocity mode
    pb.set_sdo_controlword(NODE_ALL, STATUS_OPERATION_ENABLED);
    pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);
    pb.update();

    // 7) Main log file (NO VSM fields)
    std::ofstream dataFile(DATA_DIR + "/admittance.csv");
    dataFile << "Time_us,"
             << "Q1,Q2,Q3,Q4,Q5,Q6,"
             << "Qdot1,Qdot2,Qdot3,Qdot4,Qdot5,Qdot6,"
             << "x,y,z,"
             << "xdot,ydot,zdot,"
             << "FT1,FT2,FT3,FT4,FT5,FT6\n";

    cout << "Admittance loop started. Press Enter to stop.\n";

    // start Myo logging too
    Start_record = true;

    // 8) Control loop
    while (!stopFlag)
    {
        timeLoop = std::chrono::system_clock::now();

        pb.update();
        Q = pb.get_pos();
        Qdot_a = pb.get_vel();

        // compute Qdot from FT
        admittance_compute();

        // safety saturation
        if (TooN::norm_inf(Qdot) > 50.0f * (float)(M_PI/180.0))
            Qdot = Zeros;

        pb.set_vel(Qdot);

        // log
        long long t = now_us();
        dataFile << t << ","
                 << Q[0] << "," << Q[1] << "," << Q[2] << "," << Q[3] << "," << Q[4] << "," << Q[5] << ","
                 << Qdot[0] << "," << Qdot[1] << "," << Qdot[2] << "," << Qdot[3] << "," << Qdot[4] << "," << Qdot[5] << ","
                 << X[0] << "," << X[1] << "," << X[2] << ","
                 << vel[0] << "," << vel[1] << "," << vel[2] << ","
                 << FT[0] << "," << FT[1] << "," << FT[2] << "," << FT[3] << "," << FT[4] << "," << FT[5] << "\n";

        // keep dt timing
        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        float sleep_s = dt - elaps_loop.count();
        if (sleep_s > 0) usleep((useconds_t)(sleep_s * 1e6));
    }

    // 9) Shutdown
    dataFile.close();
    Start_record = false;

    Myo_thread.interrupt();
    FT_thread.interrupt();
    stop_thread.interrupt();
    if (use_vrep) vrep_thread.interrupt();

    pb.update();
    cout << "Exiting...\n";
    return 0;
}