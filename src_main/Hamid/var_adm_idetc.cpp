// Cleaned version (NO new libraries added)
// - Removed unused pieces (sgn, timeval/gettimeofday timestamp)
// - Removed "thread per loop iteration" (computations now called directly)
// - Made V-REP thread stoppable
// - Made TCP receive thread stoppable (checks stop flag)
// - Kept the same libraries you already included

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>

#include <chrono>

#include "powerball/schunk_powerball.h"
#include "vrep/v_repClass.h"
#include "powerball/schunk_kinematics.h"

#include <TooN/LU.h>
#include <TooN/SVD.h>

using namespace std;
using boost::asio::ip::tcp;
using namespace TooN;

// -------------------- Globals (shared between threads) --------------------
Vector<6,float> FT = Zeros;
simxFloat newPos[3] = {0.0f, 0.439f, 0.275f};

// Control loop
static const float dt = 0.005f;

Vector<6,float> Q      = Zeros;
Vector<6,float> Qe     = Zeros;
Vector<6,float> Qdot   = Zeros;
Vector<6,float> Qdot_a = Zeros;

// Admittance time (seconds)
float adm_time = 0.0f;

// Oscillation parameters
const float osc_start_time = 5.0f;
const float osc_duration   = 10.0f;
const float osc_omega      = 2.0f * M_PI * 0.5f; // 0.5 Hz
const float osc_amplitude  = 0.01f;              // [m/s]

// Admittance parameters
float Damp = 10.0f;

Vector<6,float> Md_diag = makeVector(1,1,1,1,1,1) * 0.3f;
Matrix<6,6,double> Md_inv = Md_diag.as_diagonal();

Vector<6,float> Cd_diag = makeVector(1.2f,1.0f,1.0f,1.0f,1.0f,1.0f) * Damp;
Matrix<6,6,double> Cd = Cd_diag.as_diagonal();

Vector<6,float> vel = Zeros;

// Fixed force-frame rotation offset
Matrix<6,6,float> R_F_offset = Data(
    cos(M_PI/2), -sin(M_PI/2), 0,  0,0,0,
    sin(M_PI/2),  cos(M_PI/2), 0,  0,0,0,
    0,            0,           1,  0,0,0,
    0,0,0,  cos(M_PI/2), -sin(M_PI/2), 0,
    0,0,0,  sin(M_PI/2),  cos(M_PI/2), 0,
    0,0,0,  0,            0,           1
);

// -------------------- Small helpers --------------------
static long long now_us()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

static void sleep_to_keep_dt(std::chrono::time_point<std::chrono::system_clock> t0)
{
    using namespace std::chrono;
    duration<float> elapsed = system_clock::now() - t0;
    float sleep_s = dt - elapsed.count();
    if (sleep_s > 0) usleep((useconds_t)(sleep_s * 1e6));
}

// Stop thread: press Enter
void stop(bool* stopFlag)
{
    char in;
    cin.get(in);
    *stopFlag = true;
}

// V-REP thread (stoppable)
int vrep_draw(bool* stopFlag)
{
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

// Compute admittance -> Qdot
void computations()
{
    Kin kin;

    Matrix<6,6,float> J = Zeros;
    Matrix<3,3,float> R = Zeros;
    Matrix<6,6,float> Rmat = Zeros;
    Vector<3,float> X = Zeros;

    // Kinematics
    kin.Jacob(Q, &J);
    kin.FK_R(Q, &R);
    kin.FK_pos(Q, &X);

    // update V-REP visualization
    newPos[0] = -X[1];
    newPos[1] =  X[0];
    newPos[2] =  X[2];

    // build 6x6 rotation block matrix from R
    Rmat.slice<0,0,3,3>() = R;
    Rmat.slice<3,3,3,3>() = R;

    // Use only planar forces derived from FT moments (your original logic)
    Vector<6,float> F_modified = Zeros;
    F_modified[0] = -FT[3] * 10.0f;
    F_modified[1] = -FT[4] * 10.0f;

    // Add oscillation along the applied planar force direction
    if (adm_time >= osc_start_time && adm_time < (osc_start_time + osc_duration))
    {
        float t_rel = adm_time - osc_start_time;
        float osc = osc_amplitude * sinf(osc_omega * t_rel);

        float fx = F_modified[0];
        float fy = F_modified[1];
        float n  = sqrtf(fx*fx + fy*fy);

        if (n > 1e-4f)
        {
            F_modified[0] += osc * fx / n;
            F_modified[1] += osc * fy / n;
        }
    }

    // Geometrical adaptation (your original)
    float C1 = 0.075f, C2 = 0.125f;
    float alpha = 1.0f;
    float xx = 0.0f;

    if (X[1] <= C1)
    {
        alpha = 1.0f;
    }
    else if (X[1] < C2)
    {
        xx = (X[1] - C1) * 4.0f / (C2 - C1) - 2.0f;
        alpha = tanh(xx) / 0.964f * 1.5f + 2.5f;
    }
    else
    {
        alpha = 3.5f;
    }

    // Admittance: vel += dt * (M^-1 * (R*offset*F) - M^-1 * alpha*C*vel)
    vel = vel + dt * (Md_inv * (Rmat * (R_F_offset * F_modified)) - Md_inv * (alpha * Cd) * vel);

    // Solve J * Qdot = vel using SVD pseudo-inverse
    SVD<6,6,float> svdJ(J);
    Qdot = svdJ.backsub(vel);
}

// TCP FT receive thread (stoppable)
void TCP_receive(bool* stopFlag)
{
    boost::asio::io_service io_service;
    tcp::endpoint sender_endpoint(
        boost::asio::ip::address::from_string("192.168.1.30"),
        boost::lexical_cast<int>("1000")
    );

    tcp::socket socket(io_service);
    socket.connect(sender_endpoint);

    boost::system::error_code ignored_error;
    char recv_buf[128];

    // TARE
    {
        std::string msg = "TARE(1)\n";
        socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
        socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    }

    // Continuous stream
    {
        std::string msg = "L1()\n";
        socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
        socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    }

    while (!(*stopFlag))
    {
        int len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
        if (len <= 0) continue;

        int timeStamp = 0;
        sscanf(recv_buf, "F={%f,%f,%f,%f,%f,%f},%d",
               &FT[0], &FT[1], &FT[2], &FT[3], &FT[4], &FT[5], &timeStamp);
    }
}

// -------------------- main --------------------
int main(int argc, char** argv)
{
    string SubName;
    cout << "What is subject name? ";
    cin >> SubName;
    cout << "Please wait " << SubName << endl;
    cin.get(); // consume newline

    // Threads stop flag
    bool stop_flag = false;

    // V-REP thread
    boost::thread vrep_thread(vrep_draw, &stop_flag);

    // Log file
    std::string filepath = "../data/hamid/exp_1/" + SubName + "_damp_" + std::to_string((int)Damp) + "_01.csv";
    std::ofstream dataFile(filepath);
    dataFile << "Time_us,"
             << "Q1,Q2,Q3,Q4,Q5,Q6,"
             << "dQ1,dQ2,dQ3,dQ4,dQ5,dQ6,"
             << "FT1,FT2,FT3,FT4,FT5,FT6,"
             << "Vx,Vy,Vz,omega_x,omega_y,omega_z\n";

    // Robot connect
    SchunkPowerball pb;
    pb.update();
    Q = pb.get_pos();

    // Stop thread
    boost::thread stop_thread(stop, &stop_flag);

    // Go to start pose (simple cosine blend)
    Qe = Data(0.0f, -M_PI/6, M_PI/2, 0.0f, M_PI/3, 0.0f);
    Vector<6,float> dQ = Qe - Q;

    float maxq = 0.0f;
    for (int i = 0; i < 6; i++) maxq = std::max(maxq, (float)fabs(dQ[i]));

    float Ttravel = std::max(1.0f, maxq * 3.0f);
    int itNum = (int)(Ttravel / dt);

    Vector<6,float> Qhold = Q;
    for (int k = 1; k <= itNum && !stop_flag; k++)
    {
        float s = (1.0f - cos((float)k / (float)itNum * (float)M_PI)) * 0.5f;
        Vector<6,float> Qt = s * dQ + Qhold;

        pb.set_pos(Qt);
        pb.update();
        Q = pb.get_pos();
        usleep((useconds_t)(dt * 1e6));
    }

    // Velocity mode
    pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);
    pb.update();

    // FT thread
    boost::thread FT_thread(TCP_receive, &stop_flag);

    cout << "Admittance loop started! (press Enter to stop)\n";

    // Run for 90 seconds max (same behavior as your old cnt<90/dt)
    const int max_steps = (int)(90.0f / dt);

    for (int cnt = 0; cnt < max_steps && !stop_flag; cnt++)
    {
        auto t0 = std::chrono::system_clock::now();

        adm_time = cnt * dt;

        // Update robot state first
        pb.update();
        Q = pb.get_pos();
        Qdot_a = pb.get_vel();

        // Compute Qdot (NO thread creation here)
        computations();

        // Velocity saturation (same threshold you used)
        if (TooN::norm_inf(Qdot) <= 32.5f * (float)M_PI / 180.0f)
            pb.set_vel(Qdot);

        // log
        long long t_us = now_us();
        dataFile << t_us << ","
                 << Q[0] << "," << Q[1] << "," << Q[2] << "," << Q[3] << "," << Q[4] << "," << Q[5] << ","
                 << Qdot_a[0] << "," << Qdot_a[1] << "," << Qdot_a[2] << "," << Qdot_a[3] << "," << Qdot_a[4] << "," << Qdot_a[5] << ","
                 << FT[0] << "," << FT[1] << "," << FT[2] << "," << FT[3] << "," << FT[4] << "," << FT[5] << ","
                 << vel[0] << "," << vel[1] << "," << vel[2] << "," << vel[3] << "," << vel[4] << "," << vel[5] << "\n";

        sleep_to_keep_dt(t0);
    }

    // Shutdown
    dataFile.close();

    stop_flag = true; // tell threads to exit

    FT_thread.interrupt();
    stop_thread.interrupt();
    vrep_thread.interrupt();

    pb.shutdown_motors();
    pb.update();

    usleep(500 * 1000);
    cout << "Exiting ...\n";
    return 0;
}