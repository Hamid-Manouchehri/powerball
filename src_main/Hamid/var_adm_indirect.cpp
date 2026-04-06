/*
Implementation of the paper: Variable Admittance Control of Robot Manipulators Based on Human Intention
Gitae Kang (2019)
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

#include "powerball/schunk_powerball.h"
#include "vrep/v_repClass.h"
#include "powerball/schunk_kinematics.h"

#include <TooN/LU.h>
#include <TooN/SVD.h>

// Myo band headers
#include "myolinux/myoclient.h"
#include "myolinux/serial.h"

using namespace std;
using boost::asio::ip::tcp;
using namespace TooN;
using namespace myolinux;

// -------------------- Globals (shared between threads) --------------------
// FT is defined in powerball_utils.cpp and shared across the program.
extern Vector<6,float> FT;
simxFloat newPos[3] = {0.0f, 0.439f, 0.275f};

// Control loop
static const float dt = 0.005f;
static const float z_ref = 0.11563f;

// Set to true to read and record Myo band (EMG/IMU) to a separate CSV; false to skip Myo.
static const bool USE_MYO = false;  // TODO
static const bool CONSTANT_DAMP = false; // TODO true: const damping, false: variable damping
float const_damp = 10;  // TODO
float cmin_pos = 20.0f;  // TODO
float cmax_pos = 100.0f; // TODO
float M_inertia = 8.0f;  // TODO

float ft_scale_x = 1.0f;
float ft_scale_y = 1.0f;

float ft_lpf_alpha   = 0.5f;   // smaller = smoother alpha=[0 1]
float ft_deadband    = 0.05f;   // N

float eps_v_meas     = 0.02f;   // TODO; avoid huge damping near zero speed
float c_rate_max     = 100.0f;   // TODO; damping slew rate
float dc_max = c_rate_max * dt;  // damping slew-rate limit (rate limiter)

float lambda_dls     = 0.12f;   // damped least-squares IK

float qdot_lpf_alpha = 0.20f;   // joint velocity smoothing; smaller = smoother
float qdot_limit     = 30.0f * (float)M_PI / 180.0f;  //  deg/s -> rad/s

float ref_var_x_prev = 0.0f
float ref_var_y_prev = 0.0f
float ref_var_lpf_alpha = 0.5f;  // TODO

float Fstd_pos = 3.0f;    // TODO
float Fx_filt  = 0.0f;
float Fy_filt  = 0.0f;
float adm_time = 0.0f;

float c_pos_x_prev = cmax_pos;
float c_pos_y_prev = cmax_pos;

Vector<6,float> Q      = Zeros;
Vector<6,float> Qe     = Zeros;
Vector<6,float> Qdot   = Zeros;
Vector<6,float> Qdot_a = Zeros;  // Schunk jonit velocity

Vector<6,float> v_meas_lpf = Zeros;
Vector<6,float> v_meas_lpf_prev = Zeros;
float v_meas_lpf_alpha = 0.2f;

Vector<3,float> X = Zeros;
Vector<6,float> F_modified = Zeros;
Vector<6,float> F_cmd = Zeros;
Vector<6,float> v_meas = Zeros;
Vector<6,float> Qdot_cmd = Zeros;
Vector<2,float> c_des = Zeros;

// Vector<6,float> Md_diag = makeVector(0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f);  // virtual inertia vector
Vector<6,float> Md_diag = Ones * M_inertia;  // virtual inertia vector
Matrix<6,6,float> Md_inv = Zeros;

Vector<6,float> Cd_diag = makeVector(cmax_pos, cmax_pos, cmax_pos, cmax_pos, cmax_pos, cmax_pos);  // virtual damping vector
Matrix<6,6,float> Cd = Cd_diag.as_diagonal();

Vector<6,float> vel = Zeros;
Vector<6,float> vel_prev = Zeros;
Vector<6,float> ref_var = Zeros;  // reference variable
Vector<6,float> ref_var_prev = Zeros;  // reference variable
Vector<6,float> Qdot_prev = Zeros;
std::mutex ft_mutex;

Vector<3,float> B_vec = makeVector(0,0,1);
Vector<3,float> T_vec = Zeros;
Vector<3,float> T_vec_prev = Zeros;
Vector<3,float> N_vec = Zeros;

static float deadbandf(float x, float db)
{
    if (fabs(x) < db) return 0.0f;
    return x;
}   

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

struct velHysteresis{
    float v_enter;
    float v_exit;
    bool moving;
    
    velHysteresis(float enter_th, float exit_th): v_enter(enter_th), v_exit(exit_th), moving(false){}
    
    float apply(float v){
        float av = fabsf(v);
        if(!moving){
            if (av > v_enter)
                moving = true;
        }
        else{
            if(av < v_exit)
                moving = false;
        }
        
        return moving ? v : 0.0f;
    }
};

velHysteresis hyst_x(0.02f, 0.01f);
velHysteresis hyst_y(0.02f, 0.01f);

static void write_run_hyperparams_json(
    const std::string& json_path,
    const std::string& subject,
    const std::string& data_csv_path,
    bool use_myo
)
{
    std::ofstream f(json_path);
    if (!f.is_open())
    {
        std::cerr << "WARN: unable to open hyperparams file: " << json_path << std::endl;
        return;
    }

    // Keep it dependency-free: write JSON manually (numbers/bools/strings only).
    f << "{\n";
    f << "  \"subject\": " << "\"" << subject << "\",\n";
    f << "  \"data_csv_path\": " << "\"" << data_csv_path << "\",\n";
    f << "  \"use_myo\": " << (use_myo ? "true" : "false") << ",\n";
    f << "  \"build\": {\n";
    f << "    \"date\": " << "\"" << __DATE__ << "\",\n";
    f << "    \"time\": " << "\"" << __TIME__ << "\"\n";
    f << "  },\n";
    f << "  \"hyperparams\": {\n";
    f << "    \"dt\": " << dt << ",\n";
    f << "    \"ft_scale_x\": " << ft_scale_x << ",\n";
    f << "    \"ft_scale_y\": " << ft_scale_y << ",\n";
    f << "    \"ft_lpf_alpha\": " << ft_lpf_alpha << ",\n";
    f << "    \"ft_deadband\": " << ft_deadband << ",\n";
    f << "    \"eps_v_meas\": " << eps_v_meas << ",\n";
    f << "    \"c_rate_max\": " << c_rate_max << ",\n";
    f << "    \"lambda_dls\": " << lambda_dls << ",\n";
    f << "    \"qdot_lpf_alpha\": " << qdot_lpf_alpha << ",\n";
    f << "    \"qdot_limit\": " << qdot_limit << ",\n";
    f << "    \"Fstd_pos\": " << Fstd_pos << ",\n";
    f << "    \"cmin_pos\": " << cmin_pos << ",\n";
    f << "    \"cmax_pos\": " << cmax_pos << ",\n";
    f << "    \"Md_diag\": [" << Md_diag[0] << "," << Md_diag[1] << "," << Md_diag[2] << "," << Md_diag[3] << "," << Md_diag[4] << "," << Md_diag[5] << "]\n";
    f << "  }\n";
    f << "}\n";
}

static void sleep_to_keep_dt(std::chrono::time_point<std::chrono::system_clock> t0)
{
    using namespace std::chrono;
    duration<float> elapsed = system_clock::now() - t0;
    float sleep_s = dt - elapsed.count();
    if (sleep_s > 0) usleep((useconds_t)(sleep_s * 1e6));
}

// Stop thread: press Enter
void stop(std::atomic<bool>* stopFlag)
{
    char in;
    cin.get(in);
    stopFlag->store(true);
}

// V-REP thread (stoppable)
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

// -------------------- Myo logging (separate file) --------------------

void Myo_log(std::atomic<bool>* stopFlag, std::string subject)
{
    // Create local Myo client used only in this thread
    myolinux::myo::Client client(myolinux::Serial{"/dev/ttyACM0", 115200});

    client.connect();
    if (!client.connected())
    {
        cout << "Unable to connect to Myo band" << endl;
        return;
    }

    client.setSleepMode(myolinux::myo::SleepMode::NeverSleep);
    client.setMode(myolinux::myo::EmgMode::SendEmg,
                   myolinux::myo::ImuMode::SendData,
                   myolinux::myo::ClassifierMode::Disabled);

    int   emg[8]  = {0};
    float ori[4]  = {0};
    float acc[3]  = {0};
    float gyr[3]  = {0};

    client.onEmg([&](myolinux::myo::EmgSample sample)
    {
        for (int i = 0; i < 8; ++i)
            emg[i] = sample[i];
    });

    client.onImu([&](myolinux::myo::OrientationSample o,
                     myolinux::myo::AccelerometerSample a,
                     myolinux::myo::GyroscopeSample g)
    {
        for (int i = 0; i < 4; ++i)
        {
            ori[i] = o[i];
            if (i < 3)
            {
                acc[i] = a[i];
                gyr[i] = g[i];
            }
        }
    });

    std::string myo_path = "/home/srisadha/hamid_powerball/src_main/data/admittance/" + subject + "myo.csv";
    std::ofstream myoFile(myo_path);
    myoFile << "Time_us,"
            << "EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8,"
            << "ORI1,ORI2,ORI3,ORI4,"
            << "ACC1,ACC2,ACC3,"
            << "GYR1,GYR2,GYR3\n";

    while (!stopFlag->load())
    {
        client.listen(); // blocks until a packet arrives and triggers callbacks

        long long t = now_us();

        myoFile << t;
        for (int i = 0; i < 8; ++i) myoFile << "," << emg[i];
        myoFile << "," << ori[0] << "," << ori[1] << "," << ori[2] << "," << ori[3]
                << "," << acc[0] << "," << acc[1] << "," << acc[2]
                << "," << gyr[0] << "," << gyr[1] << "," << gyr[2] << "\n";
    }

    myoFile.close();
}


static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}


// Compute admittance -> Qdot
void computations()
{
    Kin kin;

    Matrix<6,6,float> J = Zeros;
    Matrix<3,3,float> R = Zeros;
    Matrix<6,6,float> Rmat = Zeros;
    Matrix<6,6,float> I6 = Zeros;
    Matrix<6,6,float> A = Zeros;

    for (int i = 0; i < 6; i++)
        I6[i][i] = 1.0f;

    kin.Jacob(Q, &J);
    kin.FK_R(Q, &R);
    kin.FK_pos(Q, &X);
    
    // cout << "R:" << "\n" << R << "\n";
    // cout << "Q: " << Q << "\n";
    // cout << "X: " << X << "\n";
    // cout << "vel[0,1]: " << vel[0] << vel[1] << "\n";

    newPos[0] = -X[1];
    newPos[1] =  X[0];
    newPos[2] =  X[2];

    Rmat.slice<0,0,3,3>() = R;
    Rmat.slice<3,3,3,3>() = R;

    // FT -> planar Cartesian force (thread-safe snapshot)
    Vector<6,float> FT_local = Zeros;
    {
        std::lock_guard<std::mutex> lock(ft_mutex);
        FT_local = FT;
    }
    float Fx_raw =  FT_local[1] * ft_scale_x;
    float Fy_raw = -FT_local[0] * ft_scale_y;

    // low-pass filter (LPF)
    Fx_filt = (1.0f - ft_lpf_alpha) * Fx_filt + ft_lpf_alpha * Fx_raw;
    Fy_filt = (1.0f - ft_lpf_alpha) * Fy_filt + ft_lpf_alpha * Fy_raw;
    
    // deadband
    Fx_filt = deadbandf(Fx_filt, ft_deadband);
    Fy_filt = deadbandf(Fy_filt, ft_deadband);

    F_modified[0] = Fx_filt;
    F_modified[1] = Fy_filt;
    F_modified[2] = 0.0f;
    F_modified[3] = 0.0f;
    F_modified[4] = 0.0f;
    F_modified[5] = 0.0f;

    F_cmd = Rmat * (R_F_offset * F_modified);  // Convert the force from sensor frame into robot base frame.

    v_meas = J * Qdot_a;
    v_meas_lpf = (1.0f - v_meas_lpf_alpha) * v_meas_lpf_prev + v_meas_lpf_alpha * v_meas;
    
    ref_var = v_meas_lpf ^ v_meas_lpf_prev / (norm(v_meas_lpf_prev) * dt);  // equ(15)
    ref_var_lpf = ref_var_lpf_alpha * ref_var_prev + (1 - ref_var_lpf_alpha) * ref_var  // equ(16)
    ref_var_prev = ref_var;

    T_vec = v_meas_lpf.slice<0,3>() / norm(v_meas_lpf.slice<0,3>());  // 1*3
    B_vec = ref_var_lpf.slice<0,3>() / norm(ref_var_lpf.slice<0,3>());  // 1*3
    N_vec = B_vec ^ T_vec;

    float kappa = norm(T_vec - T_vec_prev) / (norm(v_meas_lpf_prev.slice<0,3>())*dt);  // equ(13)

    T_vec_prev = T_vec;
    v_meas_lpf_prev = v_meas_lpf;


    float c = 20.0f;
    float m = 10.0f;
    float F_exp = m*kappa*pow(norm(v_meas_lpf_prev.slice<0,3>()),2)*N_vec + C*norm(v_meas_lpf_prev.slice<0,3>())*T_vec;  // equ(17)

    Matrix<3,3,float> C = Data(c, 0, 0,
                               0, beta_c, 0,
                               0, 0, beta_c);

    Matrix<3,3,float> Ci = Rmat*C*inv(Rmat);

    Matrix<3,3,float> M = Data(m, 0, 0,
                               0, beta_m, 0,
                               0, 0, beta_m);

    Matrix<3,3,float> Mi = Rmat*M*inv(Rmat);

    vel.slice<0,2>() = inv(Mi)*(F_cmd.slice<0,3>() - Ci*vel.slice<0,2>())*dt + vel_prev.slice<0,3>();
    
    // vel[2] = 0.0f;
    vel[3] = 0.0f;
    vel[4] = 0.0f;
    vel[5] = 0.0f;
    
    // Constrain robot workspace
    float xmin = 0.18f, xmax = 0.5f;  // TODO
    float ymin = -0.4f, ymax = 0.4f;  // TODO
    float zmin = 0.11f, zmax = 0.12f;  // TODO
    
    if (X[0] <= xmin && vel[0] < 0.0f) vel[0] = 0.0f;
    if (X[0] >= xmax && vel[0] > 0.0f) vel[0] = 0.0f;
    
    if (X[1] <= ymin && vel[1] < 0.0f) vel[1] = 0.0f;
    if (X[1] >= ymax && vel[1] > 0.0f) vel[1] = 0.0f;
    
    if (X[2] <= zmin && vel[2] < 0.0f) vel[2] = 0.0f;
    if (X[2] >= zmax && vel[2] > 0.0f) vel[2] = 0.0f; 
    

    // damped least-squares IK: qdot = J^T (J J^T + lambda^2 I)^-1 vel
    A = J * J.T() + I6 * (lambda_dls * lambda_dls);

    SVD<6,6,float> svdA(A);
    Vector<6,float> y = svdA.backsub(vel);
    Qdot_cmd = J.T() * y;

    // joint velocity smoothing
    Qdot = (1.0f - qdot_lpf_alpha) * Qdot_prev + qdot_lpf_alpha * Qdot_cmd;
    Qdot_prev = Qdot;
    
    // saturation
    float qn = norm_inf(Qdot);
    if (qn > qdot_limit)
        Qdot = Qdot * (qdot_limit / qn);
        
}


// TCP FT receive thread (stoppable)
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

    while (!stopFlag->load())
    {
        int len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
        if (ignored_error == boost::asio::error::would_block ||
            ignored_error == boost::asio::error::try_again)
        {
            usleep(1000);
            continue;
        }
        if (ignored_error || len <= 0) continue;
        if (len >= (int)sizeof(recv_buf)) len = (int)sizeof(recv_buf) - 1;
        recv_buf[len] = '\0';

        int timeStamp = 0;
        Vector<6,float> FT_tmp = Zeros;
        int matched = sscanf(recv_buf, "F={%f,%f,%f,%f,%f,%f},%d",
                             &FT_tmp[0], &FT_tmp[1], &FT_tmp[2], &FT_tmp[3], &FT_tmp[4], &FT_tmp[5], &timeStamp);
        if (matched == 7)
        {
            std::lock_guard<std::mutex> lock(ft_mutex);
            FT = FT_tmp;
        }
    }
}


// -------------------- main --------------------
int main(int argc, char** argv)
{
    // Myo: default from USE_MYO, override with -no-myo / --no-myo
    bool use_myo = USE_MYO;
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "-no-myo") == 0 || strcmp(argv[i], "--no-myo") == 0)
        {
            use_myo = false;
            break;
        }
    }

    for (int i = 0; i < 6; i++)
        Md_inv[i][i] = 1.0f / Md_diag[i];

    string SubName;
    cout << "What is subject name? ";
    cin >> SubName;
    cout << "Please wait " << SubName << endl;
    cin.get(); // consume newline

    // Threads stop flag
    std::atomic<bool> stop_flag(false);

    // Myo logging thread (only if use_myo is true)
    boost::thread myo_thread;
    if (use_myo)
        myo_thread = boost::thread(Myo_log, &stop_flag, SubName);

    // V-REP thread
    boost::thread vrep_thread(vrep_draw, &stop_flag);

    // Log file
    std::string filepath = "/home/srisadha/hamid_powerball/src_main/data/admittance/" + SubName + "_schunk.csv";
    std::ofstream dataFile(filepath);
    dataFile << "Time_us,"
             << "Q1,Q2,Q3,Q4,Q5,Q6,"
             << "Qdot_a1,Qdot_a2,Qdot_a3,Qdot_a4,Qdot_a5,Qdot_a6,"
             << "Qdot1,Qdot2,Qdot3,Qdot4,Qdot5,Qdot6,"
             << "FT1,FT2,FT3,FT4,FT5,FT6,"
             << "F_cmd1,F_cmd2,F_cmd3,F_cmd4,F_cmd5,F_cmd6,"
             << "v_meas1,v_meas2,v_meas3,v_meas4,v_meas5,v_meas6,"
             << "v_meas_lpf1,v_meas_lpf2,v_meas_lpf3,v_meas_lpf4,v_meas_lpf5,v_meas_lpf6,"
             << "vel1,vel2,vel3,vel4,vel5,vel6,"
             << "c_des1,c_des2,"
             << "Cd1,Cd2,Cd3,Cd4,Cd5,Cd6\n";

    // Hyperparameters / run metadata sidecar (written once per run)
    {
        std::string jsonpath = "/home/srisadha/hamid_powerball/src_main/data/admittance/json/" + SubName + "_schunk.hyperparams.json";
        write_run_hyperparams_json(jsonpath, SubName, filepath, use_myo);
    }

    // Robot connect
    SchunkPowerball pb;
    pb.update();
    Q = pb.get_pos();

    // Stop thread
    boost::thread stop_thread(stop, &stop_flag);

    // Go to start pose (simple cosine blend)
    Qe = Data(0.0f, -M_PI/6, M_PI/2, 0.0f, M_PI/3, 0.0f);  // init configuration for admittance control
    // Qe = Data(0.0f, -0.935235f, 0.88284f, 0.0f, 1.31909f, 0.0f);  // further in x direction
    
    Vector<6,float> dQ = Qe - Q;

    float maxq = 0.0f;
    for (int i = 0; i < 6; i++) maxq = std::max(maxq, (float)fabs(dQ[i]));

    float Ttravel = std::max(1.0f, maxq * 3.0f);
    int itNum = (int)(Ttravel / dt);

    Vector<6,float> Qhold = Q;
    for (int k = 1; k <= itNum && !stop_flag.load(); k++)
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

    for (int cnt = 0; cnt < max_steps && !stop_flag.load(); cnt++)
    {
        auto t0 = std::chrono::system_clock::now();

        adm_time = cnt * dt;
        
        // Update robot state first
        pb.update();
        Q = pb.get_pos();
        Qdot_a = pb.get_vel();

        // Compute Qdot (NO thread creation here)
        computations();

        pb.set_vel(Qdot);

        // log
        long long t_us = now_us();
        Vector<6,float> FT_log = Zeros;
        {
            std::lock_guard<std::mutex> lock(ft_mutex);
            FT_log = FT;
        }
        dataFile << t_us << ","
                 << Q[0] << "," << Q[1] << "," << Q[2] << "," << Q[3] << "," << Q[4] << "," << Q[5] << ","
                 << Qdot_a[0] << "," << Qdot_a[1] << "," << Qdot_a[2] << "," << Qdot_a[3] << "," << Qdot_a[4] << "," << Qdot_a[5] << ","
                 << Qdot[0] << "," << Qdot[1] << "," << Qdot[2] << "," << Qdot[3] << "," << Qdot[4] << "," << Qdot[5] << ","
                 << FT_log[0] << "," << FT_log[1] << "," << FT_log[2] << "," << FT_log[3] << "," << FT_log[4] << "," << FT_log[5] << ","
                 << F_cmd[0] << "," << F_cmd[1] << "," << F_cmd[2] << "," << F_cmd[3] << "," << F_cmd[4] << "," << F_cmd[5] << ","
                 << v_meas[0] << "," << v_meas[1] << "," << v_meas[2] << "," << v_meas[3] << "," << v_meas[4] << "," << v_meas[5] << ","
                 << v_meas_lpf[0] << "," << v_meas_lpf[1] << "," << v_meas_lpf[2] << "," << v_meas_lpf[3] << "," << v_meas_lpf[4] << "," << v_meas_lpf[5] << ","
                 << vel[0] << "," << vel[1] << "," << vel[2] << "," << vel[3] << "," << vel[4] << "," << vel[5] << ","
                 << c_des[0] << "," << c_des[1] << ","
                 << Cd_diag[0] << "," << Cd_diag[1] << "," << Cd_diag[2] << "," << Cd_diag[3] << "," << Cd_diag[4] << "," << Cd_diag[5] << "\n";

        sleep_to_keep_dt(t0);
    }

    // Shutdown
    dataFile.close();

    stop_flag.store(true); // tell threads to exit

    if (myo_thread.joinable())
        myo_thread.join();
    if (FT_thread.joinable())
        FT_thread.join();
    if (vrep_thread.joinable())
        vrep_thread.join();
    if (stop_thread.joinable())
        stop_thread.detach();

    pb.shutdown_motors();
    pb.update();

    usleep(500 * 1000);
    cout << "Exiting ...\n";
    return 0;
}