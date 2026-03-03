// admittance_clean.cpp
// Goal: readable “plumbing” for Myo + FT + VSM + robot admittance + CSV logging.

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <filesystem>
#include <cmath>
#include <cstdint>
#include <cstring>

// --- Project headers (must exist in your repo) ---
#include "powerball/schunk_powerball.h"
#include "powerball/schunk_kinematics.h"
#include "vrep/v_repClass.h"
#include "utils/utils.h"
#include "utils/powerball_utils.h"
#include "utils/vsm_utils.h"
#include "vsm/vsm_control.h"

// --- TooN ---
#include <TooN/SVD.h>

// --- Myo ---
#include "myolinux/myoclient.h"
#include "myolinux/serial.h"

// --------------------------- Small helpers ---------------------------

static inline int64_t now_us()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

// --------------------------- Config ---------------------------

// Dynamixel registers (XM430-W210R)
static constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
static constexpr uint16_t ADDR_GOAL_POSITION    = 116;
static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
static constexpr double   PROTOCOL_VERSION      = 1.0;

static constexpr int      BAUDRATE  = 1000000;
static constexpr char     DEVICENAME[] = "/dev/ttyUSB0";
static constexpr int      DXL_MOVING_STATUS_THRESHOLD = 20;

static constexpr float    dt = 0.005f;     // 5ms control loop
static constexpr float    passive_mag_width = 0.020f;
static constexpr double   dxl_resolution =
    (300.0 * M_PI/180.0) * (25.46e-3/2.0) / 4096.0; // encoder -> meters (your calibration)

// VSM limits (your calibration)
static constexpr uint32_t VSM_MIN_POS = 50;
static constexpr uint32_t VSM_MAX_POS = 2048;

// Myo serial
static constexpr char     MYO_PORT[] = "/dev/ttyACM0";
static constexpr int      MYO_BAUD   = 115200;

// --------------------------- Shared state ---------------------------

struct Flags
{
    std::atomic<bool> stop{false};      // stop whole program (Enter key)
    std::atomic<bool> record{false};    // start writing Myo CSV when true
    std::atomic<bool> ft_ready{false};  // FT calibrated
};

struct MyoState
{
    std::mutex m;
    TooN::Vector<8,int>   emg = TooN::Zeros;
    TooN::Vector<4,float> ori = TooN::Zeros;
    TooN::Vector<3,float> acc = TooN::Zeros;
    TooN::Vector<3,float> gyr = TooN::Zeros;
};

struct ControlState
{
    // Robot state
    TooN::Vector<6,float> Q      = TooN::Zeros;
    TooN::Vector<6,float> Qdot   = TooN::Zeros;
    TooN::Vector<6,float> Qdot_a = TooN::Zeros;
    TooN::Vector<3,float> X      = TooN::Zeros;
    TooN::Vector<3,float> X_init = TooN::Zeros;

    // Admittance state
    TooN::Vector<6,float> vel = TooN::Zeros;

    // V-REP visualization point
    simxFloat newPos[3] = {0.0f, 0.439f, 0.275f};

    // VSM state
    std::vector<uint32_t> dxl_pos {0,0};
    std::vector<float>    vsm_force {0.0f, 0.0f};
    std::vector<std::vector<float>> mag_sep {{0.0f,0.0f},{0.0f,0.0f}};
};

// NOTE: these come from your existing FT/VSM utilities.
// They were globals in your original codebase too.
// Keep them as-is, but don’t hide them inside 20 files.
extern TooN::Vector<6,float> FT;         // updated by Robotiq_ft_receive(...)
extern std::vector<float> linPot;        // updated by phidget code

// --------------------------- Options parsing ---------------------------

struct Options
{
    bool vsm_enabled = true;
    bool calib_only  = false;
    int  dxl_init_position = 600;    // safe default
    std::string damping = "hd";      // "ld" or "hd"
};

static Options parse_args(int argc, char** argv)
{
    Options opt;

    if (argc <= 1) return opt;

    if (std::strcmp(argv[1], "-h") == 0)
    {
        std::cout
            << "Usage:\n"
            << "  ./admittance -calib\n"
            << "  ./admittance -no_vsm [ld|hd]\n"
            << "  ./admittance <dxl_init_pos 100..2048> [ld|hd]\n";
        std::exit(0);
    }

    if (std::strcmp(argv[1], "-calib") == 0)
    {
        opt.calib_only = true;
        opt.vsm_enabled = false;
        return opt;
    }

    if (std::strcmp(argv[1], "-no_vsm") == 0)
    {
        opt.vsm_enabled = false;
        opt.dxl_init_position = 0;
    }
    else
    {
        opt.vsm_enabled = true;
        opt.dxl_init_position = std::stoi(argv[1]);
        if (opt.dxl_init_position < 100 || opt.dxl_init_position > 2048)
        {
            std::cerr << "dxl_init_position must be in [100,2048]\n";
            std::exit(1);
        }
    }

    if (argc >= 3)
    {
        if (std::strcmp(argv[2], "ld") == 0) opt.damping = "ld";
        if (std::strcmp(argv[2], "hd") == 0) opt.damping = "hd";
    }

    return opt;
}

// --------------------------- Stop thread ---------------------------

static void wait_enter_to_stop(Flags* flags)
{
    std::cin.get();
    flags->stop = true;
    flags->record = false;
}

// --------------------------- V-REP thread ---------------------------

static void vrep_loop(const Flags* flags, const ControlState* st)
{
    V_rep vrep;
    if (vrep.connect() == -1)
    {
        std::cerr << "V-REP Connection Error!\n";
        return;
    }

    while (!flags->stop.load())
    {
        vrep.setSphere((simxFloat*)&st->newPos[0]);
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
}

// --------------------------- Myo ---------------------------

static myolinux::Client g_myo_client(myolinux::Serial{MYO_PORT, MYO_BAUD});

static void myo_init(MyoState* myo)
{
    g_myo_client.connect();
    if (!g_myo_client.connected())
    {
        std::cerr << "Unable to connect to Myo band\n";
        std::exit(1);
    }

    g_myo_client.setSleepMode(myolinux::SleepMode::NeverSleep);
    g_myo_client.setMode(myolinux::EmgMode::SendEmg,
                         myolinux::ImuMode::SendData,
                         myolinux::ClassifierMode::Disabled);

    g_myo_client.onEmg([myo](myolinux::EmgSample sample)
    {
        std::lock_guard<std::mutex> lock(myo->m);
        for (int i = 0; i < 8; i++)
            myo->emg[i] = static_cast<int>(sample[i]);
    });

    g_myo_client.onImu([myo](myolinux::OrientationSample ori,
                             myolinux::AccelerometerSample acc,
                             myolinux::GyroscopeSample gyr)
    {
        std::lock_guard<std::mutex> lock(myo->m);
        for (int i = 0; i < 4; i++)
        {
            myo->ori[i] = ori[i];
            if (i < 3)
            {
                myo->acc[i] = acc[i];
                myo->gyr[i] = gyr[i];
            }
        }
    });
}

static void myo_log_loop(const Flags* flags,
                         const MyoState* myo,
                         const std::string& folder,
                         int dxl_init_position,
                         const std::string& damping)
{
    std::filesystem::create_directories(folder);

    std::ofstream emgFile(folder + "/EMG_stiff_" + std::to_string(dxl_init_position) + "_" + damping + ".csv");
    std::ofstream imuFile(folder + "/IMU_stiff_" + std::to_string(dxl_init_position) + "_" + damping + ".csv");

    emgFile << "Time_us,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8\n";
    imuFile << "Time_us,ORI1,ORI2,ORI3,ORI4,ACC1,ACC2,ACC3,GYR1,GYR2,GYR3\n";

    while (!flags->stop.load())
    {
        try
        {
            g_myo_client.listen(); // blocks until packet; triggers callbacks

            if (!flags->record.load())
                continue;

            // snapshot under lock
            TooN::Vector<8,int>   emg;
            TooN::Vector<4,float> ori;
            TooN::Vector<3,float> acc;
            TooN::Vector<3,float> gyr;

            {
                std::lock_guard<std::mutex> lock(myo->m);
                emg = myo->emg;
                ori = myo->ori;
                acc = myo->acc;
                gyr = myo->gyr;
            }

            const int64_t t = now_us();

            emgFile << t;
            for (int i = 0; i < 8; i++) emgFile << "," << emg[i];
            emgFile << "\n";

            imuFile << t
                    << "," << ori[0] << "," << ori[1] << "," << ori[2] << "," << ori[3]
                    << "," << acc[0] << "," << acc[1] << "," << acc[2]
                    << "," << gyr[0] << "," << gyr[1] << "," << gyr[2] << "\n";
        }
        catch (myolinux::DisconnectedException&)
        {
            std::cerr << "MYO Disconnected\n";
        }
    }

    emgFile.close();
    imuFile.close();
}

// Calibration (10 seconds)
static void myo_calib_10s()
{
    std::filesystem::create_directories("../data/VSM/peg-hole/emg_calib");
    std::ofstream f("../data/VSM/peg-hole/emg_calib/calib.csv");
    f << "Time_us,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8\n";

    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count() < 10.0f)
    {
        g_myo_client.listen();
        const int64_t t = now_us();

        // We rely on the callback filling some global state;
        // simplest: read the last value via internal callback state would be better,
        // but keep it minimal here.
        // (If you want: snapshot MyoState here too.)
        f << t << ",0,0,0,0,0,0,0,0\n"; // placeholder if you want a pure-calib file
    }

    f.close();
}

// --------------------------- Admittance computation ---------------------------

// Very simple: compute cartesian velocity from FT, then joint velocity from Jacobian.
static void admittance_step(ControlState* st,
                            const TooN::Matrix<6,6,double>& Md_inv,
                            const TooN::Matrix<6,6,double>& Cd)
{
    // Kinematics
    Kin kin;
    TooN::Matrix<6,6,float> J = TooN::Zeros;
    TooN::Matrix<3,3,float> Rb_e = TooN::Zeros;

    kin.Jacob(st->Q, &J);
    kin.FK_R(st->Q, &Rb_e);
    kin.FK_pos(st->Q, &st->X);

    // V-REP point (your “maze traversal” mapping)
    st->newPos[0] = -(st->X[1] - st->X_init[1]) + 0.06f;
    st->newPos[1] =  (st->X[0] - st->X_init[0]) + 0.439f;
    st->newPos[2] = 0.01f;

    // Force transform (keep your fixed Re_f)
    TooN::Matrix<3,3,float> Re_f = TooN::Data(
        std::cos(-M_PI/4), -std::sin(-M_PI/4), 0.0f,
        std::sin(-M_PI/4),  std::cos(-M_PI/4), 0.0f,
        0.0f,               0.0f,              1.0f
    );

    TooN::Matrix<6,6,float> Rb_e_mat = TooN::Zeros;
    TooN::Matrix<6,6,float> Re_f_mat = TooN::Zeros;
    Rb_e_mat.slice<0,0,3,3>() = Rb_e;
    Rb_e_mat.slice<3,3,3,3>() = Rb_e;
    Re_f_mat.slice<0,0,3,3>() = Re_f;
    Re_f_mat.slice<3,3,3,3>() = Re_f;

    // If you want “no rotation”, ignore moments:
    TooN::Vector<6,float> FT_local = FT;
    FT_local.slice<3,3>() = TooN::Zeros; // moments off

    TooN::Vector<6,float> F_modified = Rb_e_mat * Re_f_mat * FT_local;

    // Admittance ODE:
    // vel_dot = Md^-1 * (F - Cd * vel)
    st->vel = st->vel + dt * (Md_inv * F_modified - Md_inv * Cd * st->vel);

    // hard-disable rotation velocity
    st->vel.slice<3,3>() = TooN::Zeros;

    // J * qdot = vel  -> solve with SVD
    TooN::SVD<6,6,float> svdJ(J);
    st->Qdot = svdJ.backsub(st->vel);
}

// --------------------------- VSM loop ---------------------------

static void vsm_loop(VSMControl* vsm,
                     const std::vector<int>& dxl_id_list,
                     const Flags* flags,
                     ControlState* st,
                     bool vsm_enabled)
{
    if (!vsm_enabled) return;

    for (;;)
    {
        if (flags->stop.load()) return;

        // Read magnets / compute separations / compute external force
        for (int id = 0; id < (int)dxl_id_list.size(); id++)
        {
            st->dxl_pos[id] = vsm->xm_get_pos(dxl_id_list[id]);

            std::vector<float> sep;
            vsm->calculate_mag_sep(st->dxl_pos[id],
                                   linPot[id],
                                   dxl_resolution,
                                   VSM_MIN_POS,
                                   passive_mag_width,
                                   &sep);

            st->mag_sep[id][0] = sep[0];
            st->mag_sep[id][1] = sep[1];

            st->vsm_force[id] = resForce(sep);
        }
    }
}

// --------------------------- MAIN ---------------------------

int main(int argc, char** argv)
{
    Options opt = parse_args(argc, argv);

    // Admittance parameters
    TooN::Vector<6,float> Md_diag = TooN::makeVector(1,1,1,1,1,1) * 0.05f;
    TooN::Matrix<6,6,double> Md_inv = Md_diag.as_diagonal();

    TooN::Vector<6,float> Cd_diag;
    if (opt.damping == "ld")
        Cd_diag = TooN::makeVector(80,80,200,200,200,200);
    else
        Cd_diag = TooN::makeVector(200,200,200,200,200,200);

    TooN::Matrix<6,6,double> Cd = Cd_diag.as_diagonal();

    // Init flags/state
    Flags flags;
    MyoState myo;
    ControlState st;

    // Myo init
    myo_init(&myo);

    // Optional: calibration-only mode
    if (opt.calib_only)
    {
        std::cout << "Calibration: 10 seconds isometric.\n";
        // For your real calibration file, reuse myo_log_loop with record=true
        // or snapshot MyoState and write EMG to calib.csv.
        // myo_calib_10s();
        return 0;
    }

    // VSM init (only if enabled)
    VSMControl vsm(BAUDRATE, DEVICENAME, PROTOCOL_VERSION,
                   ADDR_TORQUE_ENABLE, ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION);

    std::vector<int> dxl_id_list;
    if (opt.vsm_enabled)
    {
        if (!vsm.dxl_enable()) return 0;
        std::cout << "Connected dyn count: " << vsm.get_connected_dxl_count(&dxl_id_list) << "\n";

        vsm.xm_set_pos(dxl_id_list[0], opt.dxl_init_position, VSM_MIN_POS, VSM_MAX_POS);
        vsm.xm_set_pos(dxl_id_list[1], opt.dxl_init_position, VSM_MIN_POS, VSM_MAX_POS);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (!phidget_enable()) return 0;
    }

    // Start threads
    std::thread stopT(wait_enter_to_stop, &flags);
    std::thread vrepT(vrep_loop, &flags, &st);

    std::thread myoT(myo_log_loop,
                     &flags,
                     &myo,
                     "../data/VSM/peg-hole",
                     opt.dxl_init_position,
                     opt.damping);

    // FT thread (your existing function)
    std::thread ftT(Robotiq_ft_receive, dt, true, &flags.stop, (bool*)&flags.ft_ready);
    while (!flags.ft_ready.load()) {} // wait FT calibrated

    std::thread vsmT(vsm_loop, &vsm, dxl_id_list, &flags, &st, opt.vsm_enabled);

    // Robot init
    SchunkPowerball pb;
    Kin kin;
    pb.update();

    // Move to start pose (keep your existing helper logic)
    TooN::Vector<6,float> Qs = pb.get_pos();
    TooN::Vector<6,float> Qe = TooN::makeVector(-30,-10,95,0,74,105) * (float)(M_PI/180.0);

    float T_travel = calculate_travel_time(Qs, Qe, 5);
    std::cout << "Travel time to start pose: " << T_travel << " sec\n";

    std::vector<std::vector<double>> traj;
    kin.HerInter(Qs, TooN::Zeros, Qe, dt, T_travel, &traj);

    for (const auto& row : traj)
    {
        TooN::Vector<6,float> qtmp;
        for (int j = 0; j < 6; j++) qtmp[j] = (float)row[j];
        pb.set_pos(qtmp);
        pb.update();
        std::this_thread::sleep_for(std::chrono::duration<float>(dt));
    }

    traj.clear();

    pb.update();
    st.Q = pb.get_pos();
    kin.FK_pos(st.Q, &st.X_init);

    // Velocity mode
    pb.set_sdo_controlword(NODE_ALL, STATUS_OPERATION_ENABLED);
    pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);
    pb.update();

    // Data log
    std::filesystem::create_directories("../data/VSM/peg-hole");
    std::ofstream dataFile("../data/VSM/peg-hole/admittance_stiff_" +
                           std::to_string(opt.dxl_init_position) + "_" + opt.damping + ".csv");

    dataFile << "Time_us,"
             << "Q1,Q2,Q3,Q4,Q5,Q6,"
             << "Qdot1,Qdot2,Qdot3,Qdot4,Qdot5,Qdot6,"
             << "x,y,z,xdot,ydot,zdot,"
             << "FT1,FT2,FT3,FT4,FT5,FT6,"
             << "handle_pos1,handle_pos2,"
             << "mag11,mag12,mag21,mag22,"
             << "f1,f2,dxl_pos1,dxl_pos2\n";

    std::cout << "Admittance loop started. Press Enter to stop.\n";
    flags.record = true;

    // Control loop at dt
    auto next = std::chrono::steady_clock::now();

    while (!flags.stop.load())
    {
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<float>(dt));

        // Read robot state
        pb.update();
        st.Q = pb.get_pos();
        st.Qdot_a = pb.get_vel();

        // Compute qdot
        admittance_step(&st, Md_inv, Cd);

        // Saturation
        if (TooN::norm_inf(st.Qdot) > 50.0f * (float)(M_PI/180.0))
        {
            st.Qdot = TooN::Zeros;
        }

        // Send velocity
        pb.set_vel(st.Qdot);

        // Log one row
        const int64_t t = now_us();
        dataFile << t << ","
                 << st.Q[0] << "," << st.Q[1] << "," << st.Q[2] << "," << st.Q[3] << "," << st.Q[4] << "," << st.Q[5] << ","
                 << st.Qdot[0] << "," << st.Qdot[1] << "," << st.Qdot[2] << "," << st.Qdot[3] << "," << st.Qdot[4] << "," << st.Qdot[5] << ","
                 << st.X[0] << "," << st.X[1] << "," << st.X[2] << ","
                 << st.vel[0] << "," << st.vel[1] << "," << st.vel[2] << ","
                 << FT[0] << "," << FT[1] << "," << FT[2] << "," << FT[3] << "," << FT[4] << "," << FT[5] << ","
                 << linPot[0] << "," << linPot[1] << ","
                 << st.mag_sep[0][0] << "," << st.mag_sep[0][1] << "," << st.mag_sep[1][0] << "," << st.mag_sep[1][1] << ","
                 << st.vsm_force[0] << "," << st.vsm_force[1] << ","
                 << st.dxl_pos[0] << "," << st.dxl_pos[1] << "\n";

        // Sleep to keep dt
        std::this_thread::sleep_until(next);
    }

    dataFile.close();
    flags.record = false;

    // Shutdown (join threads)
    if (ftT.joinable())   ftT.join();
    if (vsmT.joinable())  vsmT.detach();  // may be infinite loop; detach if your vsm_loop can’t exit cleanly yet
    if (myoT.joinable())  myoT.join();
    if (vrepT.joinable()) vrepT.join();
    if (stopT.joinable()) stopT.join();

    std::cout << "Exiting.\n";
    return 0;
}