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
#include "powerball/schunk_powerball.h"
#include "vrep/v_repClass.h"
#include "powerball/schunk_kinematics.h"
#include <TooN/LU.h>
#include <TooN/SVD.h>

// Myo band headers
#include "myolinux/myoclient.h"
#include "myolinux/serial.h"

using namespace::std;
using boost::asio::ip::tcp;
using namespace::TooN;
using namespace myolinux;

// global vars
Vector<6,float> FT;
simxFloat newPos[3]={0.0f,0.439,0.275};

// global vars computation
// float dt = 0.003f;
float dt = 0.005f;
Vector<6,float> Q = Zeros;
Vector<6,float> Qe = Zeros;
Vector<6,float> Qdot = Zeros;
Vector<6,float> Qdot_a = Zeros;
Vector<2,float> v_xy = Zeros;

// global time for admittance loop (seconds)
float adm_time = 0.0f;
// oscillation parameters
const float osc_start_time = 5.0f;    // start oscillation 5s after admittance start
const float osc_duration   = 10.0f;    // oscillation lasts for 3s
const float osc_omega      = 2.0f * M_PI * 0.5f; // angular frequency (0.5 Hz)
const float osc_amplitude  = 0.01f;   // linear velocity amplitude [m/s]


float Damp = 100;  // TODO

// Set to true to read and record Myo band (EMG/IMU) to a separate CSV; false to skip Myo.
static const bool USE_MYO = true;  // TODO

string SubName;

// Cartesian admittance parameters_ one time define
Vector<6,float> Md_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*0.3f; // for constant m
//Vector<6,float> Md_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*(1.0f/15.0f); // for var m (min:3.5 max:)
Matrix<6,6,double> Md_inv = Md_diag.as_diagonal();

//Vector<6,float> Cd_diag = makeVector(1.2f,1.0f,1.0f,1.0f,1.0f,1.0f)*80; // for fine Low
// Vector<6,float> Cd_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*25; // for fine High
//Vector<6,float> Cd_diag = makeVector(1.2f,1.0f,1.0f,1.0f,1.0f,1.0f)*90; // for gross Low
//Vector<6,float> Cd_diag = makeVector(1.4f,1.0f,1.0f,1.0f,1.0f,1.0f)*30; // for gross High
Vector<6,float> Cd_diag = makeVector(1.2f,1.0f,1.0f,1.0f,1.0f,1.0f)*Damp; // for comb High
//Vector<6,float> Cd_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*1.0f; // for velocity-based adapt

Matrix<6,6,double> Cd = Cd_diag.as_diagonal();
Vector<6,float> vel = Zeros;
Matrix<6,6,float> R_F_offset = Data(cos(M_PI/2),-sin(M_PI/2),0, 0,0,0,
                                    sin(M_PI/2),cos(M_PI/2),0,  0,0,0,
                                    0,0,1,                       0,0,0,
                                    0,0,0, cos(M_PI/2),-sin(M_PI/2),0,
                                    0,0,0, sin(M_PI/2),cos(M_PI/2),0,
                                    0,0,0,                      0,0,1);



static long long now_us()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}


void stop(bool* flag){
    char in;
    cin.get(in);
    *flag = true;
}

int sgn(float val)
{
    if (val > 0)
        return 1;
    else
        return -1;
}

int vrep_draw(){
    // connect to vrep
    int res = -1;
    V_rep vrep;
    res = vrep.connect();
    if (res==-1)
    {
        cout << "V-REP Connection Error!" << endl;
        return 0;
    }

    for (;;)
    {
        vrep.setSphere(&newPos[0]);
        usleep(40*1000);
    }
}

// -------------------- Myo logging (separate file) --------------------

void Myo_log(bool* stopFlag, std::string subject)
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

    std::string myo_path = "/home/srisadha/powerball/src_main/Hamid/data/admittance/" + SubName + "_myo_damp_" + std::to_string(Damp) + ".csv";
    std::ofstream myoFile(myo_path);
    myoFile << "Time_us,"
            << "EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8,"
            << "ORI1,ORI2,ORI3,ORI4,"
            << "ACC1,ACC2,ACC3,"
            << "GYR1,GYR2,GYR3\n";

    while (!(*stopFlag))
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

void computations(){
    Matrix<6,6,float> J = Zeros;
    Matrix<6,6,float> Rmat = Zeros;  // 6 by 6 matrix with R partitioned
    Matrix<3,3,float> R = Zeros;
    Vector<3,float> X = Zeros;
    Vector<6,float> F_modified = Zeros;

    Kin kin;

    kin.Jacob(Q,&J);
    kin.FK_R(Q,&R);
    kin.FK_pos(Q,&X);
    newPos[0] = -X[1];
    newPos[1] = X[0];
    newPos[2] = X[2];

    Rmat.slice<0,0,3,3>() = R.slice<0,0,3,3>();
    Rmat.slice<3,3,3,3>() = R.slice<0,0,3,3>();

    // prevent rotation and z motion
    F_modified[0] = -FT[3]*10.0f;
    F_modified[1] = -FT[4]*10.0f;

    // add oscillatory component along user-applied force direction after a delay
    // the oscillation is aligned with the planar external force vector (F_modified[0], F_modified[1])
    // if (adm_time >= osc_start_time && adm_time < (osc_start_time + osc_duration))
    // {
    //     float t_rel = adm_time - osc_start_time;
    //     float osc = osc_amplitude * sinf(osc_omega * t_rel);
    //     float fx = F_modified[0];
    //     float fy = F_modified[1];
    //     float norm_f = sqrtf(fx*fx + fy*fy);
    //     if (norm_f > 1e-4f)
    //     {
    //         F_modified[0] += osc * fx / norm_f;
    //         F_modified[1] += osc * fy / norm_f;
    //     }
    // }

    // no adaptation
    //vel = vel + dt*(Md_inv*(Rmat*(R_F_offset*F_modified)) - Md_inv*Cd*vel);


    // adaptation 1 (geometrical)
    float C1=0.075, C2=0.125, alpha=1.0f, xx=0.0;
    if (X[1]<=C1)
    {
        alpha=1.0f;
    } else if ((X[1]>C1) && (X[1]<C2))
    {
        xx = (X[1]-C1)*4/(C2-C1) - 2;
        alpha = tanh(xx)/0.964*1.5 + 2.5;
    } else
    {
        alpha=3.5f;
    }
    vel = vel + dt*(Md_inv*(Rmat*(R_F_offset*F_modified)) - Md_inv*alpha*Cd*vel);

    /*
    // adaptation velocity-based (Fanny)
    Vector<6,float> v_vec = J*Qdot_a;
    v_xy[0] = v_vec[0];
    v_xy[1] = v_vec[1];
    float v_norm = TooN::norm_2(v_xy);
    float Dcoeff = 100.0f*exp(-5.0f*v_norm);
    vel = vel + dt*(Md_inv*(Rmat*(R_F_offset*F_modified)) - Md_inv*Cd*Dcoeff*vel);
    */

    // solve inv(A)*b using LU
    SVD<6,6,float> luJ(J);
    Qdot = luJ.backsub(vel);

}

void TCP_receive(bool *errFlag)
{

    boost::asio::io_service io_service;
    tcp::endpoint sender_endpoint = boost::asio::ip::tcp::endpoint(
                boost::asio::ip::address::from_string("192.168.1.30"),  boost::lexical_cast<int>("1000"));
    tcp::socket socket(io_service);
    socket.connect(sender_endpoint);

    boost::system::error_code ignored_error;
    int len=0;
    char recv_buf[128];

    // TARE the sensor
    std::string msg="TARE(1)\n";
    socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
    len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    cout << "TCP recieved: " << recv_buf << endl;

    // continous receiving
    msg="L1()\n";
    socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
    len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    cout << "TCP recieved: " << recv_buf << endl;

    // Force data
    for (;;)
    {
        //msg="F()\n";
        //socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
        len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
        int timeStamp;
        sscanf(recv_buf,"F={%f,%f,%f,%f,%f,%f},%d",&FT[0],&FT[1],&FT[2],&FT[3],&FT[4],&FT[5],&timeStamp);

    }
}

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

    // subject information
    cout << "What is subject name? ";
    cin >> SubName;
    cout << "Please wait " << SubName << endl;
    char in;
    cin.get(in);

    // Set sampling and timing options
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;

    // connect to V-rep
    boost::thread vrep_thread(vrep_draw);

    // Open recording file
    std::ofstream dataFile;
    std::string filepath;
    
    filepath = "/home/srisadha/powerball/src_main/Hamid/data/admittance/" + SubName + "_schunk_damp_" + std::to_string(Damp) + ".csv";
    
    std::ofstream dataFile(filepath);
    dataFile << "Time_us,"
             << "Q1,Q2,Q3,Q4,Q5,Q6,"
             << "dQ1,dQ2,dQ3,dQ4,dQ5,dQ6,"
             << "FT1,FT2,FT3,FT4,FT5,FT6,"
             << "Vx,Vy,Vz,omega_x,omega_y,omega_z,"
             << "Cd1,Cd2,Cd3,Cd4,Cd5,Cd6\n";
             
    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;

    // connect to robot
    SchunkPowerball pb;
    pb.update();
    Q = pb.get_pos();

    // stop by Enter key thread
    bool stop_flag = false;
    boost::thread stop_thread(stop,&stop_flag);

    // Myo logging thread (only if use_myo is true)
    boost::thread myo_thread;
    if (use_myo)
        myo_thread = boost::thread(Myo_log, &stop_flag, SubName);

    // go to start pose
    Qe = Data(0.0f,-M_PI/6,M_PI/2,0.0f,M_PI/3,0.0f);
    Vector<6,float> dQ = Qe-Q;
    float maxq=0;
    for (int n=0;n<6;n++)
    {
        if (abs(dQ[n])>maxq) {maxq=abs(dQ[n]);}
    }
    float Ttravel = maxq*3;
    if (Ttravel<1.0){Ttravel=1.0;}
    int itNum = Ttravel/dt;
    Vector<6,float> Qhold = Q;
    int n = 1;
    while((n<=itNum) && (!stop_flag))
    {
        Vector<6,float> Qt = (1-cos(float(n)/itNum*M_PI))/2 * dQ + Qhold;
        pb.set_pos(Qt);
        pb.update();
        Q = pb.get_pos();
        usleep( dt*1000*1000 );
        n++;
    }

    // set velocity mode active
    pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);
    pb.update();

    // Initializing FT sensor
    bool errFlag=false;
    boost::thread FT_thread(TCP_receive,&errFlag);

    int cnt = 0;
    cout << "Admittance loop started!" << endl;
    while((cnt<90/dt) && (!stop_flag))
    {
        timeLoop = std::chrono::system_clock::now();

        // update global admittance time (seconds) for oscillatory motion
        adm_time = cnt * dt;

        boost::thread threaded_computation(computations);

        // velocity saturation
        if (TooN::norm_inf(Qdot)>32.5*M_PI/180)
        {
            cout << "saturation!" << endl;
            //Qdot[n] = 32.5*M_PI/180*sgn(Qdot[n]);
        }
        else
        {
            pb.set_vel(Qdot);
        }

        pb.update();
        Q = pb.get_pos();
        Qdot_a = pb.get_vel();

        long long t_us = now_us();
        dataFile << t_us << ","
                 << Q[0] << "," << Q[1] << "," << Q[2] << "," << Q[3] << "," << Q[4] << "," << Q[5] << ","
                 << Qdot_a[0] << "," << Qdot_a[1] << "," << Qdot_a[2] << "," << Qdot_a[3] << "," << Qdot_a[4] << "," << Qdot_a[5] << ","
                 << FT[0] << "," << FT[1] << "," << FT[2] << "," << FT[3] << "," << FT[4] << "," << FT[5] << ","
                 << vel[0] << "," << vel[1] << "," << vel[2] << "," << vel[3] << "," << vel[4] << "," << vel[5] << ","
                 << Cd_diag[0] << "," << Cd_diag[1] << "," << Cd_diag[2] << "," << Cd_diag[3] << "," << Cd_diag[4] << "," << Cd_diag[5] << "\n";

        threaded_computation.join();

        cnt++;
        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
        else {
            cout << "Communication Time Out!" << endl;
        }

    }


    dataFile.close(); // close file

    // tell background threads to exit
    stop_flag = true;

    // kill FT, Myo, and other threads
    if (myo_thread.joinable())
        myo_thread.interrupt();
    FT_thread.interrupt();
    stop_thread.interrupt();   // kill the thread
    vrep_thread.interrupt();
    // set pb off since it is in vel mode
    pb.shutdown_motors(); 
    pb.update();
    usleep(1000*500);
    cout << "Exiting ..." << endl;

}
