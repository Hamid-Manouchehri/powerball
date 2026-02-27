#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <ostream> // included for color output to the terminal
#include <typeinfo>
#include <cmath>

// boost headers
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>

// schunk powerball headers
#include "powerball/schunk_powerball.h"
#include "vrep/v_repClass.h"
#include "powerball/schunk_kinematics.h"
#include "utils/utils.h"
#include "utils/powerball_utils.h"
#include "utils/vsm_utils.h"
#include "vsm/vsm_control.h"

// Toon headers
#include <TooN/LU.h>
#include <TooN/SVD.h>

// myo band headers
#include "myolinux/myoclient.h"
#include "myolinux/serial.h"

// plotting and linear algebra libraries
// #include "matplotlibcpp.h"
#include "sigpack.h"
#include <armadillo>

using namespace::std;
using boost::asio::ip::tcp;
using namespace::TooN;
using namespace myolinux;
// namespace plt = matplotlibcpp;

// Initializing the myo connection
myo::Client client(Serial{"/dev/ttyACM0", 115200});

// Replace the control table parameters according to the dynamixel model
// These parameters are for the new dynamixel motors (XM430-W210R) used in the VSM
#define ADDR_MX_TORQUE_ENABLE           64                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           116
#define ADDR_MX_PRESENT_POSITION        132

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID1                         1                   // Dynamixel ID: 1
#define DXL_ID2                         2                   // Dynamixel ID: 2
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#define DXL_MOVING_STATUS_THRESHOLD     20

#define ESC_ASCII_VALUE                 0x1b
#define UPARROW_ASCII_VALUE             0x18
#define DOWNARROW_ASCII_VALUE           0x19

//Global variables
float dt = 0.005f; // sampling time


int exp_id = 100;
Vector<8,int> EMG = Zeros;
Vector<4,float> ORI = Zeros;
Vector<3,float> ACC = Zeros;
Vector<3,float> GYR = Zeros;
Vector<6,float> Q   = Zeros;
Vector<6,float> Qs  = Zeros;
Vector<6,float> Qe  = Zeros;
Vector<6, float> Q_interm;
Vector<3,float> X   = Zeros;
Vector<3,float> X_init = Zeros;
Vector<6,float> Qdot = Zeros;
Vector<6,float> Qdot_a = Zeros;
Vector<6,float> joints_vrep = Zeros;
simxFloat newPos[3] ={0.0f,0.439,0.275};
bool Start_record   = true;

int counter = 0;
int windLen = 30;
int numChannels = 8;
arma::imat emgVec(windLen,8,arma::fill::zeros);


void stop(bool* flag){
    char in;
    cin.get(in);
    *flag = true;
    Start_record = false;
}


// Myo armband files
myo::Client Myo_init()
{
    client.connect();// Autoconnect to the first Myo device
    if (!client.connected()) {
        cout<< RED<< "Unable to connect to Myo band"<<DEFAULT <<endl;
    }else{
        cout<< GREEN <<"Connection established with the Myo band .........."<<DEFAULT <<endl;
    }

    client.setSleepMode(myo::SleepMode::NeverSleep);// Set sleep mode
    client.setMode(myo::EmgMode::SendEmg, myo::ImuMode::SendData, myo::ClassifierMode::Disabled);// Read EMG and IMU
    client.onEmg([](myo::EmgSample sample)
    {
        for (std::size_t i = 0; i < 8; i++) {
            EMG[i] = static_cast<int>(sample[i]);
        }
    });

    client.onImu([](myo::OrientationSample ori, myo::AccelerometerSample acc, myo::GyroscopeSample gyr)
    {
        for (size_t i = 0; i < 4 ; i++){
            ORI[i] = ori[i];
            if (i < 3){
                ACC[i] = acc[i];
                GYR[i] = gyr[i];
            }
        }
    });
    // auto name = client.deviceName();
    return client;
}

void Myo_receive(bool *errFlag, std::string folderpath)
{
    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;

    // Open recording file
    std::ofstream EMGFile, IMUFile;
    EMGFile.open(folderpath + "EMG_obj_Fatigue_trial_" + std::to_string(exp_id) + ".csv");
    EMGFile << "Time,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8"<< endl;

    IMUFile.open(folderpath + "IMU_obj_Fatigue_trial_" + std::to_string(exp_id) + ".csv");
    IMUFile << "Time,ORI1,ORI2,ORI3,ORI4,ACC1,ACC2,ACC3,GYR1,GYR2,GYR3"<< endl;
    
    while(!*errFlag){
        try {
            client.listen();
            if(Start_record){
              gettimeofday(&tv, NULL);
              curtime=tv.tv_sec;
              strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));

              EMGFile << buffer << ":" << tv.tv_usec << ",";
              IMUFile << buffer << ":" << tv.tv_usec << ",";

              emgVec.rows(arma::span(0,windLen-2)) = emgVec.rows(arma::span(1,windLen-1)); 
              for(int i = 0; i < 8;i++){
                emgVec(windLen-1,i) = abs(EMG[i]);
              }  

              // for(int i = 0; i < 8;i++){
              //   emgVec(counter,i) = abs(EMG[i]);
              // }

              
            
              counter += 1;

              EMGFile<< EMG[0]<< "," << EMG[1]<< "," << EMG[2]<< "," << EMG[3]<< "," << EMG[4]<< "," << EMG[5]<< "," << EMG[6]<< "," << EMG[7]<< endl;
              IMUFile<< ORI[0]<< "," << ORI[1]<< "," << ORI[2]<< "," << ORI[3]<< "," << ACC[0]<< "," << ACC[1]<< "," << ACC[2]<< "," << GYR[0]<< "," << GYR[1]<< "," << GYR[2] <<endl;
            }
        }
        catch(myo::DisconnectedException &) {
            cout << "MYO Disconnected" << endl;
        }
    }
}

// measure the EMG signals during isometric contractions
void calib_EMG(bool *calibFlag, string folderpath)
{
    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;

    // Open recording file
    std::ofstream EMGFile, IMUFile;
    EMGFile.open(folderpath + "calib.csv");
    // EMGFile << "Time,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8"<< endl;

    while(*calibFlag){
        try {
            client.listen();
            gettimeofday(&tv, NULL);
            curtime=tv.tv_sec;
            strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));

            // EMGFile << buffer << ":" << tv.tv_usec << ",";
            EMGFile<< EMG[0]<<","<<EMG[1]<< "," << EMG[2]<< "," << EMG[3]<< "," << EMG[4]<< "," << EMG[5]<< "," << EMG[6]<< "," << EMG[7]<<endl;
        }
        catch(myo::DisconnectedException &) {
            cout << "MYO Disconnected" << endl;
        }
    }
}

// estimate the max voluntary contraction from the recorded values
// @return: 0 - terminate
//          1 - all good
int EMG_max_contrx(string folderpath, std::vector<int>* emg_calib_vec)
{       
    struct stat info;
    std::string filepath = folderpath + "emg_calib/calib.csv";
    const char *path1 = filepath.c_str();
    if (stat(path1, &info) == -1){
        std::cout << RED << "Calibration file does not exist" << DEFAULT << std::endl;
        return 0;
    }

    Kin kin;
    std::vector< std::vector<double> > matrix;
    kin.inputFile(filepath, &matrix, ",");

    int traj_col = 0;
    int  ncols, nrows=0;
    for (std::vector< std::vector<double> >::const_iterator it = matrix.begin(); it != matrix.end(); ++ it)
    {
        nrows++;
        ncols = 0;
        for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
        {
            ncols++;
        }
    }
    cout << "size of imported matrix = " << nrows << "*" << ncols << endl;
    TooN::Matrix<Dynamic,Dynamic,int> imported_matrix(nrows, ncols);

    // first 6 cols for the joint angles and the 7th col for the magnet position
    if (ncols ==9){

        // Put the matrix into TooN matrix
        nrows = 0;  ncols = 0;
        for (std::vector< std::vector<double> >::const_iterator it = matrix.begin(); it != matrix.end(); ++ it)
        {
            ncols = 0;
            for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
            {
                imported_matrix(nrows,ncols) = static_cast<int>(abs(*itit));
                ncols++;
            }
            nrows++;
            traj_col = ncols;
        }
    }else{
        cout<< RED <<"Inconsistent matrix size for trajectory generation" << DEFAULT << endl;
        return 0;
    }
    cout << "File read successful" <<endl;

    std::vector<int> temp(ncols,0);
    for (int j=0; j<ncols; j++){
        for(int i=0; i<nrows; i++){
            if (imported_matrix(i,j) > temp[j]){
                temp[j] = imported_matrix(i,j);
            }
        }
    }

    *emg_calib_vec = temp;
    return 1;
}



// ----------------------------Main function------------------------------
int main(int argc, char** argv)
{
    bool vsmFlag = true; // true - if you want to connect to dynamixel else false
    bool vrepFlag = true;
    bool Calib_Flag = false;
    bool FT_calibrated = false;
    bool high_grasp_flag = false;
    stringstream strValue;

    // arg to pass to the code is in the format "sudo ./admittance_3d_velMode -v/vo arg2 arg3 ..."
    if (argc>1){
        if (strcmp(argv[1], "-h") == 0) {
            printf(GREEN "Supported arguments in order \n ./admittance_3d_velMode -calib \n (or) \n ./admittance_3d_velMode -no_vsm -ld..\n");
            printf("arg1: -calib : prepare just for EMG calibration of the subject\n");
            printf("arg1: -no_vsm : if you don't want to use VSM (or) \n      dynamixel initial position value in between [100, 2048] \n");
            printf("arg2: ld : for low damping (or) var: for variable damping (or) hd: for high damping \n by default it is set to high damping\n" DEFAULT);

            return 0;
        }
        else if ((strcmp(argv[1], "-calib") == 0)) {
            printf("Calibration of EMG using isometric contractions \n");  
            Calib_Flag = true;
            vsmFlag = false;
        }
    }
    
 
    std::cout<< "Please enter experiment id:"<<std::endl;
    std::cin>>exp_id;
    std::cin.get();
    
    
    

    struct stat info;
    std::string folderpath;
    folderpath = "../data/hamid/emg_test/";

    const char *path1 = folderpath.c_str();

    // Create a folder to save subject's data
    if (stat(path1, &info) == -1)   mkdir(path1, 0777);



    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;

    // Initialize Myo band
    myo::Client client = Myo_init(); // initializing the myo band here works (sometimes it works here and sometimes after initializing the admittance control thread).

    // Muscle activity calibration using the MYO armband
    if (Calib_Flag) 
    {    
        printf("Relax your muscles and get ready to lift the weight and hold it in upright position for 15 seconds...\n" );
        usleep(1*1000*1000);

        boost::thread Calib_thread(calib_EMG,&Calib_Flag, folderpath);

        usleep(15*1000*1000);
        Calib_Flag = false;
        printf(GREEN "Calibration completed, you can relax now\n" DEFAULT);
        
        Calib_thread.interrupt();
        return 0;
    }
    
    // estimate the maximum contraction values from the calibration exercise if exists.
    // std::vector<int> emg_calib_vec(numChannels,0);
    // if (!EMG_max_contrx(folderpath, &emg_calib_vec)) return 0;

    
    // Myo thread for the actual experiment
    bool Myo_errFlag = false;
    boost::thread Myo_thread(Myo_receive,&Myo_errFlag, folderpath);

    bool running = true;  

   
    // stop the program using the keyboard's Enter key
    bool stopFlag = false;
    boost::thread stop_thread(stop,&stopFlag);

    
    
    std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
    std::chrono::duration<float> pause_loop = std::chrono::system_clock::now() - start_time;



    start_time = std::chrono::system_clock::now();
    pause_loop = std::chrono::system_clock::now() - start_time;
    std::cout << pause_loop.count()*1000*1000 << std::endl;
    std::cout << "Simulation started " << std::endl;
   


    
    
    timeLoop = std::chrono::system_clock::now();
    while (!stopFlag) {
        
        // printf("running");
        
        
        pause_loop = std::chrono::system_clock::now() - timeLoop;
        
        if (pause_loop.count() > 10)
        {
            stopFlag = true;
        }
        

        
    }
        



    // stop the threads
    Myo_thread.interrupt();
   
    stop_thread.interrupt();
    
    
    //   usleep(200*1000);
    cout << "Exiting ..." << endl;

}
