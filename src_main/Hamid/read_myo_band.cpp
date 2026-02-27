#include <iostream>
#include <fstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <vector>
#include <sys/stat.h>
#include "myolinux/myoclient.h"
#include "myolinux/serial.h"

using namespace std;
using namespace myolinux;

emg_csv_file = "emg_test.csv";
imu_csv_file = "imu_test.csv";
string folder = "../../data/Hamid/myo/";

constexpr float dt = 0.005f;
constexpr int windLen = 30;
constexpr int numChannels = 8;

Client client(Serial{"/dev/ttyACM0", 115200});

atomic<bool> running(true);
atomic<bool> recording(true);

mutex dataMutex;

vector<int> EMG(8,0);
vector<float> ORI(4,0);
vector<float> ACC(3,0);
vector<float> GYR(3,0);

void Myo_init()
{
    client.connect();
    if (!client.connected()) {
        cout << "Unable to connect to Myo\n";
        exit(1);
    }

    client.setSleepMode(SleepMode::NeverSleep);
    client.setMode(EmgMode::SendEmg,
                   ImuMode::SendData,
                   ClassifierMode::Disabled);

    client.onEmg([](EmgSample sample){
        lock_guard<mutex> lock(dataMutex);
        for(int i=0;i<8;i++)
            EMG[i] = sample[i];
    });

    client.onImu([](OrientationSample ori,
                    AccelerometerSample acc,
                    GyroscopeSample gyr){
        lock_guard<mutex> lock(dataMutex);
        for(int i=0;i<4;i++){
            ORI[i]=ori[i];
            if(i<3){
                ACC[i]=acc[i];
                GYR[i]=gyr[i];
            }
        }
    });
}

void Myo_receive(const string& folder)
{
    ofstream emgFile(folder+emg_csv_file);
    ofstream imuFile(folder+imu_csv_file);

    emgFile << "Time,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8"<< endl;
    imuFile << "Time,ORI1,ORI2,ORI3,ORI4,ACC1,ACC2,ACC3,GYR1,GYR2,GYR3"<< endl;

    while(running)
    {
        client.listen();

        if(!recording) continue;

        auto now = chrono::system_clock::now();
        auto ms = chrono::duration_cast<chrono::milliseconds>(
                  now.time_since_epoch()).count();

        vector<int> emgCopy(8);
        vector<float> oriCopy(4), accCopy(3), gyrCopy(3);

        {
            lock_guard<mutex> lock(dataMutex);
            emgCopy = EMG;
            oriCopy = ORI;
            accCopy = ACC;
            gyrCopy = GYR;
        }

        emgFile << ms;
        for(auto v:emgCopy) emgFile<<","<<v;
        emgFile<<"\n";

        imuFile << ms;
        for(auto v:oriCopy) imuFile<<","<<v;
        for(auto v:accCopy) imuFile<<","<<v;
        for(auto v:gyrCopy) imuFile<<","<<v;
        imuFile<<"\n";
    }

    emgFile.close();
    imuFile.close();
}

void stopThread()
{
    cin.get();
    running = false;
}

int main()
{
    mkdir(folder.c_str(),0777);

    Myo_init();

    thread myoThread(Myo_receive,folder);
    thread stopper(stopThread);

    myoThread.join();
    stopper.join();

    cout<<"Exiting cleanly\n";
    return 0;
}