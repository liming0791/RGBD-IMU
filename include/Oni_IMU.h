#include "stdlib.h"
#include "stdio.h"

#include <chrono>

#include "boost/thread/mutex.hpp"
#include "boost/thread/locks.hpp"

#include "AsyncSerial.h"
#include "OniReader.h"

class Oni_IMU
{
    public:
        Oni_IMU();
        ~Oni_IMU();
        bool init(char* port, int baud, int* width, int* height);
        bool initIMU(char* port, int baud);
        bool initOni(int* width = NULL, int* height = NULL);
        void setFrameCallback(
                boost::function< void (int16_t*, unsigned char*, long long , long long) >& callback);
        void setIMUCallback(boost::function< void (float*, long long ) >& callback);
        void getData(int16_t* depthPtr, 
                unsigned char* colorPtr, 
                float* IMUData,
                long long& depthTime,
                long long& colorTime,
                long long& imuTime);
        void getFrameData(int16_t* depthPtr, 
                unsigned char* colorPtr, 
                long long& depthTime, 
                long long& colorTime);
        void getIMUData(float* IMUData, long long& IMUTime);
        void close();
        void begin();
        void stop();
        void IMUReceived(const char* data, size_t len);
        void FrameReceived(VideoFrameRef& depthFrame, VideoFrameRef& colorFrame);
        void getFloats(char* str, float* f);

    private:
        CallbackAsyncSerial serial;
        OniReader oni;
        
        stringstream ssline;
        char line[200];
        int char_idx;

        float R[9]; // IMU data sync with Frame
        float RealTimeR[9]; // IMU data with real frequence
        unsigned char* ColorFrame;
        int16_t* DepthFrame;

        int _width, _height;
        long long base_time_stamp;
        long long color_time_stamp;
        long long depth_time_stamp;
        long long imu_time_stamp;
        long long realtime_imu_time_stamp;
        long long last_color_time_stamp;
        long long last_depth_time_stamp;
        long long last_imu_time_stamp;

        int d_count;
        int c_count;
        int i_count;

        bool firstFrame;
        bool frameReady;
        bool firstIMU;
        bool IMUReady;

        boost::mutex frame_mutex;
        boost::mutex data_mutex;
        boost::mutex imu_mutex;

        bool imuInited;
        bool oniInited;

        bool started;

        std::chrono::high_resolution_clock::time_point imu_base_time;
        std::chrono::high_resolution_clock::time_point imu_last_time;

        boost::function< void (int16_t*, unsigned char*, long long , long long) > frameCallback;
        boost::function< void (float*, long long) > imuCallback;
};
