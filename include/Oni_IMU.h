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
        void getData(int16_t* depthPtr, 
                unsigned char* colorPtr, 
                float* IMUData,
                long long& depthTime,
                long long& colorTime,
                long long& imuTime);
        void close();
        void begin();
        void stop();
        void IMUReceived(const char* data, size_t len);
        void FrameReceived(VideoFrameRef& depthFrame, VideoFrameRef& colorFrame);

    private:
        CallbackAsyncSerial serial;
        OniReader oni;
        
        stringstream ssline;

        float R[9];
        unsigned char* ColorFrame;
        int16_t* DepthFrame;

        int _width, _height;
        long long base_time_stamp;
        long long color_time_stamp;
        long long depth_time_stamp;
        long long imu_time_stamp;
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

        bool imuInited;
        bool oniInited;

        bool started;

        std::chrono::high_resolution_clock::time_point imu_base_time;
        std::chrono::high_resolution_clock::time_point imu_last_time;
};
