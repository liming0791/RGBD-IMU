#include "Oni_IMU.h"
#include <chrono>
#include "opencv2/opencv.hpp"

using namespace std;

Oni_IMU::Oni_IMU()
{
    firstFrame = true;
    frameReady = false;
    firstIMU = true;
    IMUReady = false;
    imuInited = false;
    oniInited = false;
    started = false;

    ColorFrame = NULL;
    DepthFrame = NULL;

    d_count = i_count = c_count = 0;
}

Oni_IMU::~Oni_IMU()
{
    if(ColorFrame){
        delete(ColorFrame);
        ColorFrame = NULL;
    }

    if(DepthFrame){
        delete(DepthFrame);    
        DepthFrame = NULL;
    }
}

/*
 * INIT
 */

bool Oni_IMU::init(char* port, int baud, int* width, int* height)
{
    return (initIMU(port, baud) && initOni(width, height));
}

bool Oni_IMU::initIMU(char* port, int baud)
{
    printf("init imu......\n");
    if(imuInited){
        printf("IMU has inited !");
        return true;
    }
    serial.open(std::string(port), (unsigned int)baud);
    serial.setCallback(boost::bind(&Oni_IMU::IMUReceived, this, _1, _2));
    imu_base_time = chrono::high_resolution_clock::now();
    imu_last_time = chrono::high_resolution_clock::now();
    imuInited = true;

    printf("init imu done\n");
    return true;
}

bool Oni_IMU::initOni(int* width, int* height)
{
    printf("init oni......\n");
    if(oniInited){
        printf("OpenNI has inited !");
        return true;
    }
    if(!oni.init(&_width, &_height)) 
        return false;
    oni.setCallback(boost::bind(&Oni_IMU::FrameReceived, this, _1, _2));
    if(width!=NULL) *width = _width;
    if(height!=NULL) *height = _height;
    ColorFrame = (unsigned char*)malloc(_width*_height*3*sizeof(char));
    DepthFrame = (int16_t *)malloc(_width*_height*2*sizeof(char));
    oniInited = true;
    //cv::namedWindow("test_depth");
    //cv::namedWindow("test_color");
    //
    printf("init oni done\n");
    return true;
}

void Oni_IMU::IMUReceived(const char *data, size_t len)
{
    vector<char> v(data,data+len);
    for(unsigned int i=0, _end = v.size();i<_end;i++)
    {
        if(v[i]=='\n')
        {
            ssline<<endl;

            //time it
            i_count++;
            if(i_count==10){
                i_count = 0;
                auto nowTime = chrono::high_resolution_clock::now();
                long long dua = (long long)chrono::duration_cast<chrono::microseconds>(nowTime - imu_last_time).count();
                printf("imu rate: %f\n", 1.f/ (dua/1000000.f/10.f));
                imu_last_time = chrono::high_resolution_clock::now();
            }

            {
                boost::lock_guard<boost::mutex> lock(frame_mutex);

                if(frameReady){
                    {
                        boost::lock_guard<boost::mutex> lock(data_mutex);

                        if(firstIMU){
                            firstIMU = false;
                            imu_base_time = chrono::high_resolution_clock::now();
                        }
                        auto nowTime = chrono::high_resolution_clock::now();
                        imu_time_stamp = (long long)chrono::duration_cast<chrono::microseconds>(nowTime - imu_base_time).count();
                        printf("imu timestam: %lld\n", imu_time_stamp);

                        ssline >> R[0] >> R[1] >> R[2]
                            >> R[3] >> R[4] >> R[5]
                            >> R[6] >> R[7] >> R[8];
                    }
                    frameReady = false;
                }
            }
            ssline.str(std::string());
        } else {
            ssline << v[i];
        }
    }
}

void Oni_IMU::FrameReceived(
        VideoFrameRef& depthFrame, 
        VideoFrameRef& colorFrame)
{

    DepthPixel* pDepth;
    RGB888Pixel* pColor;
    int height = colorFrame.getHeight();
    int width = colorFrame.getWidth();

    pDepth = (DepthPixel*)depthFrame.getData();
    pColor = (RGB888Pixel*)colorFrame.getData();

    {
        boost::lock_guard<boost::mutex> lock(data_mutex);

        //depth frame
        //cv::Mat depthMat(height, width, CV_16UC1);
        //memcpy(depthMat.data, pDepth, width*height*2);
        //cv::imshow("test_depth", depthMat);

        memcpy(DepthFrame, pDepth, height*width*2);
        depth_time_stamp = (long long)depthFrame.getTimestamp();
        if(firstFrame){
            base_time_stamp = depth_time_stamp;
            firstFrame = false;
            printf("base frame timestamp: %lld\n", base_time_stamp);
        }else{
            depth_time_stamp -= base_time_stamp;
        } 
        printf("depth timestamp: %lld\n", depth_time_stamp);

        //eime it
        d_count++;
        if(d_count==10){
            d_count = 0;
            printf("depth rate: %f\n", 
                    1.f/((depth_time_stamp - last_depth_time_stamp)/1000000.f/10.f));
            last_depth_time_stamp = depth_time_stamp;
        }

        //color frame
        //cv::Mat colorMat(height, width, CV_8UC3);
        //memcpy(colorMat.data, pColor, width*height*3);
        //cv::imshow("test_color", colorMat);

        memcpy(ColorFrame, pColor, height*width*3);
        color_time_stamp = (long long)colorFrame.getTimestamp();
        if(firstFrame){
            base_time_stamp = color_time_stamp;
            firstFrame = false;
            printf("base frame timestamp: %lld\n", base_time_stamp);
        }else{
            color_time_stamp -= base_time_stamp;
        }
        printf("color timestamp: %lld\n", color_time_stamp);

        c_count++;
        if(c_count==10){
            c_count = 0;
            printf("color rate: %f\n", 
                    1.f/((color_time_stamp - last_color_time_stamp)/1000000.f/10.f));
            last_color_time_stamp = color_time_stamp;
        }
    }

    {
        boost::lock_guard<boost::mutex> lock(frame_mutex);
        frameReady = true;
    }
}

/*
 * BEGIN
 */

void Oni_IMU::begin()
{
    if(started){
        printf("already started !");
        return;
    }

    started = true;
    oni.begin();

    printf("start oni_imu\n");
}

/*
 * GETDATA
 */

void Oni_IMU::getData(int16_t* depthPtr, unsigned char* colorPtr, float* IMUData, 
        long long& depthTime, long long& colorTime, long long& IMUTime)
{
    if(!started){
        printf("Has not started !\n");
        memset(depthPtr, 0, _width*_height*2);
        memset(colorPtr, 0, _width*_height*3);
        memset(IMUData, 0, 9*sizeof(float));
        return;
    }

    bool ready = true;
    while(ready){
        {
            boost::lock_guard<boost::mutex> lock(frame_mutex); 
            ready = frameReady;
        }
       usleep(2); 
    }

    {
        boost::lock_guard<boost::mutex> lock(data_mutex);

        memcpy(depthPtr, DepthFrame, _width*_height*2*sizeof(char));
        memcpy(colorPtr, ColorFrame, _width*_height*3*sizeof(char));  
        memcpy(IMUData, R, 9*sizeof(float));

        depthTime = depth_time_stamp;
        colorTime = color_time_stamp;
        IMUTime = imu_time_stamp;
    }

}

/*
 *STOP
 */

void Oni_IMU::stop()
{

}

/*
 * CLOSE
 */

void Oni_IMU::close()
{
    serial.close();
    oni.close();

}
