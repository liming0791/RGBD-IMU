#include <stdio.h>
#include <stdlib.h>

#include <ctime>
#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>

#include <termios.h>

#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>

#include <OpenNI.h> 
#include "OniSampleUtilities.h"

#include "opencv2/opencv.hpp"

#include "AsyncSerial.h"

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

using namespace openni;
using namespace std;

stringstream ssline;
auto start_time = chrono::high_resolution_clock::now(); 

bool ifrecord = false;
int depthNo = 0;
int colorNo = 0;

int imuIdx = 0;
int depth_expImuIdx = 1;
int color_expImuIdx = 1;

ofstream imuFile;

void writePPM(int width, int height, unsigned char* data, int imageNo, long long time)
{
    char strtemp[20] = "";
    sprintf(strtemp,"%04d.ppm",imageNo);
    //sprintf(strtemp,"%04d.%.6f.ppm",imageNo, time/1000000.);
    ofstream output(strtemp, ios::binary|ios::out);
    if(!output){
        cout << " unable to open the output file "<< strtemp << endl;
    }
    else{
        output << "P6"<< endl ;
        output << width;
        output << " ";
        output << height;
        output << endl;
        output << 255 << endl;
        output.write( (const char *)data, height*width*3);
        output.close();
    };//end of else
}

void writePGM(int width, int height, int16_t* data, int imageNo, long long time)
{
    char strtemp[20] = "";
    sprintf(strtemp,"%04d.pgm",imageNo);
    //sprintf(strtemp,"%04d.%.6f.pgm",imageNo, time/1000000.);
    ofstream output(strtemp, ios::binary|ios::out);
    if(!output){
        cout << " unable to open the output file " << strtemp << endl;
    }else{
        output << "P5\n" << width << " " << height << "\n" << 65535 << "\n";
        output.write((const char*)data, width*height*2);
        output.close();
    }
}

void analyzeFrame(const VideoFrameRef& frame)
{
    DepthPixel* pDepth;
    RGB888Pixel* pColor;

    int height = frame.getHeight();
    int width = frame.getWidth();
    cv::Mat dMat(height, width, CV_16UC1);
    cv::Mat cMat(height, width, CV_8UC3);
    char name[20] = "";
    double min, max;

    printf("width: %d, height: %d\n", width, height);

    int middleIndex = (height+1)*width/2;

    switch (frame.getVideoMode().getPixelFormat())
    {
        case PIXEL_FORMAT_DEPTH_1_MM:
        case PIXEL_FORMAT_DEPTH_100_UM:
            pDepth = (DepthPixel*)frame.getData();
            //printf("[%08llu] %8d\n", (long long)frame.getTimestamp(),
            //      pDepth[middleIndex]);
            memcpy(dMat.data, pDepth, height*width*2);
            cv::imshow("liming", dMat);
            if(ifrecord)
                if(depth_expImuIdx <= imuIdx){
                    sprintf(name, "d%04d.png", depthNo++);
                    cv::imwrite(name, dMat);
                    //writePGM(width, height, (int16_t *)pDepth, depthNo++, 
                    //      (long long)frame.getTimestamp());
                    while(depth_expImuIdx <= imuIdx)
                        depth_expImuIdx++;
                }
            break;
        case PIXEL_FORMAT_RGB888:
            pColor = (RGB888Pixel*)frame.getData();
            //printf("[%08llu] 0x%02x%02x%02x\n", (long long)frame.getTimestamp(),
            //        pColor[middleIndex].r&0xff,
            //        pColor[middleIndex].g&0xff,
            //        pColor[middleIndex].b&0xff);
            memcpy(cMat.data, pColor, height*width*3);
            cv::imshow("color", cMat);
            if(ifrecord)
                if(color_expImuIdx <= imuIdx){
                    sprintf(name, "c%04d.png", colorNo++);
                    cv::imwrite(name, cMat);
                    //writePPM(width, height, (unsigned char *)pColor, colorNo++, 
                    //      (long long)frame.getTimestamp());
                    while(color_expImuIdx <= imuIdx)
                        color_expImuIdx++;
                }
            break;
        default:
            printf("Unknown format\n");
    }
}

void put(stringstream &ss){
    //auto end_time = chrono::high_resolution_clock::now();
    char name[20] = "";
    sprintf(name, "%4d", imuIdx);
    if(ifrecord){
        imuFile << /*chrono::duration_cast<chrono::microseconds>(end_time - start_time).count()/1000000.f 
                << " " <<*/ name << " " << ss.str();
        imuIdx++;
    }
}

void received(const char *data, unsigned int len)
{
    vector<char> v(data,data+len);
    for(unsigned int i=0, _end = v.size();i<_end;i++)
    {
        if(v[i]=='\n')
        {
            ssline<<endl;
            put(ssline);
            ssline.str(string());
        } else {
            ssline << v[i];
        }
    }
}

int main(int argc, char* argv[])
{
    if(argc!=3)
    {
        cerr<<"Usage: serial port baudrate"<<endl<<
                "To quit type Ctrl-C x"<<endl<<
                "To send Ctrl-C type Ctrl-C Ctrl-C"<<endl;
        return 1;
    }

    termios stored_settings;
    tcgetattr(0, &stored_settings);
    termios new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_lflag &= (~ISIG); // don't automatically handle control-C
    new_settings.c_lflag &= ~(ECHO); // no echo
    tcsetattr(0, TCSANOW, &new_settings);

    cout<<"\e[2J\e[1;1H"; //Clear screen and put cursor to 1;1

    try {

        //open serial
        imuFile.open("imu.txt");
        CallbackAsyncSerial serial(argv[1],
                boost::lexical_cast<unsigned int>(argv[2]));
        serial.setCallback(received);


        //open kinect
        Status rc = OpenNI::initialize();
        if (rc != STATUS_OK)
        {
            printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
            return 1;
        }

        Device device;
        rc = device.open(ANY_DEVICE);
        if (rc != STATUS_OK)
        {
            printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
            return 2;
        }

        VideoStream depth, color;

        if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
        {
            rc = depth.create(device, SENSOR_DEPTH);
            if (rc == STATUS_OK)
            {
                rc = depth.start();
                if (rc != STATUS_OK)
                {
                    printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
                }
            }
            else
            {
                printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
            }
        }

        if (device.getSensorInfo(SENSOR_COLOR) != NULL)
        {
            rc = color.create(device, SENSOR_COLOR);
            if (rc == STATUS_OK)
            {
                rc = color.start();
                if (rc != STATUS_OK)
                {
                    printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
                }
            }
            else
            {
                printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
            }
        }

        if (device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
        {
            device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
            cout <<"Registration OK!" << endl;
        }else{
            cout << "Registration not surported !" << endl;
        }

        VideoFrameRef frame;

        VideoStream* streams[] = {&depth, &color};

        cv::namedWindow("liming");

        start_time = chrono::high_resolution_clock::now();

        while (cv::waitKey(1)!='r')
        {
            //read kinect
            int readyStream = -1;
            rc = OpenNI::waitForAnyStream(streams, 2, &readyStream, SAMPLE_READ_WAIT_TIMEOUT);
            if (rc != STATUS_OK)
            {
                printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError())      ;
                break;
            }

            switch (readyStream)
            {
                case 0:
                    // Depth
                    depth.readFrame(&frame);
                    break;
                case 1:
                    // Color
                    color.readFrame(&frame);
                    break;
                default:
                    printf("Unxpected stream\n");
            }

            analyzeFrame(frame);

            //check serial
            if(serial.errorStatus() || serial.isOpen()==false)
            {
                cerr<<"Error: serial port unexpectedly closed"<<endl;
                break;
            }

        }

        //start record
        ifrecord = true; 
        //start record end
        
        while (cv::waitKey(1)!='s')
        {
            //read kinect
            int readyStream = -1;
            rc = OpenNI::waitForAnyStream(streams, 2, &readyStream, SAMPLE_READ_WAIT_TIMEOUT);
            if (rc != STATUS_OK)
            {
                printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError())      ;
                break;
            }

            switch (readyStream)
            {
                case 0:
                    // Depth
                    depth.readFrame(&frame);
                    break;
                case 1:
                    // Color
                    color.readFrame(&frame);
                    break;
                default:
                    printf("Unxpected stream\n");
            }

            analyzeFrame(frame);

            //check serial
            if(serial.errorStatus() || serial.isOpen()==false)
            {
                cerr<<"Error: serial port unexpectedly closed"<<endl;
                break;
            }

        }

        //stop record
        ifrecord = false;
        //stop record end

        depth.stop();
        color.stop();
        depth.destroy();
        color.destroy();
        device.close();
        OpenNI::shutdown();
        serial.close();
        imuFile.close();

    } catch (std::exception& e) {
        cerr<<"Exception: "<<e.what()<<endl;
    }

    tcsetattr(0, TCSANOW, &stored_settings);

    return 0;
}
