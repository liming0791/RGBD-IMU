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
#include "opencv2/opencv.hpp"
#include "AsyncSerial.h"
#include "OniReader.h"

using namespace openni;
using namespace std;

stringstream ssline;
bool ifrecord = false;
int depthNo = 0;
int colorNo = 0;
int imuIdx = 0;
int depth_expImuIdx = 1;
int color_expImuIdx = 1;
ofstream imuFile, depth_frameFile, color_frameFile;
auto d_start_time = chrono::high_resolution_clock::now(); int d_count = 0;
auto c_start_time = chrono::high_resolution_clock::now(); int c_count = 0;
auto i_start_time = chrono::high_resolution_clock::now(); int i_count = 0;

auto i_base_time = chrono::high_resolution_clock::now(); 
float base_timeStamp = 0 ;

bool firstFrame = true;
bool firstIMU = true;
int frameReady = 0;
int colorReady = 0;

void analyzeFrame(const VideoFrameRef& frame)
{
    DepthPixel* pDepth;
    RGB888Pixel* pColor;

    int height = frame.getHeight();
    int width = frame.getWidth();
    cv::Mat dMat(height, width, CV_16UC1);
    cv::Mat cMat(height, width, CV_8UC3);
    char name[20] = "";
    //printf("width: %d, height: %d\n", width, height);
    //int middleIndex = (height+1)*width/2;

    switch (frame.getVideoMode().getPixelFormat())
    {
        case PIXEL_FORMAT_DEPTH_1_MM:
        case PIXEL_FORMAT_DEPTH_100_UM:
            frameReady = 1;
            pDepth = (DepthPixel*)frame.getData();
            //printf("[%08llu] %8d\n", (long long)frame.getTimestamp(),
            //      pDepth[middleIndex]);
            memcpy(dMat.data, pDepth, height*width*2);
            cv::imshow("depth", dMat);
            cv::waitKey(1);
            if(ifrecord){
                    float timeStamp = (long long)frame.getTimestamp()/1000000.f;
                    if(firstFrame){
                        base_timeStamp = timeStamp;
                        firstFrame = false;
                    } sprintf(name, "d%04d %.6f", depthNo, timeStamp - base_timeStamp);
                    depth_frameFile << name << endl;
                    sprintf(name, "d%04d.png", depthNo++ );
                    cv::imwrite(name, dMat);
            }
            //time it
            d_count++;
            if(d_count==10){
                d_count = 0;
                auto d_end_time = chrono::high_resolution_clock::now();
                cout << "depth: " << 1.f / (chrono::duration_cast<chrono::microseconds>(d_end_time-d_start_time).count()/1000000.f/10.f) << "Hz" << endl;
                d_start_time = chrono::high_resolution_clock::now();
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
            cv::waitKey(1);
            if(ifrecord){
                float timeStamp = (long long)frame.getTimestamp()/1000000.f;
                if(firstFrame){
                    base_timeStamp = timeStamp;
                    firstFrame = false;
                }
                sprintf(name, "c%04d %.6f", colorNo, timeStamp - base_timeStamp);
                color_frameFile << name << endl;
                sprintf(name, "c%04d.png", colorNo++ );
                cv::imwrite(name, cMat);
            }
            //time it
            c_count++;
            if(c_count==10){
                c_count = 0;
                auto c_end_time = chrono::high_resolution_clock::now();
                cout << "color: " << 1.f / (chrono::duration_cast<chrono::microseconds>(c_end_time-c_start_time).count()/1000000.f/10.f) << "Hz" << endl;
                c_start_time = chrono::high_resolution_clock::now();
            }
            break;
        default:
            printf("Unknown format\n");
    }
}

void put(stringstream &ss){
    char name[20] = "";
    if(ifrecord){
        if(frameReady==1){
            auto time = chrono::high_resolution_clock::now();
            if(firstIMU){
                i_base_time = time;
                firstIMU = false;
            }
            sprintf(name, "%04d %.6f", imuIdx, chrono::duration_cast<chrono::microseconds>(time-i_base_time).count()/1000000.f);
            imuFile << name << " " << ss.str();
            imuIdx++;
            frameReady=0;
        }
    }
    //time it
    i_count++;
    if(i_count==10){
        i_count = 0;
        auto i_end_time = chrono::high_resolution_clock::now();
        cout << "imu: " << 1.f / (chrono::duration_cast<chrono::microseconds>(i_end_time-i_start_time).count()/1000000.f/10.f) << "Hz" << endl;
        i_start_time = chrono::high_resolution_clock::now();
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

    cv::namedWindow("command");
    cv::namedWindow("depth");
    cv::namedWindow("color");

    try {

        //open serial
        imuFile.open("imu.txt");
        CallbackAsyncSerial serial(argv[1],
                boost::lexical_cast<unsigned int>(argv[2]));
        serial.setCallback(received);

        //open kinect
        depth_frameFile.open("depthframe.txt");
        color_frameFile.open("colorframe.txt");
        OniReader oniReader;
        oniReader.setCallback(analyzeFrame);
        oniReader.begin();

        while(true){
            char key = cv::waitKey(1);
            //cout << "Press key: " << key << endl;
            if(key == 'b')
                ifrecord = true;
            else if(key == 's'){
                ifrecord = false;
                break;
            }
        }
        
        serial.close();
        oniReader.close();
        imuFile.close();
        depth_frameFile.close();
        color_frameFile.close();

    } catch (std::exception& e) {
        cerr<<"Exception: "<<e.what()<<endl;
    }

    tcsetattr(0, TCSANOW, &stored_settings);

    return 0;
}


//void writePPM(int width, int height, unsigned char* data, int imageNo, long long time)
//{
//    char strtemp[20] = "";
//    sprintf(strtemp,"%04d.ppm",imageNo);
//    //sprintf(strtemp,"%04d.%.6f.ppm",imageNo, time/1000000.);
//    ofstream output(strtemp, ios::binary|ios::out);
//    if(!output){
//        cout << " unable to open the output file "<< strtemp << endl;
//    }
//    else{
//        output << "P6"<< endl ;
//        output << width;
//        output << " ";
//        output << height;
//        output << endl;
//        output << 255 << endl;
//        output.write( (const char *)data, height*width*3);
//        output.close();
//    };//end of else
//}
//
//void writePGM(int width, int height, int16_t* data, int imageNo, long long time)
//{
//    char strtemp[20] = "";
//    sprintf(strtemp,"%04d.pgm",imageNo);
//    //sprintf(strtemp,"%04d.%.6f.pgm",imageNo, time/1000000.);
//    ofstream output(strtemp, ios::binary|ios::out);
//    if(!output){
//        cout << " unable to open the output file " << strtemp << endl;
//    }else{
//        output << "P5\n" << width << " " << height << "\n" << 65535 << "\n";
//        output.write((const char*)data, width*height*2);
//        output.close();
//    }
//}
