#include "stdio.h"
#include "stdlib.h"

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "Oni_IMU.h"

using namespace std;

int main(int argc, char** argv)
{

    int width, height;
    int16_t *depthPtr;
    unsigned char * colorPtr;
    long long depthTime = 0, colorTime = 0 ;
    int frameid = 0;
    char name[50] = "";

    ofstream log("log.txt");
    log << "depth         color" << endl;

    cv::namedWindow("depth");
    cv::namedWindow("color");

    Oni_IMU oni_imu;
    oni_imu.initOni(&width, &height);

    depthPtr = (int16_t *) malloc(width*height*2*sizeof(char));
    colorPtr = (unsigned char *)malloc(width*height*3*sizeof(char));

    cv::Mat depthMat(height, width, CV_16UC1, (char *)depthPtr);
    cv::Mat colorMat(height, width, CV_8UC3, (char *)colorPtr);

    while(cv::waitKey(1)!='b'){}

    oni_imu.begin();

    while( cv::waitKey(3) != 'q')
    {
        oni_imu.getFrameData((int16_t *)depthMat.data, (unsigned char*)colorMat.data, depthTime, colorTime );
        log << depthTime << "  " << colorTime <<  endl;

        cv::cvtColor(colorMat, colorMat, CV_RGB2BGR);
        cv::imshow("color", colorMat);
        cv::imshow("depth", depthMat);

        //sprintf(name, "%04d_d.png", frameid);
        //cv::imwrite(name, depthMat);
        //sprintf(name, "%04d_c.png", frameid);
        //cv::imwrite(name, colorMat);

        frameid++;
    }

    oni_imu.close();
    log.close();

    return 0;
}
