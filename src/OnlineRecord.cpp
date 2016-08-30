#include "stdio.h"
#include "stdlib.h"

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "Oni_IMU.h"

using namespace std;

int main(int argc, char** argv)
{
    if(argc < 3){
        printf("need more args ");
        return 0;
    }

    int width, height;
    int16_t *depthPtr;
    unsigned char * colorPtr;
    float imuR[9];

    long long depthTime = 0, colorTime = 0, imuTime = 0;


    ofstream of("log.txt");
    of << "depth         color          imu" << endl;

    cv::namedWindow("depth");
    cv::namedWindow("color");


    Oni_IMU oni_imu;
    oni_imu.init(argv[1], atoi(argv[2]), &width, &height);

    depthPtr = (int16_t *) malloc(width*height*2*sizeof(char));
    colorPtr = (unsigned char *)malloc(width*height*3*sizeof(char));

    cv::Mat depthMat(height, width, CV_16UC1, (char *)depthPtr);
    cv::Mat colorMat(height, width, CV_8UC3, (char *)colorPtr);

    while(cv::waitKey(1)!='b'){}

    oni_imu.begin();

    while( cv::waitKey(33) != 'q')
    {
        oni_imu.getData((int16_t *)depthMat.data, (unsigned char*)colorMat.data, imuR, depthTime, colorTime, imuTime);
        of << depthTime << "  " << colorTime << "  " << imuTime << endl;
        cv::cvtColor(colorMat, colorMat, CV_RGB2BGR);
        cv::imshow("color", colorMat);
        cv::imshow("depth", depthMat);
    }

    oni_imu.close();
    of.close();

    return 0;
}
