# RGBD-IMU

Tool to get RGB-D image sequence from kinect and IMU rotation matrix from GY-85, and the data will be used to test [InfiniTAMv2](https://github.com/victorprad/InfiniTAM/). 

It use a serial port reader based on boost library to read IMU data from an Arduino-like device (Arduino code on [GY85_EKF](https://github.com/liming0791/GY85_EKF)). 

It use OpenNI2 library and libfreenect-OpenNIDriver to get RGB-D images from a kinect v1 device.

# Dependecy

1. Boost
2. OpenCV
3. OpenNI2(libfreenect-OpenNIDriver for kinect)

