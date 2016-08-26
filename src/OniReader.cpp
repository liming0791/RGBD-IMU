#include "OniReader.h"

using namespace openni;
using namespace std;

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

OniReader::OniReader()
{
    //open kinect
    rc = OpenNI::initialize();
    if (rc != STATUS_OK)
    {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
    }

    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK)
    {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
    }

    //set DepthColorSync
    rc = device.setDepthColorSyncEnabled(true);
    if(rc==STATUS_OK){
        cout << "set DepthColorSync ok !" << endl;
    }else{
        cout << "set DepthColorSync failed !" << endl;
    }

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

    //set registration
    if (device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
    {
        device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
        cout <<"Registration OK!" << endl;
    }else{
        cout << "Registration not surported !" << endl;
    }

    

    //VideoStream* streams[] = {&depth, &color};
    streams[0] = &depth;
    streams[1] = &color;

    started = false;
       
}

OniReader::~OniReader()
{
    started = false;
    depth.stop();
    color.stop();
    depth.destroy();
    color.destroy();
    device.close();
    OpenNI::shutdown();
}

void OniReader::close()
{
    started = false;
}

void OniReader::begin()
{
    started = true;
    boost::thread thrd(boost::bind(&OniReader::loop, this));
    thrd.detach();
}

void OniReader::loop()
{
    while (started)
    {
        //read kinect
        int readyStream = -1;
        rc = OpenNI::waitForAnyStream(streams, 2, &readyStream, SAMPLE_READ_WAIT_TIMEOUT);
        if (rc != STATUS_OK)
        {
            printf("Wait failed! (timeout is %d ms)\n%s\n", 
                    SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
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

        if(_callback){
            //boost::thread thrd(boost::bind(&OniReader::_callback, this, frame));
            boost::thread thrd(_callback, frame);
            thrd.detach();
            //_callback(frame);
        }

    }
}

void OniReader::setCallback(const boost::function<void (VideoFrameRef&) >& callback)
{
    _callback = callback;
}
