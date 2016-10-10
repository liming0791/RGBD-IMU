#include <sys/syscall.h>
#define gettid() syscall(__NR_gettid)

#include "OniReader.h"

using namespace openni;
using namespace std;

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

OniReader::OniReader()
{
    started = false;
    inited = false; 
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

bool OniReader::init(int* width, int* height){
    //open kinect
    rc = OpenNI::initialize();
    if (rc != STATUS_OK)
    {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        return false;
    }

    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK)
    {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        return false;
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
                printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
                return false;
            }
        }
        else
        {
            printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
            return false;
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
                return false;
            }
            if(width!=NULL && height!=NULL){
               *width = color.getVideoMode().getResolutionX();
               *height = color.getVideoMode().getResolutionY(); 
               printf("open stream width: %d height: %d\n", *width, *height);
            }
        }
        else
        {
            printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
            return false;
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
    inited = true;
    return true;
}

void OniReader::close()
{
    started = false;
}

void OniReader::begin()
{
    if(!inited){
        printf("OniReader has not inited !\n");
        return;
    }
    started = true;
    boost::thread thrd(boost::bind(&OniReader::loop, this));
    thrd.detach();
}

void OniReader::loop()
{

    std::cout << "OniReader pid: "<< gettid() << std::endl;

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

        // Depth
        depth.readFrame(&depthFrame);
        // Color
        color.readFrame(&colorFrame);

        if(_callback){
            //boost::thread thrd(boost::bind(&OniReader::_callback, this, frame));
            //boost::thread thrd(_callback, depthFrame, colorFrame);
            //thrd.detach();
            _callback(depthFrame, colorFrame);
        }

    }
}

void OniReader::setCallback(const boost::function<void (VideoFrameRef&, VideoFrameRef&) >& callback)
{
    _callback = callback;
}
