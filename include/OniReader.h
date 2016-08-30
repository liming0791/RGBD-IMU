#include "stdio.h"
#include "stdlib.h"

#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>

#include <OpenNI.h> 

using namespace openni;
using namespace std;

class OniReader
{
    public:
        OniReader();
        ~OniReader();
        bool init(int* width = NULL, int* height = NULL);
        void setCallback(const boost::function<void (VideoFrameRef&, VideoFrameRef&)>& callback);
        void close();
        void begin();
    private:
        Device device;
        VideoStream depth, color;
        VideoFrameRef colorFrame, depthFrame;
        VideoStream* streams[2];
        boost::function<void (VideoFrameRef&, VideoFrameRef&)> _callback;
        void loop();
        bool started;
        Status rc;
};

