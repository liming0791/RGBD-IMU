#include "stdio.h"
#include "stdlib.h"

#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>

#include <OpenNI.h> 
#include "OniSampleUtilities.h"

using namespace openni;
using namespace std;

class OniReader
{
    public:
        OniReader();
        ~OniReader();
        void setCallback(const boost::function<void (VideoFrameRef&)>& callback);
        void close();
        void begin();
    private:
        Device device;
        VideoStream depth, color;
        VideoFrameRef frame;
        VideoStream* streams[2];
        boost::function<void (VideoFrameRef&)> _callback;
        void loop();
        bool started;
        Status rc;
};

