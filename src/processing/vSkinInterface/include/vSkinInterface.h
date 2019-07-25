#ifndef __VSKININTERFACE__
#define __VSKININTERFACE__

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <iCub/eventdriven/all.h>
#include <iCub/eventdriven/deprecated.h>

using namespace ev;

/*////////////////////////////////////////////////////////////////////////////*/
// SKININTERFACE
/*////////////////////////////////////////////////////////////////////////////*/

class skinInterface : public yarp::os::Thread
{
private:

    //data structures and ports
    vReadPort< std::vector<ev::SkinEvent> > inputPort;

    double count;

    //diagnostics
    yarp::os::BufferedPort<yarp::os::Bottle> scopePort;

public:

    skinInterface() {}

    bool open(std::string name);

    //bool threadInit();
    void onStop();
    void run();
    //void threadRelease();

};





#endif
