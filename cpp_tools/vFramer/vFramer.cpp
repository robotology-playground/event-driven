/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "drawers.h"
#include <sstream>
#include <yarp/cv/Cv.h>
//#include "event-driven/vDrawSkin.h"

using namespace ev;
using namespace yarp::os;
using namespace yarp::sig;
using std::string;


/*////////////////////////////////////////////////////////////////////////////*/
// drawer factory
/*////////////////////////////////////////////////////////////////////////////*/
// vDraw * createDrawer(std::string tag)
// {

//     if(tag == addressDraw::drawtype)
//         return new addressDraw();
//     if(tag == binaryDraw::drawtype)
//         return new binaryDraw();
//     if(tag == grayDraw::drawtype)
//         return new grayDraw();
//     if(tag == blackDraw::drawtype)
//         return new blackDraw();
//     if(tag == isoDraw::drawtype)
//         return new isoDraw();
//     if(tag == interestDraw::drawtype)
//         return new interestDraw();
//     if(tag == circleDraw::drawtype)
//         return new circleDraw();
//     if(tag == flowDraw::drawtype)
//         return new flowDraw();
//     if(tag == clusterDraw::drawtype)
//         return new clusterDraw();
//     if(tag == blobDraw::drawtype)
//         return new blobDraw();
//     if(tag == skinDraw::drawtype)
//         return new skinDraw();
//     if(tag == skinsampleDraw::drawtype)
//         return new skinsampleDraw();
//     if(tag == isoDrawSkin::drawtype)
//         return new isoDrawSkin();
//     if(tag == taxelsampleDraw::drawtype)
//         return new taxelsampleDraw();
//     if(tag == taxeleventDraw::drawtype)
//         return new taxeleventDraw();
//     if(tag == accDraw::drawtype)
//         return new accDraw();
//     if(tag == isoInterestDraw::drawtype)
//         return new isoInterestDraw();
//     if(tag == isoCircDraw::drawtype)
//         return new isoCircDraw();
//     if(tag == overlayStereoDraw::drawtype)
//         return new overlayStereoDraw();
//     if(tag == saeDraw::drawtype)
//         return new saeDraw();
//     if(tag == imuDraw::drawtype)
//         return new imuDraw();
//     if(tag == cochleaDraw::drawtype)
//         return new cochleaDraw();
//     if(tag == rasterDraw::drawtype)
//         return new rasterDraw();
//     if(tag == rasterDrawHN::drawtype)
//         return new rasterDrawHN();
//     return 0;

// }

/*////////////////////////////////////////////////////////////////////////////*/
//module
/*////////////////////////////////////////////////////////////////////////////*/
class vFramerModule : public yarp::os::RFModule
{

private:
    std::vector<drawerInterface *> publishers;

public:
    ~vFramerModule()
    {
        //while(publishers.size())
        //    delete publishers.front();
    }

    // configure all the module parameters and return true if successful
    bool configure(yarp::os::ResourceFinder &rf) override
    {
        //admin options
        std::string moduleName = rf.check("name", Value("/vFramer")).asString();
        setName(moduleName.c_str());

        int height = rf.check("height", Value(2048)).asInt();
        int width = rf.check("width", Value(2048)).asInt();

        double eventWindow = rf.check("eventWindow", Value(0.1)).asDouble();
        eventWindow = ev::secondsToTicks(eventWindow);
        eventWindow = std::min(eventWindow, ev::max_stamp / 2.0);

        double isoWindow = rf.check("isoWindow", Value(1.0)).asDouble();
        isoWindow = ev::secondsToTicks(isoWindow);
        isoWindow = std::min(isoWindow, ev::max_stamp / 2.0);

        int frameRate = rf.check("frameRate", Value(30)).asInt();
        double period = 1.0 / frameRate;

        //bool useTimeout =
        //        rf.check("timeout") && rf.check("timeout", Value(true)).asBool();
        bool flip =
            rf.check("flip") && rf.check("flip", Value(true)).asBool();
        //bool forceRender =
        //        rf.check("forcerender") &&
        //        rf.check("forcerender", Value(true)).asBool();
        //    if(forceRender) {
        //        vReader.setStrictUpdatePeriod(vtsHelper::vtsscaler * period);
        //        period = 0;
        //    }

        //viewer options
        //set up the default channel list
        // yarp::os::Bottle tempDisplayList, *bp;
        // tempDisplayList.addString("/Left");
        // bp = &(tempDisplayList.addList()); bp->addString("AE");
        // tempDisplayList.addString("/Right");
        // bp = &(tempDisplayList.addList()); bp->addString("AE");

        // //set the output channels
        // yarp::os::Bottle * displayList = rf.find("displays").asList();
        // if(!displayList)
        //     displayList = &tempDisplayList;

        // yInfo() << displayList->toString();

        // if(displayList->size() % 2) {
        //     yError() << "Error: display list configured incorrectly" << displayList->size();
        //     return false;
        // }

        // int nDisplays = displayList->size() / 2;

        // for(int i = 0; i < nDisplays; i++) {

        //     string channel_name =
        //             moduleName + displayList->get(i*2).asString();

        //     channelInstance * new_ci = new channelInstance(channel_name);
        //     new_ci->setRate(period);

        //     Bottle * drawtypelist = displayList->get(i*2 + 1).asList();
        //     for(unsigned int j = 0; j < drawtypelist->size(); j++)
        //     {
        //         string draw_type = drawtypelist->get(j).asString();
        //         if(draw_type == "F") {
        //             new_ci->addFrameDrawer(width, height);
        //         }
        //         else if(!new_ci->addDrawer(draw_type, width, height, eventWindow, isoWindow, flip))
        //         {
        //             yError() << "Could not create specified publisher"
        //                      << channel_name << draw_type;
        //             return false;
        //         }
        //     }
        yInfo() << "Making publisher ...";
        publishers.push_back(new greyDrawer);
        if (!publishers.front()->initialise(getName("/grey"), 360, 480))
        {
            yError() << "Could not initialise publisher";
            return false;
        }
        publishers.front()->setPeriod(period);

        if (!yarp::os::Network::connect("/atis3/AE:o", getName("/grey/AE:i"), "fast_tcp"))
            yError() << "Did not conenct to source";

        //}

        yInfo() << "Starting publishers";
        for (auto pub_i = publishers.begin(); pub_i != publishers.end(); pub_i++)
        {
            if (!(*pub_i)->start())
            {
                yError() << "Could not start publisher" << (*pub_i)->drawerName();
                return false;
            }
        }

        yInfo() << "Configure done";
        return true;
    }

    bool interruptModule() override
    {
        for (auto pub_i = publishers.begin(); pub_i != publishers.end(); pub_i++)
            (*pub_i)->stop();

        return true;
    }

    bool close() override
    {
        for (auto pub_i = publishers.begin(); pub_i != publishers.end(); pub_i++)
            (*pub_i)->stop();

        return true;
    }

    //when we call update module we want to send the frame on the output port
    //we use the framerate to determine how often we do this
    bool updateModule() override
    {
        return !isStopping();
    }

    double getPeriod() override
    {
        return 1.0;
    }
};

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork(2.0))
    {
        yError() << "Could not find yarp network";
        return 1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("event-driven");
    rf.setDefaultConfigFile("vFramer.ini");
    rf.configure(argc, argv);

    vFramerModule framerModule;
    return framerModule.runModule(rf);
}