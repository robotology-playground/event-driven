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

#include "vFramer.h"
#include <sstream>

using namespace ev;

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        yError() << "Could not find yarp network";
        return 1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "vFramer.ini" );
    rf.configure( argc, argv );

    vFramerModule framerModule;
    return framerModule.runModule(rf);
}

/*////////////////////////////////////////////////////////////////////////////*/
//vFramerModule
/*////////////////////////////////////////////////////////////////////////////*/


bool vFramerModule::configure(yarp::os::ResourceFinder &rf)
{
    pyarptime = 0;
    stopped = false;

    //admin options
    std::string moduleName =
            rf.check("name", yarp::os::Value("/vFramer")).asString();
    setName(moduleName.c_str());

    int retinaHeight = rf.check("height", yarp::os::Value(240)).asInt();
    int retinaWidth = rf.check("width", yarp::os::Value(304)).asInt();

    double eventWindow =
            rf.check("eventWindow", yarp::os::Value(0.1)).asDouble();
    eventWindow = eventWindow * vtsHelper::vtsscaler;

    int frameRate = rf.check("frameRate", yarp::os::Value(30)).asInt();
    period = 1.0 / frameRate;

    bool flip = rf.check("flip") &&
            rf.check("flip", yarp::os::Value(true)).asBool();

    bool forceRender = rf.check("forcerender") &&
            rf.check("forcerender", yarp::os::Value(true)).asBool();
    if(forceRender)
        vReader.setStrictUpdatePeriod(vtsHelper::vtsscaler * period);

    //viewer options
    //set up the default channel list
    yarp::os::Bottle tempDisplayList, *bp;
    tempDisplayList.addInt(0);
    tempDisplayList.addString("/Left");
    bp = &(tempDisplayList.addList()); bp->addString("AE");
    tempDisplayList.addInt(1);
    tempDisplayList.addString("/Right");
    bp = &(tempDisplayList.addList()); bp->addString("AE");

    //set the output channels
    yarp::os::Bottle * displayList = rf.find("displays").asList();
    if(!displayList)
        displayList = &tempDisplayList;

    yInfo() << displayList->toString();

    if(displayList->size() % 3) {
        std::cerr << "Error: display incorrectly configured in provided "
                     "settings file." << std::endl;
        return false;
    }

    int nDisplays = displayList->size() / 3;

    //for each channel open a vFrame and an output port
    channels.resize(nDisplays);
    outports.resize(nDisplays);
    drawers.resize(nDisplays);

    for(int i = 0; i < nDisplays; i++) {

        //extract the channel integer value
        channels[i] = displayList->get(i*3).asInt();

        //extract the portname
        outports[i] = new yarp::os::BufferedPort<
                yarp::sig::ImageOf<yarp::sig::PixelBgr> >;
        //outports[i]->setStrict();
        std::string outportname = displayList->get(i*3 + 1).asString();
        if(!outports[i]->open(moduleName + outportname))
            return false;

        yarp::os::Bottle * drawtypelist = displayList->get(i*3 + 2).asList();
        for(int j = 0; j < drawtypelist->size(); j++) {
            vDraw * newDrawer = createDrawer(drawtypelist->get(j).asString());
            if(newDrawer) {
                newDrawer->setLimits(retinaWidth, retinaHeight);
                newDrawer->setWindow(eventWindow);
                newDrawer->setFlip(flip);
                newDrawer->initialise();
                drawers[i].push_back(newDrawer);
                if(!vReader.open(moduleName, newDrawer->getEventType()))
                    yError() << "Could not open input port";
            } else {
                yError() << "Could not find draw tag "
                          << drawtypelist->get(j).asString()
                          << ". No drawer created";
            }
        }
    }

    return true;

}

bool vFramerModule::interruptModule()
{
    for(unsigned int i = 0; i < outports.size(); i++)
        outports[i]->close();

    vReader.close();

    return true;
}

bool vFramerModule::close()
{
    return true;
}

bool vFramerModule::updateModule()
{

    yarp::os::Stamp yarptime = vReader.getystamp();

    //for each output image needed
    for(unsigned int i = 0; i < channels.size(); i++) {

        //make a new image
        cv::Mat canvas;

        for(unsigned int j = 0; j < drawers[i].size(); j++) {
            drawers[i][j]->draw(canvas,
                                vReader.queryWindow(drawers[i][j]->getEventType(), channels[i]),
                                vReader.getvstamp());
        }


        //then copy the image to the port and send it on
        yarp::sig::ImageOf<yarp::sig::PixelBgr> &o = outports[i]->prepare();
        o.resize(canvas.cols, canvas.rows);
        cv::Mat publishMat((IplImage *)o.getIplImage(), false);
        //cv::flip(canvas, canvas, 0);
        canvas.copyTo(publishMat);
        if(yarptime.isValid()) outports[i]->setEnvelope(yarptime);
        outports[i]->write();
    }
    return true;

}

double vFramerModule::getPeriod()
{
    return period;
}

vFramerModule::~vFramerModule()
{
    for(unsigned int i = 0; i < drawers.size(); i++) {
        for(unsigned int j = 0; j < drawers[i].size(); j++) {
            delete drawers[i][j];
        }
    }

    for(unsigned int i = 0; i < outports.size(); i++) {
        delete outports[i];
    }
}

