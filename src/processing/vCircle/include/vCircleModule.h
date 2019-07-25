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

// \defgroup Modules Modules
// \defgroup vCircle vCircle
// \ingroup Modules
// \brief detects circles using the Hough transform

#ifndef __VCIRCLE__
#define __VCIRCLE__

#include <fstream>
#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>

#include "vCircleObserver.h"

/*////////////////////////////////////////////////////////////////////////////*/
//VCIRCLEREADER
/*////////////////////////////////////////////////////////////////////////////*/
class vCircleReader : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<ev::vBottle> outPort;
    yarp::os::BufferedPort<yarp::os::Bottle> scopeOut;
    yarp::os::BufferedPort<yarp::sig::ImageOf <yarp::sig::PixelBgr> > houghOut;
    yarp::os::BufferedPort<yarp::os::Bottle> dumpOut;

    ev::vtsHelper unwrap;
    double pTS;

    bool strictness;
    yarp::os::Stamp pstamp;
    int pstampcounter;

    bool singleq;
    double tsoffset;


public:

    vCircleMultiSize * cObserverL;
    vCircleMultiSize * cObserverR;
    double inlierThreshold;
    bool hough;
    double timecounter;

    vCircleReader();

    void setSingleQ(bool singleq = true) { this->singleq = singleq; }

    bool    open(const std::string &name, bool strictness = false);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(ev::vBottle &inBot);

};

/*////////////////////////////////////////////////////////////////////////////*/
//VCIRCLEMODULE
/*////////////////////////////////////////////////////////////////////////////*/
class vCircleModule : public yarp::os::RFModule
{
    //the event bottle input and output handler
    vCircleReader      circleReader;

public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

};


#endif
