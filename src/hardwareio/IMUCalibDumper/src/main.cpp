/******************************************************************************
                                                                              *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
                                                                              *
                                                                              */

/**
 * @file: IMUCalibDumper/src/main.cpp
 * @authors: Leandro de Souza Rosa <leandro.desouzarosa@iit.it>
*/

#include "IMUCalibDumper.h"

using namespace yarp::os;

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(2)) {
        std::cout << "Could not connect to YARP" << std::endl;
        return -1;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( false );
    rf.setDefaultContext( "event-driven" );
    rf.setDefaultConfigFile( "IMUCalibDumper.ini" );
    rf.configure( argc, argv );

    /* create the module */
    IMUCalibDumper instance;
    return instance.runModule(rf);
}
