/*******************************************************************************
                                                                               *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)         *
 * All Rights Reserved.                                                        *
                                                                               *
                                                                              */

/**
 * @file: IMUCalibDumper/src/IMUCalibDumper.cpp
 * @authors:    Leandro de Souza Rosa <leandro.desouzarosa@iit.it>
 */

#include "IMUCalibDumper.h"

bool IMUCalibDumper::configure(yarp::os::ResourceFinder& rf)
{
    //set the module name used to name ports
    setName((rf.check("name", Value("/IMUCalibDumper")).asString()).c_str());

    //open io ports
    if(!imu_port.open(getName() + "/IMU:i")) {
        yError() << "Could not open input port";
        return false;
    }

    //imu calibration
    Bottle *imucalibration = rf.find("imu_calib").asList();
    if(imu_helper.configure(imucalibration)) {
        yInfo() << "IMU calibration:" << imucalibration->toString();
    }

    //inilialize the vector which saves the values
    imu_state = std::vector<double>(10, 0);

    accFileName = static_cast<std::string>(rf.check("accFileName", Value(std::string(std::getenv("HOME"))+"/results/IMUDataDump/acc.mat")).asString());
    gyrFileName = static_cast<std::string>(rf.check("gyrFileName", Value(std::string(std::getenv("HOME"))+"/results/IMUDataDump/gyr.mat")).asString());
    accFilePath = accFileName;
    gyrFilePath = gyrFileName;
    
    //start the asynchronous and synchronous threads
    return Thread::start();
}

double IMUCalibDumper::getPeriod()
{
    return 0.1; //period of synchrnous thread
}

bool IMUCalibDumper::interruptModule()
{
    //if the module is asked to stop ask the asynchrnous thread to stop
    return Thread::stop();
}

void IMUCalibDumper::onStop()
{
    // Create files and save the data
    if(!fs::exists(accFilePath.parent_path()))
    {
        yInfo() << "creating log file folder";
        fs::create_directories(accFilePath.parent_path());
    }
    std::ofstream acclog, gyrlog;
    acclog.open(accFileName, std::ofstream::out | std::ofstream::trunc);
    gyrlog.open(gyrFileName, std::ofstream::out | std::ofstream::trunc);
    acclog << std::scientific;
    gyrlog << std::scientific; 
    // dump acc 
    for(auto vec : accData)
    {
       for(auto entry : vec)
       {
           acclog << "\t" << entry;
       }
       acclog << std::endl;
    }
    acclog.close();

    // dump gyro
    for(auto vec : gyrData)
    {
       for(auto entry : vec)
       {
           gyrlog << "\t" << entry;
       }
       gyrlog << std::endl;
    }
    gyrlog.close();
    
    // clean up
    imu_port.close();
}

//synchronous thread
bool IMUCalibDumper::updateModule()
{
    return Thread::isRunning();
}

//asynchronous thread run forever
void IMUCalibDumper::run()
{
    Stamp yarpstamp_imu;
    DEF_SENSORS(); // SENSORS keeps track of which sensors are set
    double t0;
    bool firstTime = true;

    while(true) {
        const vector<IMUevent> * q_imu = imu_port.read(yarpstamp_imu);
        if(!q_imu || Thread::isStopping()) return;


        for( ev::IMUevent e_imu : *q_imu)
        {
            SET_SENSOR(e_imu.sensor); // set sensor e_imu.sensor as received
            imu_state[e_imu.sensor] =  e_imu.value;
                
            // continue receiving data until the 10th reading (index 9)
            if(e_imu.sensor != 9) continue;

            // If the 10 sensors were not received, restarts gathering the data
            if(SENSORS != ALL_SENSORS)
            {
                RESET_SENSORS();
                continue;
            }

            //resets for the next iteration
            RESET_SENSORS();
            
            vtsHelper(e_imu.stamp);
            double currTime = static_cast<double>(vtsHelper.currentTime()*vtsHelper.tsscaler);
            if(firstTime)
            {
                t0 = currTime;
                firstTime = false;
            }

            std::vector<double> dacc, dgyr;

            dacc.push_back(currTime-t0);
            dacc.push_back(imu_state[0]);
            dacc.push_back(imu_state[1]);
            dacc.push_back(imu_state[2]);

            // The minus sign on Y is to transform in the camera position
            // Following imuHelper in vtsHelper.h
            std::cout << "acc reding: ";
            for(auto e : dacc)
               std::cout << e << ", ";
            std::cout << "\n"; 

            dgyr.push_back(currTime-t0);
            dgyr.push_back(imu_state[3]);
            dgyr.push_back(imu_state[4]);
            dgyr.push_back(imu_state[5]);
            
            accData.push_back(dacc);
            gyrData.push_back(dgyr);
            
        }
    }
}       
