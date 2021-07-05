/******************************************************************************
                                                                              *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)		  *
 * All Rights Reserved.														  *
                                                                              *
                                                                              */

/**
 * @file: IMUCalibDumper/include/IMUcalibDumper.h
 * @authors:	Leandro de Souza Rosa <leandro.desouzarosa@iit.it>
 */

#ifndef __IMU_CALIB_DUMP_H
#define __IMU_CALIB_DUMP_H

#include <yarp/sig/Vector.h>
#include <yarp/os/all.h>
#include <event-driven/all.h>

// These macros serve to anotate which sensors' data were received.
// If the i^th sensor is received, it adds 1 to the i^th bit of the variable SENSORS
#define SENSORS sensors
#define DEF_SENSORS() unsigned int SENSORS = 0
#define SET_SENSOR(x) SENSORS += (1 << x)
#define ALL_SENSORS 1023 // If the 10 sensors are set, the value is 1023
#define RESET_SENSORS() SENSORS = 0

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

using namespace ev;
using namespace yarp::os;

class IMUCalibDumper : public RFModule, public Thread {

private:
    vReadPort< vector<IMUevent>  > imu_port; //!< input port to receive data from the IMU

    std::vector<double> imu_state; //!< saves the IMU values in order

    imuHelper imu_helper;

    ev::vtsHelper vtsHelper; // common for all LOGs
    std::string accFileName, gyrFileName;
    fs::path accFilePath, gyrFilePath;
    std::vector<std::vector<double>> accData;
    std::vector<std::vector<double>> gyrData;
public:

    /*!
     * Open and configure all the resources
     *
     * \param rf contains the command-line options
     *
     * \return bool true/false if success/fail
     */
    virtual bool configure(yarp::os::ResourceFinder& rf);

    /*!
     *	Establish the period of the background service thread
     *
     * \return double the period given in seconds
     */
    virtual double getPeriod();

    /*!
     * Stops the thread associated with the IMUCalibDumper module
     *
     * \return bool true/false if success/fail
     */
    bool interruptModule();

    /*!
     * Close the resources
     */
    void onStop();

    /*!
     * Background service thread (synchronous)
     *
     * \return bool true/false if success/fail
     */
    virtual bool updateModule();

    /*!
     * Main functionality of the module. Reads the IMU events stream, gathers the IMU reading from the different sensors and outputs them together in a single vector
     */
    void run();
};

#endif
//empty line to make gcc happy
