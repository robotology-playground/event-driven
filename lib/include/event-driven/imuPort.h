#ifndef __IMU_PORT_H__
#define __IMU_PORT_H__

#include "event-driven/vPort.h"
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <opencv2/opencv.hpp>

using std::string;
using std::array;
using namespace ev;
using namespace yarp::os;
using namespace yarp::sig;

class imuPort : public Thread {
    private: 
        vReadPort< vector<IMUevent>  > imu_port;
        array<double, 10> imu_state;
        imuHelper imu_helper;

        cv::Mat calib_acc, calib_gyr;
        double g_mag;        

        cv::Mat acc_missalign, gyr_missalign;
        cv::Mat acc_scale, acc_bias, gyr_scale, gyr_bias;
    public:
   
    enum
    {
        ACC_Y = 0, ACC_X = 1, ACC_Z = 2,
        GYR_Y = 3, GYR_X = 4, GYR_Z = 5,
        TEMP  = 6,
        MAG_Y = 7, MAG_X = 8, MAG_Z = 9
    };


    virtual bool configure(yarp::os::ResourceFinder& rf, string name);
    void onStop();
    void run();

    void getGyr(array<double,3> &gyr);
    void getAcc(array<double,3> &acc);
};

#endif
// empty line for gcc
