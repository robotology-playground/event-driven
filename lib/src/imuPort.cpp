#include "event-driven/imuPort.h"

bool imuPort::configure(yarp::os::ResourceFinder& rf, string name)
{
    
    if(!imu_port.open(name + "/IMU:i")) {
        yError() << "Could not open input port";
        return false;
    }
    yInfo() << "IMU reader configured";

    // Calibration Parameters
    acc_missalign = (cv::Mat_<double>(3, 3) <<  1, 0, 0,
                                                0, 1, 0,
                                                0, 0, 1);

    gyr_missalign = (cv::Mat_<double>(3, 3) <<  1, 0, 0,
                                                0, 1, 0,
                                                0, 0, 1);

    acc_scale = (cv::Mat_<double>(3, 3) <<  1, 0, 0,
                                            0, 1, 0,
                                            0, 0, 1);
                                                           
    gyr_scale = (cv::Mat_<double>(3, 3) <<  1, 0, 0,
                                            0, 1, 0,
                                            0, 0, 1);

    acc_bias = (cv::Mat_<double>(3,1) << 0, 0, 0);
    
    gyr_bias = (cv::Mat_<double>(3,1) << 0, 0, 0);

    return true;
}

void imuPort::onStop()
{
    imu_port.close();
}


void imuPort::getGyr(array<double,3> &gyr)
{
    gyr[0] = imu_state[GYR_X];
    gyr[1] = imu_state[GYR_Y];
    gyr[2] = imu_state[GYR_Z];

    return;
}

void imuPort::getAcc(array<double,3> &acc)
{
    acc[0] = imu_state[ACC_X];
    acc[1] = imu_state[ACC_Y];
    acc[2] = imu_state[ACC_Z];

    return;
}                  

void imuPort::run()
{
    Stamp yarpstamp_imu;
    int timestamp = -1, p_timestamp = -1;
    cv::Mat imu_buffer = cv::Mat(10,1, CV_64F, cv::Scalar(0));
    
    while(true) {
        const vector<IMUevent> * q_imu = imu_port.read(yarpstamp_imu);
        if(!q_imu || Thread::isStopping()) return;

        for(auto &v : *q_imu)
        {
            imu_buffer.at<double>(v.sensor) = imu_helper.convertToSI(v.value, v.sensor);
            if(v.sensor == 9)
            {
                std::cout << "\n\nimu before: " << imu_buffer;
                //perform the calibration
                imu_buffer(cv::Range(0,2), cv::Range::all()) = acc_missalign*acc_scale*(imu_buffer(cv::Range(0,2), cv::Range::all()) + acc_bias);
                imu_buffer(cv::Range(3,5), cv::Range::all()) = gyr_missalign*gyr_scale*(imu_buffer(cv::Range(3,5), cv::Range::all()) + gyr_bias);
                std::cout << "\nimu  after: " << imu_buffer;
                //yInfo() << "flushing";
                for(int i=0; i < 10; i++)
                {
                    imu_state[i] = imu_buffer.at<double>(i);
                }
            }
            
            if(p_timestamp < 0) p_timestamp = q_imu->front().stamp;
            timestamp = q_imu->back().stamp;
        }

        p_timestamp = timestamp;

    }
    
    //is it stops
    imu_port.close();
}

