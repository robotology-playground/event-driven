#include "event-driven/imuPort.h"

bool imuPort::configure(yarp::os::ResourceFinder& rf, string name)
{
    
    if(!imu_port.open(name + "/IMU:i")) {
        yError() << "Could not open input port";
        return false;
    }
    
    calibrate = rf.check("calibrate", Value(false)).asBool();

    double g_mag = rf.check("g_mag", Value(9.805622)).asDouble();
    double conversion_rate = rf.check("g_mag", Value(16384.0)).asDouble();
    double gyr_rate = rf.check("gyr_rate", Value(250.0)).asDouble();
    
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


    if(calibrate)
    {
        std::string home = std::string(std::getenv("HOME"));
        std::string acc_file_name = rf.check("acc_calib_params", yarp::os::Value(home+std::string("/acc.calib"))).asString();
        std::string gyr_file_name = rf.check("gyr_calib_params", yarp::os::Value(home+std::string("/gyr.calib"))).asString();
        
        std::ifstream acc_calib_params(acc_file_name);
        std::ifstream gyr_calib_params(gyr_file_name);
        if(!acc_calib_params.is_open() || !gyr_calib_params.is_open())
        {
            yInfo() << "Files with calibration parameters not found. Specify with '--acc_calib_params <file>' and 'gyr_calib_params <file>'";
            return false;
        }

        for(auto v = acc_missalign.begin<double>(); v!= acc_missalign.end<double>(); ++v)
        {
            acc_calib_params >> *v;
        }
        //std::cout << "acc_missalign: \n" << acc_missalign << "\n";
       
        for(auto v = gyr_missalign.begin<double>(); v!= gyr_missalign.end<double>(); ++v)
        {
            gyr_calib_params >> *v;
        }
        //std::cout << "gyr_missalign: \n" << gyr_missalign << "\n";

        for(auto v = acc_scale.begin<double>(); v!= acc_scale.end<double>(); ++v)
        {
            acc_calib_params >> *v;
        }
        //std::cout << "acc_scale: \n" << acc_scale << "\n";

        for(auto v = gyr_scale.begin<double>(); v!= gyr_scale.end<double>(); ++v)
        {
            gyr_calib_params >> *v;
        }
        //std::cout << "gyr_scale: \n" << gyr_scale << "\n";
    
        for(auto v = acc_bias.begin<double>(); v!= acc_bias.end<double>(); ++v)
        {
            acc_calib_params >> *v;
        }
        //std::cout << "acc_bias: \n" << acc_bias << "\n";
                                                                                  
        for(auto v = gyr_bias.begin<double>(); v!= gyr_bias.end<double>(); ++v)
        {
            gyr_calib_params >> *v;
        }
        //std::cout << "gyr_bias: \n" << gyr_bias << "\n";
    }
    else // just convert to SI
    {
        acc_scale = acc_scale * g_mag/conversion_rate;
        gyr_scale = gyr_scale * (gyr_rate * M_PI / (2.0 * 180.0 * conversion_rate));
    }
    
    yInfo() << "IMU Port configured";
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
            imu_buffer.at<double>(v.sensor) = v.value;
            
            std::cout << "\nev: " << v.sensor << " - " << v.value;
            if(v.sensor == 9)
            {
                std::cout << "\n\nimu before: " << imu_buffer;
                //perform the calibration
                imu_buffer(cv::Rect(0, 0, 1, 3)) = acc_missalign*acc_scale*(imu_buffer(cv::Rect(0, 0, 1, 3)) + acc_bias);
                imu_buffer(cv::Rect(0, 3, 1, 3)) = gyr_missalign*gyr_scale*(imu_buffer(cv::Rect(0, 3 , 1, 3)) + gyr_bias);
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

