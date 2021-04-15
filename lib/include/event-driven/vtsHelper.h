/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           valentina.vasco@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __VTSHELPER__
#define __VTSHELPER__

#include <yarp/os/all.h>
#include <event-driven/api.h>
#include <fstream>
#include <math.h>
#include <vector>

namespace ev {

/// \brief helper class to deal with timestamp conversion and wrapping
class EV_API vtsHelper {

private:

    int last_stamp;
    unsigned int n_wraps;

public:

    /// the maximum value of the timestamp before a wrap occurs
    static unsigned int max_stamp;
    /// a multiplier to convert an event timestamp to seconds
    static double tsscaler;
    /// a multiplier to convert seconds to an event timestamp
    static double vtsscaler;

    /// \brief constructor
    vtsHelper(): last_stamp(0), n_wraps(0) {}

    /// \brief unwrap a timestamp, given previously unwrapped timestamps
    unsigned long int operator() (int timestamp) {
        if(last_stamp > timestamp)
            n_wraps++;
        last_stamp = timestamp;
        return currentTime();
    }

    /// \brief DEPRECATED - access to max_stamp member variable is public
    static long int maxStamp() { return max_stamp; }
    /// \brief DEPRECATED - access to timestamp conversion member variables is
    /// public
    static double tstosecs() { return tsscaler; }
    /// \brief ask for the current unwrapped time, without updating the time.
    unsigned long int currentTime() { return last_stamp + ((unsigned long int)max_stamp*n_wraps); }

    static int deltaTicks(const int current_tick, const int prev_tick)
    {
        int dt = current_tick - prev_tick;
        if(dt < 0) dt += max_stamp;
        return dt;
    }

    static double deltaS(const int current_tick, const int prev_tick)
    {
        return deltaTicks(current_tick, prev_tick) * tsscaler;
    }

    static double deltaMS(const int current_tick, const int prev_tick)
    {
        return deltaS(current_tick, prev_tick) * 1000.0;
    }

};

class benchmark {
private:

    bool initialised;
    std::ifstream totals_file;
    std::ifstream process_file;

    unsigned long int prevTotal;
    unsigned long int prevProcess;
    double perc;

public:

    benchmark();
    ~benchmark();
    bool isReady();
    double getProcessorUsage();

};

/// \brief an efficient structure for storing sensor resolution
struct resolution {
    unsigned int width:10;
    unsigned int height:10;
};

class imuHelper {
private:
    std::vector<double> bias;
    std::vector<double> gain;
public:

    const static unsigned int _max_value = 32768;

    enum
    {
        ACC_Y = 0, ACC_X = 1, ACC_Z = 2,
        GYR_Y = 3, GYR_X = 4, GYR_Z = 5,
        TEMP  = 6,
        MAG_Y = 7, MAG_X = 8, MAG_Z = 9
    };

    imuHelper()
    {
        bias = {0.0, 0.0, 0.0};
        gain = {9.80665/16384.0, 9.80665/16384.0, 9.80665/16384.0};
    }

    bool configure(const yarp::os::Bottle* config)
    {
        if(!config) {
            yWarning() << "No IMU config given";
            return false;
        } else if(config->size() != 6) {
            yWarning() << "IMU config (6) incorrect size = " << config->size();
            return false;
        }
        bias[ACC_X] = config->get(0).asDouble();
        gain[ACC_X] = config->get(1).asDouble();
        bias[ACC_Y] = config->get(2).asDouble();
        gain[ACC_Y] = config->get(3).asDouble();
        bias[ACC_Z] = config->get(4).asDouble();
        gain[ACC_Z] = config->get(5).asDouble();

        return true;
    }

    double convertToSI(const int value, const int index) const
    {
//        static constexpr double ax_bias = -344.536150;
//        static constexpr double ay_bias = -207.041644;
//        static constexpr double az_bias = -127.343397;

//        static constexpr double ax_gain = 0.000623;
//        static constexpr double ay_gain = 0.000613;
//        static constexpr double az_gain = 0.000599;


        switch(index) {
        case(ACC_X):
        case(ACC_Z):
            return  (value + bias[index]) * gain[index];
        case(ACC_Y):
            return -(value + bias[index]) * gain[index];
        case(GYR_X):
        case(GYR_Z):
            return value * (250.0 * M_PI / (2.0 * 180.0 * 16384.0));
        case(GYR_Y):
            return -(value * (250.0 * M_PI / (2.0 * 180.0 * 16384.0)));
        case(TEMP):
            return (value / 333.87) + (273.15 + 21.0);
        case(MAG_X):
        case(MAG_Z):
            return value * (/*10e6 **/ 4900.0 / 16384.0);
        case(MAG_Y):
            return -(value * (/*10e6 **/ 4900.0 / 16384.0));
        }

        return -1;
    }

};

class CochleaEvent;
class cochleaHelper {
    private:

    public:
        // NAS fixed values
        static const int nas_num_frequency_channels = 32;
        static const int nas_mono_stereo = 2;
        static const int nas_polarity_type = 2;
        static const int nas_addrresses_offset = nas_num_frequency_channels * nas_mono_stereo * nas_polarity_type; 
        // [0, 127] --> 128 addresses for 32-ch stereo NAS with meged polarity

        // MSO fixed values
        static const int mso_start_freq_channel = 13;
        static const int mso_end_freq_channel = 16;
        static const int mso_num_freq_channels = mso_end_freq_channel - mso_start_freq_channel + 1;
        static const int mso_num_neurons_per_channel = 16;
        static const int mso_addresses_offset = (mso_end_freq_channel - mso_start_freq_channel + 1) * mso_num_neurons_per_channel;

        // LSO fixed values
        static const int lso_start_freq_channel = 5;
        static const int lso_end_freq_channel = 8;
        static const int lso_num_neurons_per_channel = 1;
        static const int lso_addresses_offset = (lso_end_freq_channel - lso_start_freq_channel + 1) * lso_num_neurons_per_channel;

        // CochleaEvent values
        static const int max_num_addresses = nas_addrresses_offset + mso_addresses_offset + lso_addresses_offset;

        static int getAddress(const CochleaEvent &ce);
};

}

#endif //__VTSHELPER
