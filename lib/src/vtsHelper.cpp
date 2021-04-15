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

#ifndef WIN32 
#include <unistd.h>
#else
#define EXPORT//warning, positional define. This should be define before including the vCodec.h
#endif

#include "event-driven/vtsHelper.h"
#include "event-driven/vCodec.h"

#include <sstream>

#include <limits>


namespace ev {

unsigned int vtsHelper::max_stamp = (1 << TIMER_BITS) - 1;
double vtsHelper::tsscaler = 0.000000001 * CLOCK_PERIOD;
double vtsHelper::vtsscaler = 1.0 / vtsHelper::tsscaler;

benchmark::benchmark()
{
    initialised = true;
    totals_file.open("/proc/stat");
    if(!totals_file.is_open()) {
        yWarning() << "Could not open global cpu statistics (linux only)";
        initialised = false;
    }

    std::stringstream path_proc;
    path_proc << "/proc/" << getpid() << "/stat";
    process_file.open(path_proc.str());
    if(!process_file.is_open()) {
        yWarning() << "Could not open process cpu statistics (linux only)";
        initialised = false;
    }

    if(!initialised) {
        if(totals_file.is_open())
            totals_file.close();
        if(process_file.is_open())
            process_file.close();
    }

    prevTotal = 0;
    prevProcess = 0;
    perc = 0.0;
}

bool benchmark::isReady()
{
    return initialised;
}

double benchmark::getProcessorUsage()
{
    unsigned long int temp;
    unsigned long int total = 0;
    unsigned long int process = 0;
    std::string line;
    std::stringstream lineparser;

    if(!initialised) return 0;

    //get the cpu totals

    //get the relevant line
    totals_file.seekg(0, totals_file.beg);
    getline (totals_file, line);
    lineparser.clear(); lineparser.str(line);
    //yInfo() << lineparser.str();

    //ignore unnecessary data
    lineparser.ignore(std::numeric_limits<std::streamsize>::max(), ' ');

    //extract the usage
    while(lineparser >> temp)
        total += temp;

    if(total == prevTotal) return perc;

    //get the process totals

    //get the relevant line
    process_file.seekg(0, process_file.beg);
    getline(process_file, line);
    lineparser.clear(); lineparser.str(line);
    //yInfo() << lineparser.str();

    //ignore unnecessary data
    for(int i = 0; i < 13; i++) {
        lineparser.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
    }

    //extract the usage
    if(lineparser >> temp) {
        process += temp;
    }
    if(lineparser >> temp) {
        process += temp;
    }

    //calculate the percentage usage since the last call
    perc = (double)(process - prevProcess) / (double)(total - prevTotal);
    prevProcess = process; prevTotal = total;

    //yInfo() << process << total << perc;

    return perc;
}

benchmark::~benchmark()
{
    if(totals_file.is_open())
        totals_file.close();
    if(process_file.is_open())
        process_file.close();
}


int cochleaHelper::getAddress(const CochleaEvent &ce)
{
    int address = 0;

    // Check if the event is coming from either the NAS or the SOC
    if ((int)ce.auditory_model == 0)
    {
        // It is a NAS event

        // Take the frequency channel id
        address = (int)ce.freq_chnn;
        // Shift left 1 position and add the polarity
        address = (address << 1) + (int)ce.polarity;
        // Add an offset if it is from the right channel
        address = address + ((int)ce.channel << 6);
    }
    else if ((int)ce.auditory_model == 1)
    {
        // It is a SOC event

        // In this case, always add the NAS max address value
        address = address + cochleaHelper::nas_addrresses_offset;

        if ((int)ce.xso_type == 0)
        {
            // It is a MSO event

            // Calculate the neuron address
            address = address + (int)ce.neuron_id + (((int)ce.freq_chnn - cochleaHelper::mso_start_freq_channel) * cochleaHelper::mso_num_neurons_per_channel);
        }
        else if ((int)ce.xso_type == 1)
        {
            // It is a LSO event

            // Add the MSO offset
            address = address + cochleaHelper::mso_addresses_offset;

            // Calculate the neuron address
            address = address + (int)ce.neuron_id + (((int)ce.freq_chnn - cochleaHelper::lso_start_freq_channel) * cochleaHelper::lso_num_neurons_per_channel);
        }
        else
        {
            // XSO type not recognized
            address = -1;
        }
    }
    else
    {
        // Auditory model not recognized
        address = -1;
    }

    return address;
}

}
