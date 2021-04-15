/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
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
#ifdef WIN32
#define EXPORT //warning, positional define. This should be define before including the vCodec.h
#endif
#include "event-driven/vCodec.h"

namespace ev {

const std::string IMUevent::tag = "IMUS";

IMUevent::IMUevent() : vEvent(), _coded_data(0) {}

IMUevent::IMUevent(const vEvent &v) : vEvent(v), _coded_data(0)
{
    const IMUevent *v2 = dynamic_cast<const IMUevent *>(&v);
    if(v2) {
        _coded_data = v2->_coded_data;
    }
}

IMUevent::IMUevent(const IMUevent &v) : vEvent(v)
{
    _coded_data = v._coded_data;
}

event<> IMUevent::clone()
{
    return std::make_shared<IMUevent>(*this);
}

void IMUevent::encode(yarp::os::Bottle &b) const
{
    vEvent::encode(b);
    b.addInt32(_coded_data);
}

void IMUevent::encode(std::vector<int32_t> &b, unsigned int &pos) const
{
    vEvent::encode(b, pos);
    b[pos++] = _coded_data;
}

void IMUevent::decode(const int32_t *&data)
{
    vEvent::decode(data);
    _coded_data = *(data++);
}

bool IMUevent::decode(const yarp::os::Bottle &packet, size_t &pos)
{
    // check length
    if (vEvent::decode(packet, pos) && pos + 1 <= packet.size())
    {
        _coded_data = packet.get(pos++).asInt();
        return true;
    }
    return false;
}

yarp::os::Property IMUevent::getContent() const
{
    yarp::os::Property prop = vEvent::getContent();
    prop.put("value", (int)value);
    prop.put("sensor", (int)sensor);
    prop.put("channel", (int)channel);
    prop.put("type", (int)type);
    return prop;
}

std::string IMUevent::getType() const
{
    return IMUevent::tag;
}

}
