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

#include <yarp/os/Bottle.h>
#ifdef WIN32
#define EXPORT //warning, positional define. This should be define before including the vCodec.h
#endif
#include "event-driven/vCodec.h"

namespace ev {

const std::string SkinSample::tag = "SKS";

SkinSample::SkinSample() : SkinEvent(), _ts(0), value(0) {}

SkinSample::SkinSample(const vEvent &v) : SkinEvent(v), _ts(0), value(0)
{
    const SkinSample *v2 = dynamic_cast<const SkinSample *>(&v);
    if(v2) {
        _ts = v2->_ts;
        value = v2->value;
    }
}

SkinSample::SkinSample(const SkinSample &v) : SkinEvent(v)
{
    _ts = v._ts;
    value = v.value;
}

event<> SkinSample::clone()
{
    return std::make_shared<SkinSample>(*this);
}

void SkinSample::encode(yarp::os::Bottle &b) const
{
    SkinEvent::encode(b);
    b.addInt32(_ts);
    b.addInt32(value);
}

void SkinSample::encode(std::vector<int32_t> &b, unsigned int &pos) const
{
    SkinEvent::encode(b, pos);
    b[pos++] = _ts;
    b[pos++] = value;
}

void SkinSample::decode(const int32_t *&data)
{
    SkinEvent::decode(data);
    _ts = *(data++);
    value = *(data++);
}

bool SkinSample::decode(const yarp::os::Bottle &packet, size_t &pos)
{
    // check length
    if (SkinEvent::decode(packet, pos) && pos + 2 <= packet.size())
    {
        _ts = packet.get(pos++).asInt();
        value = packet.get(pos++).asInt();
        return true;
    }
    return false;
}

yarp::os::Property SkinSample::getContent() const
{
    yarp::os::Property prop = SkinEvent::getContent();
    prop.put("value", (int)value);
    return prop;
}

std::string SkinSample::getType() const
{
    return SkinSample::tag;
}

}
