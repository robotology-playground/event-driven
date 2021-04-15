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

const std::string SkinEvent::tag = "SKE";

SkinEvent::SkinEvent() : vEvent(), polarity(0), taxel(0), cross_base(0),
    _sample(0), _error(0), body_part(0), side(0), type(0), skin(1) {}

SkinEvent::SkinEvent(const vEvent &v) : vEvent(v), _skei(0)
{
    const SkinEvent *v2 = dynamic_cast<const SkinEvent *>(&v);
    if(v2) {
        _skei = v2->_skei;
    }
}

SkinEvent::SkinEvent(const SkinEvent &v) : vEvent(v)
{
    _skei = v._skei;
}

event<> SkinEvent::clone()
{
    return std::make_shared<SkinEvent>(*this);
}

void SkinEvent::encode(yarp::os::Bottle &b) const
{
    vEvent::encode(b);
    b.addInt32(_skei);
}

void SkinEvent::encode(std::vector<int32_t> &b, unsigned int &pos) const
{
    vEvent::encode(b, pos);
    b[pos++] = _skei;
}

void SkinEvent::decode(const int32_t *&data)
{
    vEvent::decode(data);
    _skei = *(data++);
}

bool SkinEvent::decode(const yarp::os::Bottle &packet, size_t &pos)
{
    // check length
    if (vEvent::decode(packet, pos) && pos + 1 <= packet.size())
    {
        _skei = packet.get(pos++).asInt();
        return true;
    }
    return false;
}

yarp::os::Property SkinEvent::getContent() const
{
    yarp::os::Property prop = vEvent::getContent();
    prop.put("polarity", (int)polarity);
    prop.put("type", (int)type);
    prop.put("skin", (int)skin);
    prop.put("taxel", (int)taxel);
    prop.put("cross_base", (int)cross_base);
    prop.put("body_part", (int)body_part);
    prop.put("side", (int)side);

    return prop;
}

std::string SkinEvent::getType() const
{
    return SkinEvent::tag;
}

}
