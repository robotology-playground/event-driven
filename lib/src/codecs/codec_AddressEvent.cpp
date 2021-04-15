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

#if defined CODEC_128x128
union {
    uint32_t _coded_data;
    struct {
        unsigned int polarity:1;
        unsigned int x:7;
        unsigned int y:7;
        unsigned int channel:1;
    };
} _bitorder128;
#elif defined CODEC_304x240_20
union {
    uint32_t _coded_data;
    struct {
        unsigned int polarity:1;
        unsigned int x:9;
        unsigned int y:8;
        unsigned int type:2;
        unsigned int channel:1;
        unsigned int skin:1;
    };
} _bitorder20;
#endif

namespace ev {

const std::string AddressEvent::tag = "AE";

AddressEvent::AddressEvent() : vEvent(), polarity(0), x(0), _xfill(0), y(0),
    corner(0), channel(0), type(0), skin(0), _fill(0) {}

AddressEvent::AddressEvent(const vEvent &v) : vEvent(v), _coded_data(0)
{
    const AddressEvent *v2 = dynamic_cast<const AddressEvent *>(&v);
    if(v2) {
        _coded_data = v2->_coded_data;
    }
}

AddressEvent::AddressEvent(const AddressEvent &v) : vEvent(v)
{
    _coded_data = v._coded_data;
}

event<> AddressEvent::clone()
{
    return std::make_shared<AddressEvent>(*this);
}

void AddressEvent::encode(yarp::os::Bottle &b) const
{
    vEvent::encode(b);
#if defined CODEC_128x128
    _bitorder128.polarity = polarity;
    _bitorder128.x = x;
    _bitorder128.y = y;
    _bitorder128.channel = channel;
    b.addInt32(_bitorder128._coded_data);
#elif defined CODEC_304x240_20
    _bitorder20.polarity = polarity;
    _bitorder20.x = x;
    _bitorder20.y = y;
    _bitorder20.type = type;
    _bitorder20.channel = channel;
    _bitorder20.skin = skin;
    b.addInt32(_bitorder20._coded_data);
#else
    b.addInt32(_coded_data);
#endif

}

void AddressEvent::encode(std::vector<int32_t> &b, unsigned int &pos) const
{
    vEvent::encode(b, pos);
#if defined CODEC_128x128
    _bitorder128.polarity = polarity;
    _bitorder128.x = x;
    _bitorder128.y = y;
    _bitorder128.channel = channel;
    b[pos++] = _bitorder128._coded_data;
#elif defined CODEC_304x240_20
    _bitorder20.polarity = polarity;
    _bitorder20.x = x;
    _bitorder20.y = y;
    _bitorder20.type = type;
    _bitorder20.channel = channel;
    _bitorder20.skin = skin;
    b[pos++] = _bitorder20._coded_data;
#else
    b[pos++] = _coded_data;
#endif

}

void AddressEvent::decode(const int32_t *&data)
{
    vEvent::decode(data);
#if defined CODEC_128x128
    _bitorder128._coded_data = *(data++);
    polarity = _bitorder128.polarity;
    x = _bitorder128.x;
    y = _bitorder128.y;
    channel = _bitorder128.channel;
#elif defined CODEC_304x240_20
    _bitorder20._coded_data = *(data++);
    polarity = _bitorder20.polarity;
    x = _bitorder20.x;
    y = _bitorder20.y;
    type = _bitorder20.type;
    channel  = _bitorder20.channel;
    skin = _bitorder20.skin;
#else
    _coded_data = *(data++);
#endif
}

bool AddressEvent::decode(const yarp::os::Bottle &packet, size_t &pos)
{
    // check length
    if (vEvent::decode(packet, pos) && pos + 1 <= packet.size())
    {
#if defined CODEC_128x128
        _bitorder128._coded_data = packet.get(pos++).asInt();
        polarity = _bitorder128.polarity;
        x = _bitorder128.x;
        y = _bitorder128.y;
        channel = _bitorder128.channel;
#elif defined CODEC_304x240_20
        _bitorder20._coded_data = packet.get(pos++).asInt();
        polarity = _bitorder20.polarity;
        x = _bitorder20.x;
        y = _bitorder20.y;
        type = _bitorder20.type;
        channel  = _bitorder20.channel;
        skin = _bitorder20.skin;
#else
        _coded_data = packet.get(pos++).asInt();
#endif
        return true;
    }
    return false;
}

yarp::os::Property AddressEvent::getContent() const
{
    yarp::os::Property prop = vEvent::getContent();
    prop.put("channel", (int)channel);
    prop.put("polarity", (int)polarity);
    prop.put("type", (int)type);
    prop.put("skin", (int)skin);
    prop.put("x", (int)x);
    prop.put("y", (int)y);

    return prop;
}

std::string AddressEvent::getType() const
{
    return AddressEvent::tag;
}

int AddressEvent::getChannel() const
{
    return (int)channel;
}

void AddressEvent::setChannel(const int channel)
{
    this->channel=channel;
}

}

