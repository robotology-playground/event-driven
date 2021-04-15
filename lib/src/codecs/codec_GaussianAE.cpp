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
#include "event-driven/vtsHelper.h"

namespace ev {

const std::string GaussianAE::tag = "GAE";

GaussianAE::GaussianAE() : LabelledAE(), sigx(0), sigy(0), sigxy(0) {}

GaussianAE::GaussianAE(const vEvent &v) : LabelledAE(v), sigx(0), sigy(0), sigxy(0)
{
    const GaussianAE * v2 = dynamic_cast<const GaussianAE *>(&v);
    if(v2) {
        sigx = v2->sigx;
        sigy = v2->sigy;
        sigxy = v2->sigxy;
    }
}

GaussianAE::GaussianAE(const GaussianAE &v) : LabelledAE(v)
{
    sigx = v.sigx;
    sigy = v.sigy;
    sigxy = v.sigxy;
}

event<> GaussianAE::clone()
{
    return std::make_shared<GaussianAE>(*this);
}

void GaussianAE::encode(yarp::os::Bottle &b) const
{
    LabelledAE::encode(b);
    b.addInt32(_gaei[0]);
    b.addInt32(_gaei[1]);
    b.addInt32(_gaei[2]);
}

void GaussianAE::encode(std::vector<int32_t> &b, unsigned int &pos) const
{
    LabelledAE::encode(b, pos);
    b[pos++] = _gaei[0];
    b[pos++] = _gaei[1];
    b[pos++] = _gaei[2];
}

void GaussianAE::decode(const int32_t *&data)
{
    LabelledAE::decode(data);
    _gaei[0] = *(data++);
    _gaei[1] = *(data++);
    _gaei[2] = *(data++);
}

bool GaussianAE::decode(const yarp::os::Bottle &packet, size_t &pos)
{
    if (LabelledAE::decode(packet, pos) && pos + 3 <= packet.size())
    {

        _gaei[0] = packet.get(pos++).asInt();
        _gaei[1] = packet.get(pos++).asInt();
        _gaei[2] = packet.get(pos++).asInt();
        return true;
    }
    return false;
}

yarp::os::Property GaussianAE::getContent() const
{
    yarp::os::Property prop = LabelledAE::getContent();
    prop.put("Sigma X^2", sigx);
    prop.put("Sigma Y^2", sigy);
    prop.put("Sigma XY", sigxy);
    return prop;
}

std::string GaussianAE::getType() const
{
    return GaussianAE::tag;
}


}
