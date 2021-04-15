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

#include <algorithm>
#include <yarp/os/Bottle.h>
#include "event-driven/vCodec.h"
#include "event-driven/vtsHelper.h"

namespace ev {

event<> createEvent(const std::string &type)
{

    if(type == AddressEvent::tag)
        return make_event<AE>();
    if(type == SkinEvent::tag)
        return make_event<SkinEvent>();
    if(type == SkinSample::tag)
        return make_event<SkinSample>();
    if(type == CochleaEvent::tag)
        return make_event<CochleaEvent>();
	if(type == LabelledAE::tag)
        return make_event<LabelledAE>();
    if(type == FlowEvent::tag)
        return make_event<FlowEvent>();
    if(type == GaussianAE::tag)
        return make_event<GaussianAE>();
    if(type == IMUevent::tag)
        return make_event<IMUevent>();
    return event<>(nullptr);

}

unsigned int packetSize(const std::string &type)
{
    if(type == AddressEvent::tag)
        return 2;
    if(type == vEvent::tag)
        return 1;
    if(type == SkinEvent::tag)
        return 2;
    if(type == SkinSample::tag)
        return 4;
    if(type == CochleaEvent::tag)
        return 2;
    if(type == LabelledAE::tag)
        return 3;
    if(type == FlowEvent::tag)
        return 4;
    if(type == GaussianAE::tag)
        return 6;
    if(type == IMUevent::tag)
        return 2;
    return 0;


}

bool temporalSortStraight(const event<> &e1, const event<> &e2) {
    return e2->stamp > e1->stamp;
}

bool temporalSortWrap(const event<> &e1, const event<> &e2)
{

    if((unsigned int)(std::abs((int)e1->stamp - (int)e2->stamp)) > vtsHelper::max_stamp/2)
        return e1->stamp > e2->stamp;
    else
        return e2->stamp > e1->stamp;

}

void qsort(vQueue &q, bool respectWraps)
{

    if(respectWraps)
        std::sort(q.begin(), q.end(), temporalSortWrap);
    else
        std::sort(q.begin(), q.end(), temporalSortStraight);

}

}
