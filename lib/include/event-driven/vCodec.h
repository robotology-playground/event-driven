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

#ifndef __VCODEC__
#define __VCODEC__

#include "event-driven/vtsHelper.h"
#include <event-driven/api.h>
#include <memory>
#include <deque>
#include <math.h>
#include <vector>
#include <iostream>

namespace ev {

#define IS_SKIN(x)      x&0x01000000
#define IS_SAMPLE(x)    x&0x00804000
#define IS_SSA(x)       x&0x00004000
#define IS_SSV(x)       x&0x00800000
#define IS_IMUSAMPLE(x) x&0x02000000
#define IS_AUDIO(x)     x&0x04000000

//macros
class vEvent;

/// \brief an "event" is wrapper for the shared_ptr class
template<typename V = vEvent> using event = std::shared_ptr<V>;
/// \brief typecast the "event" safely checking the event type
template<typename V1, typename V2> inline event<V1> as_event(event<V2> orig_event) {
    return std::dynamic_pointer_cast<V1>(orig_event);
}
/// \brief typecast the "event" forcing the event type
template<typename V1, typename V2> inline event<V1> is_event(event<V2> orig_event) {
    return std::static_pointer_cast<V1>(orig_event);
}
/// \brief allocate memory for, and instantiate, a new event
template<typename V> event<V> inline make_event(void) {
    return std::make_shared<V>();
}
/// \brief a fast event-type conversion to access event data. Does no checking
/// that the event actually exists.
template<typename V> inline V* read_as(const event<> &orig_event) {
    return (V *)orig_event.get();
}
/// \brief make a new event, copying from an existent event. Can be used to
/// upgrade the event-type.
template<typename V1, typename V2> event<V1> make_event(event<V2> orig_event) {
    return std::make_shared<V1>(*(orig_event.get()));
}
/// \brief vQueue is a wrapper for a deque of "event"
using vQueue = std::deque< event<vEvent> >;

/// \brief sort a vQueue ensuring temporal order
void qsort(vQueue &q, bool respectWraps = false);

/// \brief create an "event" based on the string tag it uses.
event<> createEvent(const std::string &type);

/// \brief get the coded packet size of an event
unsigned int packetSize(const std::string &type);

/// \brief camera values for stereo set-up
enum { VLEFT = 0, VRIGHT = 1 } ;

//event declarations
/// \brief base event class which defines the time the event occurs
class vEvent
{
public:
    static const std::string tag;
    unsigned int stamp:31;

    vEvent();
    virtual ~vEvent();

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual void encode(std::vector<int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(const int32_t *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
    virtual int getChannel() const;
    virtual void setChannel(const int channel);
};

/// \brief an event with a pixel location, camera number and polarity
class EV_API AddressEvent : public vEvent
{
public:
    static const std::string tag;

    union
    {
        uint32_t _coded_data;
        struct {
            unsigned int polarity:1;
            unsigned int x:10;
            unsigned int _xfill:1;
            unsigned int y:9;
            unsigned int corner:1;
            unsigned int channel:1;
            unsigned int type:1;
            unsigned int skin:1;
            unsigned int _fill:7;
        };
    };

    AddressEvent();
    AddressEvent(const vEvent &v);
    AddressEvent(const AddressEvent &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual void encode(std::vector<int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(const int32_t *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
    virtual int getChannel() const;
    virtual void setChannel(const int channel);
};
using AE = AddressEvent;

/// \brief an event with a neuron ID
class neuronEvent : public vEvent
{
public:
    static const std::string tag;
    unsigned int id;

    neuronEvent();
    neuronEvent(const vEvent &v);
    neuronEvent(const neuronEvent &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual void encode(std::vector<int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(const int32_t *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;

};

/// \brief an event with a taxel location, body-part and polarity
class EV_API SkinEvent : public vEvent
{
public:
    static const std::string tag;

    union
    {
        uint32_t _skei;
        struct {
            unsigned int polarity:1;
            unsigned int taxel:10;
            unsigned int _reserved1:2;
            unsigned int cross_base:1;
            unsigned int _sample:1;
            unsigned int _error:1;
            unsigned int body_part:3;
            unsigned int _reserved2:3;
            unsigned int side:1;
            unsigned int type:1;
            unsigned int skin:1;
        };
    };

    SkinEvent();
    SkinEvent(const vEvent &v);
    SkinEvent(const SkinEvent &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual void encode(std::vector<int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(const int32_t *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;

};

/// \brief an event with a taxel location, body-part and polarity
class EV_API SkinSample : public SkinEvent
{
public:
    static const std::string tag;

    unsigned int _ts:31;
    unsigned int value:16;

    SkinSample();
    SkinSample(const vEvent &v);
    SkinSample(const SkinSample &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual void encode(std::vector<int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(const int32_t *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;

};

/// \brief an event with a frequency channel number, left/right, binaural cues info and polarity
class CochleaEvent : public vEvent
{
public:
    static const std::string tag;

    union
    {
        uint32_t _cochleaei;
        struct {
            unsigned int polarity:1;
            unsigned int freq_chnn:7;
            unsigned int xso_type:1;
            unsigned int auditory_model:1;
            unsigned int _reserved1:2;
            unsigned int neuron_id:7;
            unsigned int sensor_id:3;
            unsigned int channel:1;
            unsigned int type:1;
            unsigned int cochlea_sensor_id:3;
            unsigned int _fill:5;
        };
    };

    CochleaEvent();
    CochleaEvent(const vEvent &v);
    CochleaEvent(const CochleaEvent &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual void encode(std::vector<int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(const int32_t *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
    virtual int getChannel() const;
    virtual void setChannel(const int channel);
};

/// \brief an AddressEvent with a velocity in visual space
class FlowEvent : public AddressEvent
{
public:
    static const std::string tag;
    union {
        uint32_t _fei[2];
        struct {
            float vx;
            float vy;
        };
    };


    FlowEvent();
    FlowEvent(const vEvent &v);
    FlowEvent(const FlowEvent &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual void encode(std::vector<int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(const int32_t *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;

    int getDeath() const;
};

/// \brief an AddressEvent with an ID or class label
class LabelledAE : public AddressEvent
{
public:
    static const std::string tag;
    int ID;

    LabelledAE();
    LabelledAE(const vEvent &v);
    LabelledAE(const LabelledAE &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual void encode(std::vector<int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(const int32_t *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
};

/// \brief a LabelledAE with parameters that define a 2D gaussian
class EV_API GaussianAE : public LabelledAE
{
public:
    static const std::string tag;
    union {
        uint32_t _gaei[3];
        struct {
            float sigx;
            float sigy;
            float sigxy;
        };
    };

    GaussianAE();
    GaussianAE(const vEvent &v);
    GaussianAE(const GaussianAE &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual void encode(std::vector<int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(const int32_t *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
};

/// \brief an event with a pixel location, camera number and polarity
class EV_API IMUevent : public vEvent
{
public:
    static const std::string tag;

    union
    {
        uint32_t _coded_data;
        struct {
            int value:16;
            unsigned int sensor:4;
            unsigned int _r1:2;
            unsigned int channel:1;
            unsigned int type:1;
            unsigned int _r2:8;
        };
    };

    IMUevent();
    IMUevent(const vEvent &v);
    IMUevent(const IMUevent &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual void encode(std::vector<int32_t> &b, unsigned int &pos) const;
    virtual bool decode(const yarp::os::Bottle &packet, size_t &pos);
    virtual void decode(const int32_t *&data);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
    virtual int getChannel() const { return channel;}
    virtual void setChannel(const int channel) { this->channel = channel;}
};

/// \brief count the events in a vQueue
template <class T> size_t countEvents(const T &q) { return q.size(); }
//template <> size_t countEvents<vQueue>(const T &q) { return q.size(); }

/// \brief count the time within a vQueue
template <typename T> inline int countTime(const T &q, int &p_time)
{
    if(!p_time) p_time = q.front().stamp;
    int dt = q.back().stamp - q.front().stamp;
    p_time = q.back().stamp;
    if(dt < 0) dt += vtsHelper::max_stamp;
    return dt;
}

template <> inline int countTime<vQueue> (const vQueue &q, int &p_time)
{
    if(!p_time) p_time = q.front()->stamp;
    int dt = q.back()->stamp - q.front()->stamp;
    p_time = q.back()->stamp;
    if(dt < 0) dt += vtsHelper::max_stamp;
    return dt;
}

template <> inline int countTime< std::vector<int32_t> > (const std::vector<int32_t> &q, int &p_time)
{
    if(!p_time) p_time = q[0] & vtsHelper::max_stamp;
    int dt = (q[q.size()-2] & vtsHelper::max_stamp) - p_time;
    p_time = q[q.size()-2] & vtsHelper::max_stamp;
    if(dt < 0) dt += vtsHelper::max_stamp;
    return dt;
}



}

#endif


