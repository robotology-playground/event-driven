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

#ifndef __VGENPORT__
#define __VGENPORT__
#if WIN32
#define SCHED_FIFO 8
#endif
#include <vector>
#include <yarp/os/all.h>
#include <mutex>
#include "event-driven/vCodec.h"
#include "event-driven/vtsHelper.h"

using namespace yarp::os;
using std::vector;
using std::string;
using std::int32_t;
using std::deque;

namespace ev {

class vPortableInterface : public Portable {

protected:

    //headers
    vector<int32_t> header1;
    string header2;
    vector<int32_t> header3;

    //data
    const char * datablock;
    unsigned int datalength; //<- set the number of bytes here
    string event_type;
    unsigned int ints_to_read; //<- in integers

    //sizes
    unsigned int elementINTS;
    unsigned int elementBYTES;

public:

    vector<int32_t> internaldata;

    /// \brief instantiate the correct headers for a Bottle
    vPortableInterface() {
        header1.push_back(BOTTLE_TAG_LIST); //bottle code
        header1.push_back(2); //elements in bottle "AE" then bottle data
        header1.push_back(BOTTLE_TAG_STRING); //code for string
        header1.push_back(0); // length of string
        header2 = "";
        header3.push_back(BOTTLE_TAG_LIST|BOTTLE_TAG_INT); // bottle code + specialisation with ints
        header3.push_back(0); // <- set the number of ints here (e.g. 2 * #v's)
        elementINTS = 0;
        elementBYTES = sizeof(int32_t) * elementINTS;
    }

    /// \brief set the type of event that this vBottleMimic will send
    void setHeader(std::string eventtype) {
        header1[3] = eventtype.size();  //set the string length
        header2 = eventtype;            //set the string itself

        elementINTS = packetSize(eventtype);
        elementBYTES = sizeof(int32_t) * elementINTS;

    }

    /// \brief for data already allocated in contiguous space. Just send this
    /// data on a port without memory reallocation.
    void setExternalData(const char * datablock, unsigned int datalength) {

        //no check for event-type safety is possible..

        header3[1] = elementINTS * (datalength / elementBYTES); //forced to be x8
        this->datablock = datablock;
        this->datalength = elementBYTES * header3[1] / elementINTS; //forced to be x8

    }

    /// \brief send an entire vQueue. The queue is encoded and allocated
    /// into a single contiguous memory space. Faster than a standard vBottle.
    void setInternalData(const vQueue &q) {

        if(header2 != q.front()->getType())
            setHeader(q.front()->getType());

        header3[1] = elementINTS * q.size(); //number of ints

        if((int)internaldata.size() < header3[1]) //increase internal mem if needed
            internaldata.resize(header3[1]);

        unsigned int pos = 0;
        for(unsigned int i = 0; i < q.size(); i++)  //decode the data into
            q[i]->encode(internaldata, pos);        //internal memeory

        if(pos != (unsigned int)header3[1])
            yError() << "vBottleMimic: encoding incorrect";

        this->datablock = (const char *)internaldata.data();
        this->datalength = elementBYTES * q.size();
    }

    /// \brief send an entire vQueue. The queue is encoded and allocated
    /// into a single contiguous memory space. Faster than a standard vBottle.
    template <typename T> void setInternalData(const std::deque<T> &q) {

        if(header2 != T::tag)
            setHeader(T::tag);

        header3[1] = elementINTS * q.size(); //number of ints

        if((int)internaldata.size() < header3[1]) //increase internal mem if needed
            internaldata.resize(header3[1]);

        unsigned int pos = 0;
        for(unsigned int i = 0; i < q.size(); i++)  //decode the data into
            q[i].encode(internaldata, pos);        //internal memeory

        if(pos != (unsigned int)header3[1])
            yError() << "vPortInterface: encoding incorrect";

        this->datablock = (const char *)internaldata.data();
        this->datalength = elementBYTES * q.size();
    }

    template <typename T> void setInternalData(const std::vector<T> &q) {

        if(header2 != T::tag)
            setHeader(T::tag);

        header3[1] = elementINTS * q.size(); //number of ints

        if((int)internaldata.size() < header3[1]) //increase internal mem if needed
            internaldata.resize(header3[1]);

        unsigned int pos = 0;
        for(unsigned int i = 0; i < q.size(); i++)  //decode the data into
            q[i].encode(internaldata, pos);        //internal memeory

        if(pos != (unsigned int)header3[1])
            yError() << "vPortInterface: encoding incorrect";

        this->datablock = (const char *)internaldata.data();
        this->datalength = elementBYTES * q.size();
    }

    void setInternalData(const deque<int32_t> &q) {

        header3[1] = q.size();

        if((int)internaldata.size() < header3[1]) //increase internal mem if needed
            internaldata.resize(header3[1]);

        for(unsigned int i = 0; i < q.size(); i++)  //decode the data into
            internaldata[i] = q[i];

        this->datablock = (const char *)internaldata.data();
        this->datalength = elementBYTES * q.size();
    }

    /// \brief write the data on the connection.
    bool write(yarp::os::ConnectionWriter& connection) const {

        connection.appendBlock((const char *)header1.data(),
                                       header1.size() * sizeof(int32_t));
        connection.appendBlock(header2.c_str(), header1[3]);
        connection.appendBlock((const char *)header3.data(),
                                       header3.size() * sizeof(int32_t));
        connection.appendBlock(datablock, datalength);

        return !connection.isError();
    }

    /// \brief does nothing as this is a write-only port.
    bool read(yarp::os::ConnectionReader& connection) {

        //META DATA OF BOTTLE
        if(connection.expectInt() != BOTTLE_TAG_LIST) //a list
            return false;
        if(connection.expectInt() != 2) //of two internal bottles
            return false;

        //DATA OF FIRST INTERNAL BOTTLE (type of event)
        if(connection.expectInt() != BOTTLE_TAG_STRING) // first of two
            return false;
        int str_len = connection.expectInt();
        event_type.resize(str_len);
        connection.expectBlock((char *)event_type.data(), str_len);

        //DATA OF SECOND INTERNAL BOTTLE (data of events)
        if(connection.expectInt() != (BOTTLE_TAG_LIST|BOTTLE_TAG_INT32))
            return false;
        ints_to_read = (unsigned int)connection.expectInt(); //in integers!!

        if(ints_to_read > internaldata.size())
            internaldata.resize(ints_to_read);
        if(!connection.expectBlock((char *)internaldata.data(),
                                   sizeof(std::int32_t) * ints_to_read)) {
            yError() << "Could not read datablock";
            return false;
        }

        return true;
    }

    bool decodePacket(vQueue &read_q)
    {
        int event_size = packetSize(event_type);
        if(!event_size) {
            yError() << "Cannot get event-size of" << event_type;
            return false;
        }

        if(ints_to_read % event_size) {
            yError() << "Data corruption: incompatible data size."
                     << ints_to_read << "32 bit ints, but needed a multiple of"
                     << event_size;
            return false;
        }


        event<> v = createEvent(event_type);
        if(v == nullptr) {
            yError() << "Cannot create new event of type:" << event_type;
            return false;
        }

        const int32_t *data = internaldata.data();
        for(unsigned int i = 0; i < ints_to_read / event_size; i++) {
            v->decode(data);
            read_q.push_back(v->clone());
        }
        return true;

    }

    template <typename T> bool decodePacket(vector<T> &read_q)
    {

        if(event_type != T::tag) {
            yWarning() << "Incompatible event-type read";
            return false;
        }

        auto event_size = packetSize(event_type);
        if(ints_to_read % event_size) {
            yError() << "Data corruption: incompatible data size."
                     << ints_to_read << "32 bit ints, but needed a multiple of"
                     << event_size;
            return false;
        }

        const int32_t *data = internaldata.data();
        read_q.resize(ints_to_read / event_size);
        for(unsigned int i = 0; i < read_q.size(); i++) {
            read_q[i].decode(data);
        }
        return true;
    }

    bool decodePacket(vector<int32_t> &read_q)
    {
        read_q.resize(ints_to_read);
        for(size_t i = 0; i < ints_to_read; i++)
            read_q[i] = internaldata.at(i);

        return true;
    }


};

//this should open a yarp::os::Port
class vWritePort
{

protected:

    vPortableInterface internal_storage;
    Port port;

    bool _internal_write(Stamp &envelope)
    {
        if(!port.setEnvelope(envelope))
            return false;
        if(!port.write(internal_storage))
            return false;
        return true;
    }

public:

    bool open(std::string name)
    {
        return port.open(name);
    }

    void close()
    {
        port.close();
    }

    void setWriteType(std::string tag)
    {
        internal_storage.setHeader(tag);
    }

    int getOutputCount() {
        return port.getOutputCount();
    }

    bool write(const vector<int32_t> &q, Stamp &envelope)
    {
        internal_storage.setExternalData((const char *)q.data(),
                                         q.size() * sizeof(int32_t));
        return _internal_write(envelope);
    }

    bool write(const vector<int32_t> &q, Stamp &envelope, size_t n_to_write)
    {
        n_to_write = std::min(n_to_write, q.size());
        internal_storage.setExternalData((const char *)q.data(),
                                         n_to_write * sizeof(int32_t));
        return _internal_write(envelope);
    }


    bool write(const deque<int32_t> &q, Stamp &envelope)
    {
        internal_storage.setInternalData(q);
        return _internal_write(envelope);
    }

    bool write(const vQueue &q, Stamp &envelope)
    {
        internal_storage.setInternalData(q);
        return _internal_write(envelope);
    }

    template <class T> bool write(const std::deque<T> &q, Stamp &envelope)
    {
        internal_storage.setInternalData<T>(q);
        return _internal_write(envelope);
    }

    template <class T> bool write(const std::vector<T> &q, Stamp &envelope)
    {
        internal_storage.setInternalData<T>(q);
        return _internal_write(envelope);
    }

};

template <class T> class vReadPort : public Thread
{

protected:

    vPortableInterface internal_storage;
    Port port;

    T *working_queue;
    deque< T* > qq;
    deque<Stamp> sq;
    deque<int> t_q;
    deque<int> n_q;


    std::mutex m;
    std::mutex read_mutex;
    Semaphore dataavailable;

    unsigned int qlimit;
    unsigned int unprocdqs;
    unsigned int delay_nv;
    long unsigned int delay_t;
    double event_rate;
    int p_time;


public:

    /// \brief constructor
    vReadPort()
    {
        qlimit = 0;
        delay_nv = 0;
        delay_t = 0;
        event_rate = 0;
        unprocdqs = 0;
        working_queue = nullptr;
        p_time = 0;

        //setPriority(0, SCHED_FIFO);

        dataavailable.wait(); //init counter to 0
    }

    /// \brief desctructor
    ~vReadPort()
    {
        m.lock();
        typename deque< T* >::iterator i;
        for(i = qq.begin(); i != qq.end(); i++)
            delete *i;
        qq.clear();
        m.unlock();
    }


    bool open(std::string name)
    {
        if(!port.open(name)) {
            yError() << "Could not open vGenReadPort input port: " << name;
            return false;
        }
        start();
        return true;
    }

    void interrupt()
    {
        read_mutex.lock();
        port.interrupt();
    }

    void resume()
    {
        port.resume();
        read_mutex.unlock();
    }

    void close()
    {
        this->stop(); //make sure the isStopping() is true
        port.close(); //close the port connections
    }

    void onStop()
    {
        port.interrupt(); //port.read() will return false
        read_mutex.unlock(); //allow port.read() to be called
        dataavailable.post(); //all a this->read() to return
    }

    void run()
    {
        while(true) {

            //blocking read of data from the port
            read_mutex.lock();
            bool read_success = port.read(internal_storage);
            read_mutex.unlock();

            if(!read_success) {
                if(!isStopping())
                    yWarning() << "vGenReadPort read return false!";
                break;
            }

            if(qlimit && qq.size() >= qlimit)
                continue;

            yarp::os::Stamp yarp_stamp;
            port.getEnvelope(yarp_stamp);
            T *next_queue = new T;
            internal_storage.decodePacket(*next_queue);

            m.lock();

            qq.push_back(next_queue);
            sq.push_back(yarp_stamp);

            unprocdqs++;

            n_q.push_back(countEvents<T>(*next_queue));
            t_q.push_back(countTime<T>(*next_queue, p_time));

            delay_nv += n_q.back();
            delay_t += t_q.back();
            if(t_q.back())
                event_rate = n_q.back() / (double)(t_q.back());

            m.unlock();

            dataavailable.post();

        }

    }

    /// \brief ask for a pointer to the next vQueue.
    /// if wait is true Blocks if no data is ready.
    const T* read(yarp::os::Stamp &yarpstamp, bool wait = true)
    {
        if(working_queue) {
            m.lock();

            delay_nv -= n_q.front();
            n_q.pop_front();
            delay_t  -= t_q.front();
            t_q.pop_front();

            delete qq.front();
            qq.pop_front();
            sq.pop_front();
            m.unlock();
        }

        if(wait) {
            dataavailable.wait();

            if(qq.size()) {
                yarpstamp = sq.front();
                working_queue = qq.front();
                m.lock();
                unprocdqs--;
                m.unlock();
            }  else {
                working_queue =  0;
            }

        } else {
            if(dataavailable.check() && qq.size()) {
                yarpstamp = sq.front();
                working_queue = qq.front();
                m.lock();
                unprocdqs--;
                m.unlock();
            } else {
                working_queue = 0;
            }
        }

        return working_queue;

    }

    /// \brief set the maximum number of qs that can be stored in the buffer.
    /// A value of 0 keeps all qs.
    void setQLimit(unsigned int number_of_qs)
    {
        qlimit = number_of_qs;
    }

    /// \brief unBlocks the blocking call in getNextQ. Useful to ensure a
    /// graceful shutdown. No guarantee the return of getNextQ will be valid.
    void releaseDataLock()
    {
        dataavailable.post();
    }

    /// \brief ask for the number of vQueues currently allocated.
    unsigned int queryunprocessed()
    {
        return unprocdqs;
    }

    /// \brief ask for the number of events in all vQueues.
    unsigned int queryDelayN()
    {
        return delay_nv;
    }

    /// \brief ask for the total time spanned by all vQueues.
    double queryDelayT()
    {
        return delay_t * vtsHelper::tsscaler;
    }

    /// \brief ask for the high precision event rate
    double queryRate()
    {
        return event_rate * vtsHelper::vtsscaler;
    }

    int getInputCount(){
        return port.getInputCount();
    }

    std::string delayStatString()
    {
        std::ostringstream oss;
        oss << "qs: " << queryunprocessed() << " events: " << queryDelayN() <<
               " time(s): " << queryDelayT() << " rate: " << queryRate();
        return oss.str();
    }

    void setReporter(PortReport &reporter)
    {
        port.setReporter(reporter);
    }


};


} //end namespace ev

#endif
