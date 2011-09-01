/***************************************************************************
 *   Copyright (C) 2010 by Markus Bader and Bernhard Miller                *
 *   markus.bader@tuwien.ac.at                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#ifndef SHARED_MEM_HEADER_HPP
#define SHARED_MEM_HEADER_HPP


#include <string>
#include <sys/time.h>

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/offset_ptr.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <ak/shm/shm.hpp>
#include <ak/shm/shmmanager.hpp>

namespace ak {


/**
 * @brief Stores information about a variable in the shared memory
 * @author Markus Bader & Bernhard Miller
*/
template<typename T, uint16_t TypeVar>
class ShmHeader {
    friend class ShmManager;
    friend class ShmArray<T, TypeVar>;
    friend class ShmVar<T, TypeVar>;
    friend class ShmList<T, TypeVar>;
    friend class ShmVector<T, TypeVar>;
    //shared mem types
private:
    ///shared memory type identifier
    uint8_t typeShm;

    ///variable type identifier
    uint16_t typeVar;

    /// changed timestamp
    timeval tstamp;

    /// used to define the array size
    uint32_t array_size;

    /// used to define the variable type size in bytes
    uint32_t type_size;

    /// used to prevent concurrent access to the data
    boost::interprocess::interprocess_mutex dataMutex;

    /// used to signal if the variable has been changed
    boost::interprocess::offset_ptr<boost::interprocess::interprocess_mutex> pConditionMutex;
    boost::interprocess::offset_ptr<boost::interprocess::interprocess_condition> pCondition;
    
public:
    const timeval& getTimeStamp() {
        return tstamp;
    }
    uint32_t getSize() {
        return array_size;
    }
    /**
    * Returns the SharedVariable Type
    * @return shared memory type ARRAY, VARIABLE, VECTOR, LIST, MAP
    **/
    Shm::Type getShmType() {
        return (Shm::Type) typeShm;
    }
    /**
    * Returns the Variable Type
    * @return shared variable type INT8, UINT16, ....
    **/
    uint16_t getVarType() {
        return typeVar;
    }
    /**
    * Tries to lock
    * @return If the thread acquires ownership of the mutex, returns true
    **/
    bool try_lock() {
        return dataMutex.try_lock();
    }
    /**
    * Waits until the lock was suggessful lock
    **/
    void lock() {
        return dataMutex.lock();
    }
    /**
    * Unlook
    **/
    void unlock() {
        return dataMutex.unlock();
    }
    /**
    * Waits ms to to look the mutex
    * @param ms to wait befor the function returns it the mutex is locked
    * @return If the thread acquires ownership of the mutex, returns true
    **/
    bool timed_lock(unsigned int ms) {
        boost::posix_time::ptime now(boost::posix_time::microsec_clock::universal_time());
        boost::posix_time::ptime abs_time = now + boost::posix_time::milliseconds(ms);
        return dataMutex.timed_lock(abs_time);
    }
    /// pointer to the data itself
    boost::interprocess::offset_ptr<T> pData;

    bool isLocked() {
        if (dataMutex.try_lock()) {
            dataMutex.unlock();
            return false;
        } else return true;
    }
    std::string getShmTypeString() {
        return Shm::getShmTypeString(typeShm);
    }
    std::string getVarTypeString() {
        return Shm::getVarTypeString(typeVar);
    }
    std::string getAgeString() {
        return Shm::getAgeString(tstamp);
    }
    std::string getValueString(uint32_t array_size_max = 10) {
        std::stringstream ss;
        if ((typeShm == Shm::TYPE_VARIABLE) || (typeShm == Shm::TYPE_ARRAY)) {
            ss << Shm::getPtrValueAsString((void*) pData.get(), typeVar, 0);
            for (uint32_t i = 1; i < array_size && i < array_size_max; i++) {
                ss << " " << Shm::getPtrValueAsString((void*) pData.get(), typeVar, i);
            }
        }
        return ss.str();
    }
    void signal() {
        if (pCondition.get()) {
            pCondition->notify_all();
        }
    }
    void updateTimeStamp() {
        gettimeofday(&tstamp, NULL);
    }
private:
    void setTimeStamp(const timeval &t) {
        tstamp = t;
    }
    void setShmType(const Shm::Type type) {
        typeShm = (uint8_t) type;
    }
    void setVarType(uint16_t type) {
        typeVar = type;
    }
    void init(Shm::Type typeShm, const uint32_t &size) {
        this->typeShm = (uint8_t) typeShm;
        array_size = size;
        type_size = sizeof(T);
        typeVar = TypeVar;
        tstamp.tv_sec = 0;
        tstamp.tv_usec = 0;
    }
};

typedef ShmHeader<int8_t, 0> ShmHeaderNA;



}

#endif
