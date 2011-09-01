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

#ifndef SHARED_MEM_VAR_HPP
#define SHARED_MEM_VAR_HPP

#define BASE ShmArray<T, TypeVar>

#include <ak/shm/shmarray.hpp>

namespace ak {
template<typename T, uint16_t TypeVar>
class ShmVar : protected ShmArray <T, TypeVar> {
public:
    /**
    * @brief constructor
    **/
    ShmVar() : BASE() {
        this->shmType = Shm::TYPE_VARIABLE;
    }
    /**
    * @brief constructor
    * @param rValue init value
    **/
    ShmVar(const std::string &rName) : BASE() {
        this->shmType = Shm::TYPE_VARIABLE;
        BASE::openOrCreate(rName, 1);
    }
    /**
    * @brief constructor
    * @param rName The name
    * @param rValue init value
    * @param blocking blocking on true
    **/
    ShmVar(const std::string &rName, const T &rValue, bool blocking = true) : BASE() {
        this->shmType = Shm::TYPE_VARIABLE;
        if (BASE::openOrCreate(rName, 1) == Shm::SUCCESS) {
            if (blocking) {
                set(rValue );
            } else {
                *ptr() = rValue;
            }
        }
    }
    /**
     * @brief Opens or creates the shared memory value with the given name.
     * @param rName The name
     * \return An errorvalue indicating if the operation was successful, resp. which error occured
     */
    Shm::ErrorCode openOrCreate(const std::string &rName) {
        return BASE::openOrCreate(rName, 1);
    }

    /**
     * @brief copys the shared variable savely
     * \ToDo: disscussion ob wir diese Funktion anstelle von get verwenden sollten
     * @param rDes Destination
     **/
    void copyTo(T &rDes) {
        BASE::copyTo(rDes, 0);
    }

    /**
     * @brief returns the value of this variable
     * \return content of the shared variable or (T)0, if the index is <0 or >=size
     **/
    T get() {
        return BASE::get(0);
    }



    /**
     * @brief Sets the value of this variable
     * @param rValue The value
     * @param signalCondition Defines if the changed signal should be sent always, only if the value of the var has changed, or never. Defaults to SIGNAL_CHANGE_CHANGED
     * @param pTimeStamp the current time will be used on NULL
		 * @return returns reference to rValue
     **/
    const T& set(const T &rValue, Shm::Signals signalCondition = Shm::SIGNAL_CHANGED, const timeval *pTimeStamp = NULL) {
        BASE::set(rValue, 0, signalCondition, pTimeStamp);
				return rValue;
    }


    using BASE::shmType;
		using BASE::timed_lock;
    using BASE::open;
    using BASE::destroy;
    using BASE::wait;
    using BASE::timedWait;
    using BASE::operator->;
    using BASE::operator();
    using BASE::operator*;
    using BASE::ptr;
    using BASE::getMutex;
    using BASE::lock;
    using BASE::unlock;
    using BASE::getTypeVar;
    using BASE::getSharedTimestamp;
    using BASE::getLocalTimestamp;
    using BASE::itHasChanged;
    using BASE::hasChanged;
    using BASE::name;
    using BASE::valid;
    using BASE::getVarHeader;
};
typedef ShmVar<int8_t, Shm::VAR_NA> ShmNA;
typedef ShmVar<int8_t, Shm::VAR_INT8> ShmInt8;
typedef ShmVar<int16_t, Shm::VAR_INT16> ShmInt16;
typedef ShmVar<int32_t, Shm::VAR_INT32> ShmInt32;
typedef ShmVar<int64_t, Shm::VAR_INT64> ShmInt64;
typedef ShmVar<uint8_t, Shm::VAR_UINT8> ShmUInt8;
typedef ShmVar<uint16_t, Shm::VAR_UINT16> ShmUInt16;
typedef ShmVar<uint32_t, Shm::VAR_UINT32> ShmUInt32;
typedef ShmVar<uint64_t, Shm::VAR_UINT64> ShmUInt64;
typedef ShmVar<float, Shm::VAR_F32> ShmFloat;
typedef ShmVar<double, Shm::VAR_F64> ShmDouble;
typedef ShmVar<bool, Shm::VAR_UINT8> ShmBool;

}


#undef BASE

#endif
