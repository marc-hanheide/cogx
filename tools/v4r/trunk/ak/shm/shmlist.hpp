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

#ifndef SHARED_MEM_LIST_HPP
#define SHARED_MEM_LIST_HPP

#define BASE ShmArray< boost::interprocess::list< T, boost::interprocess::allocator< T, boost::interprocess::managed_shared_memory::segment_manager > >, TypeVar>

#include "sharedmemarray.hpp"
#include <boost/interprocess/containers/list.hpp>
#include <boost/interprocess/allocators/allocator.hpp>

namespace ak {

template<typename T, uint16_t TypeVar>
class ShmList : protected BASE  {
public:
    typedef boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager> AllocatorType;
    typedef boost::interprocess::list<T, AllocatorType> ListType;

    /**
    * \brief constructor
     **/
    ShmList () :BASE(){
        this->shmType =Shm::TYPE_LIST;
    }

    /**
     * \brief Opens or creates a list in the shared memory with the given name
     * \param rName The name
     * \return An errorvalue indicating if the operation was successful, resp. which error occured
     */
    Shm::ErrorCode openOrCreate(const std::string& rName) {
        this->varName = rName;

        this->localTimestamp.tv_sec = 0;
        this->localTimestamp.tv_usec = 0;

        try {
            if (!this->pShmManager) {
                this->pShmManager = ShmManager::getSingleton()->getShm();
            }

            this->pShmHeader = ShmManager::getSingleton()->getShm()->find< ShmHeader<ListType, TypeVar> >(this->varName.c_str()).first;
            if (this->pShmHeader) { // already exists
                return Shm::SUCCESS;
            } else {
                // create the header information...
                this->pShmHeader = ShmManager::getSingleton()->getShm()->construct< ShmHeader<ListType, TypeVar> >(this->varName.c_str())();
                this->initHeader(this->pShmHeader, 1);

                // and the data itself
                AllocatorType a(this->pShmManager->get_segment_manager());
                this->pShmHeader->pData = ShmManager::getSingleton()->getShm()->construct<ListType>(boost::interprocess::anonymous_instance)(a);

                return Shm::SUCCESS;
            }
        } catch (...) {
            //AK_LOG_ERROR << "Exception in ShmList::openOrCreate(): " << this->varName;
            std::cerr << "Exception in ShmList::openOrCreate(): " << this->varName;

            return Shm::UNSPECIFIED;
        }
    }

    /**
     * \brief Sets the values of this vector
     * \param rValue The value list
     * \param signalCondition Defines if the changed signal should be sent always, only if the value of the var has changed, or never. Defaults to SIGNAL_CHANGE_CHANGED
     **/
    void set ( const std::list<T> &rValue, Shm::Signals signalCondition = Shm::SIGNAL_CHANGED ) {
//ToDo
    }

    using BASE::shmType;
    using BASE::open;
    using BASE::wait;
    using BASE::destroy;
    using BASE::timedWait;
    using BASE::operator->;
    using BASE::operator*;
    using BASE::ptr;
    using BASE::getMutex;
		using BASE::timed_lock;
    using BASE::tryLock;
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
typedef ShmList<int8_t, Shm::VAR_INT8> ShmListInt8;
typedef ShmList<int16_t, Shm::VAR_INT16> ShmListInt16;
typedef ShmList<int32_t, Shm::VAR_INT32> ShmListInt32;
typedef ShmList<int64_t, Shm::VAR_INT64> ShmListInt64;
typedef ShmList<uint8_t, Shm::VAR_UINT8> ShmListUInt8;
typedef ShmList<uint16_t, Shm::VAR_UINT16> ShmListUInt16;
typedef ShmList<uint32_t, Shm::VAR_UINT32> ShmListUInt32;
typedef ShmList<uint64_t, Shm::VAR_UINT64> ShmListUInt64;
typedef ShmList<float, Shm::VAR_F32> ShmListFloat;
typedef ShmList<double, Shm::VAR_F63> ShmListDouble;
typedef ShmList<bool, Shm::VAR_UINT8> ShmListBool;

}

#undef BASE

#endif
