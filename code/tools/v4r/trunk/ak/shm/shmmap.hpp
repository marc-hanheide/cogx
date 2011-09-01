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


#ifndef SHARED_MEM_MAP_HPP
#define SHARED_MEM_MAP_HPP

#define BASE ShmArray< boost::interprocess::map< KeyType, ValueType, std::less<KeyType>, boost::interprocess::allocator< std::pair<const KeyType, ValueType>, boost::interprocess::managed_shared_memory::segment_manager > >, TypeVar>

#include <ak/shm/shmarray.hpp>
#include <boost/interprocess/containers/map.hpp>
#include <boost/interprocess/allocators/allocator.hpp>

namespace ak {

template<typename KeyType, typename ValueType, uint16_t TypeVar>
class ShmMap : protected BASE  {
public:
    typedef std::pair<const KeyType, ValueType> PairType;
    typedef boost::interprocess::allocator<PairType, boost::interprocess::managed_shared_memory::segment_manager> AllocatorType;
    typedef boost::interprocess::map<KeyType, ValueType, std::less<KeyType>, AllocatorType> MapType;

    /**
    * \brief constructor
     **/
    ShmMap () :BASE()  {
        shmType = Shm::TYPE_MAP;
    }

    /**
     * \brief Opens or creates a map in the shared memory with the given name
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

            // i have no idea why, but without it is not possible to access find and construct via pShmManager

            this->pShmHeader = ShmManager::getSingleton()->getShm()->find< ShmHeader<MapType, TypeVar> >(this->varName.c_str()).first;
            if (this->pShmHeader) { // already exists
                return Shm::SUCCESS;
            } else {
                // create the header information...
                this->pShmHeader = ShmManager::getSingleton()->getShm()->construct< ShmHeader<MapType, TypeVar> >(this->varName.c_str())();
                this->initHeader(this->pShmHeader, 1);

                // and the data itself
                AllocatorType a(this->pShmManager->get_segment_manager());
                this->pShmHeader->pData = ShmManager::getSingleton()->getShm()->construct<MapType>(boost::interprocess::anonymous_instance)(std::less<KeyType>(), a);

                return Shm::SUCCESS;
            }
        } catch (...) {
            //AK_LOG_ERROR << "Exception in ShmMap::openOrCreate(): " << this->varName;
                        std::cerr << "Exception in ShmMap::openOrCreate(): " << this->varName;
            return Shm::UNSPECIFIED;
        }
    }


    /**
     * \brief returns the index-th value of the vector<br>
     * If x matches the key of an element in the container, the function returns a reference to its mapped value.<br>
     * If x does not match the key of any element in the container, the function inserts a new element with that <br>
     * key and returns a reference to its mapped value. Notice that this always increases the map size by one, <br>
     * even if no mapped value is assigned to the element (the element is constructed using its default constructor).
     * \param x Key value of the element whose mapped value is accessed.<br>
     * key_type is a member type defined in map containers as an alias of Key, which is the first template parameter and the type of the element keys.
     * \return  A reference to the element with a key value equal to x<br>
     * T is the second template parameter, which defines the type of the mapped values in the container.
     **/
    ValueType& operator [] (const KeyType& x) {
        return (*((this->pShmHeader->pData->insert(make_pair(x,ValueType()))).first)).second;
    }


    using BASE::shmType;
    using BASE::open;
    using BASE::destroy;
    using BASE::wait;
    using BASE::timedWait;
    using BASE::operator->;
    using BASE::operator*;
    using BASE::ptr;
    using BASE::getMutex;
    using BASE::tryLock;
		using BASE::timed_lock;
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
}

#undef BASE

#endif
