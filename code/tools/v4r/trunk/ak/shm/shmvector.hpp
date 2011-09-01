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


#ifndef SHARED_MEM_VECTOR_HPP
#define SHARED_MEM_VECTOR_HPP

#define BASE ShmArray< boost::interprocess::vector< T, boost::interprocess::allocator< T, boost::interprocess::managed_shared_memory::segment_manager > >, TypeVar>

#include <ak/shm/shmarray.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>

namespace ak {

template<typename T, uint16_t TypeVar>
class ShmVector : protected BASE  {
public:
    typedef boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager> AllocatorType;
    typedef boost::interprocess::vector<T, AllocatorType> VectorType;

    /**
    * @brief constructor
     **/
    ShmVector () : BASE() {
        shmType =Shm::TYPE_VECTOR;
    }
    /**
    * @brief constructor
    * @param rName The name
    * @param reserve size of the reserverd memory for the vector
    **/
    ShmVector(const std::string &rName, int reserve = -1) : BASE() {
        shmType =Shm::TYPE_VECTOR;
        openOrCreate(rName);
        if (reserve > 0) {
            this->pShmHeader->pData->reserve(reserve);
        }
    }
    /**
     * @brief Opens or creates a vector in the shared memory with the given name
     * @param rName The name
     * @return An errorvalue indicating if the operation was successful, resp. which error occured
     */
    Shm::ErrorCode openOrCreate ( const std::string& rName ) {
        this->varName = rName;

        this->localTimestamp.tv_sec = 0;
        this->localTimestamp.tv_usec = 0;

        try {
            if ( !this->pShmManager ) {
                this->pShmManager = ShmManager::getSingleton()->getShm();
            }

            // i have no idea why, but without it is not possible to access find and construct via pShmManager

            this->pShmHeader = ShmManager::getSingleton()->getShm()->find< ShmHeader<VectorType, TypeVar> > ( this->varName.c_str() ).first;
            if ( this->pShmHeader ) { // already exists
                return Shm::SUCCESS;
            } else {
                // create the header information...
                this->pShmHeader = ShmManager::getSingleton()->getShm()->construct< ShmHeader<VectorType, TypeVar> > ( this->varName.c_str() ) ();
                this->initHeader ( this->pShmHeader, 1);

                // and the data itself
                AllocatorType a ( this->pShmManager->get_segment_manager() );
                this->pShmHeader->pData = ShmManager::getSingleton()->getShm()->construct<VectorType> ( boost::interprocess::anonymous_instance ) ( a );

                return Shm::SUCCESS;
            }
        } catch ( ... ) {
            //AK_LOG_ERROR << "Exception in ShmVector::openOrCreate(): " << this->varName;
            std::cerr << "Exception in ShmVector::openOrCreate(): " << this->varName;
            return Shm::UNSPECIFIED;
        }
    }

    /**
     * @brief Access element<br>
     * Returns a reference to the element at position n in the vector container.<br>
     * A similar member function, vector::at, has the same behavior as this operator function, <br>
     * except that vector::at signals if the requested position is out of range by throwing an exception.
     * @param n Position of an element in the vector.<br>
     * Notice that the first element has a position of 0, not 1.
     * @return The element at the specified position in the vector.
     **/
    T& operator [] ( unsigned int n ) const {
        return this->pShmHeader->pData->at ( n );
    }


    /**
    * @brief Access element at post n
    * @param T& returns the element
    **/
    T& get( unsigned int n ) const {
        return this->pShmHeader->pData->at( n );
    }

    /**
     * @brief Returns the number of elements in the vector container.
     * @return The number of elements that conform the vector's content.
     **/
    unsigned int size ( ) const {
        return this->pShmHeader->pData->size();
    }

    /**
     * @brief Erases all the elements of the vector.<br>
     **/
    void clear ( ) const {
        return this->pShmHeader->pData->clear();
    }
    /**
     * @brief Inserts or erases elements at the end such that<br>
    * the size becomes n. New elements are copy constructed from x.
     * @param size
     **/
    void resize (unsigned int size ) const {
        return this->pShmHeader->pData->resize(size);
    }
    /**
     * @brief Inserts or erases elements at the end such that<br>
    * the size becomes n. New elements are copy constructed from x.
     * @param size
     * @param value
     **/
    void resize (unsigned int size, const T &rValue ) const {
        return this->pShmHeader->pData->resize(size, rValue);
    }


    /**
     * @brief Sets the values of this vector
     * @param rValue The value vector
     * @param signalCondition Defines if the changed signal should be sent always, only if the value of the var has changed, or never. Defaults to SIGNAL_CHANGE_CHANGED
     * @param pTimeStamp the current time will be used on NULL
     **/
    void set ( const std::vector<T> &rValue, Shm::Signals signalCondition = Shm::SIGNAL_CHANGED, const timeval *pTimeStamp = NULL) {
        lock();
        //this->pShmHeader->pData->resize ( rValue.size() );
        this->pShmHeader->pData->reserve ( rValue.size() );
        this->pShmHeader->pData->clear();
        for ( unsigned int i = 0; i < rValue.size(); i++ ) {
            this->pShmHeader->pData->push_back(rValue[i]);
        }
        if (pTimeStamp == NULL) {
            this->updateTimestamps();
        } else {
            this->updateTimestamps(*pTimeStamp);
        }
        unlock();

        bool shouldSignal = signalCondition != Shm::SIGNAL_NEVER;
        if ( shouldSignal ) {
            this->signalChanged();
        }
    }

    /**
     * @brief copys the shared vector savely
     * @param rDes Destination
     * @param bLook it looks the shm while copy use <b>lock and unlock in combination with false</b>
     * @return True on success, false on error
     * @see lock, unlock, ()
     **/
    bool copyTo(std::vector<T> &rDes, bool bLook = true) {
        if (valid()) {
            rDes.clear();
            if (bLook) lock();
            rDes.resize(size());
            for (unsigned int i = 0; i < size(); i++) {
                rDes[i] = get(i);
            }
            if (bLook) unlock();
            return true;
        }
        return false;
    }

    using BASE::shmType;
    using BASE::open;
    using BASE::destroy;
    using BASE::wait;
    using BASE::timedWait;
    using BASE::operator();
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
typedef ShmVector<int8_t, Shm::VAR_INT8> ShmVecInt8;
typedef ShmVector<int16_t, Shm::VAR_INT16> ShmVecInt16;
typedef ShmVector<int32_t, Shm::VAR_INT32> ShmVecInt32;
typedef ShmVector<int64_t, Shm::VAR_INT64> ShmVecInt64;
typedef ShmVector<uint8_t, Shm::VAR_UINT8> ShmVecUInt8;
typedef ShmVector<uint16_t, Shm::VAR_UINT16> ShmVecUInt16;
typedef ShmVector<uint32_t, Shm::VAR_UINT32> ShmVecUInt32;
typedef ShmVector<uint64_t, Shm::VAR_UINT64> ShmVecUInt64;
typedef ShmVector<float, Shm::VAR_F32> ShmVecFloat;
typedef ShmVector<double, Shm::VAR_F64> ShmVecDouble;
typedef ShmVector<bool, Shm::VAR_UINT8> ShmVecBool;


}

#undef BASE

#endif
