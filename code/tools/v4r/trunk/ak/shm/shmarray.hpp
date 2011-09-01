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


#ifndef SHARED_MEM_ARRAY_HPP
#define SHARED_MEM_ARRAY_HPP

#include <string>
#include <sys/time.h>
#include <ak/shm/shmheader.hpp>

namespace ak {

/**
 * @brief Used to access a variable in the shared memory
 * @author Bernhard Miller
*/
template<typename T, uint16_t TypeVar>
class ShmArray {
public:
    /**
     * @brief constructor to a shared variable
     **/
    ShmArray() : pShmHeader(0), pShmManager(0) {
        localTimestamp.tv_sec = 0;
        localTimestamp.tv_usec = 0;
        this->shmType = Shm::TYPE_ARRAY;
    }

    /**
     * @brief Opens or creates the shared memory array with the given name and size.
     * @param rName The name
     * @param size array size. This param can not be <= 0
     * @return An errorvalue indicating if the operation was successful, resp. which error occured
     */
    Shm::ErrorCode openOrCreate(const std::string &rName, unsigned int size) {
        varName = rName;
        localTimestamp.tv_sec = 0;
        localTimestamp.tv_usec = 0;

        try {
            if (!pShmManager) {
                pShmManager = ShmManager::getSingleton()->getShm();
            }

            pShmHeader = pShmManager->find< ShmHeader<T, TypeVar> >(varName.c_str()).first;
            if (pShmHeader) { // already exists
                if (pShmHeader->getSize() != size) {
                    return Shm::WRONG_SIZE;
                } else {
                    return Shm::SUCCESS;
                }
            } else {
                // create the header information...
                pShmHeader = pShmManager->construct< ShmHeader<T, TypeVar> >(varName.c_str())();
                pShmHeader->init(shmType, size);

                // and the data itself
                boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> myLock(pShmHeader->dataMutex);
                pShmHeader->pData = pShmManager->construct<T>(boost::interprocess::anonymous_instance)[pShmHeader->getSize()]();

                return Shm::SUCCESS;
            }
        } catch (const std::exception& ex) {
            // AK_LOG_ERROR << "Exception in ShmVar::openOrCreate() " << varName << ": " << ex.what();
            std::cerr << "Exception in ShmVar::openOrCreate() " << varName << ": " << ex.what();
            return Shm::UNSPECIFIED;
        }
    }

    /**
     * @brief Opens the shared memory value with the given name.
     * @param rName The name
     * @param size array size. if size > 0 the open function will check the shared memory size and 
     * returns an error if it is the case
     * @return An errorvalue indicating if the operation was successful, resp. which error occured
     */
    Shm::ErrorCode open(const std::string& rName, unsigned int size = 0) {
        varName = rName;
        localTimestamp.tv_sec = 0;
        localTimestamp.tv_usec = 0;

        try {
            if (!pShmManager) {
                pShmManager = ShmManager::getSingleton()->getShm();
            }

            if (!pShmManager) {
                return Shm::NOT_EXISTANT;
            }

            pShmHeader = pShmManager->find< ShmHeader<T, TypeVar> >(varName.c_str()).first;
            if (pShmHeader) { // already exists
                if ( (size > 0) && (pShmHeader->getSize() != size) ){
                    return Shm::WRONG_SIZE;
                } else {
                    return Shm::SUCCESS;
                }
                
            } else {
                return Shm::NOT_EXISTANT;
            }
        } catch (const std::exception& ex) {
            //AK_LOG_ERROR << "Exception in ShmVar::open() " << varName << ": " << ex.what();
            std::cerr << "Exception in ShmVar::open() " << varName << ": " << ex.what();
            return Shm::UNSPECIFIED;
        }
    }

    /**
    * @brief Destroys the Shared Memory object associated to this variable.
    */
    void destroy() {
        if (!pShmManager || !pShmHeader) {
            return;
        }

        if (pShmHeader->pData) {
            lock();
            pShmManager->destroy_ptr((T*)pShmHeader->pData.get());
            unlock();
        }

        pShmManager->destroy_ptr(pShmHeader);

        varName = "";
        pShmManager = NULL;
        pShmHeader = NULL;
    }

    /**
     * @brief Waits until the variable changes
     * @see timedWait
     */
    void wait() {
        if (!pShmHeader) {
            return;
        }

        ensureWaitCondition();

        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> myLock(*(pShmHeader->pConditionMutex.get()));
        pShmHeader->pCondition->wait(myLock);
    }

    /**
     * @brief Waits until the variable changes or a timeout is reached
     * @param milliSecs Timeout in milliseconds
     * @see wait
     * @return True if the condition was triggered, false if timeout
     */
    bool timedWait(int milliSecs) {
        if (!pShmHeader) {
            return false;
        }

        if (milliSecs <= 0) {
            return false;
        }

        ensureWaitCondition();

        boost::posix_time::ptime timeout = boost::posix_time::microsec_clock::universal_time() + boost::posix_time::milliseconds(milliSecs);
        // printf("timeout: %s\n", boost::posix_time::to_simple_string(timeout).c_str());
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> myLock(*(pShmHeader->pConditionMutex.get()));
        return pShmHeader->pCondition->timed_wait(myLock, timeout);
    }

    /**
     * @brief destructor
     **/
    virtual ~ShmArray() {
    }

    /**
     * @brief copys the shared variable savely
     * <br><b>SAFE ACCESS</b>
     * @param rDes Destination
     * @param index The index you want to access.
     * @return True on success, false on error
     **/
    bool copyTo(T &rDes, unsigned int index) {
        if (valid() && index < size()) {
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> myLock(pShmHeader->dataMutex);
            rDes = pShmHeader->pData[index];
            gettimeofday(&localTimestamp, NULL);
            return true;
        }

        return false;
    }

    /**
     * @brief returns the index-th value of this variable
     * <br><b>SAFE ACCESS</b>
     * @param index The index you want to access.
     * @return content of the shared variable or (T)0, if the index is <0 or >=size
     **/
    const T get(unsigned int index) {
        if (valid() && index < size()) {
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> myLock(pShmHeader->dataMutex);
            gettimeofday(&localTimestamp, NULL);
            return pShmHeader->pData[index];
        }
        throw ShmIndexException();
    }

    /**
     * @brief returns the index-th value of this variable
     * <br><b>SAFE ACCESS</b>
     * @param index The index you want to access. If you do not want to treat this variable like an array, just leave this parameter out
     * @return content of the shared variable or (T)0, if the index is <0 or >=size
     **/
    const T operator [] (unsigned int index) {
        return get(index);
    }

    /**
     * @brief Sets the index-th value of this variable
     * <br><b>SAFE ACCESS</b>
     * @param rValue The value
     * @param index The index you want to access. If you do not want to treat this variable like an array, just leave this parameter out
     * @param signalCondition Defines if the changed signal should be sent always, only if the value of the var has changed, or never. Defaults to SIGNAL_CHANGE_CHANGED
     * @param pTimeStamp the current time will be used on NULL
     **/
    void set(const T &rValue, unsigned int index, Shm::Signals signalCondition = Shm::SIGNAL_CHANGED, const timeval *pTimeStamp = NULL) {
        if (valid() && index < size()) {
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> myLock(pShmHeader->dataMutex);

            // try to avoid the expensive memcmp function in case we already know wheter or not we want to signal
            bool shouldSignal = signalCondition != Shm::SIGNAL_NEVER && (signalCondition == Shm::SIGNAL_ALWAYS || memcmp(&pShmHeader->pData[index], &rValue, sizeof(T)) != 0);

            pShmHeader->pData[index] = rValue;
            if (pTimeStamp == NULL) {
                updateTimestamps();
            } else {
                updateTimestamps(*pTimeStamp);
            }

            if (shouldSignal) {
                signalChanged();
            }
        }
    }

    /**
     * @brief Sets all indices of this variable to a given value
     * <br><b>SAFE ACCESS</b>
     * @param rValue theValue
     */
    void setAll(const T& rValue, const timeval *pTimeStamp = NULL) {
        if (valid()) {
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> myLock(pShmHeader->dataMutex);

            for (unsigned int index = 0; index < pShmHeader->getSize(); ++index) {
                pShmHeader->pData[index] = rValue;
            }
            if (pTimeStamp == NULL) {
                updateTimestamps();
            } else {
                updateTimestamps(*pTimeStamp);
            }
        }
    }

    /**
     * @brief Returns type ID of the shared object
     * @return typeid
     **/
    int getTypeVar() const {
        if (valid()) {
            return pShmHeader->typeVar;
        }
        return -1;
    }


    /**
     * @brief Timestamp of the shared object
     * @return timstamp
     **/
    struct timeval getSharedTimestamp() const {
            if (valid()) {
                return pShmHeader->getTimeStamp();
            }
            struct timeval ret;
            ret.tv_sec = 0;
            ret.tv_usec = 0;
            return ret;
        }

    /**
     * @brief Timestamp of the local (prosses) object
     * @return timstamp
     **/
    struct timeval getLocalTimestamp() const {
            return localTimestamp;
        }

    /**
     * @brief Excplcitely signals that this variable has changed, although it might not have. Also sets the timestamps to the current time.
     */
    void itHasChanged() {
        updateTimestamps();
        signalChanged();
    }

    /**
     * @brief Excplcitely signals that this variable has changed, although it might not have. Also sets the timestamps to the current time.
     * @param tv will be used to set the time stamps
     */
    void itHasChanged(timeval &tv) {
        updateTimestamps(tv);
        signalChanged();
    }
    /**
     * @brief Returns wheter or not the variable has been updated, since this local instance last set it
     * @return true on change
     **/
    bool hasChanged() {
        if (valid()) {
            timeval t = pShmHeader->getTimeStamp();
            bool changed = (t.tv_sec > localTimestamp.tv_sec) ||
                           (t.tv_sec == localTimestamp.tv_sec && t.tv_usec > localTimestamp.tv_usec);

            gettimeofday(&localTimestamp, NULL);
            return changed;
        }

        return false;
    }

    /**
     * @brief size of the data array if it was used as such
     * @return size
     */
    unsigned int size() const {
        if (valid()) {
            return pShmHeader->getSize();
        }
        return 0;
    }

    /**
     * @brief name of the shared variable
     * @return name
     */
    const std::string& name() const {
        return varName;
    }

    /**
     * @brief indicated the shared memory was initialized
     * @return true if it was initialized
     */
    bool valid() const {
        return ((pShmHeader != NULL) && (pShmManager != NULL));
    }

    /**
     * @brief returns a pointer to the data itself
     * <br><b>!-!-!-!-!--> UNSAFE ACCESS <--!-!-!-!-!</b>
     * @see lock, unlock, ptr
     * @return unsafe pointer
     **/
    T* operator ->() const {
        return ptr();
    }

    /**
     * @brief returns reference to the data itself
     * <br><b>!-!-!-!-!--> UNSAFE ACCESS <--!-!-!-!-!</b>
     * @see lock, unlock, ()
     * @return unsafe reference
     **/
    T& operator *() const {
        return *ptr();
    }

    /**
     * @brief returns reference to the data itself
     * <br><b>!-!-!-!-!--> UNSAFE ACCESS <--!-!-!-!-!</b>
     * @see lock, unlock, ()
     * @return unsafe reference
     **/
    T& operator ()() const {
        return *ptr();
    }

    /**
     * @brief returns a pointer to the shared variable <br>
     * <br><b>!-!-!-!-!--> UNSAFE ACCESS <--!-!-!-!-!</b>
     * @see lock, unlock
     **/
    T* ptr() const {
        T *p = NULL;
        if (valid()) {
            p = pShmHeader->pData.get();
        }
        return p;
    }

    /**
     * @brief returns a pointer to the mutex for this shared variable <br>
     * <br><b>!-!-!-!-!--> UNSAFE ACCESS <--!-!-!-!-!</b>
     * @see lock, unlock
     */
    boost::interprocess::interprocess_mutex* getMutex() const {
        if (valid()) {
            return &(pShmHeader->dataMutex);
        }
        return NULL;
    }

    /**
     * @brief unlock
     * <br><b>!-!-!-!-!--> UNSAFE ACCESS <--!-!-!-!-!</b>
     * @see lock, unlock
     * @return false when mutex is already locked, true when success
     **/
    bool tryLock() {
        if (valid()) {
            return pShmHeader->try_lock();
        }
        return false;
    }

    /**
     * @brief unlock
     * <br><b>!-!-!-!-!--> UNSAFE ACCESS <--!-!-!-!-!</b>
     * @see lock, unlock
     **/
    void lock() {
        if (valid()) {
            pShmHeader->lock();
        }
    }
    /**
    * Waits ms to to look the mutex
		* @param time to wait
    * @return If the thread acquires ownership of the mutex, returns true
    **/
    bool timed_lock(unsigned int ms){
        if (valid()) {
           return pShmHeader->timed_lock(ms);
        }
        return false;
		}

    /**
     * @brief unlock
     * <br><b>!-!-!-!-!--> UNSAFE ACCESS <--!-!-!-!-!</b>
     * @param hasChangedSignal signals a change on true
     * @see lock, unlock
     **/
    void unlock(bool hasChangedSignal = false) {
        if (valid()) {
            pShmHeader->unlock();
        }
        if (hasChangedSignal) {
            itHasChanged();
        }
    }
    /**
     * @brief returns a pointer to the shared header, be careful what you are doing
     * <br><b>!-!-!-!-!--> UNSAFE ACCESS <--!-!-!-!-!</b>
     * @return shared header
     */
    ShmHeader<T, TypeVar>* getVarHeader() {
        return pShmHeader;
    }

protected:


    // The header structure in the Shared memory
    ShmHeader<T, TypeVar>* pShmHeader;

    // The Shared memory itself
    boost::interprocess::managed_shared_memory* pShmManager;

    // used to check wheter the variable has been updated since it was last accessed by this instance
    struct timeval localTimestamp;

    // The name of for this variable
    std::string varName;

    // Variable for the shared memory type
    Shm::Type shmType;

    /**
    	 * @brief Sets both the local and the shared timestamp to the current time
    	 **/
    void updateTimestamps() {
        if (pShmHeader) {
            pShmHeader->updateTimeStamp();
            localTimestamp = pShmHeader->getTimeStamp();
        }
    }
    /**
    	 * @brief Sets both the local and the shared timestamp to the current time
    	 * @param tv will be used to set the time stamps
    	 **/
    void updateTimestamps(const timeval &tv) {
        if (pShmHeader) {
            pShmHeader->setTimeStamp(tv);
            localTimestamp = pShmHeader->getTimeStamp();
        }
    }

    /** @brief Inits some values of the given Header */
    void initHeader(ShmHeader<T, TypeVar>* header, unsigned int size) const {
        header->init(shmType, size);
    }

    /** @brief Signals that this variable has changed, in case anyone is listening (i.e. there is a condition variable) */
    void signalChanged() const {
        if (pShmHeader && pShmHeader->pCondition.get()) {
            pShmHeader->pCondition->notify_all();
        }
    }

    /**
     * @brief Ensures, that it is possible to wait for this variable's change condition
     */
    void ensureWaitCondition() {
        if (pShmHeader->pConditionMutex.get() == NULL) {
            // we need to create the condition
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> myLock(pShmHeader->dataMutex);

            pShmHeader->pConditionMutex = pShmManager->construct<boost::interprocess::interprocess_mutex>(boost::interprocess::anonymous_instance)();
            pShmHeader->pCondition = pShmManager->construct<boost::interprocess::interprocess_condition>(boost::interprocess::anonymous_instance)();
        }
    }
};
typedef ShmArray<int8_t, Shm::VAR_INT8> ShmArrayInt8;
typedef ShmArray<int16_t, Shm::VAR_INT16> ShmArrayInt16;
typedef ShmArray<int32_t, Shm::VAR_INT32> ShmArrayInt32;
typedef ShmArray<int64_t, Shm::VAR_INT64> ShmArrayInt64;
typedef ShmArray<uint8_t, Shm::VAR_UINT8> ShmArrayUInt8;
typedef ShmArray<uint16_t, Shm::VAR_UINT16> ShmArrayUInt16;
typedef ShmArray<uint32_t, Shm::VAR_UINT32> ShmArrayUInt32;
typedef ShmArray<uint64_t, Shm::VAR_UINT64> ShmArrayUInt64;
typedef ShmArray<float, Shm::VAR_F32> ShmArrayFloat;
typedef ShmArray<double, Shm::VAR_F64> ShmArrayDouble;
typedef ShmArray<bool, Shm::VAR_UINT8> ShmArrayBool;
}

#endif
