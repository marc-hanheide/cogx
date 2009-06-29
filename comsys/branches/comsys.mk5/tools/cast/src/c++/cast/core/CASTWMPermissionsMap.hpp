/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef CAST_WM_PERMISSIONS_MAP_H_
#define CAST_WM_PERMISSIONS_MAP_H_

#include <cast/core/StringMap.hpp>
#include <cast/core/CASTUtils.hpp>

#include <cassert>
 
namespace cast {
 
  class CASTWMPermissionsMap {

  private:
    
    struct PermissionsStruct {
      cdl::WorkingMemoryPermissions m_permissions;
      pthread_mutex_t m_mutex;
      std::string m_owner;
      unsigned int m_lockCount;
      bool m_scheduledForDeletion;
    };

    typedef StringMap<PermissionsStruct>::map PermissionsMap;

    PermissionsMap m_permissionsMap;

    mutable pthread_mutex_t m_access;


    void lockMutex(pthread_mutex_t * _mutex) const {
      assert(_mutex != NULL);
      int err = pthread_mutex_lock(_mutex);
      if(err != 0) {
	throw CASTException(exceptionMessage(__HERE__, 
			    "failed mutex lock: %s", 
					     std::strerror(err)));
      }
    }

    void unlockMutex(pthread_mutex_t * _mutex) const {
      assert(_mutex != NULL);
      int err = pthread_mutex_unlock(_mutex);       
      if(err != 0) {
	throw CASTException(exceptionMessage(__HERE__, 
			    "failed mutex unlock: %s", 
					     std::strerror(err)));
      }      
    }

  
    void lockMap() const {
      lockMutex(&m_access);
    }
    
    void unlockMap() const {
      unlockMutex(&m_access);
    }

    bool live(const std::string & _id) const {
      PermissionsMap::const_iterator i = m_permissionsMap.find(_id);
      if(i != m_permissionsMap.end()) {
	return !(i->second.m_scheduledForDeletion);
      }
      else {
	return false;
      }
    }

  public: 
    
    CASTWMPermissionsMap();
    ~CASTWMPermissionsMap() {unlockMap();};


    /**
     * Add an entry to the map. The entry is unlocked by default.
     * 
     * @param _id
     */
    void add(const std::string & _id) throw (CASTException);
    /**
     * Acquires the lock for the entry given by the id. Blocks until the lock is
     * available.
     * 
     * @param _id
     * @throws InterruptedException
     */
    void lock(const std::string & _id, 
	      const std::string & _component,
	      const cdl::WorkingMemoryPermissions & _permissions) throw(CASTException);
    /**
     * Release the lock for the entry given by the id.
     * 
     * @param _id
     */
    void unlock(const std::string & _id,
		const std::string & _component) throw (CASTException);

    // DANGEROUS
    //     /**
    //      * Obtain then release the lock for the given id. Used to wait on
    //      * an unlock.
    //      */
    //     void block(const std::string & _id);
    

    /**
     * Trys to acquire the lock for the entry given by the id. Only obtains one
     * if one is available.
     * 
     * @param _id
     * @throws InterruptedException
     */
    bool tryLock(const std::string & _id, 
		 const std::string & _component,
		 const cdl::WorkingMemoryPermissions & _permissions)
      throw(CASTException);

//     /**
//      * Sets the permissions of the entry. This blocks to acquire the lock, then
//      * sets the permissions.
//      * 
//      * @param _id
//      * @param _permissions
//      * @throws InterruptedException
//      */
//     void setPermissions(const std::string & _id, cdl::WorkingMemoryPermissions _permissions);

    /**
     * Checks whether the given entry is locked.
     * 
     * @param _id
     * @return
     */
    bool isLocked(const std::string & _id);

    /**
     * Checks whether the given entry is locked.
     * 
     * @param _id
     * @return
     */
    bool isLockHolder(const std::string & _id, 
		      const std::string & _component) const;

    bool contains(const std::string & _id) const;

    /**
     * Gets the permissions for the given entry.
     * 
     * @param _id
     * @return
     */
    cdl::WorkingMemoryPermissions getPermissions(const std::string & _id) const;

    void remove(const std::string & _id);

    const std::string & getLockHolder(const std::string & _id) const;

  
  };
  
} 
#endif
 
