
#include "CASTWMPermissionsMap.hpp"
#include "CASTUtils.hpp"

#include <iostream>
using namespace std;

namespace cast {
 
  CASTWMPermissionsMap::CASTWMPermissionsMap() {
    pthread_mutexattr_t attr;
    // note: errors here are very unlikely, so we just do a "weaker" overall
    // checking
    int err =  pthread_mutexattr_init(&attr);
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed to create mutex: %s", strerror(err)));
    }

#ifdef __LINUX__    
    //nah: this doesn't work, and isn't necessary, on macs for some reason
    err = pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed to create mutex: %s", strerror(err)));
    }
#endif

//     err = pthread_mutexattr_settype(&attr,
// 				    PTHREAD_MUTEX_RECURSIVE);
//     if(err != 0) {
//       throw CASTException(exceptionMessage(__HERE__, "failed to create mutex: %s", strerror(err)));
//     }

    //init mutex in map
    err = pthread_mutex_init(&m_access, &attr);
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed to create mutex: %s", strerror(err)));
    }
    
  }

  /**
   * Add an entry to the map. The entry is unlocked by default.
   * 
   * @param _id
   */
  void CASTWMPermissionsMap::add(const std::string & _id) throw (CASTException) {
    
    lockMap();
    assert (m_permissionsMap.find(_id) == m_permissionsMap.end());
    
    m_permissionsMap[_id].m_permissions = cdl::UNLOCKED;
    m_permissionsMap[_id].m_owner = "";
    m_permissionsMap[_id].m_lockCount = 0;
    m_permissionsMap[_id].m_scheduledForDeletion = false;
    
    pthread_mutexattr_t attr;
    // note: errors here are very unlikely, so we just do a "weaker" overall
    // checking
    int err =  pthread_mutexattr_init(&attr);
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed to create mutex: %s", strerror(err)));
    }

#ifdef __LINUX__    
    //nah: this doesn't work, and isn't necessary, on macs for some reason
    err = pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    if(err != 0) {
      unlockMap();
      throw CASTException(exceptionMessage(__HERE__, "failed to create mutex: %s", strerror(err)));
    }
#endif

    //init mutex in map
    err = pthread_mutex_init(&(m_permissionsMap[_id].m_mutex), &attr);
    if(err != 0) {
      unlockMap();
      throw CASTException(exceptionMessage(__HERE__, "failed to create mutex: %s", strerror(err)));
    }

    unlockMap();
  }

  /**
   * Acquires the lock for the entry given by the id. Blocks until the lock is
   * available.
   * 
   * @param _id
   * @throws InterruptedException
   */
  void CASTWMPermissionsMap::lock(const std::string & _id, 
				  const std::string & _component,
				  const cdl::WorkingMemoryPermissions & _permissions) throw(CASTException) {

    lockMap();
    pthread_mutex_t * mutex = NULL;
    //use a block to double-check we don't touch that iterator
    //post-lock
    {
      //if this is an invalid entry, return;
      if(!live(_id)) {
	unlockMap();
	return;
      }
      
      PermissionsMap::iterator i = m_permissionsMap.find(_id);
      
      assert (i != m_permissionsMap.end());    
      
      //if this lock is already owned by the locking component
      if(i->second.m_owner == _component) {
	assert(_permissions == i->second.m_permissions);
	assert(i->second.m_lockCount > 0);
	i->second.m_lockCount++;
	cout<<"CASTWMPermissionsMap::lock recursive lock: "<<_id<<" "<<_component<<endl;
	unlockMap();
	return;
      }
            
      mutex = (&(i->second.m_mutex));
      
    }
    unlockMap();
      
    //block until mutex is locked
    lockMutex(mutex);      
    

    lockMap();    
    {
      //if this is now invalid entry, return;
      if(!live(_id)) {
	cout<<"CASTWMPermissionsMap::unlocking dead mutex: "<<_id<<" "<<_component<<endl;
	assert(mutex != NULL);
	unlockMutex(mutex);
      }
      else {      
	PermissionsMap::iterator i = m_permissionsMap.find(_id);
	i->second.m_permissions = _permissions;
	i->second.m_owner = _component;
	i->second.m_lockCount = 1;
	i->second.m_scheduledForDeletion = false;
      }
    }
    unlockMap();

  }

  /**
   * Release the lock for the entry given by the id.
   * 
   * @param _id
   */
  void CASTWMPermissionsMap::unlock(const std::string & _id,
				    const std::string & _component) 
    throw (CASTException) {

    lockMap();
    PermissionsMap::iterator i = m_permissionsMap.find(_id);    
    if (i == m_permissionsMap.end()) {
      cout<<"CASTWMPermissionsMap::unlock leaving deleted item: "<<_id<<" "<<_component<<endl;
    }
    else if(i->second.m_lockCount == 0) {
      cout<<"CASTWMPermissionsMap::unlock leaving unlocked item: "<<_id<<" "<<_component<<endl;
    }
    else if(i->second.m_lockCount > 1) {
      assert (i->second.m_owner ==_component);      
      cout<<"CASTWMPermissionsMap::unlock reduce recursive lock: "<<_id<<" "<<_component<<endl;
      i->second.m_lockCount--;
    }
    else {
      
      if (!deleteAllowed(i->second.m_permissions)) {
	assert (i->second.m_owner ==_component);            
      }
      i->second.m_permissions = cdl::UNLOCKED;
      i->second.m_owner = "";
      i->second.m_lockCount = 0;
      unlockMutex(&(i->second.m_mutex));
    }
    unlockMap();
 

  }


  /**
   * Trys to acquire the lock for the entry given by the id. Only obtains one
   * if one is available.
   * 
   * @param _id
   * @throws InterruptedException
   */
  bool CASTWMPermissionsMap::tryLock(const std::string & _id, 
				     const std::string & _component,
				     const cdl::WorkingMemoryPermissions & _permissions)
    throw(CASTException) {

    lockMap();
    bool ret = false;
    PermissionsMap::iterator i = m_permissionsMap.find(_id);    
    //non existant or scheduled for deletion, disallow
    if (i == m_permissionsMap.end() || i->second.m_scheduledForDeletion) {
      ret = false;
    }
    //if already locked by someone else, say not
    else if (i->second.m_lockCount > 0 && i->second.m_owner != _component) {
      ret = false;
    }
    //else we know that locking is fine here
    else {
      //if we already actually hold the lock
      if(i->second.m_lockCount > 0) {
	assert(i->second.m_permissions == _permissions);
	i->second.m_lockCount++;
	cout<<"CASTWMPermissionsMap::tryLock recursive lock: "<<_id<<" "<<_component<<endl;
      }
      //otherwise setup the details and lock away
      else {
	cout<<"CASTWMPermissionsMap::tryLock normal lock: "<<_id<<" "<<_component<<endl;
	lockMutex(&(i->second.m_mutex));
	i->second.m_permissions = _permissions;
	i->second.m_owner = _component;
	i->second.m_lockCount = 1;
	i->second.m_scheduledForDeletion = false;
      }
      ret = true;      
    }

    unlockMap();
    return ret;
    
  }

  /**
   * Checks whether the given entry is locked.
   * 
   * @param _id
   * @return
   */
  bool CASTWMPermissionsMap::isLocked(const std::string & _id)  {
    lockMap();
    PermissionsMap::iterator i = m_permissionsMap.find(_id);    
    bool ret = false;
    //if in the map, it's locked if lock count > 0
    if(i != m_permissionsMap.end()) {
      ret = i->second.m_lockCount > 0;
    }
    unlockMap();
    return ret;
  }

  /**
   * Checks whether the given entry is locked.
   * 
   * @param _id
   * @return
   */
  bool CASTWMPermissionsMap::isLockHolder(const std::string & _id, 
					  const std::string & _component) const {

    lockMap();
    PermissionsMap::const_iterator i = m_permissionsMap.find(_id);    
    bool ret = false;
    if(i != m_permissionsMap.end()) {
      ret = (_component == i->second.m_owner);
    }
    unlockMap();
    return ret;
  }

  bool CASTWMPermissionsMap::contains(const std::string & _id) const {
    
    lockMap();
    PermissionsMap::const_iterator i = m_permissionsMap.find(_id);    
    //default false
    bool ret = false;
    //if it's still in the map then only return true if not scheduled
    //for deletion
    if(i != m_permissionsMap.end()) {
      ret = !(i->second.m_scheduledForDeletion);
    }    
    unlockMap();
    
    return ret;
  }

  /**
   * Gets the permissions for the given entry.
   * 
   * @param _id
   * @return
   */
  cdl::WorkingMemoryPermissions CASTWMPermissionsMap::getPermissions(const std::string & _id) const {
    
    lockMap();
    PermissionsMap::const_iterator i = m_permissionsMap.find(_id);       
    if(i == m_permissionsMap.end() || i->second.m_scheduledForDeletion) {
      cout<<"CASTWMPermissionsMap::getPermissions: returning on missing entry"<<_id<<endl;
      unlockMap();
      return cdl::DOESNOTEXIST;
    }

    const cdl::WorkingMemoryPermissions & perms(i->second.m_permissions);
    
    unlockMap();
    
    return perms;
  }

  void CASTWMPermissionsMap::remove(const std::string & _id) {

    lockMap();    
    
    PermissionsMap::iterator i = m_permissionsMap.find(_id);       
    if(i == m_permissionsMap.end()) {
      cout<<"CASTWMPermissionsMap::remove: returning on missing entry"<<_id<<endl;
      unlockMap();
      return;
    }
    //if it's not locked, then no problem
    else if(i->second.m_lockCount == 0) {
      m_permissionsMap.erase(i);
      unlockMap();
      return;
    }
    //if it is locked, then schedule it for deletion
    else {
      i->second.m_scheduledForDeletion = true;
    }
    
    
    //if we get here, then the mutex is locked by someone... uhoh
    //get the mutex and clean up while we are in lock
    pthread_mutex_t * mutex(&(i->second.m_mutex));    
    while(i->second.m_lockCount > 0) {
      //unlock for the block
      unlockMap();
      //try to lock it
      lockMutex(mutex);
      //then lock up again when we have it
      lockMap();
      //try to lock it
      unlockMutex(mutex);
      //need  to get a new iterator
      i = m_permissionsMap.find(_id);
    }

    //once we hget here the map is locked and the lock count is 0, so
    //we're good to go again
    m_permissionsMap.erase(i);
    unlockMap();       
  }
    
  const std::string & CASTWMPermissionsMap::getLockHolder(const std::string & _id) const {
    
    lockMap();
    PermissionsMap::const_iterator i = m_permissionsMap.find(_id);           
    assert (i != m_permissionsMap.end());    
    const std::string & holder(i->second.m_owner);    
    unlockMap();
    
    return holder;
  }
  
    
} 
