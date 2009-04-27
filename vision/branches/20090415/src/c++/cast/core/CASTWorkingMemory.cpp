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

#include "CASTWorkingMemory.hpp"
#include <CASTUtils.hpp>


using namespace std;
using namespace cast::cdl;

namespace cast {



  CASTWorkingMemory::CASTWorkingMemory() {

#ifdef SYNC_MEMORY_ACCESS

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

    err = pthread_mutex_init(&m_sync, &attr);
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed to create mutex: %s", strerror(err)));
    }

    err = pthread_mutexattr_destroy(&attr);
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed to create mutex: %s", strerror(err)));
    }

#endif

  }


  /**
   * Empty   destructor.
   */
  CASTWorkingMemory::~CASTWorkingMemory() {
    //TODO memory leak: delete stored objects
  }


  void CASTWorkingMemory::lock()  throw (CASTException) {
#ifdef SYNC_MEMORY_ACCESS
    //lock mutex for wait
    int err = pthread_mutex_lock(&m_sync);
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed mutex lock: %s", strerror(err)));
    }
#endif
  }

  void CASTWorkingMemory::unlock()  throw (CASTException) {
#ifdef SYNC_MEMORY_ACCESS
    int err = pthread_mutex_unlock(&m_sync); 
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed mutex unlock: %s", strerror(err)));
    }
#endif
  }

  /**
   * Adds item to working memory with given id. Does not overwrite
   * data if id already exists.
   * 
   * @param _id
   *            The id of the entry.
   * @param _type
   *            The ontological type of the entry.
   * @param _data
   *            The data itself.
   * @return Returns true if the item is added (i.e. not a duplicate
   *         id)
   */

  bool CASTWorkingMemory::add(const string & _id, 
			      WorkingMemoryEntryPtr _pData) {
  
    assert(_pData->version == 0);

    lock();

    //check whether the id exists
    WMItemMap::iterator i = m_storage.find(_id);
  
    //if(_pData == NULL) {
    //cerr<<"adding null item into wm"<<endl;
    //}
    bool added = false;

    //if the id doesn't exist
    if(i == m_storage.end()) {
    
      //store the data
      m_storage[_id] = _pData;
    
      //store the key in addition order
      m_ids.push_front(_id);

      //     cout<<endl;
      //     cout<<"CASTWorkingMemory data ptr: count: "<<_pData.use_count()<<endl;
      //     cout<<endl;

      //needed for bug #52
      //check to see if we stored it previously
      VersionMap::const_iterator j = m_lastVersions.find(_id);
      if(j != m_lastVersions.end()) {

	//if stored before, new version is last version + 1
	m_storage[_id]->version = (j->second + 1);

	//cout<<"updated stored version from last version: "<<getOverwriteCount(_id)<<endl;
	//erase last version from map
	m_lastVersions.erase(_id);
      }
      
      added = true;
    }

    unlock();

    return added;

  }

  /**
   * Overwrites item with given id. Does not do anything if id does
   * not exist.
   * 
   * @param _id
   *            The id of the entry.
   * @param _type
   *            The ontological type of the entry.
   * @param _data
   *            The data itself.
   * @return Returns true if the id exists for overwriting
   */
  bool CASTWorkingMemory::overwrite(const string &  _id, 
				    WorkingMemoryEntryPtr _pData) {

    lock();

    //check whether the id exists
    WMItemMap::iterator i = m_storage.find(_id);
    bool overwritten = false;

    //if the id exists
    if(i != m_storage.end()) {
    
      //increment overwrite version
      _pData->version = i->second->version + 1;
      //store the data in the same place in the map
      i->second = _pData; 

      //move key to front of list
      m_ids.remove(_id);
      m_ids.push_front(_id);

      overwritten = true;
      //    cout<<"data overwritten"<<endl;
    }

    unlock();

    return overwritten;   
  }

  /**
   * Removes the item with the given id.
   * 
   * @param _id
   *            The id of the entry.
   * @return Returns true if the id is removed.
   */
  WorkingMemoryEntryPtr
  CASTWorkingMemory::remove(const string &  _id) {
  
    lock();

    //check whether the id exists
    WMItemMap::iterator i = m_storage.find(_id);
    WorkingMemoryEntryPtr pItem; //starts NULL
      
    //if the id exists
    if(i != m_storage.end()) {

      pItem = i->second;
    
      //and remove from storage
      m_storage.erase(i);

      //remove key 
      m_ids.remove(_id);

      //store last version
      m_lastVersions[_id] = pItem->version;      
    }

    unlock();

    return pItem;

  }

  /**
   * Get the item with the given id.
   * 
   * @param _id
   *            The specified item or null if it does not exist.
   */
  WorkingMemoryEntryPtr
  CASTWorkingMemory::get(const string &  _id) {

    lock();

    //check whether the id exists
    WMItemMap::iterator i = m_storage.find(_id);
    WorkingMemoryEntryPtr item; //UPGRADE check starts NULL
    
    //if the id exists
    if(i != m_storage.end()) {
      item = i->second;
    }

    //cerr<<"entry not in wm: "<<_id<<endl;
    unlock();
    
    return item;

  }
  

  bool CASTWorkingMemory::contains(const string &  _id) {

    lock();
    //cout<<"contains: "<<_id<<endl;
    //check whether the id exists
    WMItemMap::iterator i = m_storage.find(_id);
    //cout<<"contains: "<<(i != m_storage.end())<<endl;
    bool contains = (i != m_storage.end());

    unlock();

    return contains;
  }

  bool CASTWorkingMemory::hasContained(const string &  _id) {    
 
    lock();

    bool hasContained = false;

    //first check whether we contain it now
    WMItemMap::iterator i = m_storage.find(_id);    
    if(i != m_storage.end()) {
      hasContained = true;
    }
    else {
      //check whether the id exists
      VersionMap::iterator i = m_lastVersions.find(_id);
      hasContained = (i != m_lastVersions.end());
    }

    unlock();

    return hasContained;

  }



  /**
   */
  int CASTWorkingMemory::getOverwriteCount(const string &  _id) {

    lock();

    int count = -1; //TODO smarter default?

    //check whether the id exists
    WMItemMap::iterator i = m_storage.find(_id);
    if(i != m_storage.end()) {
      //cout<<"stored version: "<<_id<<" "<<i->second->version<<endl;
      count =  i->second->version;
    }
    else {
      VersionMap::const_iterator j = m_lastVersions.find(_id);
      if(j != m_lastVersions.end()) {
	//cout<<"last version: "<<_id<<" "<<j->second<<endl;
	count =  j->second;
      }
    }

    //cout<<"never version: "<<_id<<" "<<-1<<endl;
    unlock();

    //TODO make smarter?
    return count;
  }


  /**
   * Gets the n most recent ids of entries with the given type.
   * 
   * @param _type
   *            The type to check.
   * @param _count
   *            The number of ids to return. If 0 all matching items
   *            are return.
   * @return All matching items.
   */
  void
  CASTWorkingMemory::getIDsByType(const string & _type,
				  const int & _count,
				  vector<string> &_ids) {

    lock();  

    int total; 

    if (_count == 0) {
      total = m_storage.size();
    }
    else {
      total = _count;
    }

    int count = 0;

    for(StringList::iterator i = m_ids.begin();
	i != m_ids.end();
	i++) {

      WMItemMap::iterator j = m_storage.find(*i);

      if(j != m_storage.end()) {
	if(j->second->type == _type) {
	  _ids.push_back(*i);
	  count++;
	}

	//will always be at least 1
	if(count == total) {
	  break;
	}
      }
      else {
	assert(false);
      }
    }

    unlock();

  }



  /**
   * Get all working memory entries with given type.
   * 
   * @param _type
   *            The type to check.
   * @return All matching items from working memory.
   */
  void
  CASTWorkingMemory::getByType(const string & _type,
			       vector< WorkingMemoryEntryPtr > & _items) {
    //get all matching items
    getByType(_type,m_storage.size(),_items);
  } 



  /**
   * Get a collection of memory items with the given type.
   * 
   * @param _type
   *            The type to check.
   * @param _count
   *            The number of entries to return. If 0 all matching
   *            entries are returned.
   * @return A collection of matching entries.
   */
  void
  CASTWorkingMemory::getByType(const string & _type, 
			       const int & _count,
			       vector< WorkingMemoryEntryPtr > & _items)  {

    lock();

    int total; 

    if (_count == 0) {
      total = m_storage.size();
    }
    else {
      total = _count;
    }

    int count = 0;

    for(StringList::iterator i = m_ids.begin();
	i != m_ids.end();
	i++) {

      WMItemMap::iterator j = m_storage.find(*i);

      if(j != m_storage.end()) {

	if(j->second->type == _type) {
	  _items.push_back(j->second);
	  //cout<<"returning id: "<<*i<<endl;
	  count++;
	}

	//will always be at least 1
	if(count == total) {
	  break;
	}
      }
      else {
	assert(false);
      }
    }
    unlock();
  }


  void 
  CASTWorkingMemory::debug() {
//     lock();
//     cout<<"pointer counts"<<endl;
//     for(WMItemMap::iterator i = m_storage.begin();
// 	i != m_storage.end();
// 	i++) {
//       cout<<i->first<<" count "<<i->second.use_count()<<endl;
//       //cout<<i->first<<" count "<<i->second->data().use_count()<<endl;

//     }
//    unlock();
  }
}  // namespace cast
