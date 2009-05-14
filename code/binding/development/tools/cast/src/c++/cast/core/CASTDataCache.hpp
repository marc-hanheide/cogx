/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Henrik Jacobsson
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

#ifndef CACHED_CAST_DATA_HPP_
#define CACHED_CAST_DATA_HPP_

#include <cast/architecture/WorkingMemoryReaderProcess.hpp>
#include <boost/shared_ptr.hpp>
#include <map>

namespace cast {


/// loads and keeps track of the version of cached data. All is now
/// made private since the CASTCache should be used.
template<class T>
class CachedCASTData{
public:

  /// loads the data from component \p _component from WM address
  /// _id (from the subarch of the component)
  CachedCASTData(cast::WorkingMemoryReaderProcess& _component, 
		 const std::string& _id) 
    : m_version(-1),
      m_component(_component),
      m_id(_id)
  {
//    _updateData();
  }
  
  /// loads the data from component \p _component from WM address
  /// _id (from the subarch specified in \p _subarchitectureID)
  CachedCASTData(cast::WorkingMemoryReaderProcess& _component, 
		 const std::string& _id, 
		 const boost::shared_ptr<const std::string>& _subarchitectureID) 
    : m_version(-1),
      m_component(_component),
      m_id(_id),
      m_subarchitectureID(_subarchitectureID)
  {
//    _updateData();
  }
  
  /// updates the cached data if necessary and returns the result
  boost::shared_ptr<const T> getPtr() {
    _updateData();
    return m_data;
  };

  /// returns \p (*get())
  const T& operator*() {
    return *(this->getPtr());
  };

  /// returns \p (*get())
  const T* operator->() {
    return this->getPtr().get();
  };
  
  const std::string& id() const {
    return m_id;
  };

  /// returns the correct subarchitectureID, irrespectively of whether
  /// the local or an external subarchitecture is used
  inline const std::string& subarchitectureID() const {
    if(m_subarchitectureID.get())
      return *m_subarchitectureID;
    return m_component.subarchitectureID();
  };
  /// sets the subarchitectureID so that an external WM can be
  /// cached. The default is otherwise the local WM.
  void setSubarchitectureID(const boost::shared_ptr<const std::string>& _id) {
    m_subarchitectureID = _id;
  };
  /// same as \p setSubarchitectureID(const shared_ptr<const
  /// std::string>& _id), but it also allocated \p m_subarchitectureID
  void setSubarchitectureID(const std::string& _id)  {
    m_subarchitectureID = boost::shared_ptr<const std::string>(new std::string(_id));
  };
  
  /// the number of times the cache believes the cached data has been
  /// altered on the WM.
  int getVersion() const {
    return m_version;
  };

  /// compares the addresses, used for sorting sets of cached data
  bool operator<(const CachedCASTData<T>& _cd) const {

    //sort by subarchitecture then id
    if (this->subarchitectureID() != _cd.subarchitectureID()) {
      this->subarchitectureID() < _cd.subarchitectureID();
    }
    return this->m_id < _cd.m_id;    
  }

  /// just a shortcut that can be handy
  bool existsOnWorkingMemory() const {
    return const_cast<cast::WorkingMemoryReaderProcess>(m_component).existsOnWorkingMemory(m_id, subarchitectureID());
  }
  
    
private:

  /// the locally cached data
  boost::shared_ptr<const T> m_data;
  
  /// the version number of the last time the data was loaded
  int m_version;
  
  /// should be const... but later
  cast::WorkingMemoryReaderProcess& m_component;

  /// the address of the stored data
  const std::string m_id;

  /// the subarchitectur of the stored data, if NULL, then m_component.m_subarchtectureID is used
  boost::shared_ptr<const std::string> m_subarchitectureID;

  /// updates the local data if necessary
  void _updateData() throw(cast::DoesNotExistOnWMException, 
			   cast::SubarchitectureProcessException) {
    const std::string& subarch(subarchitectureID());
    if(!m_component.existsOnWorkingMemory(m_id, subarch)) {
      m_data = boost::shared_ptr<const T>(); // deletes the local content
      throw cast::DoesNotExistOnWMException(m_component.makeAdress(subarch,m_id),
					    __HERE__, 
					    "CachedCASTData::_updateData: Entry does not exist on wm. Was looking for id: %s for subarchitecture %s", 
					    m_id.c_str(),
					    subarch.c_str());
    }
    int new_version = m_component.getVersionNumber(m_id, subarch);
    //m_component.log("CACHE: new_version: " + lexical_cast<std::string>(new_version) + " m_version: " + lexical_cast<std::string>(m_version));
    if(m_version < new_version) {
      boost::shared_ptr<const cast::CASTData<T> > data_ptr(m_component.getWorkingMemoryEntry<T>(m_id, subarch));
      m_data = data_ptr->getData();
      m_version = new_version;
    }
  }  
};


/// Caching of data from WM. Use the operator[](string) for easy and
/// efficient access to WM. Only access the given subarchitecture.
template <class T> 
class CASTDataCache {
public:
  typedef CachedCASTData<T> CachedCASTDataType;
  typedef typename StringMap<CachedCASTDataType>::map CacheType;
  //typedef typename std::map<std::string,CachedCASTDataType> CacheType;

  /// creates a cache for \p _component. \p _debugstr just for debugging purposes
  CASTDataCache(cast::WorkingMemoryReaderProcess& _component, const std::string& _debugstr = "")
    : m_component(_component)
  {
    //  cout << "CREATING a CACHE for " << _component.subarchitectureID() 
    //	 << " (" << _debugstr << ")" <<  endl;
    //    sleep(1);
  }

  
  /// returns a reference to the cached data. OBS, this data must not
  /// be removed by \p erase or \garbageCollect as long as it is in
  /// scope. If you need to store the data over longer periods, get
  /// the shared pointer instead (\getPtr).
  CachedCASTData<T>& get(const std::string& _id) {
    typename CacheType::iterator pos(m_cache.find(_id));
    if(pos == m_cache.end()) {
      //std::cout << "SEARCHING: " << _id << std::endl;      
      if(m_subarchitectureID.get()) // either used shared pointer to the subarchID...
	pos = (m_cache.insert(make_pair(_id, CachedCASTDataType(m_component,_id, m_subarchitectureID)))).first;
      else // ... or, if local, let the cached data be local too 
	pos = (m_cache.insert(make_pair(_id, CachedCASTDataType(m_component,_id)))).first;
    }
    //    assert(pos == m_cache.find(_id));
    return pos->second;
  }

  /// returns \p get(_id)
  const T& operator[](const std::string& _id) {
    return *get(_id);
  }
  
  /// equivalent to get(_id).getPtr()
  boost::shared_ptr<const T> getPtr(const std::string& _id) {
    return this->get(_id).getPtr();
  }

  /// removes the locally cached data at address \p _id (which means
  /// it needs to me reloaded if later requested again)
  void purge(const std::string& _id) {
    m_cache.erase(_id);
  }

  /// removes all locally cached data
  void purgeAll() {
    m_cache.clear();
  }
  
  /// removes the cached data if it is also removed on WM,
  /// otherwised it does nothing.
  void garbageCollect(const std::string& _id) {
    typename CacheType::iterator pos = m_cache.find(_id);
    if(_id != m_cache.end())
      _garbageCollect(pos);
  }
  
  /// garbage collects all cached data. Preferrable use is to call the
  /// garbage collector for specific ids first. Even more efficient is
  /// to call the \p erase if you really know that an element is
  /// erased, as this save one O(log N) lookup.
  void garbageCollect() {
    typename CacheType::iterator pos;
    for(pos = m_cache.begin() ; pos != m_cache.end() ; ) {
      pos = _garbageCollect(pos);
    }
  }

  /// returns the correct subarchitectureID, irrespectively of whether
  /// the local or an external subarchitecture is used
  inline const std::string& subarchitectureID() const {
    if(m_subarchitectureID.get())
      return *m_subarchitectureID;
    return m_component.subarchitectureID();
  };
  /// sets the subarchitectureID so that an external WM can be
  /// cached. The default is otherwise the local WM.
  void setSubarchitectureID(const boost::shared_ptr<const std::string>& _id) {
    m_subarchitectureID = _id;
  };
  /// same as \p setSubarchitectureID(const shared_ptr<const
  /// std::string>& _id), but it also allocated \p m_subarchitectureID
  void setSubarchitectureID(const std::string& _id)  {
    m_subarchitectureID = boost::shared_ptr<const std::string>(new std::string(_id));
  };
  
    
private:
  
  cast::WorkingMemoryReaderProcess& m_component;
  boost::shared_ptr<const std::string> m_subarchitectureID;
  CacheType m_cache;

  /// used internally by \p garbageCollect, removes element at \p
  /// _pos, returns the next position (and \p _pos is also updated itself
  /// since it's a reference)
  typename CacheType::iterator
  _garbageCollect(typename CacheType::iterator& _pos) {    
    if(const_cast<cast::WorkingMemoryReaderProcess>(m_component).existsOnWorkingMemory(_pos->first, subarchitectureID())) {
      return _pos++;
    }
    m_cache.erase(_pos++);
    return _pos;
  }
  
};
    
} // namespace cast

#endif // CACHED_CAST_DATA_HPP_
