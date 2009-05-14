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

#ifndef CAST_DATA_LOCAL_CACHE_HPP_
#define CAST_DATA_LOCAL_CACHE_HPP_

#include <boost/shared_ptr.hpp>
#include <boost/concept_check.hpp>
#include <map>

namespace cast {

/// caches CASTData (of type \p CastT) and stores it as a local class
/// type (\p LocalT). The TranslatorFct is a functor that should take
/// a boost::shared_ptr<const CastT> and a const
/// string& (the ID, since sometimes it makes sense to store this in
/// the local type, you can ignore this in your TranslatorFct) and
/// return a LocalT-object
template<class CastT, 
	 class LocalT, 
	 class TranslatorFct>
class LocalCachedCASTData{
  BOOST_CLASS_REQUIRE(LocalT, boost, CopyConstructibleConcept);
//  typedef boost::shared_ptr<const CastT> CastTPtr;
//  BOOST_CLASS_REQUIRE3(TranslatorFct, LocalT, CastTPtr, boost, UnaryFunctionConcept);

public:

  /// loads the data from component \p _component from WM address
  /// _id (from the subarch of the component). 
  LocalCachedCASTData(WorkingMemoryReaderProcess& _component, 
		      const std::string& _id, 
		      const TranslatorFct& _translateFct) 
    : m_version(-1),
      m_component(_component),
      m_id(_id),
      m_translateFct(_translateFct)
  {
    //    _updateData();
    //std::cout << "LocalCachedCASTData(/*std*/) called\n";
  }
  /// loads the data from component \p _component from WM address
  /// _id (from the subarch of the component). 
  LocalCachedCASTData(WorkingMemoryReaderProcess& _component, 
		      const std::string& _id, 
		      const TranslatorFct& _translateFct,
		      const boost::shared_ptr<const std::string>& _subarchitectureID) 
    : m_version(-1),
      m_component(_component),
      m_id(_id),
      m_subarchitectureID(_subarchitectureID),
      m_translateFct(_translateFct)
  {
    //    _updateData();
    //std::cout << "LocalCachedCASTData(/*std*/) called\n";
  }

  /// copy constructor
  LocalCachedCASTData(const LocalCachedCASTData& _l) 
    : m_castData(_l.m_castData),
      m_localData(_l.m_localData),
      m_version(_l.m_version),
      m_component(_l.m_component),
      m_id(_l.m_id),
      m_subarchitectureID(_l.m_subarchitectureID),
      m_translateFct(_l.m_translateFct)
  {
    //    _updateData();
    assert(m_version == -1); // i.e. this should only be called during
			     // creation, due to the implementation of
			     // std::map
    //std::cout << "LocalCachedCASTData(const LocalCachedCASTData& _l) called\n";
  }
  
  void operator=(const LocalCachedCASTData& _t) {this->LocalCachedCASTData(_t);}


  /// updates the cached data if necessary and returns the resulting
  /// pointer to the CASTData representation
  boost::shared_ptr<const CastT> getCASTPtr() {
    _updateData();
    return m_castData;
  };

  /// updates the cached data if necessary and returns the resulting
  /// pointer to the CASTData representation
  boost::shared_ptr<LocalT> getLocalPtr() {
    _updateData();
    return m_localData;
  };

  /// returns \p (get())
  const LocalT& operator*() {
    return this->get();
  };
  
  const LocalT& get() {
    _updateData();
    return *m_localData;
  }

  const std::string& getID() const {
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


  int getVersion() const {
    return m_version;
  };
  
  /// just a shortcut that can be handy
  bool existsOnWorkingMemory() const {
    return const_cast<cast::WorkingMemoryReaderProcess>(m_component).existsOnWorkingMemory(m_id);
  }
      
private:

  /// the locally cached data
  boost::shared_ptr<const CastT> m_castData;
  /// the locally cached data, translated from m_castData
  //std::auto_ptr<const LocalT> m_localData;
  boost::shared_ptr<LocalT> m_localData;
  
  /// the version number of the last time the data was loaded
  int m_version;
  
  /// should be const... but later
  cast::WorkingMemoryReaderProcess& m_component;

  /// the address of the stored data
  const std::string m_id;

  /// the subarchitectur of the stored data, if NULL, then m_component.m_subarchtectureID is used
  boost::shared_ptr<const std::string> m_subarchitectureID;

  /// a reference to the relevant translator functor to be used
  const TranslatorFct& m_translateFct;
  
  /// updates the local data if necessary
  void _updateData() throw(cast::DoesNotExistOnWMException, 
			   cast::SubarchitectureProcessException) {
    //std::cout << "_updateData() called\n";
    const std::string& subarch(subarchitectureID());
    if(!m_component.existsOnWorkingMemory(m_id,subarch)) {
      throw cast::DoesNotExistOnWMException(m_component.makeAdress(subarch,m_id),
					    __HERE__, 
					    "CachedCASTData::_updateData: Entry does not exist on wm. Was looking for id: %s in subarchitecture %s", 
					    m_id.c_str(),
					    subarch.c_str());
    }
    int new_version = m_component.getVersionNumber(m_id, subarch);
    //m_component.log("CACHE: new_version: " + lexical_cast<std::string>(new_version) + " m_version: " + lexical_cast<std::string>(m_version));
    if(m_version < new_version) {
      boost::shared_ptr<const cast::CASTData<CastT> > data_ptr(m_component.getWorkingMemoryEntry<CastT>(m_id, subarch));
      m_castData = data_ptr->getData();
      // this is the central little "trick", the CAST data is
      // trsnformed into the local type by the functor
      //LocalT loc = m_translateFct(m_castData);
      //LocalT& loc2(loc);
      //m_localData = std::auto_ptr<LocalT>(new LocalT(loc2));
      m_localData = boost::shared_ptr<LocalT>(new LocalT(m_translateFct(m_castData, m_id, subarch, new_version)));
      //m_localData = new LocalT(m_translateFct(m_castData));
      m_version = new_version;
    }
  }
};

/// maps to lots of LocalCachedCASTData. The TranslatorFct is a
/// functor that should take an CastT-object and return a
/// LocalT-object (is passed to LocalCachedCASTData as a const
/// reference)
template<class CastT, 
	 class LocalT, 
	 class TranslatorFct>
class CASTDataLocalCache {
  BOOST_CLASS_REQUIRE(LocalT, boost, CopyConstructibleConcept);

public:
  typedef LocalCachedCASTData<CastT,LocalT,TranslatorFct> LCCDType;
  typedef typename StringMap<LCCDType>::map CacheType;

  /// creates a cache for \p _component. \p _debugstr just for debugging purposes
  CASTDataLocalCache(WorkingMemoryReaderProcess& _component, 
		     const TranslatorFct& _translateFct,
		     const std::string& _debugstr = "")
    : m_component(_component),
      m_translateFct(_translateFct) {
    //    cout << "CREATING a LOCAL CACHE for " << _component.subarchitectureID() 
    //    	 << " (" << _debugstr << ")" <<  endl;
  }
  
  /// returns a reference to the cached data. OBS, this data must not
  /// be removed by \p erase or \garbageCollect as long as it is in
  /// scope. If you need to store the data over longer periods, get
  /// the shared pointer instead (\getPtr).
  LCCDType& get(const std::string& _id) {
    typename CacheType::iterator pos(m_cache.find(_id));
    if(pos == m_cache.end()) {
      //LCCDType lccd(this->m_component,_id,this->m_translateFct);
      //std::pair<std::string,LCCDType> p(_id,lccd);
      if(m_subarchitectureID.get()) // either used shared pointer to the subarchID...
	pos = (this->m_cache.insert(std::make_pair(_id,LCCDType(this->m_component,_id,this->m_translateFct,m_subarchitectureID)))).first;
      else
	pos = (this->m_cache.insert(std::make_pair(_id,LCCDType(this->m_component,_id,this->m_translateFct)))).first;
      //pos = (this->m_cache.insert(p)).first;
    }
    //    assert(pos == m_cache.find(_id));
    return pos->second;
  }

  /// returns \p get(_id)
  const LocalT& operator[](const std::string& _id) {
    return *get(_id);
  }
  
  /// equivalent to get(_id).getPtr()
  boost::shared_ptr<const CastT> getCASTPtr(const std::string& _id) {
    return this->get(_id).getCASTPtr();
  }

  /// removes the locally cached data at address \p _id (which means
  /// it needs to be reloaded if later requested again)
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
  const TranslatorFct m_translateFct;

  /// used internally by \p garbageCollect, removes element at \p
  /// _pos, returns the next position (and \p _pos is also updated itself
  /// since it's a reference)
  typename CacheType::iterator
  _garbageCollect(typename CacheType::iterator& _pos) {    
    if(const_cast<cast::WorkingMemoryReaderProcess&>(m_component).existsOnWorkingMemory(_pos->first)) {
      return _pos++;
    }
    m_cache.erase(_pos++);
    return _pos;
  }
  
};

} // namespace cast

#endif // CAST_DATA_LOCAL_CACHE_HPP_
