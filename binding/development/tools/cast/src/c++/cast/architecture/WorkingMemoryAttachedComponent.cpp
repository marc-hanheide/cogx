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

#include "WorkingMemoryAttachedComponent.hpp"
#include "SubarchitectureWorkingMemoryProtocol.hpp"

#include <sstream>

using namespace std;
using namespace boost;

namespace cast {

  using namespace cdl;

  WorkingMemoryAttachedComponent::WorkingMemoryAttachedComponent(const string &_id) 
    : //InspectableComponent(_id),
    LocalWorkingMemoryAttachedComponent(_id),
    m_pOutputFromRemoteWorkingMemory(NULL),
    m_pOutputFromLocalWorkingMemory(NULL) {    
  }


  void 
  WorkingMemoryAttachedComponent::setPullConnector(const string & _connectionID, 
					       PullConnectorOut<cdl::WorkingMemoryEntryList> * _pOut) {
    m_pOutputFromRemoteWorkingMemory = _pOut;
  }
  void 
  WorkingMemoryAttachedComponent::setPullConnector(const string & _connectionID, 
					       PullConnectorOut<vector<CASTWorkingMemoryEntry *> > * _pOut) {
    m_pOutputFromLocalWorkingMemory = _pOut;
  }

  bool 
  WorkingMemoryAttachedComponent::existsOnWorkingMemory(const cdl::WorkingMemoryAddress & _wma) {
    return existsOnWorkingMemory(string(_wma.m_id),
				 string(_wma.m_subarchitecture));
  }

  bool 
  WorkingMemoryAttachedComponent::existsOnWorkingMemory(const string & _id, 
							const string & _subarch) {

    assert(!_id.empty());//id must not be empty
    assert(!_subarch.empty());//subarch must not be empty

    if (_subarch != m_subarchitectureID) {
      return existsOnWorkingMemoryXarch(_id, _subarch);
    }
    else {
      return LocalWorkingMemoryAttachedComponent::existsOnWorkingMemory(_id);
    }
  }

  bool 
  WorkingMemoryAttachedComponent::existsOnWorkingMemoryXarch(const string & _id, 
							     const string & _subarch) {

    string query = SubarchitectureWorkingMemoryProtocol
      ::createExistsQuery(getProcessIdentifier(),m_subarchitectureID,_subarch,_id);
    //println("subarch: " + _subarch);
    //println("query: " + query);
  
    vector< shared_ptr< const CASTData<bool> > > results;
    
    queryWorkingMemory(_subarch,query,results);
  
    //printfln("query count: %i", results.size());
  
    if(1 != results.size()) {      
      throw CASTException(__HERE__, "Incorrect count returned from working memory. Count = %i",results.size());
    }
 
    return *(results[0]->getData());
  }







  int WorkingMemoryAttachedComponent::getVersionNumber(const cdl::WorkingMemoryAddress & _wma) {
    return getVersionNumber(string(_wma.m_id),string(_wma.m_subarchitecture));
  }



  int WorkingMemoryAttachedComponent::getVersionNumberXarch(const string & _id, 
							    const string & _subarch) {
    //logReadEvent(_subarch,_id);

    string query = SubarchitectureWorkingMemoryProtocol
      ::createOverwriteCountQuery(getProcessIdentifier(),m_subarchitectureID,_subarch,_id);
    //println("subarch: " + _subarch);
    //println("query: " + query);
  
    vector< shared_ptr< const CASTData<int> > > results;
    
    queryWorkingMemory(_subarch,query,results);
  
    //printfln("query count: %i", results.size());
  
    if(1 != results.size()) {      
      throw CASTException(__HERE__, "Incorrect count returned from working memory. Count = %i",results.size());
    }

    return *(results[0]->getData());
  }
 
 
  int WorkingMemoryAttachedComponent::getVersionNumber(const string & _id, 
						       const string & _subarch) {

    assert(!_id.empty());//id must not be empty
    assert(!_subarch.empty());//subarch must not be empty

    // logReadEvent(_subarch, _id);
    int versionNumber;
    if (_subarch != m_subarchitectureID) {
      versionNumber = getVersionNumberXarch(_id, _subarch);
      if (-1 == versionNumber) {
	throw(DoesNotExistOnWMException(makeAdress(_subarch,_id),
					__HERE__, 
					"Entry has never existed on wm. Was looking in subarch %s for id %s", 
					_subarch.c_str(),_id.c_str()));
      }
    }
    else {
      //nah: why on earth does this need to be qualified?
      versionNumber = LocalWorkingMemoryAttachedComponent::getVersionNumber(_id);
    }

    return versionNumber;
  }



  bool 
  WorkingMemoryAttachedComponent::haveLatestVersion(const std::string &_id, 
						    const std::string &_subarch)
    throw(ConsistencyException, DoesNotExistOnWMException) {

    assert (isVersioned(_id));
    
    int ownedVersion = getStoredVersionNumber(_id);
    int wmVersion = getVersionNumber(_id, _subarch);
    
    debug("haveLatestVersion(%s,%s): %d == %d", _id.c_str(), _subarch.c_str(), wmVersion, ownedVersion);

    return wmVersion == ownedVersion;
  }

  void 
  WorkingMemoryAttachedComponent::checkConsistency(const string & _id, const string & _subarch)
    throw(ConsistencyException, DoesNotExistOnWMException) {


    if (!isVersioned(_id)) {
      throw(ConsistencyException(makeAdress(_subarch,_id),
				 __HERE__,
				 "!isVersioned(%s) in subarch %s. ",
				 _id.c_str(), _subarch.c_str()));
    }

    if (!haveLatestVersion(_id,_subarch)) {
      throw(ConsistencyException(makeAdress(_subarch,_id),
				 __HERE__,
				 "%s has attempted to overwrite an outdated working memory entry. Please reread and try again. WMA: %s:%s. Local version: %d. WM version: %d",
				 getProcessIdentifier().c_str(), _id.c_str(), _subarch.c_str(), getStoredVersionNumber(_id), getVersionNumber(_id, _subarch)));
    }
    
  }
    
  void 
  WorkingMemoryAttachedComponent::checkConsistency(const WorkingMemoryAddress &_wma) 
    throw(ConsistencyException, DoesNotExistOnWMException) {
    checkConsistency(string(_wma.m_id), string(_wma.m_subarchitecture));
  }


  void WorkingMemoryAttachedComponent::lockEntry(const std::string & _id, 
						 const std::string & _subarch,
						 const cdl::WorkingMemoryPermissions & _permissions)
    throw(DoesNotExistOnWMException) {
    if (_subarch == getSubarchitectureID()) {
      //TODO why is this prefix needed??
      LocalWorkingMemoryAttachedComponent::lockEntry(_id, _permissions);
    } else {
      lockEntryHelper(_id, _subarch, _permissions,
		      cdl::BLOCKING);
    }
  }
  
  void WorkingMemoryAttachedComponent::lockEntry(const cdl::WorkingMemoryAddress & _wma,
						 const cdl::WorkingMemoryPermissions & _permissions)
    throw(DoesNotExistOnWMException) {
    lockEntry(std::string(_wma.m_id), 
	      std::string(_wma.m_subarchitecture),
	      _permissions);
  }

  void WorkingMemoryAttachedComponent::unlockEntry(const std::string & _id, 
						   const std::string & _subarch)
    throw(DoesNotExistOnWMException) {
    if (_subarch == getSubarchitectureID()) {
      //TODO why is this prefix needed??
      LocalWorkingMemoryAttachedComponent::unlockEntry(_id);
    } 
    else {
      unlockEntryHelper(_id, _subarch);
    }
  }
  
  void WorkingMemoryAttachedComponent::unlockEntry(const cdl::WorkingMemoryAddress & _wma)
    throw(DoesNotExistOnWMException) {
    unlockEntry(std::string(_wma.m_id), 
		std::string(_wma.m_subarchitecture));
  }


  bool WorkingMemoryAttachedComponent::tryLockEntry(const std::string & _id, 
						    const std::string & _subarch,
						    const cdl::WorkingMemoryPermissions & _permissions)
    throw(DoesNotExistOnWMException) {
    if (_subarch == getSubarchitectureID()) {
      //TODO why is this prefix needed??
      return LocalWorkingMemoryAttachedComponent::tryLockEntry(_id, _permissions);
    } else {
      return lockEntryHelper(_id, _subarch, _permissions,
			     cdl::BLOCKING);
    }
  }
  
  bool WorkingMemoryAttachedComponent::tryLockEntry(const cdl::WorkingMemoryAddress & _wma,
						    const cdl::WorkingMemoryPermissions & _permissions)
    throw(DoesNotExistOnWMException) {
    return tryLockEntry(std::string(_wma.m_id), 
			std::string(_wma.m_subarchitecture),
			_permissions);
  }



  cdl::WorkingMemoryPermissions 
  WorkingMemoryAttachedComponent::getPermissions(const std::string &_id,
						 const std::string &_subarch) const
    throw(DoesNotExistOnWMException)  {
    return getPermissionsHelper(_id, _subarch);
  }
  
  cdl::WorkingMemoryPermissions 
  WorkingMemoryAttachedComponent::getPermissions(const cdl::WorkingMemoryAddress &_wma) const
    throw(DoesNotExistOnWMException) {
      return getPermissionsHelper(std::string(_wma.m_id), std::string(_wma.m_subarchitecture));
  }
  
  bool 
  WorkingMemoryAttachedComponent::isOverwritable(const std::string & _id, 
						 const std::string & _subarch) const
    throw(DoesNotExistOnWMException) {

    if(holdsOverwriteLock(_id, _subarch)) {
      return true;
    }
    else {
      WorkingMemoryPermissions permissions = getPermissions(_id, _subarch);
      ostringstream outStream;
      outStream<<permissions;
      debug("WorkingMemoryPermissions::isOverwritable: %s",outStream.str().c_str());
      return overwriteAllowed(permissions);
    }
  }
    
  
  bool 
  WorkingMemoryAttachedComponent::isDeletable(const std::string & _id, 
					      const std::string & _subarch) const
    throw(DoesNotExistOnWMException) {
    
    if(holdsDeleteLock(_id, _subarch)) {
      return true;
    }
    else {
      WorkingMemoryPermissions permissions = getPermissions(_id, _subarch);
      return deleteAllowed(permissions);
    }
  }
    
  
  bool 
  WorkingMemoryAttachedComponent::isReadable(const std::string & _id, 
					     const std::string & _subarch) const
    throw(DoesNotExistOnWMException) {
    
    if(holdsReadLock(_id, _subarch)) {
      return true;
    }
    else {
      WorkingMemoryPermissions permissions = getPermissions(_id, _subarch);
      return readAllowed(permissions);
    }
  }
  


}
