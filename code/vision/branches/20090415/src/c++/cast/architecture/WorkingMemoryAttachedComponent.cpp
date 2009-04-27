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


using namespace std;
using namespace boost;

namespace cast {

  using namespace cdl;

//   WorkingMemoryAttachedComponent::WorkingMemoryAttachedComponent(const string &_id) 
//     : 
//     SubarchitectureComponent(_id)  {    
//   }


  /**
   * Overrides the configure method from FrameworkProcess to use
   * _config to set the subarchitecture ID.
   * 
   * @param _config
   *            The ID of the subarchitecture which contains this
   *            component.
   */
  void WorkingMemoryAttachedComponent::configureInternal(const std::map<std::string,std::string> & _config) {
    SubarchitectureComponent::configureInternal(_config);
    m_permissions = 
      boost::shared_ptr< CASTComponentPermissionsMap >(new CASTComponentPermissionsMap(getSubarchitectureID()));
  }


  bool 
  WorkingMemoryAttachedComponent::existsOnWorkingMemory(const string & _id) {
    return existsOnWorkingMemory(_id, getSubarchitectureID());
  }



  bool 
  WorkingMemoryAttachedComponent::existsOnWorkingMemory(const cdl::WorkingMemoryAddress & _wma) 
    throw (UnknownSubarchitectureException) {
    return existsOnWorkingMemory(_wma.id,
				 _wma.subarchitecture);
  }

  bool 
  WorkingMemoryAttachedComponent::existsOnWorkingMemory(const string & _id, 
							const string & _subarch) 
    throw (UnknownSubarchitectureException) {

    assert(!_id.empty());//id must not be empty
    assert(!_subarch.empty());//subarch must not be empty

    return m_workingMemory->exists(_id,_subarch);
  }



  int 
  WorkingMemoryAttachedComponent::getVersionNumber(const string & _id) 
    throw (DoesNotExistOnWMException) {
    return getVersionNumber(_id, getSubarchitectureID());
  }  


  int WorkingMemoryAttachedComponent::getVersionNumber(const cdl::WorkingMemoryAddress & _wma) 
    throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {
    return getVersionNumber(_wma.id,_wma.subarchitecture);
  }


  int WorkingMemoryAttachedComponent::getVersionNumber(const string & _id, 
						       const string & _subarch) 
    throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {

    assert(!_id.empty());//id must not be empty
    assert(!_subarch.empty());//subarch must not be empty

    return m_workingMemory->getVersionNumber(_id,_subarch);
  }






  void 
  WorkingMemoryAttachedComponent::storeVersionNumber(const string & _id, int _version) {
    m_versionNumbers[_id] = _version;
  }

  int 
  WorkingMemoryAttachedComponent::getStoredVersionNumber(const string & _id) const
    throw(ConsistencyException) {

    IntMap::const_iterator i = m_versionNumbers.find(_id);

    if (i == m_versionNumbers.end()) {
      throw(ConsistencyException(exceptionMessage(__HERE__, "No stored version for id: %s", _id.c_str()),
				 makeWorkingMemoryAddress(subarchitectureID(),_id)));
    } else {
      return i->second;
    }
  }

  void
  WorkingMemoryAttachedComponent::increaseStoredVersion(const string & _id) throw(ConsistencyException) {
    int stored = getStoredVersionNumber(_id);
    updateVersion(_id, ++stored);
  }

  void 
  WorkingMemoryAttachedComponent::startVersioning(const string & _id) {
    assert (!isVersioned(_id));
    storeVersionNumber(_id, 0);
  }

  void 
  WorkingMemoryAttachedComponent::updateVersion(const string & _id, int _newVersion) {
    storeVersionNumber(_id, _newVersion);
  }

  
  bool
  WorkingMemoryAttachedComponent::isVersioned(const string & _id) const {
    IntMap::const_iterator i = m_versionNumbers.find(_id);
    return i != m_versionNumbers.end();
  }

  void 
  WorkingMemoryAttachedComponent::stopVersioning(const string & _id) {
    if (isVersioned(_id)) {
      removeVersionNumber(_id);
    }
  }

  void 
  WorkingMemoryAttachedComponent::removeVersionNumber(const string & _id) {
    m_versionNumbers.erase(_id);
  }

     
  bool 
  WorkingMemoryAttachedComponent::haveLatestVersion(const string &_id)
    throw(ConsistencyException, DoesNotExistOnWMException) {
    return haveLatestVersion(_id,getSubarchitectureID());
  }

  bool 
  WorkingMemoryAttachedComponent::haveLatestVersion(const std::string &_id, 
						    const std::string &_subarch)
    throw(ConsistencyException, DoesNotExistOnWMException, UnknownSubarchitectureException) {

    assert (isVersioned(_id));
    
    int ownedVersion = getStoredVersionNumber(_id);
    int wmVersion = getVersionNumber(_id, _subarch);
    
    debug("haveLatestVersion(%s,%s): %d == %d", _id.c_str(), _subarch.c_str(), wmVersion, ownedVersion);

    return wmVersion == ownedVersion;
  }


  void 
  WorkingMemoryAttachedComponent::checkConsistency(const string & _id) 
    throw(ConsistencyException, DoesNotExistOnWMException) {
    checkConsistency(_id,getSubarchitectureID());
  }

  void 
  WorkingMemoryAttachedComponent::checkConsistency(const WorkingMemoryAddress &_wma) 
    throw(ConsistencyException, DoesNotExistOnWMException, UnknownSubarchitectureException) {
    checkConsistency(_wma.id, _wma.subarchitecture);
  }


  void 
  WorkingMemoryAttachedComponent::checkConsistency(const string & _id, const string & _subarch)
    throw(ConsistencyException, DoesNotExistOnWMException, UnknownSubarchitectureException) {


    if (!isVersioned(_id)) {
      throw(ConsistencyException(exceptionMessage(__HERE__,
						  "!isVersioned(%s) in subarch %s. ",
						  _id.c_str(), _subarch.c_str()),
				 makeWorkingMemoryAddress(_subarch,_id)));
    }

    if (!haveLatestVersion(_id,_subarch)) {
      throw(ConsistencyException(exceptionMessage(__HERE__,
						  "%s has attempted to overwrite an outdated working memory entry. Please reread and try again. WMA: %s:%s. Local version: %d. WM version: %d",
						  getComponentID().c_str(), _id.c_str(), _subarch.c_str(), 
						  getStoredVersionNumber(_id), getVersionNumber(_id, _subarch)),
				 makeWorkingMemoryAddress(_subarch,_id)));
    }
    
  }
    


  void 
  WorkingMemoryAttachedComponent::lockEntry(const std::string & _id, const cdl::WorkingMemoryPermissions & _permissions)
    throw(DoesNotExistOnWMException) {
    lockEntry(_id, 
	      getSubarchitectureID(), 
	      _permissions);
  }


  void WorkingMemoryAttachedComponent::lockEntry(const cdl::WorkingMemoryAddress & _wma,
						 const cdl::WorkingMemoryPermissions & _permissions)
    throw(DoesNotExistOnWMException, UnknownSubarchitectureException) {
    lockEntry(_wma.id, 
	      _wma.subarchitecture,
	      _permissions);
  }


  void 
  WorkingMemoryAttachedComponent::lockEntry(const std::string & _id, 
					    const std::string & _subarch,
					    const cdl::WorkingMemoryPermissions & _permissions) 
    throw(DoesNotExistOnWMException, UnknownSubarchitectureException) {


    assert(!_id.empty());//id must not be empty
    assert(!_subarch.empty());//id must not be empty
    assert(m_workingMemory);

    //will throw here if doesn't exist
    m_workingMemory->lockEntry(_id,_subarch, getComponentID(),_permissions);
 
    m_permissions->setPermissions(_id, _subarch, _permissions);
  }


  bool 
  WorkingMemoryAttachedComponent::tryLockEntry(const std::string & _id,
					       const cdl::WorkingMemoryPermissions & _permissions) 
    throw(DoesNotExistOnWMException) {
    return tryLockEntry(_id, 
			getSubarchitectureID(),
			_permissions);
  }


  bool WorkingMemoryAttachedComponent::tryLockEntry(const cdl::WorkingMemoryAddress & _wma,
						    const cdl::WorkingMemoryPermissions & _permissions)
    throw(DoesNotExistOnWMException, UnknownSubarchitectureException) {
    return tryLockEntry(_wma.id, 
			_wma.subarchitecture,
			_permissions);
  }


  bool 
  WorkingMemoryAttachedComponent::tryLockEntry(const std::string & _id, 
					       const std::string & _subarch,
					       const cdl::WorkingMemoryPermissions & _permissions) 
    throw(DoesNotExistOnWMException, UnknownSubarchitectureException) {

    assert(!_id.empty());//id must not be empty    
    assert(!_subarch.empty());//id must not be empty
    assert(m_workingMemory);

    //will throw here if doesn't exist
    bool succeeded = m_workingMemory->tryLockEntry(_id,_subarch,getComponentID(),_permissions);
    
    // if we succeeded, then let's store the permissions
    if (succeeded) {
      m_permissions->setPermissions(_id, _subarch, _permissions);
    }
    
    return succeeded;
  }


  void WorkingMemoryAttachedComponent::unlockEntry(const std::string & _id) 
    throw(DoesNotExistOnWMException, ConsistencyException) {
    unlockEntry(_id, getSubarchitectureID());
  }
  
  void WorkingMemoryAttachedComponent::unlockEntry(const cdl::WorkingMemoryAddress & _wma)
    throw(DoesNotExistOnWMException, ConsistencyException, UnknownSubarchitectureException) {
    unlockEntry(_wma.id, 
		_wma.subarchitecture);
  }
  

  void
  WorkingMemoryAttachedComponent::unlockEntry(const std::string & _id, 
					      const std::string & _subarch) 
    throw(DoesNotExistOnWMException, ConsistencyException, UnknownSubarchitectureException) {
      
    // if we don't hold the lock, then don't do anything
    if (!holdsLock(_id, _subarch)) {
      debug("no lock held for: %s:$s", _id.c_str(),_subarch.c_str());
      return;
    }

    assert(!_id.empty());//id must not be empty

    //unlock entry... will throw if entry does not exist or locks are wrong
    m_workingMemory->unlockEntry(_id,_subarch,getComponentID());
    
    m_permissions->removePermissions(_id, _subarch);

    //just check up on ourselves
    assert (!holdsLock(_id, _subarch));    
  }

  bool WorkingMemoryAttachedComponent::holdsLock(const std::string & _id) const {
    return holdsOverwriteLock(_id);
  }

  bool WorkingMemoryAttachedComponent::holdsLock(const std::string & _id, const std::string & _subarch) const {
    return holdsOverwriteLock(_id, _subarch);
  }

  bool WorkingMemoryAttachedComponent::holdsOverwriteLock(const std::string & _id) const {
    // any lock is an overwrite lock
    return m_permissions->hasPermissions(_id);
  }

  bool WorkingMemoryAttachedComponent::holdsDeleteLock(const std::string & _id) const {
      
    if(m_permissions->hasPermissions(_id)) {
      WorkingMemoryPermissions permissions = m_permissions ->getPermissions(_id);
      return (permissions == LOCKEDOD 
	      || permissions == LOCKEDODR);
    }
    else {
      return false;
    }
  }

  bool WorkingMemoryAttachedComponent::holdsReadLock(const std::string & _id) const {
    if(m_permissions->hasPermissions(_id)) {
      WorkingMemoryPermissions permissions = m_permissions ->getPermissions(_id);
      return permissions == LOCKEDODR;
    }
    else {
      return false;
    }
  }

  bool WorkingMemoryAttachedComponent::holdsOverwriteLock(const std::string & _id, const std::string & _subarch) const {
    // any lock is an overwrite lock
    return m_permissions->hasPermissions(_id, _subarch);
  }

  bool WorkingMemoryAttachedComponent::holdsDeleteLock(const std::string & _id, const std::string & _subarch) const {
    if(m_permissions->hasPermissions(_id, _subarch)) { 
      WorkingMemoryPermissions permissions = m_permissions->getPermissions(_id, _subarch);
      return (permissions == LOCKEDOD || permissions == LOCKEDODR);
    }
      
    else {
      return false;
    }      
  }

  bool WorkingMemoryAttachedComponent::holdsReadLock(const std::string & _id, const std::string & _subarch) const {
    if(m_permissions->hasPermissions(_id, _subarch)) { 
      WorkingMemoryPermissions permissions = m_permissions->getPermissions(_id, _subarch);
      return permissions == LOCKEDODR;
    }
    else {
      return false;
    }

  }
  
  bool WorkingMemoryAttachedComponent::needsConsistencyCheck(const std::string & _id) const {
    return m_permissions->needsConsistencyCheck(_id);
  }
  
  bool WorkingMemoryAttachedComponent::needsConsistencyCheck(const std::string & _id, const std::string & _subarch) const {
    return m_permissions->needsConsistencyCheck(_id, _subarch);
  }
  
  void WorkingMemoryAttachedComponent::consistencyChecked(const std::string & _id) {
    m_permissions->consistencyChecked(_id);
  }
  
  void WorkingMemoryAttachedComponent::consistencyChecked(const std::string & _id, const std::string & _subarch) {
    m_permissions->consistencyChecked(_id, _subarch);
  }


  
  cdl::WorkingMemoryPermissions 
  WorkingMemoryAttachedComponent::getPermissions(const std::string & _id) const
    throw(DoesNotExistOnWMException) {
    return getPermissions(_id, getSubarchitectureID());
  }


   
  cdl::WorkingMemoryPermissions 
  WorkingMemoryAttachedComponent::getPermissions(const cdl::WorkingMemoryAddress &_wma) const
    throw(DoesNotExistOnWMException, UnknownSubarchitectureException) {
      return getPermissions(_wma.id, _wma.subarchitecture);
  }



  cdl::WorkingMemoryPermissions 
  WorkingMemoryAttachedComponent::getPermissions(const std::string & _id,
						 const std::string & _subarch) const
    throw(DoesNotExistOnWMException, UnknownSubarchitectureException) {

    assert(!_id.empty());//id must not be empty
    assert(!_subarch.empty());//id must not be empty
    assert(m_workingMemory);

    WorkingMemoryPermissions permissions = m_workingMemory->getPermissions(_id,_subarch);
    return permissions;
  }


  bool 
  WorkingMemoryAttachedComponent::isOverwritable(const std::string & _id) const 
    throw(DoesNotExistOnWMException) {
    return isOverwritable(_id,getSubarchitectureID());
  }

  bool 
  WorkingMemoryAttachedComponent::isOverwritable(const std::string & _id, 
						 const std::string & _subarch) const
    throw(DoesNotExistOnWMException, UnknownSubarchitectureException) {

    if(holdsOverwriteLock(_id, _subarch)) {
      return true;
    }
    else {
      WorkingMemoryPermissions permissions = getPermissions(_id, _subarch);
      return overwriteAllowed(permissions);
    }
  }


    
  bool 
  WorkingMemoryAttachedComponent::isDeletable(const std::string & _id) const 
    throw(DoesNotExistOnWMException) {
    return isDeletable(_id,getSubarchitectureID());
  }
    
  
  bool 
  WorkingMemoryAttachedComponent::isDeletable(const std::string & _id, 
					      const std::string & _subarch) const
    throw(DoesNotExistOnWMException, UnknownSubarchitectureException) {
    
    if(holdsDeleteLock(_id, _subarch)) {
      return true;
    }
    else {
      WorkingMemoryPermissions permissions = getPermissions(_id, _subarch);
      return deleteAllowed(permissions);
    }
  }
    

  bool 
  WorkingMemoryAttachedComponent::isReadable(const std::string & _id) const
    throw(DoesNotExistOnWMException) {
    return isReadable(_id,getSubarchitectureID());
  }

  
  bool 
  WorkingMemoryAttachedComponent::isReadable(const std::string & _id, 
					     const std::string & _subarch) const
    throw(DoesNotExistOnWMException, UnknownSubarchitectureException) {
    
    if(holdsReadLock(_id, _subarch)) {
      return true;
    }
    else {
      WorkingMemoryPermissions permissions = getPermissions(_id, _subarch);
      return readAllowed(permissions);
    }
  }


  void 
  WorkingMemoryAttachedComponent::setWorkingMemory(const interfaces::WorkingMemoryPrx& _wm, 
						    const Ice::Current& _current) {
    assert(!m_workingMemory);

    //println("WorkingMemoryAttachedComponent::setWorkingMemory");
    //_wm->ice_ping();
    //println("working memory set: %s", _wm->getID().c_str());
    m_workingMemory = _wm;    
  }


  
}
