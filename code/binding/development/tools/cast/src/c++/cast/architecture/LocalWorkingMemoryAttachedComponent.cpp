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

#include "LocalWorkingMemoryAttachedComponent.hpp"


using namespace std;
using namespace boost;

namespace cast {

  using namespace cdl;

  LocalWorkingMemoryAttachedComponent::LocalWorkingMemoryAttachedComponent(const string &_id) 
    : //InspectableComponent(_id),
    CASTProcessingComponent(_id),
    m_pExistsConnector(NULL),
    m_pVersionNumberConnector(NULL)
  {    
  }

  void 
  LocalWorkingMemoryAttachedComponent::setPullConnector(const string & _connectionID, 
							PullConnectorOut< int > * _pOut) {
    m_pVersionNumberConnector = _pOut;
  }


  void 
  LocalWorkingMemoryAttachedComponent::setPullConnector(const string & _connectionID, 
					       PullConnectorOut< bool > * _pOut) {
    m_pExistsConnector = _pOut;
  }

  bool 
  LocalWorkingMemoryAttachedComponent::existsOnWorkingMemory(const string & _id) {

    assert(!_id.empty());//id must not be empty
    assert(m_pExistsConnector != NULL);
    FrameworkQuery query(getProcessIdentifier(),_id);
    FrameworkLocalData< bool > * pData = NULL;
    m_pExistsConnector->pull(query,pData);
    bool exists = pData->getData();
    delete pData;
    return exists;

  }

  int 
  LocalWorkingMemoryAttachedComponent::getVersionNumber(const string & _id) throw (DoesNotExistOnWMException){

    assert(m_pVersionNumberConnector != NULL);    
    assert(!_id.empty());//id must not be empty

    FrameworkQuery query(getProcessIdentifier(),_id);
    FrameworkLocalData< int > * pData = NULL;
    
    m_pVersionNumberConnector->pull(query,pData);
    int versionNumber = pData->getData();
    delete pData;

    if (-1 == versionNumber) {
      throw(DoesNotExistOnWMException(makeAdress(subarchitectureID(), _id),
				      __HERE__, 
				      "Entry has never existed on wm. Was looking in subarch %s for id %s", 
				      m_subarchitectureID.c_str(),_id.c_str()));
    }
    else {
      return versionNumber;
    }

  }  

  void 
  LocalWorkingMemoryAttachedComponent::storeVersionNumber(const string & _id, int _version) {
    m_versionNumbers[_id] = _version;
  }

  int 
  LocalWorkingMemoryAttachedComponent::getStoredVersionNumber(const string & _id) const
    throw(ConsistencyException) {

    IntMap::const_iterator i = m_versionNumbers.find(_id);

    if (i == m_versionNumbers.end()) {
      throw(ConsistencyException(makeAdress(subarchitectureID(),_id),
				 __HERE__,
				 "No stored version for id: %s", 
				 _id.c_str()));
    } else {
      return i->second;
    }
  }

  void
  LocalWorkingMemoryAttachedComponent::increaseStoredVersion(const string & _id) throw(ConsistencyException) {
    int stored = getStoredVersionNumber(_id);
    updateVersion(_id, ++stored);
  }

  void 
  LocalWorkingMemoryAttachedComponent::startVersioning(const string & _id) {
    assert (!isVersioned(_id));
    storeVersionNumber(_id, 0);
  }

  void 
  LocalWorkingMemoryAttachedComponent::updateVersion(const string & _id, int _newVersion) {
    storeVersionNumber(_id, _newVersion);
  }

  
  bool
  LocalWorkingMemoryAttachedComponent::isVersioned(const string & _id) const {
    IntMap::const_iterator i = m_versionNumbers.find(_id);
    return i != m_versionNumbers.end();
  }

  void 
  LocalWorkingMemoryAttachedComponent::stopVersioning(const string & _id) {
    if (isVersioned(_id)) {
      removeVersionNumber(_id);
    }
  }

  void 
  LocalWorkingMemoryAttachedComponent::removeVersionNumber(const string & _id) {
    m_versionNumbers.erase(_id);
  }

     
  bool 
  LocalWorkingMemoryAttachedComponent::haveLatestVersion(const string &_id)
    throw(ConsistencyException, DoesNotExistOnWMException) {
    assert (isVersioned(_id));
    
    int ownedVersion = getStoredVersionNumber(_id);
    int wmVersion = getVersionNumber(_id);
    
    debug("haveLatestVersion(%s): %d == %d", _id.c_str(), wmVersion, ownedVersion);

    return wmVersion == ownedVersion;
  }


  void 
  LocalWorkingMemoryAttachedComponent::checkConsistency(const string & _id) 
    throw(ConsistencyException, DoesNotExistOnWMException)
  {

    if (!isVersioned(_id)) {
      throw(ConsistencyException(makeAdress(subarchitectureID(),_id),
				 __HERE__,
				 "!isVersioned(%s) in subarch %s. ",
				 _id.c_str(), m_subarchitectureID.c_str()));
    }


    if (!haveLatestVersion(_id)) {
      throw(ConsistencyException(makeAdress(subarchitectureID(),_id),
				 __HERE__,
				 "%s has attempted to overwrite an outdated working memory entry. Please reread and try again. WMA: %s:%s. Local version: %d. WM version: %d",
				 getProcessIdentifier().c_str(), _id.c_str(), m_subarchitectureID.c_str(), getStoredVersionNumber(_id), getVersionNumber(_id)));
    }

  }



  void 
  LocalWorkingMemoryAttachedComponent::setPullConnector(const string & _connectionID, 			
							PullConnectorOut< WorkingMemoryPermissions > * _pOut) {
    m_lockConnector = _pOut;
  }


  void LocalWorkingMemoryAttachedComponent::lockEntry(const std::string & _id, const cdl::WorkingMemoryPermissions & _permissions)
    throw(DoesNotExistOnWMException) {
    lockEntryHelper(_id, getSubarchitectureID(), _permissions, BLOCKING);
  }

  bool LocalWorkingMemoryAttachedComponent::lockEntryHelper(const std::string & _id, 
							    const std::string & _subarch,
							    const cdl::WorkingMemoryPermissions & _permissions, 
							    const cdl::OperationMode & _op) 
    throw(DoesNotExistOnWMException) {


    assert(!_id.empty());//id must not be empty
    assert(m_lockConnector!= NULL);
    FrameworkQuery query(getProcessIdentifier(), lockQueryString(_id, _subarch, _permissions, _op));
    FrameworkLocalData< WorkingMemoryPermissions > * pData = NULL;
    m_lockConnector->pull(query,pData);
    WorkingMemoryPermissions set = pData->getData();
    delete pData;

    if (set == DOES_NOT_EXIST) {
      throw(DoesNotExistOnWMException(makeAdress(_subarch,_id),
				      __HERE__,
				      "ID %s does not exist on WM in subarch %s",_id.c_str(),_subarch.c_str()));
    }
    
    bool succeeded = (_permissions == set);
    
    // if we blocked then we should always succeed
    if (_op == BLOCKING) {
      assert(succeeded);
    }
    
    // if we succeeded, then let's store the permissions
    if (succeeded) {
      m_permissions->setPermissions(_id, _subarch, set);
    }
    
    return succeeded;
    
  }

    /**
   * Overrides the configure method from FrameworkProcess to use
   * _config to set the subarchitecture ID.
   * 
   * @param _config
   *            The ID of the subarchitecture which contains this
   *            component.
   */
  void LocalWorkingMemoryAttachedComponent::configure(std::map<std::string,std::string> & _config) {
    CASTProcessingComponent::configure(_config);
    m_permissions = 
      boost::shared_ptr< CASTComponentPermissionsMap >(new CASTComponentPermissionsMap(getSubarchitectureID()));
  }


  bool LocalWorkingMemoryAttachedComponent::tryLockEntry(const std::string & _id,
							 const cdl::WorkingMemoryPermissions & _permissions) 
    throw(DoesNotExistOnWMException) {
    return lockEntryHelper(_id, getSubarchitectureID(), _permissions,
			   cdl::NON_BLOCKING);
  }

  

  void LocalWorkingMemoryAttachedComponent::unlockEntry(const std::string & _id) 
    throw(DoesNotExistOnWMException) {
    unlockEntryHelper(_id, getSubarchitectureID());
  }
  
  void LocalWorkingMemoryAttachedComponent::unlockEntryHelper(const std::string & _id, 
							      const std::string & _subarch) 
    throw(DoesNotExistOnWMException) {
      
    // if
    if (!holdsLock(_id, _subarch)) {
      debug("no lock held for: %s:$s", _id.c_str(),_subarch.c_str());
      return;
    }

    assert(!_id.empty());//id must not be empty
    assert(m_lockConnector!= NULL);
    FrameworkQuery query(getProcessIdentifier(), unlockQueryString(_id, _subarch));
    FrameworkLocalData< WorkingMemoryPermissions > * pData = NULL;
    m_lockConnector->pull(query,pData);
    WorkingMemoryPermissions set = pData->getData();
    delete pData;

    if (set == DOES_NOT_EXIST) {
      throw(DoesNotExistOnWMException(makeAdress(_subarch,_id),
				      __HERE__,
				      "ID %s does not exist on WM in subarch %s",_id.c_str(),_subarch.c_str()));
    }
    // allow for DOES_NOT_EXIST as this could happen if unlock on
    // deletion occurs
    // boolean succeeded = (WorkingMemoryPermissions.UNLOCKED == set ||
    // WorkingMemoryPermissions.DOES_NOT_EXIST == set);
    bool succeeded = (UNLOCKED == set);
    
    // we should always succeed unlocking if we hold a lock
    assert (succeeded);
    
    m_permissions->removePermissions(_id, _subarch);

    //just check up on ourselves
     assert (!holdsLock(_id, _subarch));
	
  }

  bool LocalWorkingMemoryAttachedComponent::holdsLock(const std::string & _id) const {
    return holdsOverwriteLock(_id);
  }

  bool LocalWorkingMemoryAttachedComponent::holdsLock(const std::string & _id, const std::string & _subarch) const {
    return holdsOverwriteLock(_id, _subarch);
  }

  bool LocalWorkingMemoryAttachedComponent::holdsOverwriteLock(const std::string & _id) const {
    // any lock is an overwrite lock
    return m_permissions->hasPermissions(_id);
  }

  bool LocalWorkingMemoryAttachedComponent::holdsDeleteLock(const std::string & _id) const {
      
    if(m_permissions->hasPermissions(_id)) {
      WorkingMemoryPermissions permissions = m_permissions ->getPermissions(_id);
      return (permissions == LOCKED_OD 
	      || permissions == LOCKED_ODR);
    }
    else {
      return false;
    }
  }

  bool LocalWorkingMemoryAttachedComponent::holdsReadLock(const std::string & _id) const {
    if(m_permissions->hasPermissions(_id)) {
      WorkingMemoryPermissions permissions = m_permissions ->getPermissions(_id);
      return permissions == LOCKED_ODR;
    }
    else {
      return false;
    }
  }

  bool LocalWorkingMemoryAttachedComponent::holdsOverwriteLock(const std::string & _id, const std::string & _subarch) const {
    // any lock is an overwrite lock
    return m_permissions->hasPermissions(_id, _subarch);
  }

  bool LocalWorkingMemoryAttachedComponent::holdsDeleteLock(const std::string & _id, const std::string & _subarch) const {
    if(m_permissions->hasPermissions(_id, _subarch)) { 
      WorkingMemoryPermissions permissions = m_permissions->getPermissions(_id, _subarch);
      return (permissions == LOCKED_OD || permissions == LOCKED_ODR);
    }
      
    else {
      return false;
    }      
  }

  bool LocalWorkingMemoryAttachedComponent::holdsReadLock(const std::string & _id, const std::string & _subarch) const {
    if(m_permissions->hasPermissions(_id, _subarch)) { 
      WorkingMemoryPermissions permissions = m_permissions->getPermissions(_id, _subarch);
      return permissions == LOCKED_ODR;
    }
    else {
      return false;
    }

  }
  
  bool LocalWorkingMemoryAttachedComponent::needsConsistencyCheck(const std::string & _id) const {
    return m_permissions->needsConsistencyCheck(_id);
  }
  
  bool LocalWorkingMemoryAttachedComponent::needsConsistencyCheck(const std::string & _id, const std::string & _subarch) const {
    return m_permissions->needsConsistencyCheck(_id, _subarch);
  }
  
  void LocalWorkingMemoryAttachedComponent::consistencyChecked(const std::string & _id) {
    m_permissions->consistencyChecked(_id);
  }
  
  void LocalWorkingMemoryAttachedComponent::consistencyChecked(const std::string & _id, const std::string & _subarch) {
    m_permissions->consistencyChecked(_id, _subarch);
  }


  cdl::WorkingMemoryPermissions 
  LocalWorkingMemoryAttachedComponent::getPermissionsHelper(const std::string & _id,
							    const std::string & _subarch) const
    throw(DoesNotExistOnWMException) {

    assert(!_id.empty());//id must not be empty
    assert(m_lockConnector!= NULL);
    FrameworkQuery query(getProcessIdentifier(), statusQueryString(_id, _subarch));
    FrameworkLocalData< WorkingMemoryPermissions > * pData = NULL;
    m_lockConnector->pull(query,pData);
    WorkingMemoryPermissions permissions = pData->getData();
    delete pData;

    if (permissions == DOES_NOT_EXIST) {
      throw(DoesNotExistOnWMException(makeAdress(_subarch,_id),
				      __HERE__,
				      "ID %s does not exist on WM in subarch %s",_id.c_str(), _subarch.c_str()));
    }
    
    return permissions;
  }
  
  cdl::WorkingMemoryPermissions 
  LocalWorkingMemoryAttachedComponent::getPermissions(const std::string & _id) const
    throw(DoesNotExistOnWMException) {
    return getPermissionsHelper(_id, getSubarchitectureID());
  }


  bool 
  LocalWorkingMemoryAttachedComponent::isOverwritable(const std::string & _id) const 
    throw(DoesNotExistOnWMException) {
    if(holdsOverwriteLock(_id)) {
      return true;
    }
    else {
      const WorkingMemoryPermissions & permissions = getPermissions(_id);
      return overwriteAllowed(permissions);
    }
  }
    
  bool 
  LocalWorkingMemoryAttachedComponent::isDeletable(const std::string & _id) const 
    throw(DoesNotExistOnWMException) {
    if(holdsDeleteLock(_id)) {
      return true;
    }
    else {
      const WorkingMemoryPermissions & permissions = getPermissions(_id);
      return deleteAllowed(permissions);
    }
  }

  bool 
  LocalWorkingMemoryAttachedComponent::isReadable(const std::string & _id) const
    throw(DoesNotExistOnWMException) {
    if(holdsReadLock(_id)) {
      return true;
    }
    else {
      const WorkingMemoryPermissions & permissions = getPermissions(_id);
      return readAllowed(permissions);
    }
  }

  
}
