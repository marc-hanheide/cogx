#include "CASTComponentPermissionsMap.hpp"
#include <CASTUtils.hpp>



namespace cast {
 
  CASTComponentPermissionsMap::CASTComponentPermissionsMap(const std::string & _subarch) {
    //store local permissions in global map for generality
    m_globalPermissions[_subarch] = m_localPermissions;
  }


  void CASTComponentPermissionsMap::setPermissions(const std::string & _id, 
						   const cdl::WorkingMemoryPermissions & _permissions) {
    m_localPermissions[_id].m_first =_permissions;
    m_localPermissions[_id].m_second = true; 	
  }
  
  void CASTComponentPermissionsMap::setPermissions(const std::string & _id, const std::string & _subarch, const cdl::WorkingMemoryPermissions & _permissions) {
      
    m_globalPermissions[_subarch][_id].m_first = _permissions;
    m_globalPermissions[_subarch][_id].m_second = true;
      
  }
    
  const cdl::WorkingMemoryPermissions & CASTComponentPermissionsMap::getPermissions(const std::string & _id) const 
    throw(CASTException) {
    PermissionsMap::const_iterator i = m_localPermissions.find(_id);      
    if(i != m_localPermissions.end()) {
      return i->second.m_first;
    }

    throw(CASTException(exceptionMessage(__HERE__, 
					 "Entry does not have permissions to get: %s", 
					 _id.c_str())));	
    
  }
  
  const cdl::WorkingMemoryPermissions & CASTComponentPermissionsMap::getPermissions(const std::string & _id, 
										    const std::string & _subarch) const 
    throw(CASTException) {

    PermissionsMapMap::const_iterator i = m_globalPermissions.find(_subarch);      
    if(i != m_globalPermissions.end()) {
      PermissionsMap::const_iterator j = i->second.find(_id);
      if(j != i->second.end()) {
	return j->second.m_first;
      }
    }
    throw(CASTException(exceptionMessage(__HERE__, 
					 "Entry does not have permissions to get: %s:%s", 
					 _id.c_str(), _subarch.c_str())));	
  }

  bool CASTComponentPermissionsMap::hasPermissions(const std::string & _id) const {
    PermissionsMap::const_iterator i = m_localPermissions.find(_id);      
    return i != m_localPermissions.end();
  }
    
  bool CASTComponentPermissionsMap::hasPermissions(const std::string & _id, 
						   const std::string & _subarch) const {
    PermissionsMapMap::const_iterator i = m_globalPermissions.find(_subarch);      
    if(i != m_globalPermissions.end()) {
      PermissionsMap::const_iterator j = i->second.find(_id);
      return j != i->second.end();
    }
    else {
      return false;
    }
  }

  void CASTComponentPermissionsMap::removePermissions(const std::string & _id) {
    m_localPermissions.erase(_id);
  }
  
  void CASTComponentPermissionsMap::removePermissions(const std::string & _id, 
						      const std::string & _subarch) {
    if(hasPermissions(_id,_subarch)) {
      m_globalPermissions[_subarch].erase(_id);
    }
  }
  
  bool CASTComponentPermissionsMap::needsConsistencyCheck(const std::string & _id) const {
    if(hasPermissions(_id)) {
      //find should work if above passes
      return m_localPermissions.find(_id)->second.m_second;
    }
    else {
      return false;
    }
  }
    
  bool CASTComponentPermissionsMap::needsConsistencyCheck(const std::string & _id, 
							  const std::string & _subarch) const {
    if(hasPermissions(_id,_subarch)) {
      //find should work if above passes
      return m_globalPermissions.find(_subarch)->second.find(_id)->second.m_second;
    }
    else {
      return false;
    }
  }
  
  void CASTComponentPermissionsMap::consistencyChecked(const std::string & _id) {
    if(hasPermissions(_id)) {
      m_localPermissions[_id].m_second = false;
    }
  }
  
  void CASTComponentPermissionsMap::consistencyChecked(const std::string & _id, 
						       const std::string & _subarch) {     
    if(hasPermissions(_id,_subarch)) {
      m_globalPermissions[_subarch][_id].m_second = false;
    }
  }
  
} 
