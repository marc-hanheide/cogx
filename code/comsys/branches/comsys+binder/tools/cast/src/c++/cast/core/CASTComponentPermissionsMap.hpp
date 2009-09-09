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

#ifndef CAST_COMPONENT_PERMISSIONS_MAP_H_
#define CAST_COMPONENT_PERMISSIONS_MAP_H_

#include <cast/core/StringMap.hpp>
#include <cast/slice/CDL.hpp>
 
namespace cast {
 
  class CASTComponentPermissionsMap {

  private:
    struct PermAndCheck {
      cdl::WorkingMemoryPermissions m_first;
      bool m_second;
    };
    
    typedef StringMap<PermAndCheck>::map PermissionsMap;
    typedef StringMap<PermissionsMap>::map PermissionsMapMap;

    PermissionsMap m_localPermissions;
    PermissionsMapMap m_globalPermissions;
  
  public: 

    CASTComponentPermissionsMap(const std::string & _subarch);

    void setPermissions(const std::string & _id, 
			const cdl::WorkingMemoryPermissions & _permissions);  
    void setPermissions(const std::string & _id, const std::string & _subarch, const cdl::WorkingMemoryPermissions & _permissions);    

    const cdl::WorkingMemoryPermissions & getPermissions(const std::string & _id) const 
      throw(CASTException);
  
    const cdl::WorkingMemoryPermissions & getPermissions(const std::string & _id, 
							 const std::string & _subarch) const
      throw(CASTException);

    bool hasPermissions(const std::string & _id) const ;    
    bool hasPermissions(const std::string & _id, 
			const std::string & _subarch) const;
    void removePermissions(const std::string & _id);  
    void removePermissions(const std::string & _id, 
			   const std::string & _subarch);  
    bool needsConsistencyCheck(const std::string & _id) const;    
    bool needsConsistencyCheck(const std::string & _id, 
			       const std::string & _subarch) const;  
    void consistencyChecked(const std::string & _id);
  
    void consistencyChecked(const std::string & _id, 
			    const std::string & _subarch);


  
  };
  
} 
#endif
 
