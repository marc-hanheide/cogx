
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

#ifndef CAST_WORKING_MEMORY_CHANGE_FILTER_COMPARATOR_H_
#define CAST_WORKING_MEMORY_CHANGE_FILTER_COMPARATOR_H_

#include <cstring>
#include <iostream>

namespace cast {

  struct WorkingMemoryChangeFilterComparator {
    
    bool operator () (const cdl::WorkingMemoryChangeFilter & _f1, 
		      const cdl::WorkingMemoryChangeFilter & _f2) const {


      //used to try to keep mapping with identical Java code
      static const bool BEFORE = true;
      static const bool EQUAL = false;
      static const bool AFTER = false;

      // same object
      if (&_f1 == &_f2) {
	return EQUAL;
      }


      // change id
      int comparison = strcmp(_f1.m_address.m_id,
			  _f2.m_address.m_id);
      if (comparison < 0) {
	return BEFORE;
      }
      if (comparison > 0) {
	return AFTER;
      }
  
      // change sa
      comparison = strcmp(_f1.m_address.m_subarchitecture, 
			  _f2.m_address.m_subarchitecture);
      if (comparison < 0) {
	return BEFORE;
      }
      if (comparison > 0) {
	return AFTER;
      }


      // ontological type
      comparison = strcmp(_f1.m_type,_f2.m_type);
      if (comparison < 0) {
	return BEFORE;
      }
      if (comparison > 0) {
	return AFTER;
      }

      // xarch
      if ((_f1.m_restriction != cdl::LOCAL_SA) && (_f2.m_restriction == cdl::LOCAL_SA)) {
	return BEFORE;
      }
      if ((_f1.m_restriction == cdl::LOCAL_SA) && (_f2.m_restriction != cdl::LOCAL_SA)) {
	return AFTER;
      }

      // operation
      if (_f1.m_operation < _f2.m_operation) {
	return BEFORE;
      }
      if (_f1.m_operation > _f2.m_operation) {
	return AFTER;
      }


      // source of change
      comparison = strcmp(_f1.m_src,
			  _f2.m_src);
      if (comparison < 0) {
	return BEFORE;
      }
      if (comparison > 0) {
	return AFTER;
      }

      // originator
      // change id
      comparison = strcmp(_f1.m_origin,
			  _f2.m_origin);
      if (comparison < 0) {
	return BEFORE;
      }
      if (comparison > 0) {
	return AFTER;
      }

  
      return EQUAL;
  
    }


    static
    bool 
    allowsChange(const cdl::WorkingMemoryChangeFilter &_filter,
		 const cdl::WorkingMemoryChange &_change) {
  
      //std::cout<<"allowsChange: "<<_change.m_type<<std::endl;

      // id options
    
      // if the filter id is not empty
      if (strcmp(_filter.m_address.m_id,"") != 0) {
	// then ids much match
	if (strcmp(_filter.m_address.m_id,_change.m_address.m_id) != 0) {
	  return false;
	}    
      }

      // subarchitecture options
    
      // if filter is set for a specific subarch
      if (strcmp(_filter.m_address.m_subarchitecture,"") != 0) {      
      
	// then subarchitectures must match
	if (strcmp(_filter.m_address.m_subarchitecture,
		   _change.m_address.m_subarchitecture) != 0) {
	
	  //std::cout<<_filter.m_address.m_subarchitecture<<" == "<< _change.m_address.m_subarchitecture<<std::endl;
	
	  return false;
	}
      }
      else {
	// otherwises allow any subarchitecture in the change
      }
    
      //std::cout<<"passed sa"<<std::endl;

      //type options

      //if type set
      if (strcmp(_filter.m_type,"") != 0) {      
	// type must be the same
	if (strcmp(_filter.m_type,_change.m_type) != 0) {
	  return false;
	}
      }

  
      // operation must be the same if not a wildcard
      if(_filter.m_operation != cdl::WILDCARD) {
	if (_filter.m_operation != _change.m_operation) {
	  return false;
	}
      }

      //std::cout<<"passed op"<<std::endl;
    

      //std::cout<<"passed type"<<std::endl;

  
      //std::cout<<"passed src"<<std::endl;
  



            // src options
    
      // if filter src set
      if (strcmp(_filter.m_src,"") != 0) {      
	// filter src must match
	if (strcmp(_filter.m_src,_change.m_src) != 0) {
	  return false;
	}
    
      }

    
      //std::cout<<"passed id"<<std::endl;

      // if we get this far then we're fine
    
      return true;
    }


  };

} //namespace cast

#endif
