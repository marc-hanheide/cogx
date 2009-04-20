
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
      int comparison = _f1.address.id.compare(
			      _f2.address.id);
      if (comparison < 0) {
	return BEFORE;
      }
      if (comparison > 0) {
	return AFTER;
      }
  
      // change sa
      comparison = _f1.address.subarchitecture.compare(
			  _f2.address.subarchitecture);
      if (comparison < 0) {
	return BEFORE;
      }
      if (comparison > 0) {
	return AFTER;
      }


      // ontological type
      comparison = _f1.type.compare(_f2.type);
      if (comparison < 0) {
	return BEFORE;
      }
      if (comparison > 0) {
	return AFTER;
      }

      // xarch
      if ((_f1.restriction != cdl::LOCALSA) && (_f2.restriction == cdl::LOCALSA)) {
	return BEFORE;
      }
      if ((_f1.restriction == cdl::LOCALSA) && (_f2.restriction != cdl::LOCALSA)) {
	return AFTER;
      }

      // operation
      if (_f1.operation < _f2.operation) {
	return BEFORE;
      }
      if (_f1.operation > _f2.operation) {
	return AFTER;
      }


      // source of change
      comparison = _f1.src.compare(
			  _f2.src);
      if (comparison < 0) {
	return BEFORE;
      }
      if (comparison > 0) {
	return AFTER;
      }

      // originator
      // change id
      comparison = _f1.origin.compare(
			  _f2.origin);
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
  
      //std::cout<<"allowsChange: "<<_change.type<<std::endl;

      // id options
    
      // if the filter id is not empty
      if (_filter.address.id.length() != 0) {
	// then ids much match
	if (_filter.address.id.compare(_change.address.id) != 0) {
	  return false;
	}    
      }

      // subarchitecture options
    
      // if filter is set for a specific subarch
      if (_filter.address.subarchitecture.length() != 0) {      
      
	// then subarchitectures must match
	if (_filter.address.subarchitecture.compare(
						    _change.address.subarchitecture) != 0) {
	
	  //std::cout<<_filter.address.subarchitecture<<" == "<< _change.address.subarchitecture<<std::endl;
	
	  return false;
	}
      }
      else {
	// otherwises allow any subarchitecture in the change
      }
    
      //std::cout<<"passed sa"<<std::endl;

      //type options

      //if type set
      if (_filter.type.length() != 0) {      
	// type must be the same
	if (_filter.type.compare(_change.type) != 0) {
	  return false;
	}
      }

  
      // operation must be the same if not a wildcard
      if(_filter.operation != cdl::WILDCARD) {
	if (_filter.operation != _change.operation) {
	  return false;
	}
      }

      //std::cout<<"passed op"<<std::endl;
    

      //std::cout<<"passed type"<<std::endl;

  
      //std::cout<<"passed src"<<std::endl;
  



            // src options
    
      // if filter src set
      if (_filter.src.length() != 0) {      
	// filter src must match
	if (_filter.src.compare(_change.src) != 0) {
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
