#include <cstring>
#include <iostream>

#include "WorkingMemoryChangeFilterComparator.hpp"

namespace cast {

  
  bool WorkingMemoryChangeFilterComparator::operator () (const cdl::WorkingMemoryChangeFilter & _f1, 
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


  bool 
  WorkingMemoryChangeFilterComparator::allowsChange(const cdl::WorkingMemoryChangeFilter &_filter,
						    const cdl::WorkingMemoryChange &_change) {
  
    //std::cout<<"allowsChange: "<<_change.type<<std::endl;

    // id options
    
    // if the filter id is not empty
    if (_filter.address.id.length() != 0) {
      // then ids much match
      if (_filter.address.id != _change.address.id) {
	return false;
      }    
    }

    // subarchitecture options
    
    // if filter is set for a specific subarch
    if (_filter.address.subarchitecture.length() != 0) {      
      
      // then subarchitectures must match
      if (_filter.address.subarchitecture != _change.address.subarchitecture)  {
	//std::cout<<_filter.address.subarchitecture<<" == "<< _change.address.subarchitecture<<std::endl;
	return false;
      }
    }
    else {
      // otherwises allow any subarchitecture in the change
    }
    
    //std::cout<<"passed sa"<<std::endl;

    //type options

    //if type is set
    if (_filter.type.length() != 0) {      
      // then they must match directly or the filter type must be a member
      // of the supertypes of the change type
      
      bool allowsTypeChange = false;
      
      // direct comparison
      if (_filter.type == _change.type) {  
	//printf("allowing based on exact match: %s\n", _filter.type.c_str());
	allowsTypeChange = true;
      } 
      else {
	// sanity check to allow assumptions later
	//assert (_change.type
	//.equals(_change.superTypes[_change.superTypes.length - 1]));
	
	// go through the super types except for the most derived one
	// (which is _change.type)
	for (unsigned int i = 0; i < _change.superTypes.size() - 1; ++i) {
	  // if the filter matches against the supertype
	  if (_filter.type == _change.superTypes[i]) {
	    //printf("allowing based on supertypes: %s\n", _filter.type.c_str());
	    allowsTypeChange = true;
	    break;
	  }
	}
      }
      
      if (!allowsTypeChange) {
	  // printf("type failed");
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
      if (_filter.src != _change.src) {
	return false;
      }
    
    }

    
    //std::cout<<"passed id"<<std::endl;

    // if we get this far then we're fine
    
    return true;
  }
    

} //namespace cast
