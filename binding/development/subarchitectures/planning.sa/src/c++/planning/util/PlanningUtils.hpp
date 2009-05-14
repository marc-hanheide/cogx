
#ifndef PLANNING_UTILS_H_
#define PLANNING_UTILS_H_

#include <planning/idl/Planner.hh>

#include <iostream>


bool 
operator== (const Planner::Fact & _f1, 
	    const Planner::Fact & _f2);
  


std::ostream & 
operator<<(std::ostream &_stream, 
	   const Planner::Fact &_fact);

std::ostream & 
operator<<(std::ostream &_stream, 
	   const Planner::ObjectDeclaration &_decl);




struct ObjectDeclarationComparator {
  
  bool operator () (const Planner::ObjectDeclaration & _o1, 
		    const Planner::ObjectDeclaration & _o2) const {
    
    
    //used to try to keep mapping with identical Java code
    static const bool BEFORE = true;
    static const bool EQUAL = false;
    static const bool AFTER = false;
    
    // same object
    if (&_o1 == &_o2) {
      return EQUAL;
    }
    
    
    // change id
    int comparison = strcmp(_o1.name,
			_o2.name);
    if (comparison < 0) {
      return BEFORE;
    }
    if (comparison > 0) {
      return AFTER;
    }
    
    
    comparison = strcmp(_o1.type,
			_o2.type);
    
    if (comparison < 0) {
      return BEFORE;
    }
    if (comparison > 0) {
      return AFTER;
    }
    
    return EQUAL;
    
  }
};


struct FactComparator {
  
  bool operator () (const Planner::Fact & _f1, 
		    const Planner::Fact & _f2) const {
    
    
    //used to try to keep mapping with identical Java code
    static const bool BEFORE = true;
    static const bool EQUAL = false;
    static const bool AFTER = false;
    
    // same object
    if (&_f1 == &_f2) {
      return EQUAL;
    }
    
    
    // length of args
    int comparison = _f1.arguments.length() - _f2.arguments.length();
    if (comparison < 0) {
      return BEFORE;
    }
    if (comparison > 0) {
      return AFTER;
    }
    
    //name
    comparison = strcmp(_f1.name,
			_f2.name);
    if (comparison < 0) {
      return BEFORE;
    }
    if (comparison > 0) {
      return AFTER;
    }

    //value
    comparison = strcmp(_f1.value,
			_f2.value);
    if (comparison < 0) {
      return BEFORE;
    }
    if (comparison > 0) {
      return AFTER;
    }
    

    //each arg
    for (unsigned int i = 0; i <  _f1.arguments.length(); ++i) {
      comparison = strcmp(_f1.arguments[i],
			  _f2.arguments[i]);
      if (comparison < 0) {
	return BEFORE;
      }
      if (comparison > 0) {
	return AFTER;
      }
    }
    
    //modality
    if(_f1.modality < _f2.modality) {
      return BEFORE;
      }
    if(_f1.modality > _f2.modality) {      
      return AFTER;
    }


    comparison = strcmp(_f1.agent,
			_f2.agent);
    
    if (comparison < 0) {
      return BEFORE;
    }
    if (comparison > 0) {
      return AFTER;
    }
    
    return EQUAL;
    
  }
};



#include <planning/util/TemporaryPlanningState.hpp>

class TemporaryPlanningState;

std::ostream & 
operator<<(std::ostream &_stream, 
	   const TemporaryPlanningState &_state);



#endif
