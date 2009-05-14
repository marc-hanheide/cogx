#ifndef CAST_PLANNING_BINDING_WORKING_MEMORY_H_
#define CAST_PLANNING_BINDING_WORKING_MEMORY_H_

#include <cast/architecture/SubarchitectureWorkingMemory.hpp>
#include "binding/ontology/BindingOntology.hpp"

class PlanningBindingWorkingMemory : public SubarchitectureWorkingMemory {

public:
  PlanningBindingWorkingMemory(const string & _id);
  virtual ~PlanningBindingWorkingMemory();
    


  
};

#endif

