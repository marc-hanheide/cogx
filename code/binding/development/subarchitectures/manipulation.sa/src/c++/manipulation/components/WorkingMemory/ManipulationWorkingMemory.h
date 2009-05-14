#ifndef CAST_MANIPULATION_WORKING_MEMORY_H_
#define CAST_MANIPULATION_WORKING_MEMORY_H_

#include <cast/architecture/SubarchitectureWorkingMemory.hpp>

class ManipulationWorkingMemory : public cast::SubarchitectureWorkingMemory {
public:
  ManipulationWorkingMemory(const std::string & _id);
  virtual ~ManipulationWorkingMemory();
protected:
  virtual void redrawGraphicsText();
};

#endif
