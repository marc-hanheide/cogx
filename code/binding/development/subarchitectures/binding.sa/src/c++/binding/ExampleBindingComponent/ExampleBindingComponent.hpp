#ifndef CAST_EXAMPLE_BINDING_COMPONENT_H_
#define CAST_EXAMPLE_BINDING_COMPONENT_H_

#include <cast/architecture/ManagedProcess.hpp>
#include "../idl/BindingData..hpp"

class ExampleBindingComponent : public ManagedProcess {

public:
  ExampleBindingComponent(const string &_id);
  virtual ~ExampleBindingComponent();
  
  virtual void runComponent();

protected:
  virtual void taskAdopted(const string &_taskID);
  virtual void taskRejected(const string &_taskID);
  
};


#endif
