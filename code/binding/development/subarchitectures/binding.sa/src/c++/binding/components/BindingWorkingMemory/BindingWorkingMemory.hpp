#ifndef BINDING_WORKING_MEMORY_H_
#define BINDING_WORKING_MEMORY_H_

#include <pthread.h>
#include <cast/architecture/SubarchitectureWorkingMemory.hpp>
#include <binding/idl/BindingData.hh>
#include <binding/ontology/BindingFeatureOntology.hpp>

namespace Binding {
/// Binding working memory has a few extra interfaces w.r.t. \c SubarchitectureWorkingMemory
class BindingWorkingMemory : public cast::SubarchitectureWorkingMemory
  {
  public:
    BindingWorkingMemory(const std::string & _id);
    virtual ~BindingWorkingMemory();    
};
} // namespace Binding 
#endif // CAST_BINDING_WORKING_MEMORY_H_

