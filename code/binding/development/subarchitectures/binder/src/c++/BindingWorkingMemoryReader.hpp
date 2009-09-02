#ifndef BINDING_WORKING_MEMORY_READER_HPP
#define BINDING_WORKING_MEMORY_READER_HPP

#include <cast/architecture/ManagedComponent.hpp>

namespace binder {

  /**
   */
  class BindingWorkingMemoryReader : 
    public cast::ManagedComponent {
    
    /**
     * Constructor
     */
    BindingWorkingMemoryReader();
    
    /**
     * Destructor
     */
    virtual ~BindingWorkingMemoryReader();
    
  };
  
}

#endif
