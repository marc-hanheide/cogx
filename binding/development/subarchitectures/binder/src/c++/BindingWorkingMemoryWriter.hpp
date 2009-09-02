#ifndef BINDING_WORKING_MEMORY_WRITER_HPP
#define BINDING_WORKING_MEMORY_WRITER_HPP

#include <cast/architecture/ManagedComponent.hpp>

namespace binder {

  /**
   */
  class BindingWorkingMemoryWriter : 
    public cast::ManagedComponent {
    
    /**
     * Constructor
     */
    BindingWorkingMemoryWriter();
    
    /**
     * Destructor
     */
    virtual ~BindingWorkingMemoryWriter();
    
  };
  
}

#endif
