#ifndef BINDING_WORKING_MEMORY_READER_HPP
#define BINDING_WORKING_MEMORY_READER_HPP

#include <cast/architecture/ManagedComponent.hpp>
#include <autogen/BinderEssentials.hpp>
#include <vector>

/**
 * Abstract class for retrieving elements currently present in the
 * working memory of the binder
 * 
 * @author Pierre Lison, Nick Hawes
 * @version 02/09/2009
 */
namespace binder {

  /**
   */
  class BindingWorkingMemoryReader : 
    public virtual cast::ManagedComponent {

    const std::vector<autogen::core::UnionPtr> & 
    getUnions() const {
      return m_currentUnions;
    }

  protected:
    virtual void start();
    
    std::vector<autogen::core::UnionPtr> m_currentUnions;

  private:
    void fetchThenExtract(const cast::cdl::WorkingMemoryChange & _wmc);
    void extractUnionsFromConfig (autogen::core::UnionConfigurationPtr _config);
    
  };
  
}

#endif
