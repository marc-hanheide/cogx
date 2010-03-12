#ifndef BINDING_WORKING_MEMORY_READER_HPP
#define BINDING_WORKING_MEMORY_READER_HPP

#include <cast/architecture/ManagedComponent.hpp>
#include <autogen/BinderEssentials.hpp>
#include <map>

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

    const std::map<std::string, autogen::core::UnionPtr> & 
    getUnions() const {
      return m_currentUnions;
    }

  protected:
    virtual void start();
    void addrFetchThenExtract(const cast::cdl::WorkingMemoryAddress & _addr);

    std::map<std::string, autogen::core::UnionPtr> m_currentUnions;

    cast::cdl::WorkingMemoryAddress m_currentUnionsAddr;

    bool m_haveAddr;
    
  private:
    void fetchThenExtract(const cast::cdl::WorkingMemoryChange & _wmc);   
    void extractUnionsFromConfig (autogen::core::UnionConfigurationPtr _config);
    
  };
  
}

#endif
