#include <cast/architecture/ChangeFilterFactory.hpp>
#include "BindingWorkingMemoryReader.hpp"

namespace binder {

  using namespace autogen::core;
  using namespace cast;
  using namespace cast::cdl;

  void
  BindingWorkingMemoryReader::start() {
    
    // if the set of possible union configurations has been updated,
    // update the monitor accordingly

    
    addChangeFilter(createGlobalTypeFilter<UnionConfiguration>(),
		    new MemberFunctionChangeReceiver<BindingWorkingMemoryReader>(this,
										 &BindingWorkingMemoryReader::fetchThenExtract));
  }
  
  void 
  BindingWorkingMemoryReader::fetchThenExtract(const cast::cdl::WorkingMemoryChange & _wmc) {
    try {
      UnionConfigurationPtr config = 
	getMemoryEntry<UnionConfiguration>(_wmc.address);
      extractUnionsFromConfig(config);
    }
    catch (CASTException & e) {
      println("EXCEPTION BindingWorkingMemoryReader::fetchThenExtract: %s", e.message.c_str());
    }
  }

  void BindingWorkingMemoryReader::extractUnionsFromConfig (UnionConfigurationPtr _config) {
    m_currentUnions.clear();
    for (UnionSequence::iterator i = _config->includedUnions.begin(); 
	 i < _config->includedUnions.end() ; ++i) {
      m_currentUnions.push_back(*i);
    }

    println("Wow, I currently have %d unions", m_currentUnions.size());
  }

}


