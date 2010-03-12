#include <cast/architecture/ChangeFilterFactory.hpp>
#include "BindingWorkingMemoryReader.hpp"

namespace binder {

  using namespace std;
  using namespace autogen::core;
  using namespace cast;
  using namespace cast::cdl;

  void
  BindingWorkingMemoryReader::start() {
    
    m_haveAddr = false;

    // if the set of possible union configurations has been updated,
    // update the monitor accordingly
    
    addChangeFilter(createGlobalTypeFilter<UnionConfiguration>(cdl::ADD),
		    new MemberFunctionChangeReceiver<BindingWorkingMemoryReader>(this,
			&BindingWorkingMemoryReader::fetchThenExtract));

    addChangeFilter(createGlobalTypeFilter<UnionConfiguration>(cdl::OVERWRITE),
		    new MemberFunctionChangeReceiver<BindingWorkingMemoryReader>(this,
			&BindingWorkingMemoryReader::fetchThenExtract));
  }
  
  void 
  BindingWorkingMemoryReader::fetchThenExtract(const cast::cdl::WorkingMemoryChange & _wmc) {
      addrFetchThenExtract(_wmc.address);
      m_currentUnionsAddr = _wmc.address;
      m_haveAddr = true;
  }

  void 
  BindingWorkingMemoryReader::addrFetchThenExtract(const cast::cdl::WorkingMemoryAddress & _addr) {
    try {
      UnionConfigurationPtr config = 
	getMemoryEntry<UnionConfiguration>(_addr);
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
	UnionPtr uni = *i;
	m_currentUnions.insert(pair<string, UnionPtr>(uni->entityID, uni));
	debug("Union ID %s is part of the current configuration", uni->entityID.c_str());
    }

    log("Wow, I currently have %d unions", m_currentUnions.size());
  }

}


