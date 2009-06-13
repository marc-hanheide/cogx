#include "AbstractActiveBindingWMReader.hpp"
#include "BindingData.hpp"
#include "cast/architecture/ChangeFilterFactory.hpp"

namespace Binding {
using namespace std;
using namespace cast;
using namespace boost;

  AbstractActiveBindingWMReader::AbstractActiveBindingWMReader(const string &_id) :
    WorkingMemoryAttachedComponent(_id),
    ManagedProcess(_id) {  
    
    // not finished...

  }


  void
  AbstractActiveBindingWMReader::start() {

    ManagedProcess::start();

    MemberFunctionChangeReceiver<AbstractActiveBindingWMReader>* 
      receiver_ptr(new MemberFunctionChangeReceiver<AbstractActiveBindingWMReader>(this,&AbstractActiveBindingWMReader::proxyAdded)); 
    addChangeFilter(createGlobalTypeFilter<BindingData::BindingProxy>(cdl::ADD), receiver_ptr);
  }

  void 
  AbstractActiveBindingWMReader::configure(map<string,string> & _config) {
    
    // first let the base class configure itself
    ManagedProcess::configure(_config);
    
    if(_config[BindingData::BINDING_SUBARCH_CONFIG_KEY] != "") {
      bindingSA = _config[BindingData::BINDING_SUBARCH_CONFIG_KEY];
      log("setting binding subarch to: " + bindingSA);
    }
    else {
      log("binding subarch not specified, assuming it\'s local to monitor");
      bindingSA = subarchitectureID;
    }
    
  }
  
} // namespace Binding
