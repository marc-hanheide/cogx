#include "AbstractBinder.hpp"
#include <boost/lexical_cast.hpp>

namespace Binding {
  using namespace boost;
  using namespace std;
  using namespace cast;

AbstractBinder::AbstractBinder(const string &_id) :
  WorkingMemoryAttachedComponent(_id),
  PrivilegedManagedProcess(_id),
  AbstractBindingWMRepresenter(dynamic_cast<cast::WorkingMemoryReaderProcess&>(*this))
{
  queueBehaviour = cdl::QUEUE;
}

AbstractBinder::~AbstractBinder() {

}

void 
AbstractBinder::start() {
  
  PrivilegedManagedProcess::start();

}

void 
AbstractBinder::configure(std::map<std::string,std::string>& _config)
{
  cast::PrivilegedManagedProcess::configure(_config);
  std::map<std::string,std::string>::const_iterator itr = _config.find("-bsa");
  if(_config.end() == itr)
    itr = _config.find("--bsa");
  
  if(_config.end() != itr) {
    setBindingSubarchID(itr->second);
    log("setting binding subarch to: " + bindingSubarchID());
  }
 else {
    log("binding subarch not specified, assuming it\'s local to monitor");
    setBindingSubarchID(subarchitectureID);
  }
}

void 
AbstractBinder::changeProxyState(const LBindingProxy& _proxy, 
				 BindingData::BindingProxyState _state)
{
#ifndef NDEBUG
  if(_state == BindingData::BOUND)
    assert(!string(_proxy->unionID).empty());
  if(string(_proxy->unionID).empty())
    assert(_state != BindingData::BOUND);
#endif // NDEBUG
  if(_proxy->proxyState == _state) { // do nothing if nothing needs doing...
    return;
  }
  BindingData::BindingProxy* proxy_ptr = 
    new BindingData::BindingProxy(_proxy.get());
  proxy_ptr->proxyState = _state;
  overwriteWorkingMemory(_proxy.id(), 
			 //BindingLocalOntology::BINDING_PROXY_TYPE, 
			 proxy_ptr, 
			 cdl::BLOCKING);
}

//cast::cdl::WorkingMemoryAddress 
//AbstractBinder::_binderTokenAddress() const {
//}



void 
AbstractBinder::acquireBinderToken()
{
//  if(hasBinderTokenToken())
 //   releaseBinderTokenToken();
  acquireToken(_binderTokenAddress());
}

void 
AbstractBinder::releaseBinderToken()
{
  //acquireBinderTokenToken();
  releaseToken(_binderTokenAddress());
}

void 
AbstractBinder::acquireInternalBindingToken()
{
//  if(hasBinderTokenToken())
 //   releaseBinderTokenToken();
  acquireToken(_internalBindingTokenAddress());
}

void 
AbstractBinder::releaseInternalBindingToken()
{
  //acquireBinderTokenToken();
  releaseToken(_internalBindingTokenAddress());
}


void 
AbstractBinder::acquireBinderLockToken()
{
//  if(hasBinderLockTokenToken())
 //   releaseBinderLockTokenToken();
//  acquireToken(_binderLockTokenAddress());
}

void 
AbstractBinder::releaseBinderLockToken()
{
  //acquireBinderLockTokenToken();
 // releaseToken(_binderLockTokenAddress());
}


} //namespace Binding
