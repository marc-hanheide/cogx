#include "AbstractBindingWMRepresenter.hpp"
#include "binding/feature-specialization/helpers/SalienceHelper.hpp"
#include <boost/lexical_cast.hpp>
#include <sstream>
namespace Binding {

using namespace std;
using namespace cast;
using namespace boost;

AbstractBindingWMRepresenter::AbstractBindingWMRepresenter(cast::WorkingMemoryReaderProcess& _component) 
  : component(_component),  
    proxyLocalCache(_component,local_proxy_converter(*this),"proxy Local cache"),
    unionLocalCache(_component,local_union_converter(*this),"proxy Union cache"),
//    storeAllFeatures(false),
    featureLoader(*this)
{ };


/// uses the \p FeatureHelper to get the feature
shared_ptr<AbstractFeature>
AbstractBindingWMRepresenter::toFeature(const BindingData::FeaturePointer& _feat) {
  return featureLoader.getFeature(_feat, bindingSubarchID());
}


template<>
const LBindingProxy&
CASTDataLocalCache<BindingData::BindingProxy,LBindingProxy,AbstractBindingWMRepresenter::local_proxy_converter>::operator[](const std::string& _id) {
  return *(this->get(_id));
}

template<>
const LBindingUnion&
CASTDataLocalCache<BindingData::BindingUnion,LBindingUnion,AbstractBindingWMRepresenter::local_union_converter>::operator[](const std::string& _id) {
  return *(this->get(_id));
}


const LBindingProxy* 
AbstractBindingWMRepresenter::maybeLoadProxy(const std::string& _proxyID)
{
  const LBindingProxy* proxy_ptr = NULL;
  if(!component.existsOnWorkingMemory(_proxyID)) {
    return NULL;
  } else {
    try {
      proxy_ptr = &(this->proxyLocalCache[_proxyID]);
    } 
    catch (const cast::DoesNotExistOnWMException&) {
      proxy_ptr = NULL;
    }
  }
  return proxy_ptr;
}
 
const LBindingUnion* 
AbstractBindingWMRepresenter::maybeLoadUnion(const std::string& _unionID)
{
  const LBindingUnion* union_ptr = NULL;
  if(!component.existsOnWorkingMemory(_unionID)) {
    return NULL;
  } else {
    try {
      union_ptr = &(this->unionLocalCache[_unionID]);
    } 
    catch (const cast::DoesNotExistOnWMException&) {
      union_ptr = NULL;
    }
  }
  return union_ptr;
}

void 
AbstractBindingWMRepresenter::acquireToken(const cast::cdl::WorkingMemoryAddress& _wma) 
{
  WMASet::iterator itr = tokens.find(_wma);
  assert(itr == tokens.end()); // i.e. don't reacquire the same token again...
  stringstream str;
  str << BALTTimer::getBALTTime();
  component().debug("gonna acquire token " + string(_wma.id) + " in " +
		    string(_wma.subarchitecture) + " " +  str.str());
#ifndef NBINDING_TOKENS
  component.lockEntry(_wma, cast::cdl::LOCKED_ODR);
#endif //NBINDING_TOKENS
  stringstream str2;
  str2 << BALTTimer::getBALTTime();
  component().debug("acquired token " + string(_wma.id) + " in " +
		    string(_wma.subarchitecture)+ " " + str2.str() +
		    " (waited since "+ str.str() +")");
  tokens.insert(_wma);
}

void 
AbstractBindingWMRepresenter::releaseToken(const cast::cdl::WorkingMemoryAddress& _wma)
{
  WMASet::iterator itr = tokens.find(_wma);
  assert(itr != tokens.end());
  stringstream str;
  str << BALTTimer::getBALTTime();
  component().debug("gonna release token " + string(_wma.id) + " in " +
		    string(_wma.subarchitecture) + " " +  str.str());
#ifndef NBINDING_TOKENS
  component.unlockEntry(_wma);
#endif //NBINDING_TOKENS
  tokens.erase(itr);
}

void 
AbstractBindingWMRepresenter::releaseAllTokens()
{
  while(!tokens().empty())
    releaseToken(*tokens().begin());
}



}  // namespace Binding 
