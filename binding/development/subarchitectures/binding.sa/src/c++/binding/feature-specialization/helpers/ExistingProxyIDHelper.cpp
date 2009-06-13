#include "ExistingProxyIDHelper.hpp"

namespace Binding {
using namespace std;
using namespace boost;

boost::tribool 
ExistingProxyIDvsThisProxyIDComparator::compare(const AbstractFeature& _proxyFeature, 
						const AbstractFeature& _unionFeature) const {
  const BindingFeatures::ExistingProxyID& existingproxy(getIDLFeature1(_proxyFeature,_unionFeature));
  const BindingFeatures::ThisProxyID& thisproxy(getIDLFeature2(_proxyFeature,_unionFeature));    
  if(string(existingproxy.existingProxyID) == string(thisproxy.thisProxyID))
    return true;
  return indeterminate;
}

ExistingProxyIDHelper::ExistingProxyIDHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant    = false;
  prop.isSimplex      = true;
  prop.isDemanded     = false;
  setProperties(prop);
}

ostream& 
ExistingProxyIDHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << extract(_feat).existingProxyID;
  return _out;
}

bool 
ExistingProxyIDHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return string(extract(_feat1).existingProxyID) < string(extract(_feat2).existingProxyID);
}
  

} // namespace Binding
