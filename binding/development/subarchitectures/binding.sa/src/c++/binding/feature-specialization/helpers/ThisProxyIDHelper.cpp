#include "ThisProxyIDHelper.hpp"

namespace Binding {
using namespace std;

ThisProxyIDHelper::ThisProxyIDHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant    = true;
  prop.isSimplex      = true;
  prop.isDemanded     = true;
  setProperties(prop);
}

ostream& 
ThisProxyIDHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << extract(_feat).thisProxyID;
  return _out;
}

bool 
ThisProxyIDHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return string(extract(_feat1).thisProxyID) < string(extract(_feat2).thisProxyID);
}
  

} // namespace Binding
