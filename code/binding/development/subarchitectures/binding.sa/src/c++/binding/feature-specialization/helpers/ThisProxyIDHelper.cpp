#include "ThisProxyIDHelper.hpp"

namespace Binding {
using namespace std;

ThisProxyIDHelper::ThisProxyIDHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant    = true;
  prop.m_isSimplex      = true;
  prop.m_isDemanded     = true;
  setProperties(prop);
}

ostream& 
ThisProxyIDHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << extract(_feat).m_thisProxyID;
  return _out;
}

bool 
ThisProxyIDHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return string(extract(_feat1).m_thisProxyID) < string(extract(_feat2).m_thisProxyID);
}
  

} // namespace Binding
