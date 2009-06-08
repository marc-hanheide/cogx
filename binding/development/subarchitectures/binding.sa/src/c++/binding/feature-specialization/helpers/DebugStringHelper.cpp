#include "DebugStringHelper.hpp"

namespace Binding {
using namespace std;
 
DebugStringHelper::DebugStringHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant    = false;
  prop.m_isSimplex      = false;
  prop.m_isDemanded     = false;
  setProperties(prop);
}

ostream& 
DebugStringHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << "\\\"" << extract(_feat).m_debugString <<"\\\"";
  return _out;
}


} // namespace Binding
