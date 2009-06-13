#include "DebugStringHelper.hpp"

namespace Binding {
using namespace std;
 
DebugStringHelper::DebugStringHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant    = false;
  prop.isSimplex      = false;
  prop.isDemanded     = false;
  setProperties(prop);
}

ostream& 
DebugStringHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << "\\\"" << extract(_feat).debugString <<"\\\"";
  return _out;
}


} // namespace Binding
