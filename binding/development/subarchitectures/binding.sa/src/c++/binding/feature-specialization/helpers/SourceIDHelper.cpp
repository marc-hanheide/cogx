#include "SourceIDHelper.hpp"

namespace Binding {
using namespace std;

SourceIDHelper::SourceIDHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant    = true;
  prop.isSimplex      = true;
  prop.isDemanded     = true;
  setProperties(prop);
}

ostream& 
SourceIDHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << extract(_feat).sourceID << "(";
  _out << extract(_feat).monitorID << ")";
  return _out;
}

bool 
SourceIDHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  if(string(extract(_feat1).sourceID) != string(extract(_feat2).sourceID))
    return string(extract(_feat1).sourceID) < string(extract(_feat2).sourceID);
  return string(extract(_feat1).monitorID) < string(extract(_feat2).monitorID);
}
  

} // namespace Binding
