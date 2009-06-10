#include "SourceIDHelper.hpp"

namespace Binding {
using namespace std;

SourceIDHelper::SourceIDHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant    = true;
  prop.m_isSimplex      = true;
  prop.m_isDemanded     = true;
  setProperties(prop);
}

ostream& 
SourceIDHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << extract(_feat).m_sourceID << "(";
  _out << extract(_feat).m_monitorID << ")";
  return _out;
}

bool 
SourceIDHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  if(string(extract(_feat1).m_sourceID) != string(extract(_feat2).m_sourceID))
    return string(extract(_feat1).m_sourceID) < string(extract(_feat2).m_sourceID);
  return string(extract(_feat1).m_monitorID) < string(extract(_feat2).m_monitorID);
}
  

} // namespace Binding
