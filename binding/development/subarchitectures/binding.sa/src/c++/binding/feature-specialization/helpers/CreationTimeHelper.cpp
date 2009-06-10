#include "CreationTimeHelper.hpp"
#include "SalienceHelper.hpp"

namespace Binding {
using namespace std;

CreationTimeHelper::CreationTimeHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant    = false;
  prop.m_isSimplex      = true;
  prop.m_isDemanded     = false;
  setProperties(prop);
}

ostream& 
CreationTimeHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << extract(_feat).m_creationTime;
  return _out;
}

bool 
CreationTimeHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return extract(_feat1).m_creationTime < extract(_feat2).m_creationTime;
}

} // namespace Binding
