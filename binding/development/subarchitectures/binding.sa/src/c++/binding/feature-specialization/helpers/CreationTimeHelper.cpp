#include "CreationTimeHelper.hpp"
#include "SalienceHelper.hpp"

namespace Binding {
using namespace std;

CreationTimeHelper::CreationTimeHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant    = false;
  prop.isSimplex      = true;
  prop.isDemanded     = false;
  setProperties(prop);
}

ostream& 
CreationTimeHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << extract(_feat).creationTime;
  return _out;
}

bool 
CreationTimeHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return extract(_feat1).creationTime < extract(_feat2).creationTime;
}

} // namespace Binding
