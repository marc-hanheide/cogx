#include "PersonIDHelper.hpp"

namespace Binding {
using namespace std;

boost::tribool 
PersonIDComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::PersonID& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::PersonID& unionFeature(getIDLFeature(_unionFeature));    
  return proxyFeature.id == unionFeature.id;
}

PersonIDHelper::PersonIDHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant    = true;
  prop.isSimplex      = true;
  prop.isDemanded     = false;
  setProperties(prop);
}

ostream& 
PersonIDHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << "#"<< extract(_feat).id;
  return _out;
}

bool 
PersonIDHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return extract(_feat1).id < extract(_feat2).id;
}
  

} // namespace Binding
