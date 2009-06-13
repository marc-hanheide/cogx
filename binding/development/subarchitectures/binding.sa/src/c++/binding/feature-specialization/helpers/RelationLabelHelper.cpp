#include "RelationLabelHelper.hpp"

namespace Binding {
using namespace std;

boost::tribool 
RelationLabelComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::RelationLabel& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::RelationLabel& unionFeature(getIDLFeature(_unionFeature));    
  if(string(proxyFeature.label) == string(unionFeature.label))
    return true;
  return false;
}

RelationLabelHelper::RelationLabelHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant    = true;
  prop.isSimplex      = true;
  prop.isDemanded     = false;
  setProperties(prop);
}

ostream& 
RelationLabelHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
    _out << "\\\"" << extract(_feat).label <<"\\\"";
    return _out;
  }

bool 
RelationLabelHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return string(extract(_feat1).label) < string(extract(_feat2).label);
}
  

} // namespace Binding
