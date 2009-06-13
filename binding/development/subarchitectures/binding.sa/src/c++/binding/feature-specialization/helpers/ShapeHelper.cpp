#include "ShapeHelper.hpp"

namespace Binding {
using namespace std;

boost::tribool 
ShapeComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::Shape& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::Shape& unionFeature(getIDLFeature(_unionFeature));    
  if(string(proxyFeature.shape) == string(unionFeature.shape))
    return true;
  return false;
}

ShapeHelper::ShapeHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant    = true;
  prop.isSimplex      = false;
  prop.isDemanded     = false;
  setProperties(prop);
}

ostream& 
ShapeHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
    _out << extract(_feat).shape;
    return _out;
  }

bool 
ShapeHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return string(extract(_feat1).shape) < string(extract(_feat2).shape);
}
  

} // namespace Binding
