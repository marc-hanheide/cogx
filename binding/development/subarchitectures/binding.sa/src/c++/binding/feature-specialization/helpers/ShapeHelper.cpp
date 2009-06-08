#include "ShapeHelper.hpp"

namespace Binding {
using namespace std;

boost::tribool 
ShapeComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::Shape& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::Shape& unionFeature(getIDLFeature(_unionFeature));    
  if(string(proxyFeature.m_shape) == string(unionFeature.m_shape))
    return true;
  return false;
}

ShapeHelper::ShapeHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant    = true;
  prop.m_isSimplex      = false;
  prop.m_isDemanded     = false;
  setProperties(prop);
}

ostream& 
ShapeHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
    _out << extract(_feat).m_shape;
    return _out;
  }

bool 
ShapeHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return string(extract(_feat1).m_shape) < string(extract(_feat2).m_shape);
}
  

} // namespace Binding
