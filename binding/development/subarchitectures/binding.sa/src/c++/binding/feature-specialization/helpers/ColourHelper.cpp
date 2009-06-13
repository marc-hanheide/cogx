#include "ColourHelper.hpp"

namespace Binding {
using namespace std;

boost::tribool 
ColourComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::Colour& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::Colour& unionFeature(getIDLFeature(_unionFeature));    
  if(string(proxyFeature.colour) == string(unionFeature.colour))
    return true;
  return false;
}

ColourHelper::ColourHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant    = true;
  prop.isSimplex      = false;
  prop.isDemanded     = false;
  setProperties(prop);
}

ostream& 
ColourHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << extract(_feat).colour;
  return _out;
}

bool 
ColourHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return string(extract(_feat1).colour) < string(extract(_feat2).colour);
}
  

} // namespace Binding
