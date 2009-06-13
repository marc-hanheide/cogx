#include "CoordinateHelper.hpp"
#include "ExperimentalUtil.hpp"

namespace Binding {
using namespace std;

boost::tribool 
CoordinateComparator::compare(const AbstractFeature& _proxyFeature, 
			      const AbstractFeature& _unionFeature) const {
  const BindingFeatures::Coordinate& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::Coordinate& unionFeature(getIDLFeature(_unionFeature));    
  if(proxyFeature.frame != unionFeature.frame)
    return boost::indeterminate;
  return proxyFeature.coordinate == unionFeature.coordinate;
}

CoordinateHelper::CoordinateHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant    = true;
  prop.isSimplex      = true;
  prop.isDemanded     = false;
  setProperties(prop);
}

ostream& 
CoordinateHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << extract(_feat).coordinate << "["<< extract(_feat).frame <<"]";
  return _out;
}

bool 
CoordinateHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  const BindingFeatures::Coordinate& feat1(extract(_feat1));
  const BindingFeatures::Coordinate& feat2(extract(_feat2));
  if(feat1.frame != feat2.frame)
    return feat1.frame < feat2.frame;
  return feat1.coordinate < feat2.coordinate;
}


} // namespace Binding
