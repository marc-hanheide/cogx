#include "LocationHelper.hpp"
#include <cmath>

namespace Binding {
using namespace std;


  inline double distance(const BindingFeatures::Location &v1, const BindingFeatures::Location &v2)
  {
    double dx = v1.location.x - v2.location.x;
    double dy = v1.location.y - v2.location.y;
    double dz = v1.location.z - v2.location.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }


boost::tribool 
LocationComparator::compare(const AbstractFeature& _proxyFeature, 
			    const AbstractFeature& _unionFeature) const {
  const BindingFeatures::Location& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::Location& unionFeature(getIDLFeature(_unionFeature));    

  //HACK threshold
  double dist = distance(proxyFeature,unionFeature);

  //2cm magic threshold
  if(dist < 0.02) {
    return true;
  }
  else {
    return false;
  }

}

LocationHelper::LocationHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant    = true;
  prop.isSimplex      = true;
  prop.isDemanded     = false;
  setProperties(prop);
}

ostream& 
LocationHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  const BindingFeatures::Location loc(extract(_feat));
  _out << "(" 
       << loc.location.x << ", "
       << loc.location.y << ", "
       << loc.location.z << ")";
  return _out;
}

bool 
LocationHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  
  const BindingFeatures::Location loc1(extract(_feat1));
  const BindingFeatures::Location loc2(extract(_feat2));
  
  if(loc1.location.x != loc2.location.x) {
    return loc1.location.x < loc2.location.x;
  }
  if(loc1.location.y != loc2.location.y) {    
    return loc1.location.y < loc2.location.y;
  }
  return loc1.location.z < loc2.location.z;  
}
  

} // namespace Binding
