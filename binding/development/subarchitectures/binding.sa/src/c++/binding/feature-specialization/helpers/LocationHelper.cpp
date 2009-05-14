#include "LocationHelper.hpp"
#include <cmath>

namespace Binding {
using namespace std;


  inline double distance(const BindingFeatures::Location &v1, const BindingFeatures::Location &v2)
  {
    double dx = v1.m_location.m_x - v2.m_location.m_x;
    double dy = v1.m_location.m_y - v2.m_location.m_y;
    double dz = v1.m_location.m_z - v2.m_location.m_z;
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
  prop.m_isInvariant    = true;
  prop.m_isSimplex      = true;
  prop.m_isDemanded     = false;
  setProperties(prop);
}

ostream& 
LocationHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  const BindingFeatures::Location loc(extract(_feat));
  _out << "(" 
       << loc.m_location.m_x << ", "
       << loc.m_location.m_y << ", "
       << loc.m_location.m_z << ")";
  return _out;
}

bool 
LocationHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  
  const BindingFeatures::Location loc1(extract(_feat1));
  const BindingFeatures::Location loc2(extract(_feat2));
  
  if(loc1.m_location.m_x != loc2.m_location.m_x) {
    return loc1.m_location.m_x < loc2.m_location.m_x;
  }
  if(loc1.m_location.m_y != loc2.m_location.m_y) {    
    return loc1.m_location.m_y < loc2.m_location.m_y;
  }
  return loc1.m_location.m_z < loc2.m_location.m_z;  
}
  

} // namespace Binding
