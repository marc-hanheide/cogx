#include "CoordinateHelper.hpp"
#include "ExperimentalUtil.hpp"

namespace Binding {
using namespace std;

boost::tribool 
CoordinateComparator::compare(const AbstractFeature& _proxyFeature, 
			      const AbstractFeature& _unionFeature) const {
  const BindingFeatures::Coordinate& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::Coordinate& unionFeature(getIDLFeature(_unionFeature));    
  if(proxyFeature.m_frame != unionFeature.m_frame)
    return boost::indeterminate;
  return proxyFeature.m_coordinate == unionFeature.m_coordinate;
}

CoordinateHelper::CoordinateHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant    = true;
  prop.m_isSimplex      = true;
  prop.m_isDemanded     = false;
  setProperties(prop);
}

ostream& 
CoordinateHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << extract(_feat).m_coordinate << "["<< extract(_feat).m_frame <<"]";
  return _out;
}

bool 
CoordinateHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  const BindingFeatures::Coordinate& feat1(extract(_feat1));
  const BindingFeatures::Coordinate& feat2(extract(_feat2));
  if(feat1.m_frame != feat2.m_frame)
    return feat1.m_frame < feat2.m_frame;
  return feat1.m_coordinate < feat2.m_coordinate;
}


} // namespace Binding
