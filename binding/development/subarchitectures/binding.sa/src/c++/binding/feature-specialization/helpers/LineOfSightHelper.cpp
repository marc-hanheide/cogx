#include "LineOfSightHelper.hpp"
#include "ExperimentalUtil.hpp"

namespace Binding {
using namespace std;
using namespace BindingFeatures;

LineOfSightHelper::LineOfSightHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant    = true;
  prop.m_isSimplex      = true;
  prop.m_isDemanded     = false;
  setProperties(prop);
}

ostream& 
LineOfSightHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  const LineOfSight& l(extract(_feat));
  _out << "viewpoint: " << l.m_from 
       << " dir: " << l.m_direction 
       << " [" << l.m_frame <<"]";
  return _out;
}

bool 
LineOfSightHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  const BindingFeatures::LineOfSight& feat1(extract(_feat1));
  const BindingFeatures::LineOfSight& feat2(extract(_feat2));
  if(feat1.m_frame != feat2.m_frame)
    return feat1.m_frame < feat2.m_frame;
  if(feat1.m_from != feat2.m_from)
    return feat1.m_from < feat2.m_from;
  return feat1.m_direction < feat2.m_direction;
}
  

} // namespace Binding
