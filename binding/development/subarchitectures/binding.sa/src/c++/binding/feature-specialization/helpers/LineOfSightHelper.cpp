#include "LineOfSightHelper.hpp"
#include "ExperimentalUtil.hpp"

namespace Binding {
using namespace std;
using namespace BindingFeatures;

LineOfSightHelper::LineOfSightHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant    = true;
  prop.isSimplex      = true;
  prop.isDemanded     = false;
  setProperties(prop);
}

ostream& 
LineOfSightHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  const LineOfSight& l(extract(_feat));
  _out << "viewpoint: " << l.from 
       << " dir: " << l.direction 
       << " [" << l.frame <<"]";
  return _out;
}

bool 
LineOfSightHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  const BindingFeatures::LineOfSight& feat1(extract(_feat1));
  const BindingFeatures::LineOfSight& feat2(extract(_feat2));
  if(feat1.frame != feat2.frame)
    return feat1.frame < feat2.frame;
  if(feat1.from != feat2.from)
    return feat1.from < feat2.from;
  return feat1.direction < feat2.direction;
}
  

} // namespace Binding
