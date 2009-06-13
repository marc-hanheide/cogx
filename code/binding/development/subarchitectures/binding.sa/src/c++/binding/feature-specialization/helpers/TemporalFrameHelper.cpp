#include "TemporalFrameHelper.hpp"

namespace Binding {
using namespace std;
using namespace boost;


boost::tribool 
TemporalFrameComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::TemporalFrame& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::TemporalFrame& unionFeature(getIDLFeature(_unionFeature));    
  
  if(proxyFeature.temporalFrame == BindingFeaturesCommon::NA ||
     unionFeature.temporalFrame == BindingFeaturesCommon::NA)
    return indeterminate;
  if(proxyFeature.temporalFrame == unionFeature.temporalFrame)
    return indeterminate;
  switch(proxyFeature.temporalFrame) {
  case BindingFeaturesCommon::PERCEIVED:
  case BindingFeaturesCommon::ASSERTED:
    switch(unionFeature.temporalFrame) {
    case BindingFeaturesCommon::PERCEIVED:
    case BindingFeaturesCommon::ASSERTED:
      return indeterminate;
    case BindingFeaturesCommon::DESIRED:
    case BindingFeaturesCommon::PAST:
    case BindingFeaturesCommon::TYPICAL:
      return false;
    case BindingFeaturesCommon::NA:
      assert(false); // NA already tested before switch-statement...
    }
  case BindingFeaturesCommon::TYPICAL:
    return false;
  case BindingFeaturesCommon::DESIRED:
    return false; // equality already tested
  case BindingFeaturesCommon::PAST:
    return false; // equality already tested
  case BindingFeaturesCommon::NA:
    assert(false); // NA already tested before switch-statement...
  }
  throw runtime_error("execution cannot reach this point");
}

TemporalFrameHelper::TemporalFrameHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant    = false;
  prop.isSimplex      = true;
  prop.isDemanded     = false;
  setProperties(prop);
}

ostream&
operator<<(ostream& _out, const BindingFeaturesCommon::TemporalFrameType _frame)
{
  _out << "<";
  switch(_frame) {
  case BindingFeaturesCommon::PERCEIVED:
    _out << "PERCEIVED";
    break;
  case BindingFeaturesCommon::ASSERTED:
    _out << "ASSERTED";
    break;
  case BindingFeaturesCommon::TYPICAL:
    _out << "TYPICAL";
    break;
  case BindingFeaturesCommon::DESIRED:
    _out << "DESIRED";
    break;
  case BindingFeaturesCommon::PAST:
    _out << "PAST";
    break;
  case BindingFeaturesCommon::NA:
    _out << "N/A";
    break;
  default:
    throw BindingException("Incorrect BindingFeaturesCommon::TemporalFrameType specified");
  }
  _out << ">";
  return _out;
}


ostream& 
TemporalFrameHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << extract(_feat).temporalFrame;
  return _out;
}

bool 
TemporalFrameHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return extract(_feat1).temporalFrame < extract(_feat2).temporalFrame;
}
  

} // namespace Binding
