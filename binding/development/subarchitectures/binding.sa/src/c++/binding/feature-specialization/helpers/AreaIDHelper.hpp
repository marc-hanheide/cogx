#ifndef BINDING_AREA_ID_HELPER_H_
#define BINDING_AREA_ID_HELPER_H_

#include <ostream>
#include <binding/idl/BindingFeaturesCommon.hh>
#include <binding/idl/BindingFeatures.hh>
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"


namespace Binding {

/// returns true if identical, otherwise false
class AreaIDComparator : public ReflexiveInternalComparator<BindingFeatures::AreaID> 
{
public:
  AreaIDComparator() : 
    ReflexiveInternalComparator<BindingFeatures::AreaID>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class AreaIDHelper : public FeatureHelper<BindingFeatures::AreaID> {
public:
  AreaIDHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};

} // namespace Binding

#endif // BINDING_AREA_ID_HELPER_H_
