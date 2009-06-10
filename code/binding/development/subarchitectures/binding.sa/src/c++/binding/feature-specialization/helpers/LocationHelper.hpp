#ifndef BINDING_LOCATION_HELPER_H_
#define BINDING_LOCATION_HELPER_H_

#include <ostream>
#include <binding/idl/BindingFeaturesCommon.hh>
#include <binding/idl/BindingFeatures.hh>
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"


namespace Binding {

class LocationComparator : public ReflexiveInternalComparator<BindingFeatures::Location> 
{
public:
  LocationComparator() : 
    ReflexiveInternalComparator<BindingFeatures::Location>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class LocationHelper : public FeatureHelper<BindingFeatures::Location> {
public:
  LocationHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};

} // namespace Binding

#endif // BINDING_LOCATION_HELPER_H_
