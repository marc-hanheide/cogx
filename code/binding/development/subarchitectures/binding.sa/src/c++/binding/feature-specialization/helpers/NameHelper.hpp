#ifndef BINDING_NAME_HELPER_H_
#define BINDING_NAME_HELPER_H_

#include <ostream>
#include <binding/idl/BindingFeaturesCommon.hh>
#include <binding/idl/BindingFeatures.hh>
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"


namespace Binding {

class NameComparator : public ReflexiveInternalComparator<BindingFeatures::Name> 
{
public:
  NameComparator() : 
    ReflexiveInternalComparator<BindingFeatures::Name>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class NameHelper : public FeatureHelper<BindingFeatures::Name> {
public:
  NameHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};

} // namespace Binding

#endif // BINDING_NAME_HELPER_H_
