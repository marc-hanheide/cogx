#ifndef BINDING_GROUP_HELPER_H_
#define BINDING_GROUP_HELPER_H_

#include <ostream>
#include <binding/idl/BindingFeaturesCommon.hh>
#include <binding/idl/BindingFeatures.hh>
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"


namespace Binding {

/// just compares the sizes of groups (returns indeterminate if any of
/// them are unbounded)
class GroupComparator : public ReflexiveInternalComparator<BindingFeatures::Group> 
{
public:
  GroupComparator() : 
    ReflexiveInternalComparator<BindingFeatures::Group>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class GroupHelper : public FeatureHelper<BindingFeatures::Group> {
public:
  GroupHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};

} // namespace Binding

#endif // BINDING_GROUP_HELPER_H_
