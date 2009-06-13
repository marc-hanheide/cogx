#ifndef BINDING_SINGULAR_HELPER_H_
#define BINDING_SINGULAR_HELPER_H_

#include <ostream>
#include <BindingFeaturesCommon.hpp>
#include <binding/idl/BindingFeatures.hh>
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"


namespace Binding {

/// returns true if they're NOT equal... this to prevent binding of singulars from the same group
class SingularComparator : public ReflexiveInternalComparator<BindingFeatures::Singular> 
{
public:
  SingularComparator() : 
    ReflexiveInternalComparator<BindingFeatures::Singular>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class SingularHelper : public FeatureHelper<BindingFeatures::Singular> {
public:
  SingularHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};

} // namespace Binding

#endif // BINDING_SINGULAR_HELPER_H_
