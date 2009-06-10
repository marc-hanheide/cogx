#ifndef BINDING_DEBUGSTRING_HELPER_H_
#define BINDING_DEBUGSTRING_HELPER_H_

#include <ostream>
#include <binding/idl/BindingFeaturesCommon.hh>
#include <binding/idl/BindingFeatures.hh>
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"


namespace Binding {

class DebugStringHelper : public FeatureHelper<BindingFeatures::DebugString> {
public:
  DebugStringHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  //virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};

} // namespace Binding

#endif // BINDING_DEBUGSTRING_HELPER_H_
