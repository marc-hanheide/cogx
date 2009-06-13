#ifndef BINDING_THISPROXYID_HELPER_H_
#define BINDING_THISPROXYID_HELPER_H_

#include <ostream>
#include <BindingFeaturesCommon.hpp>
#include <binding/idl/BindingFeatures.hh>
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"


namespace Binding {

class ThisProxyIDHelper : public FeatureHelper<BindingFeatures::ThisProxyID> {
public:
  ThisProxyIDHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};

} // namespace Binding

#endif // BINDING_THISPROXYID_HELPER_H_
