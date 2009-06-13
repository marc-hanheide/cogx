#ifndef BINDING_TEMPORALFRAME_HELPER_H_
#define BINDING_TEMPORALFRAME_HELPER_H_

#include <ostream>
#include <BindingFeaturesCommon.hpp>
#include <binding/idl/BindingFeatures.hh>
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"


namespace Binding {

class TemporalFrameComparator : public ReflexiveInternalComparator<BindingFeatures::TemporalFrame> 
{
public:
  TemporalFrameComparator() : 
    ReflexiveInternalComparator<BindingFeatures::TemporalFrame>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class TemporalFrameHelper : public FeatureHelper<BindingFeatures::TemporalFrame> {
public:
  TemporalFrameHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};

} // namespace Binding

#endif // BINDING_TEMPORALFRAME_HELPER_H_
