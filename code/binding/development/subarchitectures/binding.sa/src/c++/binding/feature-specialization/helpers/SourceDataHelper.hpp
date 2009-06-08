#ifndef BINDING_SOURCEDATA_HELPER_H_
#define BINDING_SOURCEDATA_HELPER_H_

#include <ostream>
#include <binding/idl/BindingFeaturesCommon.hh>
#include <binding/idl/BindingFeatures.hh>
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"


namespace Binding {

class SourceDataComparator : public ReflexiveInternalComparator<BindingFeatures::SourceData> 
{
public:
  SourceDataComparator() : 
    ReflexiveInternalComparator<BindingFeatures::SourceData>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class SourceDataHelper : public FeatureHelper<BindingFeatures::SourceData> {
public:
  SourceDataHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};

} // namespace Binding

#endif // BINDING_SOURCEDATA_HELPER_H_
