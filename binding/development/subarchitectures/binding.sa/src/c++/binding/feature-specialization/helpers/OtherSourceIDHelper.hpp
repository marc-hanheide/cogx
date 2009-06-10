#ifndef BINDING_OTHERSOURCEID_HELPER_H_
#define BINDING_OTHERSOURCEID_HELPER_H_

#include <ostream>
#include <binding/idl/BindingFeaturesCommon.hh>
#include <binding/idl/BindingFeatures.hh>
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"


namespace Binding {
  
/// compares with the \p SourceID of existing unions. Only compares \p
/// BindingFeatures::OtherSourceID::m_sourceID, not the monitor ID
class OtherSourceIDvsSourceIDComparator 
  : public SymmetricInternalComparator<BindingFeatures::OtherSourceID, BindingFeatures::SourceID> 
{
public:
  OtherSourceIDvsSourceIDComparator() : 
    SymmetricInternalComparator<BindingFeatures::OtherSourceID, BindingFeatures::SourceID>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class OtherSourceIDHelper : public FeatureHelper<BindingFeatures::OtherSourceID> {
public:
  OtherSourceIDHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};

} // namespace Binding

#endif // BINDING_OTHERSOURCEID_HELPER_H_
