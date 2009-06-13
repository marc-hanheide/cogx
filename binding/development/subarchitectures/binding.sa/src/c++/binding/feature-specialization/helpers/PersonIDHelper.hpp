#ifndef BINDING_PERSON_ID_HELPER_H_
#define BINDING_PERSON_ID_HELPER_H_

#include <ostream>
#include <BindingFeaturesCommon.hpp>
#include <binding/idl/BindingFeatures.hh>
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"


namespace Binding {

/// returns true if identical, otherwise false
class PersonIDComparator : public ReflexiveInternalComparator<BindingFeatures::PersonID> 
{
public:
  PersonIDComparator() : 
    ReflexiveInternalComparator<BindingFeatures::PersonID>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class PersonIDHelper : public FeatureHelper<BindingFeatures::PersonID> {
public:
  PersonIDHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};

} // namespace Binding

#endif // BINDING_PERSON_ID_HELPER_H_
