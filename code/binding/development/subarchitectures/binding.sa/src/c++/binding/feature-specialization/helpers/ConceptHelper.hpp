#ifndef BINDING_CONCEPT_HELPER_H_
#define BINDING_CONCEPT_HELPER_H_

#include <ostream>
#include <BindingFeaturesCommon.hpp>
#include <binding/idl/BindingFeatures.hh>
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"


namespace Binding {

class ConceptComparator : public ReflexiveInternalComparator<BindingFeatures::Concept> 
{
public:
  ConceptComparator() : 
    ReflexiveInternalComparator<BindingFeatures::Concept>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class ConceptHelper : public FeatureHelper<BindingFeatures::Concept> {
public:
  ConceptHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};

} // namespace Binding

#endif // BINDING_CONCEPT_HELPER_H_
