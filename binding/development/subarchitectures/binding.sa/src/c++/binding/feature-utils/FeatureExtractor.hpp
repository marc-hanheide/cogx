#ifndef BINDING_FEATURE_EXTRACTOR_H_
#define BINDING_FEATURE_EXTRACTOR_H_

#include <boost/shared_ptr.hpp>
#include <balt/core/StringMap.hpp>
#include <stdexcept>

//#include "feature-utils/AbstractFeature.hpp"
#include "binding/feature-utils/Features.hpp"
#include "binding/ontology/BindingFeatureOntology.hpp"
#include "binding/BindingException.hpp"

namespace Binding {


/// A helper function to make it easier to cast from an abstract
/// feature
template <typename IDLFeatureT>
const Feature<IDLFeatureT>& 
extractFeature(const AbstractFeature& _abstractFeature)
{
  const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  if(_abstractFeature.name() != ontology.featureName(typeid(IDLFeatureT))) {
    throw BindingException("Error in casting abstract feature into concrete feature: " + _abstractFeature.name() + " -> " + ontology.featureName(typeid(IDLFeatureT)));
  }
  const Feature<IDLFeatureT>& feature(dynamic_cast<const Feature<IDLFeatureT>&>(_abstractFeature));
  return feature;
}

template <typename IDLFeatureT>
const Feature<IDLFeatureT>& 
extractFeature(const boost::shared_ptr<AbstractFeature>& _abstractFeature)
{
  return extractFeature<IDLFeatureT>(*_abstractFeature);
}

/// A helper function to make it easier to cast from an abstract
/// feature to the original feature type as specified in the IDL
template <typename IDLFeatureT>
const IDLFeatureT& 
extractIDLFeature(const AbstractFeature& _abstractFeature) 
{
  return extractFeature<IDLFeatureT>(_abstractFeature).idlFeature();
}

template <typename IDLFeatureT>
const IDLFeatureT& 
extractIDLFeature(const boost::shared_ptr<AbstractFeature>& _abstractFeature) 
{
  return extractFeature<IDLFeatureT>(*_abstractFeature).idlFeature();  
}


} // namespace Binding

#endif // BINDING_FEATURE_ONTOLOGY_H_
