#ifndef BINDING_FEATURE_REGISTRANT_H_
#define BINDING_FEATURE_REGISTRANT_H_
#include <boost/shared_ptr.hpp>

namespace Binding {
class BindingFeatureOntology;
  class AbstractInternalComparator;
class FeatureRegistrant {
  friend class BindingFeatureOntology;
  /// registers features to the BindingFeatureOntology
  static void registerFeatures(BindingFeatureOntology&);
  static void registerInternalComparator(BindingFeatureOntology&, 
					 boost::shared_ptr<const AbstractInternalComparator>);
};

} // namespace Binding

#endif // BINDING_FEATURE_ONTOLOGY_H_
