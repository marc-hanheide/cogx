#ifndef BINDING_FEATURE_LOADER_H_
#define BINDING_FEATURE_LOADER_H_

#include <boost/shared_ptr.hpp>
#include <balt/core/StringMap.hpp>

namespace BindingData {
class FeaturePointer;
}
namespace Binding {

class BindingFeatureOntology;
class AbstractBindingWMRepresenter;
class AbstractFeature;

/// helps loading features into local memory
class FeatureLoader {
public:
  FeatureLoader(AbstractBindingWMRepresenter&);
  /// just loads the feature, and store it locally forever (for now, will be refactored). A feature
  /// is 'const' on WM anyway.
  boost::shared_ptr<AbstractFeature>& getFeature(const BindingData::FeaturePointer& _feat, const std::string& _bindingSAID = "");
  
private:  
  FeatureLoader(const FeatureLoader&);
  FeatureLoader& operator=(const FeatureLoader&);


  AbstractBindingWMRepresenter& representer;
  cast::StringMap<boost::shared_ptr<AbstractFeature> >::map features;
};


} // namespace Binding

#endif // BINDING_FEATURE_ONTOLOGY_H_
