#include "FeatureLoader.hpp"
#include "binding/feature-utils/AbstractFeature.hpp"
#include "FeatureHelper.hpp"
#include <string>
#include <BindingData.hpp>


namespace Binding{
  using namespace boost;
  using namespace std;
  using namespace cast;

FeatureLoader::FeatureLoader(AbstractBindingWMRepresenter& _binder) : representer(_binder) {}

shared_ptr<AbstractFeature>&
FeatureLoader::getFeature(const BindingData::FeaturePointer& _feat, 
			  const std::string&  _bindingSubarchID) {
  
  string address(string(_feat.address));
  StringMap<shared_ptr<AbstractFeature> >::map::iterator itr = features.find(address);  
  
  if(itr == features.end()) {
    const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
    shared_ptr<AbstractFeature> feature = ontology.featureHelper(string(_feat.type)).getFeatureFromWM(representer,_feat,_bindingSubarchID);
    itr = (features.insert(make_pair(address,feature))).first;
  }
  return itr->second;
}

} //namespace Binding
