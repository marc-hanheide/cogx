#include "FeatureLoader.hpp"
#include "binding/feature-utils/AbstractFeature.hpp"
#include "FeatureHelper.hpp"
#include <string>
#include <binding/idl/BindingData.hh>


namespace Binding{
  using namespace boost;
  using namespace std;
  using namespace cast;

FeatureLoader::FeatureLoader(AbstractBindingWMRepresenter& _binder) : m_representer(_binder) {}

shared_ptr<AbstractFeature>&
FeatureLoader::getFeature(const BindingData::FeaturePointer& _feat, 
			  const std::string&  _bindingSubarchID) {
  
  string address(string(_feat.m_address));
  StringMap<shared_ptr<AbstractFeature> >::map::iterator itr = m_features.find(address);  
  
  if(itr == m_features.end()) {
    const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
    shared_ptr<AbstractFeature> feature = ontology.featureHelper(string(_feat.m_type)).getFeatureFromWM(m_representer,_feat,_bindingSubarchID);
    itr = (m_features.insert(make_pair(address,feature))).first;
  }
  return itr->second;
}

} //namespace Binding
