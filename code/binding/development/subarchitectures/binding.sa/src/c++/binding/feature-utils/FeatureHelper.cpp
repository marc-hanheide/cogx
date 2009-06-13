#include "FeatureHelper.hpp"
#include "AbstractFeature.hpp"
#include "binding/ontology/BindingFeatureOntology.hpp"
#include "FeaturePropertiesUtils.hpp"

namespace Binding {
  using namespace std;

bool 
AbstractFeatureHelper::operatorLess(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  assert(_feat1.typeInfo() == _feat2.typeInfo());
  return operatorLessImpl(_feat1,_feat2);
}

bool 
AbstractFeatureHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return _feat1.featureID() < _feat2.featureID();
}

const std::type_info& 
getTypeInfo(const AbstractFeature& _feat) 
{
  return _feat.typeInfo();
}

void 
AbstractFeatureHelper::setProperties(const FeatureProperties& _properties) {
  if(properties.get() == NULL) {
    properties = auto_ptr<FeatureProperties>(new FeatureProperties(_properties));
  } else {
    *properties = _properties;
  }
}


} //namespace Binding 
