#include "OtherSourceIDHelper.hpp"

namespace Binding {
using namespace std;
using namespace boost;

boost::tribool 
OtherSourceIDvsSourceIDComparator::compare(const AbstractFeature& _proxyFeature, 
					   const AbstractFeature& _unionFeature) const 
{
  
  const BindingFeatures::OtherSourceID& other_source_id(getIDLFeature1(_proxyFeature,_unionFeature));
  const BindingFeatures::SourceID& source_id(getIDLFeature2(_proxyFeature,_unionFeature));

/*
  const BindingFeatures::OtherSourceID other_source_id(NULL);
  const BindingFeatures::SourceID* source_id(NULL);

  if(_proxyFeature.typeInfo() == typeid(BindingFeatures::OtherSourceID)) {
    assert(_unionFeature.typeInfo() == typeid(BindingFeatures::SourceID));
    other_source_id = &getIDLFeature1(_proxyFeature);
    source_id = &getIDLFeature2(_unionFeature);
  } else {
    assert(_proxyFeature.typeInfo() == typeid(BindingFeatures::SourceID));
    assert(_unionFeature.typeInfo() == typeid(BindingFeatures::OtherSourceID));
    other_source_id = &getIDLFeature1(_unionFeature);
    source_id = &getIDLFeature2(_proxyFeature);
  }
*/
  

  //const BindingFeatures::OtherSourceID& proxyFeature(getIDLFeature1(_proxyFeature));
  //const BindingFeatures::SourceID& unionFeature(getIDLFeature2(_unionFeature));    
  //  
  //cout << "comparing othersourceID \"" 
  //     << other_source_id->m_otherSourceID 
  //     << "\" vs. SourceID \"" 
  //     << source_id->m_sourceID << "\"\n";
  
  // do not compare if source id and other source id is in same proxy
  if(_proxyFeature.immediateProxyID() == _unionFeature.immediateProxyID()) {
    return indeterminate;
  }
  if(string(other_source_id.m_otherSourceID) == string(source_id.m_sourceID)) {
    return true;
  } 
  return false;
}

OtherSourceIDHelper::OtherSourceIDHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant    = true;
  prop.m_isSimplex      = true;
  prop.m_isDemanded     = false;
  setProperties(prop);
}

ostream& 
OtherSourceIDHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
    _out << extract(_feat).m_otherSourceID;
    return _out;
}

bool 
OtherSourceIDHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return string(extract(_feat1).m_otherSourceID) < string(extract(_feat2).m_otherSourceID);
}


} // namespace Binding
