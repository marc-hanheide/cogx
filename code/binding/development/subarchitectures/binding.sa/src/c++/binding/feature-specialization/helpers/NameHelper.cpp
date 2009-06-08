#include "NameHelper.hpp"

namespace Binding {
using namespace std;

boost::tribool 
NameComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::Name& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::Name& unionFeature(getIDLFeature(_unionFeature));    
  if(string(proxyFeature.m_name) == string(unionFeature.m_name))
    return true;
  return false;
}

NameHelper::NameHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant    = true;
  prop.m_isSimplex      = true;
  prop.m_isDemanded     = false;
  setProperties(prop);
}

ostream& 
NameHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
    _out << "\\\"" << extract(_feat).m_name <<"\\\"";
    return _out;
  }

bool 
NameHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return string(extract(_feat1).m_name) < string(extract(_feat2).m_name);
}
  

} // namespace Binding
