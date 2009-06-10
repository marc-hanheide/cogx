#include "ConceptHelper.hpp"
#include "boost/algorithm/string/predicate.hpp"

namespace Binding {
using namespace std;


boost::tribool 
ConceptComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::Concept& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::Concept& unionFeature(getIDLFeature(_unionFeature));    
  using boost::iequals;
  string union_concept(unionFeature.m_concept);
  string proxy_concept(proxyFeature.m_concept);
  if(!union_concept.empty() && union_concept[0] == '?')
    return true;
  if(!proxy_concept.empty() && proxy_concept[0] == '?')
    return true;
  if(iequals(proxy_concept,union_concept))
    return true;
  return false;
}

ConceptHelper::ConceptHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant    = true;
  prop.m_isSimplex      = true;
  prop.m_isDemanded     = false;
  setProperties(prop);
}

ostream& 
ConceptHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
    _out << "\\\"" << extract(_feat).m_concept <<"\\\"";
    return _out;
}

bool 
ConceptHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return string(extract(_feat1).m_concept) < string(extract(_feat2).m_concept);
}
  

} // namespace Binding
