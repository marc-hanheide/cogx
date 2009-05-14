#include "AspectualStateHelper.hpp"
#include "boost/algorithm/string/predicate.hpp"

namespace Binding {
using namespace std;


boost::tribool 
AspectualStateComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::AspectualState& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::AspectualState& unionFeature(getIDLFeature(_unionFeature));    
  using boost::iequals;
  if(iequals(string(proxyFeature.m_aspectualState), 
	     string(unionFeature.m_aspectualState)))
    return true;
  return false;
}

AspectualStateHelper::AspectualStateHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant    = true;
  prop.m_isSimplex      = true;
  prop.m_isDemanded     = false;
  setProperties(prop);
}

ostream& 
AspectualStateHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
    _out << "\\\"" << extract(_feat).m_aspectualState <<"\\\"";
    return _out;
}

bool 
AspectualStateHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return string(extract(_feat1).m_aspectualState) < string(extract(_feat2).m_aspectualState);
}
  

} // namespace Binding
