#include "SizeHelper.hpp"

namespace Binding {
using namespace std;

boost::tribool 
SizeComparator::compare(const AbstractFeature& _proxyFeature, 
			   const AbstractFeature& _unionFeature) const {
  const BindingFeatures::Size& proxyFeature(getIDLFeature(_proxyFeature));
  const BindingFeatures::Size& unionFeature(getIDLFeature(_unionFeature));    

  if(string(proxyFeature.m_size) == string(unionFeature.m_size))
    return true;
  return false;
}

SizeHelper::SizeHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant    = true;
  prop.m_isSimplex      = false;
  prop.m_isDemanded     = false;
  setProperties(prop);
}

ostream& 
SizeHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  _out << extract(_feat).m_size;
  return _out;
}

bool 
SizeHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return string(extract(_feat1).m_size) < string(extract(_feat2).m_size);
}
  

} // namespace Binding
