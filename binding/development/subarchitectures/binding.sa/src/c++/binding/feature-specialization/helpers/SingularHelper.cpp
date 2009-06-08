#include "SingularHelper.hpp"

namespace Binding {
using namespace std;
using namespace boost;

tribool 
SingularComparator::compare(const AbstractFeature& _proxyFeature, 
			 const AbstractFeature& _unionFeature) const {
  const BindingFeatures::Singular& proxySingular(getIDLFeature(_proxyFeature));
  const BindingFeatures::Singular& unionSingular(getIDLFeature(_unionFeature));    
  //cout << "SingularComparator::compare: I am called with " << proxyFeature.m_singular << " vs. " << unionFeature.m_singular << endl;
  if(string(proxySingular.m_groupID) == string(unionSingular.m_groupID))
    return false;
  return indeterminate;
}

SingularHelper::SingularHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant   = false;
  prop.m_isSimplex     = true;
  prop.m_isDemanded    = false;
  setProperties(prop);
}

ostream& 
SingularHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  const BindingFeatures::Singular& singular(extract(_feat));
  _out << "element #" << singular.m_elementNumber <<" of " << singular.m_groupID;
  return _out;
}

bool 
SingularHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return string(extract(_feat1).m_groupID) < string(extract(_feat2).m_groupID);
}
  

} // namespace Binding
