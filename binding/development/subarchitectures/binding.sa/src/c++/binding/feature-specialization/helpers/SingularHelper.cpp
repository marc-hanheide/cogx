#include "SingularHelper.hpp"

namespace Binding {
using namespace std;
using namespace boost;

tribool 
SingularComparator::compare(const AbstractFeature& _proxyFeature, 
			 const AbstractFeature& _unionFeature) const {
  const BindingFeatures::Singular& proxySingular(getIDLFeature(_proxyFeature));
  const BindingFeatures::Singular& unionSingular(getIDLFeature(_unionFeature));    
  //cout << "SingularComparator::compare: I am called with " << proxyFeature.singular << " vs. " << unionFeature.singular << endl;
  if(string(proxySingular.groupID) == string(unionSingular.groupID))
    return false;
  return indeterminate;
}

SingularHelper::SingularHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant   = false;
  prop.isSimplex     = true;
  prop.isDemanded    = false;
  setProperties(prop);
}

ostream& 
SingularHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  const BindingFeatures::Singular& singular(extract(_feat));
  _out << "element #" << singular.elementNumber <<" of " << singular.groupID;
  return _out;
}

bool 
SingularHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return string(extract(_feat1).groupID) < string(extract(_feat2).groupID);
}
  

} // namespace Binding
