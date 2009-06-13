#include "GroupHelper.hpp"

namespace Binding {
using namespace std;
using namespace boost;

tribool 
GroupComparator::compare(const AbstractFeature& _proxyFeature, 
			 const AbstractFeature& _unionFeature) const {
  const BindingFeatures::Group& proxyGroup(getIDLFeature(_proxyFeature));
  const BindingFeatures::Group& unionGroup(getIDLFeature(_unionFeature));    
//  cout << "GroupComparator::compare: I am called with " << proxyFeature.group << " vs. " << unionFeature.group << endl;

  if(proxyGroup.size == 0 || unionGroup.size == 0)
    return indeterminate;
  return proxyGroup.size == unionGroup.size;
}

GroupHelper::GroupHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant   = false;
  prop.isSimplex     = true;
  prop.isDemanded    = false;
  setProperties(prop);
}

ostream& 
GroupHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  const BindingFeatures::Group& group(extract(_feat));
  if(group.size == 0) { 
    _out << "<unbounded>"; 
  } else {
    _out << group.size; 
  }
  return _out;
}

bool 
GroupHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return extract(_feat1).size < extract(_feat2).size;
}
  

} // namespace Binding
