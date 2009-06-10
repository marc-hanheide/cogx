#include "GroupHelper.hpp"

namespace Binding {
using namespace std;
using namespace boost;

tribool 
GroupComparator::compare(const AbstractFeature& _proxyFeature, 
			 const AbstractFeature& _unionFeature) const {
  const BindingFeatures::Group& proxyGroup(getIDLFeature(_proxyFeature));
  const BindingFeatures::Group& unionGroup(getIDLFeature(_unionFeature));    
//  cout << "GroupComparator::compare: I am called with " << proxyFeature.m_group << " vs. " << unionFeature.m_group << endl;

  if(proxyGroup.m_size == 0 || unionGroup.m_size == 0)
    return indeterminate;
  return proxyGroup.m_size == unionGroup.m_size;
}

GroupHelper::GroupHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant   = false;
  prop.m_isSimplex     = true;
  prop.m_isDemanded    = false;
  setProperties(prop);
}

ostream& 
GroupHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  const BindingFeatures::Group& group(extract(_feat));
  if(group.m_size == 0) { 
    _out << "<unbounded>"; 
  } else {
    _out << group.m_size; 
  }
  return _out;
}

bool 
GroupHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  return extract(_feat1).m_size < extract(_feat2).m_size;
}
  

} // namespace Binding
