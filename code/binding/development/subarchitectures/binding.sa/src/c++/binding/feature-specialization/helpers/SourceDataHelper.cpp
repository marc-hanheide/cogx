#include "SourceDataHelper.hpp"

namespace Binding {
using namespace std;
using namespace boost;

boost::tribool 
SourceDataComparator::compare(const AbstractFeature& _proxyFeature, 
			      const AbstractFeature& _unionFeature) const {
  const BindingFeatures::SourceData& proxyData(getIDLFeature(_proxyFeature));
  const BindingFeatures::SourceData& unionData(getIDLFeature(_unionFeature));    
  // only compare those features that are specified to be comparable
  if(!proxyData.m_comparable || !unionData.m_comparable)
    return indeterminate;
  // only compare those of the same type
  if(string(proxyData.m_type) != string(unionData.m_type)) {
    return indeterminate;
  }
  // otherwise, match their addresses
  if(string(proxyData.m_address.m_id) == string(unionData.m_address.m_id) &&
     string(proxyData.m_address.m_subarchitecture) == string(unionData.m_address.m_subarchitecture) ) 
    return true;
  return false;
}

SourceDataHelper::SourceDataHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant    = false;
  prop.m_isSimplex      = false;
  prop.m_isDemanded     = false;
  setProperties(prop);
}

ostream& 
SourceDataHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  const BindingFeatures::SourceData& data(extract(_feat));
  _out << "(" + toString(data.m_type) << "," 
       << data.m_address.m_subarchitecture << ":" 
       << data.m_address.m_id <<  ")";
  return _out;
}

bool 
SourceDataHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  const BindingFeatures::SourceData& data1(extract(_feat1));
  const BindingFeatures::SourceData& data2(extract(_feat2));
  // primarily, compare ontological data type
  if(      string(data1.m_type) != string(data2.m_type))
    return string(data1.m_type) <  string(data2.m_type);
  // secondarily, compare subarchitectures
  if(      string(data1.m_address.m_subarchitecture) != string(data2.m_address.m_subarchitecture))
    return string(data1.m_address.m_subarchitecture) <  string(data2.m_address.m_subarchitecture);
  // and last, compare the adresses
  return   string(data1.m_address.m_id) != string(data2.m_address.m_id);
}
  

} // namespace Binding
