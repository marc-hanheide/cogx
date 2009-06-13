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
  if(!proxyData.comparable || !unionData.comparable)
    return indeterminate;
  // only compare those of the same type
  if(string(proxyData.type) != string(unionData.type)) {
    return indeterminate;
  }
  // otherwise, match their addresses
  if(string(proxyData.address.id) == string(unionData.address.id) &&
     string(proxyData.address.subarchitecture) == string(unionData.address.subarchitecture) ) 
    return true;
  return false;
}

SourceDataHelper::SourceDataHelper() 
{
  FeatureProperties prop; 
  prop.isInvariant    = false;
  prop.isSimplex      = false;
  prop.isDemanded     = false;
  setProperties(prop);
}

ostream& 
SourceDataHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  const BindingFeatures::SourceData& data(extract(_feat));
  _out << "(" + toString(data.type) << "," 
       << data.address.subarchitecture << ":" 
       << data.address.id <<  ")";
  return _out;
}

bool 
SourceDataHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  const BindingFeatures::SourceData& data1(extract(_feat1));
  const BindingFeatures::SourceData& data2(extract(_feat2));
  // primarily, compare ontological data type
  if(      string(data1.type) != string(data2.type))
    return string(data1.type) <  string(data2.type);
  // secondarily, compare subarchitectures
  if(      string(data1.address.subarchitecture) != string(data2.address.subarchitecture))
    return string(data1.address.subarchitecture) <  string(data2.address.subarchitecture);
  // and last, compare the adresses
  return   string(data1.address.id) != string(data2.address.id);
}
  

} // namespace Binding
