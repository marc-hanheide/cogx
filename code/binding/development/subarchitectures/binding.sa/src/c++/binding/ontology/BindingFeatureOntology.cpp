
#include "BindingFeatureOntology.hpp"
#include "binding/feature-specialization/FeatureRegistrant.hpp"
#include "binding/utils/BindingUtils.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"
#include "BindingData.hpp"
#include "binding/feature-utils/FeatureHelper.hpp"

#include <boost/assign.hpp>
#include <set>
#include <map>

namespace Binding {

using namespace boost::assign;
using namespace std;
using namespace boost;
using namespace cast;


/*set<string> BindingFeatureOntology::features = set<string>();
set<string> BindingFeatureOntology::comparableFeatures = set<string>();
map<string,set<string> > BindingFeatureOntology::proxy2unionComparable = map<string,set<string> >();
map<string,set<string> > BindingFeatureOntology::union2proxyComparable = map<string,set<string> >();
map<string,Binding::FeatureProperties> BindingFeatureOntology::featurePropertyMap = map<string,Binding::FeatureProperties>();
BindingFeatureOntology::TrustSpecMap BindingFeatureOntology::internalTrust = TrustSpecMap();
StringMap<BindingFeatureOntology::FeatureStats>::map BindingFeatureOntology::featureStats = StringMap<BindingFeatureOntology::FeatureStats>::map();
*/

ostream& operator<<(ostream& out, const set<string>& strings) {
  for(set<string>::const_iterator i = strings.begin() ; i != strings.end() ; i++)
    out << *i << " ";
  return out;
}

BindingFeatureOntology::BindingFeatureOntology() 
  :
    ontologyState(PRE_CONSTRUCT)
{
  FeatureRegistrant::registerFeatures(*this);
  constructorFinished();
}
 

void 
BindingFeatureOntology::updateFeatureProperties(const string& _featureType, const AbstractFeatureHelper& _helper)
{
  updateFeatureProperties(_featureType, _helper.properties());
}
void 
BindingFeatureOntology::updateFeatureProperties(const string& _featureType, const Binding::FeatureProperties& _properties) 
{
  featurePropertyMap[_featureType] = _properties;
  features.insert(_featureType);
  
  const set<string>& comparableInternally(_properties.comparableInternally);
  const set<string>& comparableExternally(_properties.comparableExternally);
  proxy2unionComparable[_featureType].insert(comparableInternally.begin(),comparableInternally.end());
  proxy2unionComparable[_featureType].insert(comparableExternally.begin(),comparableExternally.end());
  if(!comparableInternally.empty() ||
     !comparableExternally.empty()) {
    comparableFeatures.insert(_featureType);
    comparableFeatures.insert(comparableInternally.begin(),comparableInternally.end());
    comparableFeatures.insert(comparableExternally.begin(),comparableExternally.end());
    for(set<string>::const_iterator itr = comparableInternally.begin() ; 
	itr != comparableInternally.end() ; 
	  ++itr) {
      union2proxyComparable[*itr].insert(_featureType);
    }
    for(set<string>::const_iterator itr = comparableExternally.begin() ; 
	itr != comparableExternally.end() ; 
	  ++itr) {
      union2proxyComparable[*itr].insert(_featureType);
    }
  } 
}


void 
BindingFeatureOntology::registerComparatorCompetence(const BindingData::FeatureComparisonCompetence& _competence) {
  assert(ontologyState == CONSTRUCTED);
      
  stringstream str;
  Binding::operator<<(str,_competence);
  cout << "\ncomparator competence registered:\n" << str.str() << endl;
//  const map<string,Binding::FeatureProperties>& properties(BindingFeatureOntology::featurePropertyMap());
  const string proxyFeatureType(_competence.proxyFeatureType);
  const string unionFeatureType(_competence.unionFeatureType);
  
  comparableExternally[featureNumber(proxyFeatureType)].insert(unionFeatureType);  
  Binding::FeatureProperties new_prop = featureHelper(proxyFeatureType).properties();//properties.find(proxyFeatureType)->second;
  new_prop.comparableExternally.insert(unionFeatureType);
  BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  ontology.updateFeatureProperties(proxyFeatureType, new_prop);    
  if(internalTrust.find(proxyFeatureType) != internalTrust.end() &&
     internalTrust.find(proxyFeatureType)->second.find(unionFeatureType) != 
     internalTrust.find(proxyFeatureType)->second.end()) {
    stringstream str2;
    Binding::operator<<(str2,internalTrust[proxyFeatureType][unionFeatureType]);
    cerr << "WARNING! Same comparator competence registered twice:\n" <<  str.str() << " and previously with " << str2.str() << endl;
  }
  internalTrust[proxyFeatureType][unionFeatureType]= _competence.comparisonTrustSpecification;
}

  
const std::string& 
BindingFeatureOntology::featureName(const std::type_info& _info) const 
{
  assert(ontologyState > FEATURE_REGISTRATION); 
  TypeMap<std::string>::map::const_iterator itr = featureName.find(&_info);
  if(itr == featureName.end()) {
    throw(BindingException("in BindingFeatureOntology::featureName(const std::type_info& _info): Not a feature, it seems: " + string(_info.name())));
  }
  return itr->second;
}

unsigned int 
BindingFeatureOntology::featureNumber(const std::type_info& _info) const 
{
  assert(ontologyState > FEATURE_REGISTRATION); 
  TypeMap<unsigned int>::map::const_iterator itr = featureNumber.find(&_info);
  if(itr == featureNumber.end()) {
    throw(BindingException("in BindingFeatureOntology::featureNumber(const std::type_info& _info): Not a feature, it seems: " + string(_info.name())));
  }
  return itr->second;
}

unsigned int 
BindingFeatureOntology::featureNumber(const std::string& _name) const 
{
  assert(ontologyState > FEATURE_REGISTRATION); 
  StringMap<unsigned int>::map::const_iterator itr = featureNameNumber.find(_name);
  if(itr == featureNameNumber.end()) {
    throw(BindingException("BindingFeatureOntology::featureNumber(const std::string& _name): Not a feature, it seems: " + _name));
  }
  return itr->second;
}

const AbstractFeatureHelper& 
BindingFeatureOntology::featureHelper(const std::type_info& _info) const
{
  assert(ontologyState > FEATURE_REGISTRATION); 
  return *featureHelpers[featureNumber(_info)];
  /*TypeMap<shared_ptr<const AbstractFeatureHelper> >::map::const_iterator itr = featureHelpers.find(&_info);
  if(itr == featureHelpers.end()) {
    throw(BindingException("A binding feature without a helper: " + string(_info.name())));
  }
  return *(itr->second);
*/
}

const AbstractInternalComparator* 
BindingFeatureOntology::internalComparator(const std::type_info& _proxyFeatureType, 
					   const std::type_info& _unionFeatureType) const 
{
  assert(ontologyState > COMPARATOR_REGISTRATION); 
  return internalComparators[featureNumber(_proxyFeatureType)][featureNumber(_unionFeatureType)].get();
}

  
void 
BindingFeatureOntology::registerInternalComparator(const std::type_info& _proxyFeatureType, 
						   const std::type_info& _unionFeatureType,
						   boost::shared_ptr<const AbstractInternalComparator>& _comparator) 
{
  std::cout << std::string("Feature comparator registered: ") 
	    << featureName(_proxyFeatureType) << " vs. \t" 
	    << featureName(_unionFeatureType)  << std::endl; 
  comparableFeatures.insert(this->featureName(_proxyFeatureType));
  comparableFeatures.insert(this->featureName(_unionFeatureType));
  assert(ontologyState == COMPARATOR_REGISTRATION); 
  internalComparators[featureNumber(_proxyFeatureType)][featureNumber(_unionFeatureType)] = _comparator;
  comparableInternally[featureNumber(_proxyFeatureType)].insert(featureName(_unionFeatureType));  
  
  proxy2unionComparable[featureName(_proxyFeatureType)].insert(featureName(_unionFeatureType));
  union2proxyComparable[featureName(_unionFeatureType)].insert(featureName(_proxyFeatureType));
}

void 
BindingFeatureOntology::openFeatureRegistration() { 
  assert(ontologyState == PRE_CONSTRUCT); 
  ontologyState = FEATURE_REGISTRATION; 
};
void 
BindingFeatureOntology::freezeFeatures() { 
  assert(ontologyState == FEATURE_REGISTRATION); 
  ontologyState = COMPARATOR_REGISTRATION; 
  //cout << "registered feature names:\n";
  for(TypeMap<std::string>::map::const_iterator itr = featureName.begin() ;
      itr != featureName.end() ;
      ++itr) {
    //cout << (itr->first)  << " " << itr->first->name() << " " << itr->second << endl;
    assert(featureName(*(itr->first)) == itr->second);
  }
};
void 
BindingFeatureOntology::freezeInternalComparators() { 
  assert(ontologyState == COMPARATOR_REGISTRATION); 
  ontologyState = REGISTRATION_FROZEN; 
};

void 
BindingFeatureOntology::constructorFinished() { 
  assert(ontologyState == REGISTRATION_FROZEN); 
  ontologyState = CONSTRUCTED; 
}

const std::set<std::string>& 
BindingFeatureOntology::comparableInternally(const std::type_info& _info) const
{
  //return featureHelper(_info).properties().comparableInternally;
  return comparableInternally[featureNumber(_info)];
}

const std::set<std::string>& 
BindingFeatureOntology::comparableExternally(const std::type_info& _info) const
{
  //return featureHelper(_info).properties().comparableExternally;
  return comparableExternally[featureNumber(_info)];
}
const std::set<std::string>& 
BindingFeatureOntology::comparableInternally(const std::string& _name) const
{
  //return featureHelper(_name).properties().comparableInternally;
  return comparableInternally[featureNumber(_name)];
}

const std::set<std::string>& 
BindingFeatureOntology::comparableExternally(const std::string& _name) const
{
  //return featureHelper(_name).properties().comparableExternally;
  return comparableExternally[featureNumber(_name)];
}


void
BindingFeatureOntology::assertHelperType(const AbstractFeatureHelper& _featureHelper, const type_info& _info) const
{
  assert(_featureHelper.typeInfo() == _info);
}

} // namespace Binding 
