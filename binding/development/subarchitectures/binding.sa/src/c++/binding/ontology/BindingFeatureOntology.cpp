
#include "BindingFeatureOntology.hpp"
#include "binding/feature-specialization/FeatureRegistrant.hpp"
#include "binding/utils/BindingUtils.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"
#include "binding/idl/BindingData.hh"
#include "binding/feature-utils/FeatureHelper.hpp"

#include <boost/assign.hpp>
#include <set>
#include <map>

namespace Binding {

using namespace boost::assign;
using namespace std;
using namespace boost;
using namespace cast;


/*set<string> BindingFeatureOntology::m_features = set<string>();
set<string> BindingFeatureOntology::m_comparableFeatures = set<string>();
map<string,set<string> > BindingFeatureOntology::m_proxy2unionComparable = map<string,set<string> >();
map<string,set<string> > BindingFeatureOntology::m_union2proxyComparable = map<string,set<string> >();
map<string,Binding::FeatureProperties> BindingFeatureOntology::m_featurePropertyMap = map<string,Binding::FeatureProperties>();
BindingFeatureOntology::TrustSpecMap BindingFeatureOntology::m_internalTrust = TrustSpecMap();
StringMap<BindingFeatureOntology::FeatureStats>::map BindingFeatureOntology::m_featureStats = StringMap<BindingFeatureOntology::FeatureStats>::map();
*/

ostream& operator<<(ostream& out, const set<string>& strings) {
  for(set<string>::const_iterator i = strings.begin() ; i != strings.end() ; i++)
    out << *i << " ";
  return out;
}

BindingFeatureOntology::BindingFeatureOntology() 
  :
    m_ontologyState(PRE_CONSTRUCT)
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
  m_featurePropertyMap[_featureType] = _properties;
  m_features.insert(_featureType);
  
  const set<string>& comparableInternally(_properties.m_comparableInternally);
  const set<string>& comparableExternally(_properties.m_comparableExternally);
  m_proxy2unionComparable[_featureType].insert(comparableInternally.begin(),comparableInternally.end());
  m_proxy2unionComparable[_featureType].insert(comparableExternally.begin(),comparableExternally.end());
  if(!comparableInternally.empty() ||
     !comparableExternally.empty()) {
    m_comparableFeatures.insert(_featureType);
    m_comparableFeatures.insert(comparableInternally.begin(),comparableInternally.end());
    m_comparableFeatures.insert(comparableExternally.begin(),comparableExternally.end());
    for(set<string>::const_iterator itr = comparableInternally.begin() ; 
	itr != comparableInternally.end() ; 
	  ++itr) {
      m_union2proxyComparable[*itr].insert(_featureType);
    }
    for(set<string>::const_iterator itr = comparableExternally.begin() ; 
	itr != comparableExternally.end() ; 
	  ++itr) {
      m_union2proxyComparable[*itr].insert(_featureType);
    }
  } 
}


void 
BindingFeatureOntology::registerComparatorCompetence(const BindingData::FeatureComparisonCompetence& _competence) {
  assert(m_ontologyState == CONSTRUCTED);
      
  stringstream str;
  Binding::operator<<(str,_competence);
  cout << "\ncomparator competence registered:\n" << str.str() << endl;
//  const map<string,Binding::FeatureProperties>& properties(BindingFeatureOntology::featurePropertyMap());
  const string proxyFeatureType(_competence.m_proxyFeatureType);
  const string unionFeatureType(_competence.m_unionFeatureType);
  
  m_comparableExternally[featureNumber(proxyFeatureType)].insert(unionFeatureType);  
  Binding::FeatureProperties new_prop = featureHelper(proxyFeatureType).properties();//properties.find(proxyFeatureType)->second;
  new_prop.m_comparableExternally.insert(unionFeatureType);
  BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  ontology.updateFeatureProperties(proxyFeatureType, new_prop);    
  if(m_internalTrust.find(proxyFeatureType) != m_internalTrust.end() &&
     m_internalTrust.find(proxyFeatureType)->second.find(unionFeatureType) != 
     m_internalTrust.find(proxyFeatureType)->second.end()) {
    stringstream str2;
    Binding::operator<<(str2,m_internalTrust[proxyFeatureType][unionFeatureType]);
    cerr << "WARNING! Same comparator competence registered twice:\n" <<  str.str() << " and previously with " << str2.str() << endl;
  }
  m_internalTrust[proxyFeatureType][unionFeatureType]= _competence.m_comparisonTrustSpecification;
}

  
const std::string& 
BindingFeatureOntology::featureName(const std::type_info& _info) const 
{
  assert(m_ontologyState > FEATURE_REGISTRATION); 
  TypeMap<std::string>::map::const_iterator itr = m_featureName.find(&_info);
  if(itr == m_featureName.end()) {
    throw(BindingException("in BindingFeatureOntology::featureName(const std::type_info& _info): Not a feature, it seems: " + string(_info.name())));
  }
  return itr->second;
}

unsigned int 
BindingFeatureOntology::featureNumber(const std::type_info& _info) const 
{
  assert(m_ontologyState > FEATURE_REGISTRATION); 
  TypeMap<unsigned int>::map::const_iterator itr = m_featureNumber.find(&_info);
  if(itr == m_featureNumber.end()) {
    throw(BindingException("in BindingFeatureOntology::featureNumber(const std::type_info& _info): Not a feature, it seems: " + string(_info.name())));
  }
  return itr->second;
}

unsigned int 
BindingFeatureOntology::featureNumber(const std::string& _name) const 
{
  assert(m_ontologyState > FEATURE_REGISTRATION); 
  StringMap<unsigned int>::map::const_iterator itr = m_featureNameNumber.find(_name);
  if(itr == m_featureNameNumber.end()) {
    throw(BindingException("BindingFeatureOntology::featureNumber(const std::string& _name): Not a feature, it seems: " + _name));
  }
  return itr->second;
}

const AbstractFeatureHelper& 
BindingFeatureOntology::featureHelper(const std::type_info& _info) const
{
  assert(m_ontologyState > FEATURE_REGISTRATION); 
  return *m_featureHelpers[featureNumber(_info)];
  /*TypeMap<shared_ptr<const AbstractFeatureHelper> >::map::const_iterator itr = m_featureHelpers.find(&_info);
  if(itr == m_featureHelpers.end()) {
    throw(BindingException("A binding feature without a helper: " + string(_info.name())));
  }
  return *(itr->second);
*/
}

const AbstractInternalComparator* 
BindingFeatureOntology::internalComparator(const std::type_info& _proxyFeatureType, 
					   const std::type_info& _unionFeatureType) const 
{
  assert(m_ontologyState > COMPARATOR_REGISTRATION); 
  return m_internalComparators[featureNumber(_proxyFeatureType)][featureNumber(_unionFeatureType)].get();
}

  
void 
BindingFeatureOntology::registerInternalComparator(const std::type_info& _proxyFeatureType, 
						   const std::type_info& _unionFeatureType,
						   boost::shared_ptr<const AbstractInternalComparator>& _comparator) 
{
  std::cout << std::string("Feature comparator registered: ") 
	    << featureName(_proxyFeatureType) << " vs. \t" 
	    << featureName(_unionFeatureType)  << std::endl; 
  m_comparableFeatures.insert(this->featureName(_proxyFeatureType));
  m_comparableFeatures.insert(this->featureName(_unionFeatureType));
  assert(m_ontologyState == COMPARATOR_REGISTRATION); 
  m_internalComparators[featureNumber(_proxyFeatureType)][featureNumber(_unionFeatureType)] = _comparator;
  m_comparableInternally[featureNumber(_proxyFeatureType)].insert(featureName(_unionFeatureType));  
  
  m_proxy2unionComparable[featureName(_proxyFeatureType)].insert(featureName(_unionFeatureType));
  m_union2proxyComparable[featureName(_unionFeatureType)].insert(featureName(_proxyFeatureType));
}

void 
BindingFeatureOntology::openFeatureRegistration() { 
  assert(m_ontologyState == PRE_CONSTRUCT); 
  m_ontologyState = FEATURE_REGISTRATION; 
};
void 
BindingFeatureOntology::freezeFeatures() { 
  assert(m_ontologyState == FEATURE_REGISTRATION); 
  m_ontologyState = COMPARATOR_REGISTRATION; 
  //cout << "registered feature names:\n";
  for(TypeMap<std::string>::map::const_iterator itr = m_featureName.begin() ;
      itr != m_featureName.end() ;
      ++itr) {
    //cout << (itr->first)  << " " << itr->first->name() << " " << itr->second << endl;
    assert(featureName(*(itr->first)) == itr->second);
  }
};
void 
BindingFeatureOntology::freezeInternalComparators() { 
  assert(m_ontologyState == COMPARATOR_REGISTRATION); 
  m_ontologyState = REGISTRATION_FROZEN; 
};

void 
BindingFeatureOntology::constructorFinished() { 
  assert(m_ontologyState == REGISTRATION_FROZEN); 
  m_ontologyState = CONSTRUCTED; 
}

const std::set<std::string>& 
BindingFeatureOntology::comparableInternally(const std::type_info& _info) const
{
  //return featureHelper(_info).properties().m_comparableInternally;
  return m_comparableInternally[featureNumber(_info)];
}

const std::set<std::string>& 
BindingFeatureOntology::comparableExternally(const std::type_info& _info) const
{
  //return featureHelper(_info).properties().m_comparableExternally;
  return m_comparableExternally[featureNumber(_info)];
}
const std::set<std::string>& 
BindingFeatureOntology::comparableInternally(const std::string& _name) const
{
  //return featureHelper(_name).properties().m_comparableInternally;
  return m_comparableInternally[featureNumber(_name)];
}

const std::set<std::string>& 
BindingFeatureOntology::comparableExternally(const std::string& _name) const
{
  //return featureHelper(_name).properties().m_comparableExternally;
  return m_comparableExternally[featureNumber(_name)];
}


void
BindingFeatureOntology::assertHelperType(const AbstractFeatureHelper& _featureHelper, const type_info& _info) const
{
  assert(_featureHelper.typeInfo() == _info);
}

} // namespace Binding 
