
#include "binding/utils/BindingUtils.hpp"
#include "binding/utils/Predicates.hpp"
#include "binding/ontology/BindingFeatureOntology.hpp"
#include "ComparatorCache.hpp"
#include "BindingScorer.hpp"
#include "binding/utils/BindingScoreUtils.hpp"

#include <set>
#include <string>


namespace Binding {
using namespace std;
using namespace boost;
using namespace cast;
using namespace boost::logic;

boost::logic::tribool 
ComparatorCache::get(const AbstractFeature& _proxyFeaturePtr, 
		     const AbstractFeature& _unionFeaturePtr,
		     const string& _proxyID) {
  return _get(_proxyFeaturePtr, _unionFeaturePtr, _proxyID);
}


boost::logic::tribool 
ComparatorCache::_get(const AbstractFeature& _proxyF, 
		      const AbstractFeature& _unionF,
		      const string& _proxyID)
{
  
  // easiest case, a feature will not be compared with one from the same proxy itself
  if(_proxyF.immediateProxyID() == _unionF.immediateProxyID()) {
    return indeterminate;
  }
  
  /*  const std::set<std::string>& 
      externally_comparable_set(_proxyF.properties().m_comparableExternally);
      const std::set<std::string>& 
      internally_comparable_set(_proxyF.properties().m_comparableInternally);*/
  static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  
  const std::set<std::string>& 
    externally_comparable_set(ontology.comparableExternally(_proxyF.typeInfo()));
  const std::set<std::string>& 
    internally_comparable_set(ontology.comparableInternally(_proxyF.typeInfo()));
    
  bool externally_comparable(externally_comparable_set.find(_unionF.name()) != 
			     externally_comparable_set.end());
  bool internally_comparable(internally_comparable_set.find(_unionF.name()) != 
			     internally_comparable_set.end());
  const BindingData::ComparisonTrustSpecification* spec = NULL;
  if(externally_comparable) {
    BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
    const BindingFeatureOntology::TrustSpecMap& internalTrust(ontology.internalTrust());
    assert(internalTrust.find(_proxyF.name()) != internalTrust.end());
    assert(internalTrust.find(_proxyF.name())->second.find(_unionF.name()) != internalTrust.find(_proxyF.name())->second.end());
    spec = &(internalTrust.find(_proxyF.name())->second.find(_unionF.name())->second);
//    cout << "spec: " << *spec << endl; 
  }
  
  if(!externally_comparable && !internally_comparable)
    throw BindingException("ComparatorCache was requested to retrieve comparison for incomparable features. This shouldn't happen.");
  
  const string comparisonID(combinedID(_proxyF.featureID(),_unionF.featureID()));
  
  cast::StringMap<boost::logic::tribool>::map::const_iterator 
    pos(m_internalCache.find(comparisonID));
  
  // if the comparison is already internally represented, then just
  // return what is cached
  if(pos != m_internalCache.end()) {
    m_bindingScorer.log("used internalcache for " + comparisonID);
    return pos->second;
  }

  tribool equivalent = indeterminate;

  if(internally_comparable && 
     (!spec ||
      spec->m_trustInternalTrue != BindingData::DONT_USE && 
      spec->m_trustInternalIndeterminate != BindingData::DONT_USE && 
      spec->m_trustInternalFalse != BindingData::DONT_USE)) {    
    equivalent = _proxyF.compare(_unionF);
    m_internalCache[comparisonID] = equivalent;
  }
  
  if(externally_comparable) {
    // if it is externally comparable, but not on the internal cache,
    // the comparison must be stated as a task for whoever can do
    // it. The result will be written onto the WM which will change
    // the m_internalCache eventually by an event.
    
    assert(spec != 0);
    BindingData::ComparisonTrust trust;
    switch(equivalent.value) {
    case boost::logic::tribool::true_value:
      trust = spec->m_trustInternalTrue;
      break;
    case boost::logic::tribool::false_value:
      trust = spec->m_trustInternalFalse;;
      break;
    case  boost::logic::tribool::indeterminate_value:
      trust = spec->m_trustInternalIndeterminate;
      break;
    }
    if(trust == BindingData::TRUST_COMPLETELY) {
      // cool, we can use the internal comparison, no fuss
      return equivalent;
    }
    // 1st, make sure comparison is not already being processed but
    // has not yet been received.
    if(m_bindingScorer.existsOnWorkingMemory(comparisonID)) {
      //      m_bindingScorer.lockEntry(comparisonID,cast::cdl::LOCKED_ODR);
      throw(ExternalScoreNotReadyException(comparisonID));
      //return equivalent;
    }

    BindingData::FeatureComparison* comp = 
      new BindingData::FeatureComparison();
    comp->m_proxyFeature.m_address = CORBA::string_dup(_proxyF.featureID().c_str());
    comp->m_proxyFeature.m_type    = CORBA::string_dup(_proxyF.name().c_str());
    comp->m_proxyID                = CORBA::string_dup(_proxyID.c_str());
    comp->m_unionFeature.m_address = CORBA::string_dup(_unionF.featureID().c_str());
    comp->m_unionFeature.m_type    = CORBA::string_dup(_unionF.name().c_str());
    comp->m_featuresEquivalent = tribool_cast(equivalent);
    comp->m_bindingSubarchitectureID = CORBA::string_dup(m_bindingScorer.m_subarchitectureID.c_str());
    if(trust == BindingData::TRUST_BUT_VALIDATE) {
      comp->m_insistOnExternalComparison = false;
    } else {
      comp->m_insistOnExternalComparison = true;
    }
    m_bindingScorer.log("stored comparison (+task): " + _proxyF.featureID() +" vs. "+_unionF.featureID() + " : " + Binding::triboolToString(equivalent));
    // store the comparison itself
    m_bindingScorer.addToWorkingMemory(comparisonID,
				       //BindingLocalOntology::FEATURE_COMPARISON_TYPE, 
				       comp,
				       cdl::BLOCKING);
    m_bindingScorer.lockEntry(comparisonID,cast::cdl::LOCKED_ODR);

    // and store the task
    BindingData::FeatureComparisonTask* task = 
      new BindingData::FeatureComparisonTask();
    task->m_comparisonID = CORBA::string_dup(comparisonID.c_str());
    task->m_bindingSubarchitectureID = CORBA::string_dup(m_bindingScorer.m_subarchitectureID.c_str());
    m_bindingScorer.addToWorkingMemory(m_bindingScorer.newDataID(),
				       //BindingLocalOntology::FEATURE_COMPARISON_TASK_TYPE, 
				       task,
				       cdl::BLOCKING);
    
    if(trust == BindingData::TRUST_BUT_VALIDATE) {
      // cool, we can use the internal comparison, and then we'll see
      // if there is some external stuff coming in. If not, then we
      // stick to the internal one.
      return equivalent;
    }
    throw(ExternalScoreNotReadyException(comparisonID));
  }
  
  return equivalent;

}


void 
ComparatorCache::set(const string& _comparisonID)
{
  shared_ptr<const BindingData::FeatureComparison> comp;
  try{
    comp = m_bindingScorer.loadBindingDataFromWM<BindingData::FeatureComparison>(_comparisonID);
  } catch(const DoesNotExistOnWMException& _e) {
    m_bindingScorer.log("Caught this in ComparatorCache (FeatureComparison not existing in ComparatorCache::set(...), which is odd but may be normal, aborting now to see it in tests): " + string(_e.what()));
    abort();
  }

  m_internalCache[_comparisonID] = tribool_cast(comp->m_featuresEquivalent);

  BindingScorer& s(m_bindingScorer); // too lazy to type...
  // 1st, check if the returned answer is part of a basic query
  if(!s.m_openBasicQueryFeatureComparisonIDs.empty()) {
    map<string,string>::iterator itr = 
      s.m_openBasicQueryFeatureComparisonIDs.find(_comparisonID);
    if(itr != s.m_openBasicQueryFeatureComparisonIDs.end()) {
      m_bindingScorer.log("results for a basic query received from an external comparator");
      string queryID(itr->second);
      shared_ptr<const BindingQueries::BasicQuery> 
	query(s.loadBindingDataFromWM<BindingQueries::BasicQuery>(queryID));
      s.m_openBasicQueryFeatureComparisonIDs.erase(itr);
      s._answerBasicQuery(*query,queryID);
    }
  }
  // 2nd, check if the returned answer is part of an advanced query
  if(!s.m_openAdvancedQueryFeatureComparisonIDs.empty()) {
    map<string,string>::iterator itr = s.m_openAdvancedQueryFeatureComparisonIDs.find(_comparisonID);
    if(itr != s.m_openAdvancedQueryFeatureComparisonIDs.end()) {
      m_bindingScorer.log("results for an advanced query received from an external comparator");
      string queryID(itr->second);
      shared_ptr<const BindingQueries::AdvancedQuery> 
	query(s.loadBindingDataFromWM<BindingQueries::AdvancedQuery>(queryID));
      s.m_openAdvancedQueryFeatureComparisonIDs.erase(itr);
      s._answerAdvancedQuery(*query,queryID);
    }
  }
#warning rescoring blindly triggered, and this may be very inefficient
  BindingData::BindTheseProxies* bindThese = new BindingData::BindTheseProxies();
  bindThese->m_proxyIDs.length(1);
  bindThese->m_proxyIDs[0] = CORBA::string_dup(comp->m_proxyID);
  m_bindingScorer.addToWorkingMemory(m_bindingScorer.newDataID(), 
				     m_bindingScorer.m_subarchitectureID, 
				     //BindingLocalOntology::BIND_THESE_PROXIES_TYPE, 
				     bindThese, 
				     cdl::BLOCKING);
  
  
  m_bindingScorer.log(_comparisonID + " results received: " + string(comp->m_proxyFeature.m_address) + " vs " + string(comp->m_unionFeature.m_address) + " for proxy " + string(comp->m_proxyID));

}

// implemented here due to linking issues
bool
ComparatorCacheCheck::test(const ProxyPtr& _ptr) const
{
  const FeatureSetWithRepetitions* fset_ptr = NULL;
  if(m_queryParameters.m_boundProxyInclusion == BindingQueries::EXCLUDE_BOUND) {
    fset_ptr = &(_ptr->comparableFeatureSetWithRepetitions());
  } else {
    fset_ptr = &(_ptr->bindingUnion().comparableFeatureSetWithRepetitions());
  }
  const FeatureSetWithRepetitions& fset(*fset_ptr);
  FeatureSetWithRepetitions::const_iterator itr = fset.find(m_feature.name());
  if(itr == fset.end()) { // no such feature, result is indeterminate, return true if that is what we're checking for
    m_answer = indeterminate;
    return m_desiredTribool.value == tribool::indeterminate_value;
  } else {
    //tribool result = indeterminate;
    m_answer = indeterminate;
    bool at_least_one_match = false; // assume no match as long as all results are indeterminate
    foreach(const OneTypeOfFeaturesWithRepetitions::value_type& feature, itr->second) { // feature is of type shared_ptr<AbstractFeature&>
      try {
	m_answer = m_comparator.get(*feature,m_feature,string(feature->immediateProxyID()));
	if(m_answer.value == tribool::false_value) { // no need to continue at a mismatch
	  return m_desiredTribool.value == tribool::false_value;
	} else 	if(m_answer.value == tribool::true_value) { 
	  at_least_one_match = true;
	}
      } catch(const ExternalScoreNotReadyException& _e) {
	m_featureComparisonIDs.insert(_e.m_featureComparisonID);
      }
      if(at_least_one_match)
	m_answer = true;
    }
    return m_answer.value == m_desiredTribool.value;
  }
}

} // namespace Binding
