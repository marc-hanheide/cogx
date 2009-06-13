#include "binding/ontology/BindingFeatureOntology.hpp"
#include "binding/abstr/AbstractMonitor.hpp"
#include "BindingScorer.hpp"
#include "binding/feature-utils/Features.hpp"
#include "binding/feature-utils/FeatureExtractor.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-specialization/helpers/SalienceHelper.hpp"
#include "binding/utils/BindingScoreUtils.hpp"
#include "binding/utils/BindingUtils.hpp"
#include "binding/utils/Predicates.hpp"
#include "binding/utils/GraphLoader.hpp"
#include "binding/utils/QueryUtils.hpp"
#include "binding/idl/BindingQueries.hh"

#include <cstdlib>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/assign.hpp>
#include <ext/hash_set>


using namespace std;
using namespace cast;
using namespace boost;
using namespace boost::assign;
//typedef std::set<std::string> StringSet;
typedef __gnu_cxx::hash_set<std::string> StringSet;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Binding::BindingScorer(_id);
  }
}

namespace Binding {

BindingScorer::BindingScorer(const string &_id) : 
  
  WorkingMemoryAttachedComponent(_id),
  AbstractBinder(_id),
  comparatorCache(*this)//  featureComparisonCache(this)
{ 
}

void
BindingScorer::start() {
  
  AbstractBinder::start();
  
  addChangeFilter(createLocalTypeFilter<BindingData::ScoringTask>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingScorer>(this,
								  &BindingScorer::scoringTaskAdded));

  addChangeFilter(createLocalTypeFilter<BindingData::BindingUnion>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingScorer>(this,
								  &BindingScorer::scoreThisUnion));
  
  addChangeFilter(createLocalTypeFilter<BindingData::BindingUnion>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<BindingScorer>(this,
								  &BindingScorer::scoreThisUnion));  

  addChangeFilter(createLocalTypeFilter<BindingData::BindTheseProxies>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingScorer>(this,
								  &BindingScorer::bindTheseProxiesAdded));
  
  addChangeFilter(createLocalTypeFilter<BindingData::FeatureComparison>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<BindingScorer>(this,
								  &BindingScorer::updateComparison));
  
  addChangeFilter(createLocalTypeFilter<BindingData::FeatureComparisonCompetence>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingScorer>(this,
								  &BindingScorer::registerComparatorCompetence));
  
  addChangeFilter(createLocalTypeFilter<BindingQueries::BasicQuery>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingScorer>(this,
								  &BindingScorer::answerBasicQuery));
  addChangeFilter(createLocalTypeFilter<BindingQueries::AdvancedQuery>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingScorer>(this,
								  &BindingScorer::answerAdvancedQuery));

  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<BindingScorer>(this,
								  &BindingScorer::proxyUpdateReceived));

  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<BindingScorer>(this,
								  &BindingScorer::proxyUpdateReceived));

  addChangeFilter(createLocalTypeFilter<BindingData::BinderStatus>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<BindingScorer>(this,
								  &BindingScorer::binderStatusReceived));

  addChangeFilter(createLocalTypeFilter<BindingData::ProxyProcessingFinished>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingScorer>(this,
								  &BindingScorer::proxyProcessingFinished));

}

void 
BindingScorer::configure(map<string, string>& _config)
{
  AbstractBinder::configure(_config);
  nth = 0;
  maxN = 1;
  if(_config["-nth"] != "") {
    nth = lexical_cast<unsigned int>(_config["-nth"]);
    log("nth: " + nth);
    if(_config["-maxN"] != "") {
      maxN = lexical_cast<unsigned int>(_config["-maxN"]);
      if(maxN <= nth)
	throw(BindingException("-maxN <= nth!\n"));
    } else {
      throw(BindingException("-maxN must be specified if -nth is!\n"));
    }
  }
  dummy = false;
  if(_config["--dummy"] != "") {
    dummy = true;
  }
  
  log("maxN = " + lexical_cast<string>(maxN) + " nth = " + lexical_cast<string>(nth));
  setBindingSubarchID(subarchitectureID());
} 


BindingScorer::~BindingScorer() {}

void 
BindingScorer::scoreThisUnion(const cdl::WorkingMemoryChange & _wmc) {
  const string unionID(_wmc.address.id);
  log("scoreThisUnion: " + unionID);
  if(!existsOnWorkingMemory(unionID)) {
    log("union not existing in scoreThisUnion(...)): ");
    return;
  }
  const LBindingUnion* uptr = NULL;
  try {
    uptr = &(unionLocalCache[unionID]);
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in BindingScorer (union not existing in scoreThisUnion(...)): " + string(_e.what()));
  }
  const LBindingUnion& binding_union(*uptr);
  
  vector<shared_ptr<const CASTData<BindingData::BindingProxy> > > proxies;
  
  getWorkingMemoryEntries<BindingData::BindingProxy>(//BindingLocalOntology::BINDING_PROXY_TYPE,
			  0,
			  proxies);
  
  StringSet proxyIDs;
  for(vector<shared_ptr<const CASTData<BindingData::BindingProxy> > >::const_iterator i = proxies.begin();
      i != proxies.end();
      ++i) {
    proxyIDs.insert((*i)->getID());
    
    const LBindingProxy* proxy_ptr(maybeLoadProxy(string((*i)->getID())));
    if(proxy_ptr){
      const LBindingProxy& proxy(*proxy_ptr);
      log(string("BASED ON UNION UPDATE: Going to score proxy ") + (*i)->getID() + " vs. binding " + unionID);
      try{
	scoreAndStore(proxy, binding_union);
      } catch(const DoesNotExistOnWMException& _e) {
	cerr << "thrown from BindingScorer::scoreAndStore: " << _e.what() 
	     << "should be caugth within that function. Aborting." << endl;
	abort();
      }
    } else {
      log("Proxy does not exist in scoreThisUnion: " + string((*i)->getID()));
    }
  }
  
  /* This is to be used if... well.. unbounded proxies remain a problem
    vector<shared_ptr<const CASTData<BindingData::BindingProxy> > > updated_proxies;
  getWorkingMemoryEntries(BindingLocalOntology::BINDING_PROXY_TYPE,
			  0,
			  updated_proxies);
  StringSet proxyIDsNotScored;
  for(vector<shared_ptr<const CASTData<BindingData::BindingProxy> > >::const_iterator i = updated_proxies.begin();
      i != updated_proxies.end();
      ++i) {
    StringSet::iterator itr = proxyIDs.find((*i)->getID());
    if(itr == proxyIDs.end()) {
      proxyIDsNotScored.insert((*i)->getID());
    }
  }
  //assert(proxyIDsNotScored.empty());
*/
}

void
BindingScorer::bindTheseProxiesAdded(const cdl::WorkingMemoryChange & _wmc) {
  //  cout << "BindingScorer::bindTheseProxiesAdded" << endl;
  log("bindTheseProxiesAdded: " + string(_wmc.address.subarchitecture) + " id: " + string(_wmc.address.id));
  if(string(_wmc.address.subarchitecture) != subarchitectureID)
    throw BindingException("ERROR: bindTheseProxies written to wrong subarchitecture WM\nMake sure to set the -bsa flag to the binding subarchitecture ID");
  // get the data from working memory
  shared_ptr<const BindingData::BindTheseProxies> 
    bindTheseProxies(loadBindingDataFromWM<BindingData::BindTheseProxies>(_wmc));
  
  log("bindThese.proxyIDs.length(): " + lexical_cast<string>(bindTheseProxies->proxyIDs.length()));
    
  if(!hasBinderToken())
    acquireBinderToken(); // will be released again, when all seems bound
  //  try {
  
  for(unsigned int i = 0; i < bindTheseProxies->proxyIDs.length(); ++i) {
    string proxyID(bindTheseProxies->proxyIDs[i]);
   
    log("bindTheseProxies->proxyIDs[" +lexical_cast<string>(i)+ "]: " 
	+ string(bindTheseProxies->proxyIDs[i]));
    log("gonna score " + proxyID);
    
    const LBindingProxy* proxy_ptr = maybeLoadProxy(proxyID);
    if(proxy_ptr) {
      processedProxies.insert(proxyID);
      const LBindingProxy& proxy(*proxy_ptr);
      
      //      assert(proxy->proxyState != BindingData::BOUND);
      
      scoreAndStoreVsNIL(proxy);
      vector<shared_ptr<const CASTData<BindingData::BindingUnion> > > unions;
      getWorkingMemoryEntries<BindingData::BindingUnion>(0, unions);
      // used to avoid accessing the cache too much since we can assume the unions are constant during scoring (generally)
      vector<const LBindingUnion*> lunions(unions.size(),NULL); 
      unsigned int j = 0;
      for(vector<shared_ptr<const CASTData<BindingData::BindingUnion> > >::const_iterator 
	    i = unions.begin();
	  i != unions.end();
	  ++i,++j) {
	string unionID((*i)->getID());
	try {
	  if(lunions[j] == NULL) {
	    lunions[j] = &(unionLocalCache[unionID]);
	  }
	} catch(const DoesNotExistOnWMException& _e) {
	  log("Caught this in BindingScorer (union not existing in bindTheseProxiesAdded(...)): " + string(_e.what()));
	}
	// if the exception did not prevent setting the pointer, then we can safely continue...
	if(lunions[j] != NULL) {
	  log(string("BASED ON BIND_THESE_PROXIES ADD: Going to score proxy ") + proxyID + " vs. union " + unionID);
	  scoreAndStore(proxy,*lunions[j]);
	}
      }
    }
  }
  //} catch(...) {
  //  releaseBinderToken();
  //  throw;
 // }
 // releaseBinderToken();
  // now delete the BindTheseProxies list
#warning BindTheseProxies IS now deleted, beware!
  deleteFromWorkingMemory(string(_wmc.address.id));
}

void 
BindingScorer::scoringTaskAdded(const cdl::WorkingMemoryChange & _wmc) {
  log("scoringTaskAdded : " + string(_wmc.address.id));
  shared_ptr<const BindingData::ScoringTask>
    ctask(loadBindingDataFromWM<BindingData::ScoringTask>(_wmc.address.id));

  const string proxyUnionScoreID(ctask->proxyUnionScoreID);

  // We have what we need, delete the task
  deleteFromWorkingMemory(string(_wmc.address.id));  
  shared_ptr<const BindingData::ProxyUnionScore> comparison;
  try {
    comparison = loadBindingDataFromWM<BindingData::ProxyUnionScore>(proxyUnionScoreID);
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in BindingScorer (the score didn't exist in scoringTaskAdded(...)): " + string(_e.what()));
    abort();
  }
    
  const string proxyID(comparison->proxyID);
  const string unionID(comparison->unionID);
  
  log(string("BASED ON TASK: Going to score proxy ") + proxyID + " vs. binding " + unionID);

  try {
    const LBindingProxy& proxy(proxyLocalCache[proxyID]);
    if(unionID != string(BindingData::NO_UNION)) {
      const LBindingUnion& binding_union(unionLocalCache[unionID]);
      scoreAndStore(proxy,binding_union);
    } else {
      scoreAndStoreVsNIL(proxy);
    }
  } catch(const DoesNotExistOnWMException& _e) {
    cerr << "Caught this in BindingScorer (proxy or union not existing in scoringTaskAdded(...)):\n" 
	 << "should be caugth earlier:\n" << string(_e.what()) << endl;
    abort();
  }
}
  
void 
BindingScorer::scoreAndStoreVsNIL(const LBindingProxy& _proxy)
{
  // default score with no matches or mismatches
  BindingData::BindingScore score = defaultBindingScore();
  score.comparable = true;
  score.sticky = 1;
  score.proxyUpdatesWhenThisComputed = _proxy->updates;
  score.proxyID = CORBA::string_dup(_proxy.id().c_str());
  score.proxyFeatureSignature = CORBA::string_dup(_proxy->featureSignature);
  score.unionID = CORBA::string_dup(BindingData::NO_UNION);
  _storeScore(_proxy.id(),string(BindingData::NO_UNION),score);
}
/// used to throw
struct BestListNotUpToDate{};

void 
BindingScorer::scoreAndStore(const LBindingProxy& _proxy,
			     const LBindingUnion& _union)
{  

  const string& proxyID(_proxy.id());
  const string& unionID(_union.id());
  log("gonna to score proxy " + proxyID + " vs. binding " + unionID + "!");
  
  if(proxyID[0] % maxN != nth) {
    log("NOT my proxy: " + proxyID);
    abort();
  }
    
  BindingData::BindingScore score = defaultBindingScore();
  score.proxyID = CORBA::string_dup(_proxy.id().c_str());
  score.unionID = CORBA::string_dup(_union.id().c_str());
  score.proxyUpdatesWhenThisComputed = _proxy->updates;
      
  try {
    
    if(!existsOnWorkingMemory(proxyID)) {
      log("----- Attempting to score proxy " + proxyID + " vs. binding " + unionID + " but the former does not exist. Deleted? Doing nothing!");
#warning SILLY TOKEN HACK, should not be necessary
      BindingData::ProxyProcessingFinished* p = new BindingData::ProxyProcessingFinished;
      p->proxyID = CORBA::string_dup(proxyID.c_str());
      addToWorkingMemory(newDataID(),p, cast::cdl::BLOCKING);
      return;
    }
    //  shared_ptr<const BindingData::BindingProxy>
    //    proxy(proxyCache.getPtr(proxyID));
    
    
    //shared_ptr<const BindingData::BindingProxy>
    //  proxy(proxyCache.getPtr(proxyID));
    
    //log(Binding::toString(*proxy));
    
    assert(string(unionID) != string(BindingData::NO_UNION));
    
    if(!existsOnWorkingMemory(unionID)) {
      log("----- Attempting to score proxy " + proxyID + " vs. binding " + unionID + " but the latter does not exist. Deleted? Doing nothing!");
      return;
    }
    if(_union->type == _proxy->type) {
      
      score = scoreProxyVsUnion(_proxy, _union);
      unsigned int relationMatches;
      bool relationMismatch;
      try{
	_relational_score(_proxy, 
			  _union,
			  relationMatches,
			  relationMismatch);
      } catch (const DoesNotExistOnWMException& _e) {
	cerr << "thrown from BindingScorer::_relational_score:\n"
	     << _e.what() << "\n should be caught earlier. Aborting." << endl; 
	abort();
      }
      score.relationMatches = relationMatches;
      score.relationMismatch = relationMismatch;
      log("After relational_score: score between Proxy " + _proxy.id() + " and Union " + _union.id() + " : "+ Binding::scoreToString(score));
      if(relationMatches > 0 || relationMismatch)
	score.comparable = true;
      if(score.matches == 0 &&
	 score.relationMatches == 0 &&
	 score.mismatch == false &&
	 score.relationMismatch == false &&
	 string(_proxy->unionID) == _union.id()) {
	log(_proxy.id() +  " is already bound to "+ _union.id() +", sticky score is increased");
	score = defaultBindingScore();
	score.comparable = true;
	score.sticky = 2;
	score.proxyUpdatesWhenThisComputed = _proxy->updates;
	score.proxyID = CORBA::string_dup(_proxy.id().c_str());
	score.proxyFeatureSignature = CORBA::string_dup(_proxy->featureSignature);
	score.unionID = CORBA::string_dup(_union.id().c_str());
      }
    } else { // not the same type of proxies/bindings
      score = defaultBindingScore();
      score.comparable = false;
      score.proxyUpdatesWhenThisComputed = _proxy->updates;
      score.proxyID = CORBA::string_dup(_proxy.id().c_str());
      score.proxyFeatureSignature = CORBA::string_dup(_proxy->featureSignature);
      score.unionID = CORBA::string_dup(_union.id().c_str());
    }
    
//    if(!unionFeaturesUpToDate(_union,_proxy)) {
//      score.mismatch = true;
//    }
    
    //log("going to update " + proxyUnionScoreID + " a (" + BindingLocalOntology::FEATURE_SET_COMPARISON_TYPE + ")");
    _storeScore(proxyID, unionID, score);
    //log("updated " + proxyUnionScoreID + " a (" + BindingLocalOntology::FEATURE_SET_COMPARISON_TYPE + ")");
    
    log(string("Scored proxy ") + proxyID + " vs. union " + unionID);
    log(string("Score: ") + Binding::scoreToString(score));
  }
  catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in BindingScorer::scoreAndStore " + string(_e.what()));
    cerr << "should have been caught earlier: " << _e.what() << endl;
    abort();
  }
  catch(const ExternalScoreNotReadyException& _e) {
    log("ExternalScoreNotReadyException: " + _proxy.id() + " vs. " + _union.id());
    unlockEntry(_e.featureComparisonID);
  }
  catch(const BestListNotUpToDate&) {
    log("BestListNotUpToDate "  + _proxy.id() + " vs. " + _union.id()  + " (aborting now to catch this in testing, but this should be changed later)");
    abort();
  }
}
  
BindingData::BindingScore 
BindingScorer::scoreProxyVsUnion(const LBindingProxy& _proxy,
				 const LBindingUnion& _union) {
  log(string("scoreProxyVsUnion ") + _proxy.id() + " vs " + _union.id());
  const BindingFeatureOntology& ont(BindingFeatureOntology::construct());
  
  BindingData::BindingScore ret = defaultBindingScore();
  ret.comparable = false;
  ret.proxyUpdatesWhenThisComputed = _proxy->updates;
  ret.proxyID = CORBA::string_dup(_proxy.id().c_str());
  ret.unionID = CORBA::string_dup(_union.id().c_str());
  
  const string& proxyID(_proxy.id());
  //const string& unionID(_union.id());
  
//#warning test
  const FeatureSet& proxyset(_proxy.comparableFeatureSet());
  //const FeatureSet& proxyset(_proxy.featureSet());
  const FeatureSetWithRepetitions& unionset(_union.comparableFeatureSetWithRepetitions());
  //const FeatureSetWithRepetitions& unionset(_union.featureSetWithRepetitions());
  typedef OneTypeOfFeatures ProxyFeatures;
  typedef OneTypeOfFeaturesWithRepetitions UnionFeatures;

  log("proxyset.size() : " + lexical_cast<string>(proxyset.size()));
  log("unionset.size() : " + lexical_cast<string>(unionset.size()));
  
  FeatureSet::const_iterator p_i(proxyset.begin());
  FeatureSetWithRepetitions::const_iterator u_i(unionset.begin());
  
  p_i = proxyset.begin();
  u_i = unionset.begin();

  bool finished = false;

  vector<BindingFeatures::Salience> proxy_saliences;
  vector<BindingFeatures::Salience> union_saliences;
  vector<BindingFeatures::CreationTime> proxy_creation_times;
  vector<BindingFeatures::CreationTime> union_creation_times;

  bool union_is_singular = false;
  unsigned int union_element_nr = 0;
  static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  while(p_i != proxyset.end() && !finished) {
    //const set<string>& externally_comparable_set((*(p_i->second.begin()))->properties().comparableExternally);
    //const set<string>& internally_comparable_set((*(p_i->second.begin()))->properties().comparableInternally);
    const set<string>& externally_comparable_set(ontology.comparableExternally((*p_i->second.begin())->typeInfo()));
    const set<string>& internally_comparable_set(ontology.comparableInternally((*p_i->second.begin())->typeInfo()));
//    log("*** PROXY FEATURE:" + (*(p_i->second.begin()))->name());
    u_i = unionset.begin();
    bool one_match_for_union_feature(false); // only allow one match per union feature
    while(u_i != unionset.end() && !finished) {
      //log("PROXY FEATURE:" + (*(p_i->second.begin()))->name());
      //log("UNION FEATURE:" + u_i->first);
      string proxyFeatureType((*(p_i->second.begin()))->name());
      string unionFeatureType(u_i->first);
      if(unionFeatureType == ont.featureName(typeid(BindingFeatures::Singular))) {//BindingFeatureOntology::SINGULAR_TYPE) {
	union_is_singular = true;
	//union_element_nr = dynamic_cast<const Feature<BindingFeatures::Singular>* >((*(u_i->second.begin())).get())->idlFeature().elementNumber;
	union_element_nr = extractIDLFeature<BindingFeatures::Singular>(*(u_i->second.begin())).elementNumber;
      }
      if(externally_comparable_set.find(u_i->first) != externally_comparable_set.end() ||
	 internally_comparable_set.find(u_i->first) != internally_comparable_set.end() ) {
	// whatever the results, the features are comparable
	ret.comparable = true;
	typedef OneTypeOfFeatures ProxyFeatures;
	typedef OneTypeOfFeaturesWithRepetitions UnionFeatures;
	const ProxyFeatures& proxy_features(p_i->second);
	const UnionFeatures& union_features(u_i->second);
	tribool equivalent(indeterminate);
	for(ProxyFeatures::const_iterator pf_i = proxy_features.begin(); 
	    pf_i != proxy_features.end() && !finished; 
	    ++pf_i) {
	  //bool one_noninequivalent_found = false;
	  for(UnionFeatures::const_iterator uf_i = union_features.begin(); 
	      uf_i != union_features.end() && !finished; 
	      ++uf_i) {
	    if(proxyFeatureType == ont.featureName(typeid(BindingFeatures::Salience))) { //BindingFeatureOntology::SALIENCE_TYPE) {
	      proxy_saliences.push_back(dynamic_cast<const Feature<BindingFeatures::Salience>* >((*pf_i).get())->idlFeature());
	    } else if(proxyFeatureType == ont.featureName(typeid(BindingFeatures::CreationTime))) { //BindingFeatureOntology::CREATION_TIME_TYPE) {
	      proxy_creation_times.push_back(dynamic_cast<const Feature<BindingFeatures::CreationTime>* >((*pf_i).get())->idlFeature());
	    }
	    if(unionFeatureType == ont.featureName(typeid(BindingFeatures::Salience))) {//BindingFeatureOntology::SALIENCE_TYPE) {
	      union_saliences.push_back(dynamic_cast<const Feature<BindingFeatures::Salience>* >((*uf_i).get())->idlFeature());
	    } else if(unionFeatureType == ont.featureName(typeid(BindingFeatures::CreationTime))) {//BindingFeatureOntology::CREATION_TIME_TYPE) {
	      union_creation_times.push_back(dynamic_cast<const Feature<BindingFeatures::CreationTime>* >((*uf_i).get())->idlFeature());
	    }
	    //cout << "calling comparator cache's get function" << endl;
	    equivalent = comparatorCache.get(*(*pf_i),*(*uf_i), proxyID);
	    log("equivalent(" + (*pf_i)->featureID() + "," 
		+ (*uf_i)->featureID() + ") = " 
		+ Binding::triboolToString(equivalent));
	    if(equivalent.value == tribool::true_value && !one_match_for_union_feature) {
	      ret.matches++;
	      one_match_for_union_feature = true;
	    }
	    if(equivalent.value == tribool::false_value) {
	      ret.mismatch = true;
	      finished = true;
	    }
//	    if(equivalent.value != tribool::indeterminate_value) {
//	      one_noninequivalent_found = true;
//	    }
	  } // individual union features	 
	}  // individual proxy features
      } // internally comparable
      u_i++;
    } // union features
    p_i++;
  } // proxy features
  log(scoreToString(ret) + "\n");
  ret.salienceHeuristics = _salienceHeuristics(proxy_saliences, union_saliences, proxy_creation_times, union_creation_times) 
    + static_cast<double>(union_element_nr); // a hack to make older singulars "look" better
  return ret;
}

double
_salienceHeuristicsHelper(const BindingFeatures::Salience& _s1,
			  const BindingFeatures::Salience& _s2) {
  if(overlap(_s1,_s2)) {
    return 0.0;
  }
  BindingFeaturesCommon::EndTime diff1 = diff(_s1.start, _s2.end);
  BindingFeaturesCommon::EndTime diff2 = diff(_s2.start, _s1.end);
  BindingFeaturesCommon::EndTime diff;
  if(end_time_leq(diff1,diff2)) {
    diff = diff1;
  } else {
    diff = diff2;
  }
  if(diff.t.length() == 0)
    return defaultBindingScore().salienceHeuristics;
  return -baltTime(diff.t[0]); // negative since the intervals are crosschecked
}

double 
BindingScorer::_salienceHeuristics(const vector<BindingFeatures::Salience>& _proxy_saliences,
				   const vector<BindingFeatures::Salience>& _union_saliences,
				   const vector<BindingFeatures::CreationTime>& _proxy_creation_times,
				   const vector<BindingFeatures::CreationTime>& _union_creation_times) const
{
  double ret = defaultBindingScore().salienceHeuristics;
  const vector<BindingFeatures::Salience>& ps(_proxy_saliences);
  const vector<BindingFeatures::Salience>& us(_union_saliences);
  const vector<BindingFeatures::CreationTime>& pc(_proxy_creation_times);
  const vector<BindingFeatures::CreationTime>& uc(_union_creation_times);
  for(vector<BindingFeatures::Salience>::const_iterator i = ps.begin(); i != ps.end() ; ++i){
    for(vector<BindingFeatures::Salience>::const_iterator j = us.begin(); j != us.end() ; ++j){
      if(immediateProxyID(*i) != immediateProxyID(*j)) {
	double h = _salienceHeuristicsHelper(*i,*j);
	if(h < ret) ret = h;
      }
    }
    for(vector<BindingFeatures::CreationTime>::const_iterator j = uc.begin(); j != uc.end() ; ++j){
      if(immediateProxyID(*i) != immediateProxyID(*j)) {
	BindingFeatures::Salience us;
	us.start = startTime(j->creationTime);
	us.end = endTime(j->creationTime);
	double h = _salienceHeuristicsHelper(*i,us);
	if(h < ret) ret = h;
      }
    }
  }
  for(vector<BindingFeatures::CreationTime>::const_iterator i = pc.begin(); i != pc.end() ; ++i){
    BindingFeatures::Salience ps;
    ps.start = startTime(i->creationTime);
    ps.end = endTime(i->creationTime);    
    for(vector<BindingFeatures::Salience>::const_iterator j = us.begin(); j != us.end() ; ++j){
      if(immediateProxyID(*i) != immediateProxyID(*j)) {	
	double h = _salienceHeuristicsHelper(*j,ps);
	if(h < ret) ret = h;
      }
    }
    for(vector<BindingFeatures::CreationTime>::const_iterator j = uc.begin(); j != uc.end() ; ++j){
      if(immediateProxyID(*i) != immediateProxyID(*j)) {
	BindingFeatures::Salience us;
	us.start = startTime(j->creationTime);
	us.end = endTime(j->creationTime);
	double h = _salienceHeuristicsHelper(us,ps);
	if(h < ret) ret = h;
      }
    }
  }
  return ret;
}

void
BindingScorer::updateComparison(const cdl::WorkingMemoryChange & _wmc) {
  log("BindingScorer::updateComparison(const cdl::WorkingMemoryChange & _wmc) {");
  comparatorCache.set(string(_wmc.address.id));
}


// returns true if the unionID is in the best-list
bool 
unionInBestList(const string& _unionID, const BindingData::BestUnionsForProxy& _best)
{
  for(unsigned int i = 0 ; i < _best.unionIDs.length() ; ++i) {
    if(string(_best.unionIDs[i]) == _unionID) { 
      return true;
    }
  }
  return false;
}

// returns true if the unionID is in the nonmatch-list
bool unionInNonMatchingList(const string& _unionID, const BindingData::NonMatchingUnions& _nonmatch)
{
  for(unsigned int i = 0 ; i < _nonmatch.nonMatchingUnionIDs.length() ; ++i) {
    if(string(_nonmatch.nonMatchingUnionIDs[i]) == _unionID) { 
      return true;
    }
  }
  return false;
}


enum ProxyCompatibility {
  MISMATCH, // each are bound to unions that can't be matching
  MATCH,    // each are bound to the same union
  UNKNOWN   // neither a match nor mismatch, these proxies COULD in
	    // principle be bound to the same union, possibly
};

bool
proxiesBound(const LBindingProxy& _proxy1, 
	     const LBindingProxy& _proxy2)
{
  if(_proxy1.id() == _proxy2.id()) { 
    // sure... a proxy is by definition bound to itself
    return true;
  }
  if(!_proxy1.bound() || !_proxy2.bound()) {
    // if one of the proxies are not bound at all, then they can also
    // not be bound to eachother. Perhaps they will be later, but then
    // related proxies should get rescored
    return false;
  }
  string unionID1(_proxy1->unionID);
  string unionID2(_proxy2->unionID);
  if(unionID1 == unionID2) { 
    // the proxies are bound to the same union,then they obviously match
    return true; 
  }
  // not the same union...
  return false;
}


ProxyCompatibility 
proxyCompatibility(const LBindingProxy& _proxy1, 
		   const LBindingProxy& _proxy2)
{
  if(_proxy1.id() == _proxy2.id()) { 
    // trivially a match since it is the same proxy
    return MATCH;
  }
  
//  if(!_proxy1.bound() || !_proxy2.bound()) {
    if(!_proxy1.scored() || !_proxy2.scored()) {
    // we can't know if they match if they're not scored yet
    return UNKNOWN;
  }
  string unionID1(_proxy1->unionID);
  string unionID2(_proxy2->unionID);
  if(unionID1 == unionID2) { 
    // the proxies are bound to the same union,then they obviously match
    return MATCH; 
  }
  const BindingData::BestUnionsForProxy& best1(_proxy1.bestUnionsForProxy());
  const BindingData::BestUnionsForProxy& best2(_proxy2.bestUnionsForProxy());  

  if(!(best1.unionIDs.length() == 1 || unionInBestList(unionID1,best1)) ||
     !(best2.unionIDs.length() == 1 || unionInBestList(unionID2,best2)))
    throw(BestListNotUpToDate());

  //#warning I don't understand what I did here, and thus do not understand what I now undo
  assert(best1.unionIDs.length() == 1 || unionInBestList(unionID1,best1)); // should be true, unless some synchronization issue messes it up
  //cout << "best2.unionIDs.length(): " << best2.unionIDs.length() << endl;
  assert(best2.unionIDs.length() == 1 || unionInBestList(unionID2,best2)); // should be true, unless some synchronization issue messes it up

  if(unionInBestList(unionID1,best2) || unionInBestList(unionID2,best1) ) {
    // the proxy is bound to a union which in principle COULD have been bound to the other proxy since it is equally scored
    return MATCH;
  }
  const BindingData::NonMatchingUnions& nonmatch1(_proxy1.nonMatchingUnions());
  const BindingData::NonMatchingUnions& nonmatch2(_proxy2.nonMatchingUnions());
  
  if(unionInNonMatchingList(unionID1,nonmatch2) || unionInNonMatchingList(unionID2,nonmatch1) ) {
    // the proxy is bound to a union which in principle COULD have been bound to the other proxy since it is equally scored
    return MISMATCH;
  }
  // last option... these proxies do not mismatch nor match
  return UNKNOWN;
}


/// scores a proxy vs a union
void 
BindingScorer::_relational_score(const LBindingProxy& _proxy,
				 const LBindingUnion& _union,
				 unsigned int& _score,
				 bool& _mismatch){
  _mismatch = false;
  _score = 0;
  // 1st, check the outports. These need to match by arity, port label
  // and the ported proxies may not be mismatching.
  { // outport scope
    const PortMap& proxyOutports(_proxy.outPorts());
    const PortMap& unionOutports(_union.outPorts());
    if(proxyOutports.size() != unionOutports.size()) {
      _mismatch = true;
      return;
    }
    assert(proxyOutports.size() == unionOutports.size());
    PortMap::const_iterator proxyOutports_i(proxyOutports.begin());
    PortMap::const_iterator unionOutports_i(unionOutports.begin());
    while(proxyOutports_i != proxyOutports.end()) {
      if(proxyOutports_i->first != unionOutports_i->first) {
	_mismatch = true;
	return;
      }
      for(set<BindingData::ProxyPort, proxyPortLess>::const_iterator proxy_port(proxyOutports_i->second.begin());
	  proxy_port != proxyOutports_i->second.end();
	  ++proxy_port) {
	for(set<BindingData::ProxyPort, proxyPortLess>::const_iterator union_port(unionOutports_i->second.begin());
	    union_port != unionOutports_i->second.end();
	    ++union_port) {
	  if(string(proxy_port->ownerProxyID) != string(union_port->ownerProxyID)) { // ports not compared to eachother if they stem from same proxy
	    ProxyCompatibility comp = UNKNOWN;
	    try{
	      comp = proxyCompatibility(proxyLocalCache[string(proxy_port->proxyID)],
					proxyLocalCache[string(union_port->proxyID)]);
	    } catch(const DoesNotExistOnWMException& _e) {
	      cerr << "This is sort of expected and should set comp to UNKNOWN here:\n" << _e.what() << endl;
#warning changed this since Alen had this abortion a lot
	      comp = UNKNOWN;
	      //abort();
	    }
	    switch(comp) {
	    case MISMATCH:
	      _mismatch = true;
	      return;
	    case MATCH:
	      _score++;
	      break;
	    case UNKNOWN:
	      break;
	    }
	  }
	}
      }
      ++proxyOutports_i;
      ++unionOutports_i;
    }
  } // outport scope
  // 2nd, check the inports, these never lead to mismatches (they
  // can't help who is referring to them...)
  const BindingData::ProxyPorts* p_ptr(NULL);
  const BindingData::ProxyPorts* u_ptr(NULL);
  try {
    p_ptr = &_proxy.inPorts();
    u_ptr = &_union.inPorts();
  } catch(const DoesNotExistOnWMException& _e) {
    cerr << "inports not existing in BindingScorer::_relational_score(...)\n " << _e.what() << endl;
    abort();
  }
  const BindingData::ProxyPorts& proxyInports(*p_ptr);
  const BindingData::ProxyPorts& unionInports(*u_ptr);
  for(unsigned int i = 0; i < unionInports.ports.length() ; ++i) {
    bool scored(false); // each union port only counted once
    for(unsigned int j = 0; j < proxyInports.ports.length() && !scored ; ++j) {
      if(string(unionInports.ports[i].ownerProxyID) != string(proxyInports.ports[j].ownerProxyID)) { // ports not compared to eachother if they stem from same proxy
	if(string(unionInports.ports[i].label) == string(proxyInports.ports[j].label)) { // only if they're related in the same way 
	  /*ProxyCompatibility comp = 
	    proxyCompatibility(proxyLocalCache[string(unionInports.ports[i].proxyID)],
	    proxyLocalCache[string(proxyInports.ports[j].proxyID)]);
	    switch(comp) {
	    case MISMATCH:
	    //#warning inports causing mismatches...
	    //	  _mismatch = true;
	    //	  return;
	    break;
	    case MATCH:
	    _score++;
	    scored = true;
	    break;
	    case UNKNOWN:
	    break;
	    }
	  */
	  try {
	    if(proxiesBound(proxyLocalCache[string(unionInports.ports[i].proxyID)],
			    proxyLocalCache[string(proxyInports.ports[j].proxyID)])) 
	      {	    
		_score++;
		scored = true;
	      }
	  } catch (const DoesNotExistOnWMException& _e) {
	    //cerr << "This is sort of expected and should just be ignored:\n" << _e.what() << endl;
	    //abort();
	  }
	}
      }
    }
  }
}


void
BindingScorer::_storeScore(const string& _proxyID,
			   const string& _unionID,
			   const BindingData::BindingScore& _score)
{
  
  BindingData::ProxyUnionScore* new_comparison = 
    new BindingData::ProxyUnionScore;
  new_comparison->proxyID = CORBA::string_dup(_proxyID.c_str());
  new_comparison->unionID = CORBA::string_dup(_unionID.c_str());
  new_comparison->updated = true;
  new_comparison->score = _score;
  string proxyUnionScoreID = combinedID(_proxyID, _unionID);
  log("proxyUnionScoreID: " + proxyUnionScoreID);

  if(existsOnWorkingMemory(proxyUnionScoreID)) {
    //loadBindingDataFromWM<BindingData::ProxyUnionScore>(proxyUnionScoreID);
    overwriteWorkingMemory(proxyUnionScoreID, 
			   //BindingLocalOntology::PROXY_UNION_SCORE_TYPE, 
			   new_comparison,
			   cdl::BLOCKING
			   );
  } else {
    addToWorkingMemory(proxyUnionScoreID, 
		       //BindingLocalOntology::PROXY_UNION_SCORE_TYPE, 
		       new_comparison,
		       cdl::BLOCKING
		       );
    lockEntry(proxyUnionScoreID, cdl::LOCKED_O);
  }
}

/// Triggered by an update FeatureComparison
void 
BindingScorer::registerComparatorCompetence(const cdl::WorkingMemoryChange & _wmc) {
  shared_ptr<const BindingData::FeatureComparisonCompetence> 
    competence(loadBindingDataFromWM<BindingData::FeatureComparisonCompetence>(_wmc));
  BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  ontology.registerComparatorCompetence(*competence);
}

void 
BindingScorer::answerBasicQuery(const cdl::WorkingMemoryChange & _wmc) 
{
  log("going to answer a BasicQuery");
  lockEntry(string(_wmc.address.id), cast::cdl::LOCKED_ODR);
  shared_ptr<const BindingQueries::BasicQuery> 
    query(loadBindingDataFromWM<BindingQueries::BasicQuery>(_wmc));
  _answerBasicQuery(*query, string(_wmc.address.id));
  unlockEntry(string(_wmc.address.id));
}

void 
BindingScorer::_answerBasicQuery(const BindingQueries::BasicQuery& _query,
				 const std::string& _queryID)
{
  assert(_query.processed == false);
  const string proxyID(string(_query.proxyID));
  BindingGraphHandler handler(*this);
  const BindingData::FeaturePointer& featurePointer(_query.featurePointer);
  const string featureID(featurePointer.address);
  bool loaded = false;
  ProxySet proxies;
  UnionSet unions;
  boost::tribool answer = indeterminate;
//  auto_ptr<ProxySet> included_proxies;
  // first make sure there is a consisten local copy of all proxies  
  while(!loaded) {
    try{
      proxies = handler.loadProxies(proxyID);
      proxies = proxies | 
	(!ProxyUnionIDChecker(""));
      unions = handler.extractUnionsFromProxies(proxies);
      if(_query.parameters.boundProxyInclusion == BindingQueries::INCLUDE_BOUND) {
//	included_proxies = 
//	  auto_ptr<ProxySet>(new ProxySet(handler.extractProxiesFromUnions(unions)));
	handler.extractProxiesFromUnions(unions);
      }
      loaded = true;
    } catch (const DoesNotExistOnWMException& _e) {
      // noop, this is possible to happen, just try again from the top
    }
  }
  try {
  //  cout << "BindingScorer : proxies.size(): " << proxies.size() << endl;
    //    assert(proxies.size() == 1);
    if(proxies.empty()) { // not bound yet, no answer
      answer = indeterminate;
    } 
    else if(string(_query.featurePointer.address) == "") { // "null" pointer feature... only answer based on existence of proxy
      answer = false;
      if(_query.parameters.boundProxyInclusion == BindingQueries::EXCLUDE_BOUND) {
	if(true_for_all(proxies, HasFeatureCheck<ProxyPtr>(string(_query.featurePointer.type))))
	  answer = true;
      } else {
	if(true_for_all(unions, HasFeatureCheck<UnionPtr>(string(_query.featurePointer.type)))) 
	  answer = true;
      }
    } else {
      if(bLogOutput)
	cout << "Gonna answer the basic query on this set of proxies:" 
	     << proxies << endl;
      
      assert(_query.processed == false);
      
      boost::shared_ptr<AbstractFeature> feature;
      try {
	feature = featureLoader.getFeature(featurePointer);
      } catch (const DoesNotExistOnWMException& _e) {
	cout << "caught this in BindingScorer::answerBasicQuery: " << _e.what() 
	     << "\nthe reason was that a feature occuring in a BindingQueries::BasicQuery did not exist on Binding WM\naborting" << endl;
	abort();
      }
      
      ComparatorCacheCheck 
	checker(comparatorCache,*feature,true,_query.parameters);
      checker.test(proxies.begin()->second);
      if(!checker.cacheReady()) {
	foreach(const string& id, checker.featureComparisonIDs()) {
	  openBasicQueryFeatureComparisonIDs.insert(make_pair(id,_queryID));
	  assert(!id.empty());
	  assert(!_queryID.empty());
	  unlockEntry(id);
	}
	log("not ready to answer basic query yet");
	// not yet ready to answer
	return;
      }
      answer = checker.answer();	
    }
    log("The answer is " + Binding::triboolToString(answer));
    
    BindingQueries::BasicQuery* ret(new BindingQueries::BasicQuery(_query)); 
    ret->answer = tribool_cast(answer);
    ret->processed = true;
    
    overwriteWorkingMemory(_queryID, 
			   ret);
    unlockEntry(_queryID);
    if(!featureID.empty())
      deleteFromWorkingMemory(featureID);
  }
  catch(const DoesNotExistOnWMException& _e) {
    cerr << "caught this unexpectedly in BindingScorer::answerBasicQuery:\n" << _e.what() << endl;
    abort();
  }
}

void
copy_strings(const set<string>& _src, 
	     BindingData::WorkingMemoryIDList& _dst) {
  _dst.length(_src.size());
  unsigned int i = 0;
  foreach(const string& str, _src) {
    _dst[i++] = CORBA::string_dup(str.c_str());
    assert(!str.empty());
  }
}

void 
translateQueryResults(const BindingQueries::AdvancedQuery& _query,
		      const ProxySet& _proxies,
		      BindingData::WorkingMemoryIDList& _proxyIDs,
		      BindingData::WorkingMemoryIDList& _unionIDs)
{
  set<string> proxyIDs;
  set<string> unionIDs;
  insert_iterator<set<string> > proxyIDs_inserter = 
    inserter(proxyIDs,proxyIDs.begin());
  insert_iterator<set<string> > unionIDs_inserter = 
    inserter(unionIDs,unionIDs.begin());
  foreach(const ProxySet::value_type& proxy, _proxies) {
    //    if(_query.parameters.boundProxyInclusion == 
    //   BindingQueries::EXCLUDE_BOUND)
    proxyIDs_inserter = proxy.first;
    /*else // include the bound stuff in answer
      foreach(const string& id, proxy.second->bindingUnion().proxyIDs()) {
      proxyIDs_inserter = id;	  
      }
    */
    unionIDs_inserter = proxy.second->bindingUnionID();
  }
  copy_strings(proxyIDs,_proxyIDs);
  copy_strings(unionIDs,_unionIDs);
}

void 
BindingScorer::answerAdvancedQuery(const cast::cdl::WorkingMemoryChange & _wmc)
{
  cout << "BindingScorer::answerAdvancedQuery 1" << endl;
  log("going to answer an AdvancedQuery");
  assert(string(_wmc.address.subarchitecture) != "");
  cout << "BindingScorer::answerAdvancedQuery 2" << endl;
  lockEntry(_wmc.address, cast::cdl::LOCKED_ODR);
  shared_ptr<const BindingQueries::AdvancedQuery> query = loadBindingDataFromWM<BindingQueries::AdvancedQuery>(_wmc);
  _answerAdvancedQuery(*query, string(_wmc.address.id));
  unlockEntry(_wmc.address);
}
  
void 
BindingScorer::_answerAdvancedQuery(const BindingQueries::AdvancedQuery& _query,
				    const std::string& _queryID)
{
  log("answering an AdvancedQuery: " + _queryID);
  assert(_query.processed == false);
  BindingGraphHandler handler(*this);
  bool loaded = false;
  ProxySet proxies;
  auto_ptr<ProxySet> included_proxies;
  // first make sure there is a consisten local copy of all proxies
  while(!loaded) {
    try{
      cout << "gonna " << "proxies = handler.allProxiesFromWM();" << endl;
      proxies = handler.allProxiesFromWM();
      cout << "did " << "proxies = handler.allProxiesFromWM();" << endl;
      proxies = proxies | 
	(HasFeatureCheck<ProxyPtr>(string(_query.featurePointer.type)) && 
	 !ProxyUnionIDChecker(""));
      UnionSet unions = handler.extractUnionsFromProxies(proxies);
      if(_query.parameters.boundProxyInclusion == BindingQueries::INCLUDE_BOUND) {
	included_proxies = 
	  auto_ptr<ProxySet>(new ProxySet(handler.extractProxiesFromUnions(unions)));
      }
      loaded = true;
    } catch (const DoesNotExistOnWMException& _e) {
      // noop, this is possible to happen, just try again from the top
    }
  }
  cout << "Gonna answer the advanced query on this set of proxies:" << proxies << endl;
  try{
    BindingQueries::AdvancedQuery* answer = 
      new BindingQueries::AdvancedQuery(_query);
    ProxySet* prxptr =NULL;
    if(included_proxies.get())
      prxptr = included_proxies.get();
    else
      prxptr = &proxies;
    translateQueryResults(_query,
			  *prxptr,
			  answer->hasTheFeatureProxyIDs,
			  answer->hasTheFeatureUnionIDs);
    assert(prxptr->size() == answer->hasTheFeatureProxyIDs.length());
    string featureID(_query.featurePointer.address);
    log("testing vs. featureID:" + featureID);
    if(!featureID.empty()) {
      const AbstractFeature& 
	feature(*featureLoader.getFeature(_query.featurePointer));
      ComparatorCacheCheck 
	checker(comparatorCache,feature,true,_query.parameters);
      ProxySet matching_proxies = proxies | checker;
      cout << "for " << _query << "\n" << matching_proxies << endl;
      if(!checker.cacheReady()) {
	foreach(const string& id, checker.featureComparisonIDs()) {
	  openAdvancedQueryFeatureComparisonIDs.insert(make_pair(id,_queryID));
	  assert(!id.empty());
	  assert(!_queryID.empty());
	  unlockEntry(id);
	}
	log("not ready to answer query yet");
	// not yet ready to answer
	delete answer;
	return;
      }
      log("All answers received, ready to answer");
      //cout << "matching proxies: " << matching_proxies << endl;
      assert(true_for_all(matching_proxies, 
      			  ComparatorCacheCheck(comparatorCache,
					       feature,
					       tribool::true_value,
					       _query.parameters)));
      assert(true_for_all(matching_proxies, 
			  !ComparatorCacheCheck(comparatorCache,
						feature,
						tribool::false_value,
						_query.parameters)));


      UnionSet matching_unions = 
	handler.extractUnionsFromProxies(matching_proxies);
      ProxySet nonmatching_proxies = 
	proxies | 
	ComparatorCacheCheck(comparatorCache,
			     feature,
			     false, 
			     _query.parameters);
      //cout << "nonmatching proxies: " << nonmatching_proxies << endl;
      assert(true_for_all(nonmatching_proxies, 
      			  !ComparatorCacheCheck(comparatorCache,
						feature,
						tribool::true_value,
						_query.parameters) &&
			  ComparatorCacheCheck(comparatorCache,
					       feature,
					       tribool::false_value,
					       _query.parameters))) ;
	     
      UnionSet nonmatching_unions = 
	handler.extractUnionsFromProxies(nonmatching_proxies);
      translateQueryResults(_query,
			    matching_proxies,
			    answer->matchingProxyIDs,
			    answer->matchingUnionIDs);
      translateQueryResults(_query,
			    nonmatching_proxies,
			    answer->nonMatchingProxyIDs,
			    answer->nonMatchingUnionIDs);
    }
    answer->processed = true;
    // now, provide the answer to WM, blocking, just for safety
    //    log("now the answer is written back to WM:");
    //cout << *answer << endl;
    overwriteWorkingMemory(_queryID,
			   answer,
			   cdl::BLOCKING);
    
    if(!featureID.empty())
      deleteFromWorkingMemory(featureID);
  }
  catch(const DoesNotExistOnWMException& _e) {
    cerr << "caught this unexpectedly in BindingScorer::answerAdvancedQuery:\n" << _e.what() << endl;
    abort();
  }
}
void
BindingScorer::runComponent() 
{
  addToWorkingMemory(std::string(BindingData::binderTokenTokenID), new BindingData::BinderToken());
  addToWorkingMemory(std::string(BindingData::bindTheseProxiesMonitorTokenID), new BindingData::MonitorToken());
  addToWorkingMemory(std::string(BindingData::updateProxyMonitorTokenID), new BindingData::MonitorToken(),cast::cdl::BLOCKING);
  addToWorkingMemory(std::string(BindingData::binderLockTokenID), new BindingData::MonitorToken(),cast::cdl::BLOCKING);
  addToWorkingMemory(std::string(BindingData::binderTokenID), new BindingData::BinderToken(),cast::cdl::BLOCKING);
  addToWorkingMemory(std::string(BindingData::internalBindingTokenID), new BindingData::BinderToken(),cast::cdl::BLOCKING);
  acquireBinderTokenToken(); 
  if(dummy) {
    //lockProcess();
    receiveNoChanges();
  }  

  /*  const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
      
  //OK... while cast is not working... let's do all testing here...
  BindingFeatures::Concept concept1;
  concept1.concept = CORBA::string_dup("assertionconcept1");
  BindingFeatures::Concept concept2 = concept1;
  BindingFeatures::Concept concept3;
  concept3.concept = CORBA::string_dup("assertionconcept2");  
  BindingFeatures::Concept negated_concept3;
  negated_concept3.concept = CORBA::string_dup("assertionconcept2");  
  negated_concept3.parent.truthValue = BindingFeaturesCommon::NEGATIVE;
  
  shared_ptr<const AbstractFeature> feature1(new Feature<BindingFeatures::Concept>(concept1, "id1"));
  shared_ptr<const AbstractFeature> feature2(new Feature<BindingFeatures::Concept>(concept2, "id2"));
  shared_ptr<const AbstractFeature> feature3(new Feature<BindingFeatures::Concept>(concept3, "id3"));
  shared_ptr<const AbstractFeature> negated_feature3(new Feature<BindingFeatures::Concept>(negated_concept3, "nid3"));
  
  assert(!ontology.featureHelper(typeid(BindingFeatures::Concept)).operatorLess(*feature1,*feature2));
  assert(!ontology.featureHelper(typeid(BindingFeatures::Concept)).operatorLess(*feature2,*feature1));
  assert(!ontology.featureHelper(feature1->typeInfo()).operatorLess(*feature2,*feature1));
  assert(typeid(BindingFeatures::Concept) == feature1->typeInfo() && feature1->typeInfo() == feature2->typeInfo());
  assert(!ontology.featureHelper(typeid(BindingFeatures::Concept)).operatorLess(*feature3,*feature1));

  assert(ontology.internalComparator(typeid(BindingFeatures::Concept),typeid(BindingFeatures::Concept)));
  assert(!ontology.internalComparator(typeid(BindingFeatures::DebugString),typeid(BindingFeatures::Concept)));
  assert(!ontology.internalComparator(typeid(BindingFeatures::Concept),typeid(BindingFeatures::DebugString)));
  assert(!ontology.internalComparator(typeid(BindingFeatures::DebugString),typeid(BindingFeatures::DebugString)));
  
  assert(static_cast<bool>(ontology.internalComparator(typeid(BindingFeatures::Concept),typeid(BindingFeatures::Concept))->compare(*feature1,*feature2)));
  assert(static_cast<bool>(ontology.internalComparator(typeid(BindingFeatures::Concept),typeid(BindingFeatures::Concept))->compare(*feature2,*feature1)));
  assert(!static_cast<bool>(ontology.internalComparator(typeid(BindingFeatures::Concept),typeid(BindingFeatures::Concept))->compare(*feature1,*feature3)));
  assert(!static_cast<bool>(ontology.internalComparator(typeid(BindingFeatures::Concept),typeid(BindingFeatures::Concept))->compare(*feature2,*feature3)));
  assert(!static_cast<bool>(ontology.internalComparator(typeid(BindingFeatures::Concept),typeid(BindingFeatures::Concept))->compare(*feature3,*feature1)));
  assert(!static_cast<bool>(ontology.internalComparator(typeid(BindingFeatures::Concept),typeid(BindingFeatures::Concept))->compare(*feature3,*feature2)));
  assert(!static_cast<bool>(ontology.internalComparator(typeid(BindingFeatures::Concept),typeid(BindingFeatures::Concept))->compare(*feature2,*feature3)));
  //assert(!static_cast<bool>(ontology.internalComparator(typeid(BindingFeatures::Concept),typeid(BindingFeatures::Concept))->compare(*feature3,*negated_feature3)));
  //assert(!static_cast<bool>(ontology.internalComparator(typeid(BindingFeatures::Concept),typeid(BindingFeatures::Concept))->compare(*negated_feature3,*feature3)));

  BindingFeatures::DebugString debugstring1;
  debugstring1.debugString = CORBA::string_dup("testdebugstring");
  BindingFeatures::DebugString debugstring2 = debugstring1;

  shared_ptr<const AbstractFeature> feature4(new Feature<BindingFeatures::DebugString>(debugstring1, "id4"));
  shared_ptr<const AbstractFeature> feature5(new Feature<BindingFeatures::DebugString>(debugstring2, "id5"));
*/

  

};  


void 
BindingScorer::proxyUpdateReceived(const cast::cdl::WorkingMemoryChange & _wmc) 
{
  string proxyID(_wmc.address.id);
  try{
    if(_wmc.operation == cast::cdl::DELETE)
      processedProxies.erase(proxyID);
    else if(proxyLocalCache[proxyID].proxyState() == BindingData::BOUND)
      processedProxies.erase(proxyID);
    cout << "proxyUpdate received regarding proxy " << proxyID << endl;
    cout << "Token issue (after proxy update): processedProxies.size(): " << processedProxies.size() << " {";
    bool bla = false;
    foreach(const string& p, processedProxies) { 
      cout << (bla?", ":"") << p; bla = true;
    }
    cout << "}\n";
  }
  catch(const DoesNotExistOnWMException& _e) {
    log(string("In BindingScorer::proxyUpdateReceived: ") + _e.what());
    processedProxies.erase(proxyID);
  }
  if(processedProxies.empty()) {
    //    acquireBinderLockToken(); // the next parts must be atomic
    if(hasBinderToken())
      releaseBinderToken();  
    assert(hasBinderTokenToken());
    releaseBinderTokenToken(); // so that some monitor may continue
    acquireBinderTokenToken();
    
    //releaseBinderLockToken();
  }
}

void 
BindingScorer::binderStatusReceived(const cast::cdl::WorkingMemoryChange & _wmc)
{
  /* const BindingData::BinderStatus
     status(*loadBindingDataFromWM<BindingData::BinderStatus>(_wmc));
     //  cout << "binderStatusReceived: " << status.stable<< endl;
     
     if(status.stable) {
     processedProxies.clear();
     if(hasBinderToken())
     releaseBinderToken();
     assert(hasBinderTokenToken());
     releaseBinderTokenToken(); // so that some monitor may continue
     acquireBinderTokenToken();
     
     }
  */
}

void
BindingScorer::proxyProcessingFinished(const cast::cdl::WorkingMemoryChange & _wmc)
{
  string proxyID(loadBindingDataFromWM<BindingData::ProxyProcessingFinished>(_wmc)->proxyID);
  deleteFromWorkingMemory(_wmc.address);
  processedProxies.erase(proxyID);
  cout << "in Scorer: ProxyProcessingFinished: " << proxyID << endl;
  cout << "Token issue(after ProxyProcessingFinished add): processedProxies.size(): " << processedProxies.size() << " {";
  bool bla = false;
  foreach(const string& p, processedProxies) { 
    cout << (bla?", ":"") << p; bla = true;
  }
  cout << "}\n";

  if(processedProxies.empty()) {
    //    acquireBinderLockToken(); // the next parts must be atomic
    if(hasBinderToken())
      releaseBinderToken();  
    assert(hasBinderTokenToken());
    releaseBinderTokenToken(); // so that some monitor may continue
    acquireBinderTokenToken();
    //releaseBinderLockToken();
  }

}

} // namespace Binding 


