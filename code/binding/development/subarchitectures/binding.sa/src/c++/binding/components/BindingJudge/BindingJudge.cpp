#include "binding/abstr/AbstractMonitor.hpp"
#include "binding/utils/BindingUtils.hpp"
#include "BindingJudge.hpp"
#include "binding/feature-utils/Features.hpp"
#include "binding/utils/BindingScoreUtils.hpp"

#include <cstdlib>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/assign.hpp>


using namespace std;
using namespace boost;
using namespace boost::assign;
using namespace cast;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Binding::BindingJudge(_id);
  }
}

namespace Binding {

BindingJudge::BindingJudge(const string &_id) : 
  
  WorkingMemoryAttachedComponent(_id),
  AbstractBinder(_id),
  m_bindingTask(false)
{ 
}

void
BindingJudge::start() {

  AbstractBinder::start();

  //addChangeFilter(BindingLocalOntology::PROXY_UNION_SCORE_TYPE, 
  //cdl::OVERWRITE, 
  //cdl::LOCAL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<BindingJudge>(this,
  //&BindingJudge::scoreUpdated));
  addChangeFilter(createLocalTypeFilter<BindingData::ProxyUnionScore>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<BindingJudge>(this,
								 &BindingJudge::scoreUpdated));
  
  //addChangeFilter(BindingLocalOntology::PROXY_UNION_SCORE_TYPE, 
  //cdl::ADD, 
  //cdl::LOCAL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<BindingJudge>(this,
  //&BindingJudge::scoreUpdated));
  
  addChangeFilter(createLocalTypeFilter<BindingData::ProxyUnionScore>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingJudge>(this,
								 &BindingJudge::scoreUpdated));
  
  //addChangeFilter(BindingLocalOntology::BINDING_PROXY_TYPE, 
  //cdl::DELETE, 
  //cdl::ALL_SA, //only look at sa-local changes (no and
  ////yes... they are written locally, but from
  ////external sources)
  //new MemberFunctionChangeReceiver<BindingJudge>(this,
  //&BindingJudge::proxyDeleted));
  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<BindingJudge>(this,
								 &BindingJudge::proxyDeleted));

  
  //addChangeFilter(BindingLocalOntology::BINDING_UNION_TYPE, 
  //cdl::DELETE, 
  //cdl::LOCAL_SA, //only look at sa-local changes 
  //new MemberFunctionChangeReceiver<BindingJudge>(this,
  //&BindingJudge::unionDeleted));  
  addChangeFilter(createLocalTypeFilter<BindingData::BindingUnion>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<BindingJudge>(this,
								 &BindingJudge::unionDeleted));  
  

  addChangeFilter(createLocalTypeFilter<BindingData::BindingTask>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<BindingJudge>(this,
								 &BindingJudge::bindingTaskDeleted));

  

  addChangeFilter(createLocalTypeFilter<BindingData::BestUnionsForProxy>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingJudge>(this,
								 &AbstractBinder::acquireEntry<BindingData::BestUnionsForProxy>));
  addChangeFilter(createLocalTypeFilter<BindingData::NonMatchingUnions>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingJudge>(this,
								 &AbstractBinder::acquireEntry<BindingData::NonMatchingUnions>));

  

}

BindingJudge::~BindingJudge() {

}


void 
BindingJudge::scoreUpdated(const cdl::WorkingMemoryChange & _wmc)
{  
  log(string("scoreUpdated : ") + string(_wmc.m_address.m_id));
  shared_ptr<const BindingData::ProxyUnionScore> puscore;
  try{
    puscore = loadBindingDataFromWM<BindingData::ProxyUnionScore>(_wmc);
  } catch (const DoesNotExistOnWMException& _e) {
    log("caught this in BindingJudge::scoreUpdated, the just updated score is deleted. skipping it.");
    return;
  }
  
  assert(puscore->m_updated); // no motivator no more...
  if(!puscore->m_updated) {
    log("scoreUpdated by motivator, will need to wait for results, do nothing");
    return;
  }
  string proxyID(puscore->m_proxyID);
  string unionID(puscore->m_unionID);
  
  log("Score received: " + proxyID + " vs. " + unionID + " : " + scoreToString(puscore->m_score));
  
  if(m_deletedProxies.find(proxyID) != m_deletedProxies.end()) {
    log("An updated score for deleted proxy " + proxyID + " received. Ignoring this.");
    assert(m_proxyScores.find(proxyID) == m_proxyScores.end());
    return;
  }
  if(m_deletedUnions.find(unionID) != m_deletedUnions.end()) {
    log("An updated score for deleted union " + unionID + " received. Ignoring this.");
    
//#define NEVERMIND    
    //#ifndef NDEBUG    // getting paranoid
#ifdef NEVERMIND
    for(StringMap<ProxyScores>::map::iterator proxy_scores_itr = m_proxyScores.begin() ;
	proxy_scores_itr != m_proxyScores.end() ;
	++proxy_scores_itr) {
      assert(proxy_scores_itr->second.m_bestUnions.find(unionID) 
	     == proxy_scores_itr->second.m_bestUnions.end());
      assert(proxy_scores_itr->second.m_unionScore.find(unionID) 
	     == proxy_scores_itr->second.m_unionScore.end());
    }
#endif // NDEBUG
    return;
  }
  
  const BindingData::BindingScore& score = puscore->m_score;  
  
  if(m_proxyScores.find(proxyID) == m_proxyScores.end()) { // new proxy
    m_proxyScores[proxyID] = ProxyScores();
  }
  
  if(m_proxyScores[proxyID].m_unionScore.empty()) {
    m_proxyScores[proxyID].m_bestBindingScore = defaultBindingScore();
  }
  
  m_proxyScores[proxyID].m_unionScore[unionID] = score;
  
  
  try{
    _calculateBestUnionsAndStoreIfNecessary(proxyID);
  } catch (const DoesNotExistOnWMException& _e) {
    cerr << "caugth an DoesNotExistOnWMException from BindingJudge::_calculateBestUnionsAndStoreIfNecessary(\""
	 << proxyID << "\"):\n all those exceptions should be handled within the function... aborting\n" 
	 << _e.what() << endl;
    abort();
  };
}


void 
BindingJudge::unionUpdated(const cdl::WorkingMemoryChange & _wmc) {
//  cerr << "-   enter BindingJudge::unionUpdated\n"; cerr.flush();
  const string unionID(_wmc.m_address.m_id);    
  log("union updated: " + unionID);

  shared_ptr<const BindingData::BindingUnion> binding_union;
  try {
    binding_union = (loadBindingDataFromWM<BindingData::BindingUnion>(_wmc));
  } catch (const DoesNotExistOnWMException&) {return;}
//  m_uni2prox[unionID].clear();
  for(unsigned int i = 0; i < binding_union->m_proxyIDs.length(); i++) {
    const string proxyID(binding_union->m_proxyIDs[i]);
    m_prox2uni[proxyID] = unionID;
    assert(m_prox2uni[proxyID] == unionID); // duh!
    log("*** updated: " + proxyID + " -> " + unionID);
//    m_uni2prox[unionID].insert(proxyID);
  }
  //assert(m_uni2prox[unionID].size() == binding_union->m_proxyIDs.length());
#ifndef NDEBUG
/*  assert(m_uni2prox.find(unionID) != m_uni2prox.end());
  const set<string>& proxies(m_uni2prox.find(unionID)->second);
  for(set<string>::const_iterator i = proxies.begin() ;
      i != proxies.end();
      ++i){
    assert(m_prox2uni.find(*i) != m_prox2uni.end());
    assert(m_prox2uni[*i] == unionID);
  }*/
#endif //NDEBUG
  //cerr << "-    exit BindingJudge::unionUpdated\n"; cerr.flush();
}

void 
BindingJudge::unionDeleted(const cdl::WorkingMemoryChange & _wmc) {
//  cerr << "-   enter BindingJudge::unionDeleted\n"; cerr.flush();
  log(string("unionDeleted: ") + string(_wmc.m_address.m_id));
  string unionID(_wmc.m_address.m_id);
  m_deletedUnions.insert(unionID);

  
  // delete all references to the deleted union
  for(StringMap<ProxyScores>::map::iterator proxy_scores_itr = m_proxyScores.begin() ;
      proxy_scores_itr != m_proxyScores.end() ;
      ++proxy_scores_itr) {
    proxy_scores_itr->second.m_unionScore.erase(unionID);
    // since a union has been removed, it possible that the
    // highscore unions for this proxy may have changed (or, an
    // unscored binding was deleted and the full high-score list can
    // be computed)
    try {
      _calculateBestUnionsAndStoreIfNecessary(proxy_scores_itr->first);
    } catch(const DoesNotExistOnWMException&) {};
  }
  
//  cerr << "-    exit BindingJudge::unionDeleted\n"; cerr.flush();  
}

void 
BindingJudge::proxyDeleted(const cdl::WorkingMemoryChange & _wmc) {
  log(string("proxyDeleted: ") + string(_wmc.m_address.m_id));
  
  string proxyID(_wmc.m_address.m_id);
  m_proxyScores.erase(proxyID);
  m_deletedProxies.insert(proxyID);
  
}


struct HeuristicCmp {
  bool operator()(const BindingData::BindingScore& _score1, const BindingData::BindingScore& _score2) {
    assert(_score1.m_comparable == _score2.m_comparable);
    assert(_score1.m_mismatch == _score2.m_mismatch);
    assert(_score1.m_matches == _score2.m_matches);
    assert(_score1.m_relationMismatch == _score2.m_relationMismatch);
    assert(_score1.m_relationMatches == _score2.m_relationMatches);
    assert(_score1.m_sticky == _score2.m_sticky);
    assert(string(_score1.m_proxyID) == string(_score2.m_proxyID));
    return _score1.m_salienceHeuristics < _score2.m_salienceHeuristics;
  }
};

vector<string>
BindingJudge::_bestUnions(const map<string,BindingData::BindingScore>::map& _bindingScores, 
			  BindingData::BindingScore& _best_score, 
			  const BindingData::BindingScore& _threshold_score,
			  const string& _already_bound_to,
			  const string& _proxyID) {
  vector<string> ret;
//  ret.push_back(_already_bound_to); // give this a preference, if is not bound to anything, it should be BindingData::NO_UNION
  //const BindingData::BindingScore& current_score(bind->second);
//  _best_score = _threshold_score;
  assert(_bindingScores.find(string(BindingData::NO_UNION)) != _bindingScores.end());
  _best_score = _bindingScores.find(string(BindingData::NO_UNION))->second;
  multimap<BindingData::BindingScore,string,HeuristicCmp> best_ones;
  best_ones.insert(make_pair(_best_score,string(BindingData::NO_UNION)));

  assert(!_bindingScores.empty());
  
  for(map<string,BindingData::BindingScore>::const_iterator bind = _bindingScores.begin(); 
      bind != _bindingScores.end(); 
      ++bind) {
    if(bind->first != (BindingData::NO_UNION)) { // ignore the NIL
      assert(!best_ones.empty());
      assert(bind->first != "");
      const BindingData::BindingScore& current_score(bind->second);
      
      //log(scoreToString(current_score) + " < " + scoreToString(_best_score) + "?");
      if(current_score < _best_score) { // remember, it is a
	// mimimization problem!
	// BindingScore is "inverted",
	// low score is good
	_best_score = current_score;
	
	////ret.clear();
	////ret.push_back(bind->first);
	best_ones.clear();
	best_ones.insert(make_pair(current_score,bind->first));
      } else if (!(current_score < _best_score) &&
		 !(_best_score < current_score) //&&
		 ////(ret.empty() || (*ret.begin()) != string(BindingData::NO_UNION))) {
		 //(best_ones.empty()) // If the match is to no union, then no other union can beat it
		 ) {       
	////ret.push_back(bind->first);
	best_ones.insert(make_pair(current_score,bind->first));
	_best_score = current_score; // to get the misc members
      }
    }
  }
  //// if(ret.empty()) { // nothing fits the proxy anymore
  ////   ret.push_back(string(BindingData::NO_UNION));
  //// }

  assert(!best_ones.empty());  
  for(multimap<BindingData::BindingScore,string,HeuristicCmp>::const_iterator i = best_ones.begin() ; 
      i != best_ones.end();
      i++) {
    ret.push_back(i->second);
  }
  assert(!ret.empty());
  return ret;
}


vector<string>
BindingJudge::_nonMatchingUnions(const string& _proxyID, const map<string,BindingData::BindingScore>::map& _bindingScores) {
  vector<string> ret;
  for(map<string,BindingData::BindingScore>::map::const_iterator bind = _bindingScores.begin() ; 
      bind != _bindingScores.end(); 
      ++bind) {
    if(!bind->second.m_comparable || bind->second.m_mismatch || bind->second.m_relationMismatch) { 
/*      cout << "nonmatching score : " << scoreToString(bind->second) << endl;
      const LBindingProxy& proxy(m_proxyLocalCache[_proxyID]);
      if(proxy.bound() && string(proxy->m_unionID) != bind->first) {
	cout << proxy.id() << " is bound to " << proxy->m_unionID << endl;
	cout << "ERROR BANANA ERROR!\n";
	//abort();
      }*/
      ret.push_back(bind->first);
    }
  }
  return ret;
}


string
ttoString(const vector<string>& strings) {
  stringstream out;
  for(vector<string>::const_iterator i = strings.begin() ; i != strings.end() ; i++)
    out << *i << " ";
  return out.str();
}

 
void 
BindingJudge::_calculateBestUnionsAndStoreIfNecessary(const string & _proxyID )
{
  if(m_bindingTask) { // there is already a task, then just enqueue the proxy ID
    m_proxyIDQueue.enqueue(_proxyID);
    string tmp;
    for(non_repeating_queue<string>::const_iterator itr = m_proxyIDQueue.begin(); itr != m_proxyIDQueue.end() ; ++itr) {
      tmp += *itr + ", ";
    }
    log("proxy queue just grew: {" + tmp + "}");
    return;
  }
  
  vector<shared_ptr<const CASTData<BindingData::BindingUnion> > > unions;
  getWorkingMemoryEntries<BindingData::BindingUnion>(//BindingLocalOntology::BINDING_UNION_TYPE,
			  0,
			  unions);
  
  cast::StringMap<ProxyScores>::map::iterator itr = m_proxyScores.find(_proxyID);
  //assert(itr != m_proxyScores.end());
  if(itr == m_proxyScores.end()) {
    log("proxy score missing, assuming it was deleted but on the queue");
    return;
  }
  ProxyScores& proxyScores(itr->second);
  
  
  const LBindingProxy* proxy_tmp = NULL;
  try {
    proxy_tmp = &m_proxyLocalCache[_proxyID];
  } 
  catch (const DoesNotExistOnWMException&) {
    BindingData::ProxyProcessingFinished* p = new BindingData::ProxyProcessingFinished;
    p->m_proxyID = CORBA::string_dup(_proxyID.c_str());
    addToWorkingMemory(newDataID(),p, cast::cdl::BLOCKING);
    return;
  }

  const LBindingProxy& proxy(*proxy_tmp);

  log(string("proxy: ") + _proxyID);
  log(string(" # of unions on WM: ") + lexical_cast<string>(unions.size()) + " + 1");
  log(string("# of unions scored: ") + lexical_cast<string>(proxyScores.m_unionScore.size()));
#ifndef NDEBUG
//#ifndef BLURB
  string on_wm;
  string scored;
  set<string> ids_on_wm;
  for(vector<shared_ptr<const CASTData<BindingData::BindingUnion> > >::const_iterator i = unions.begin() ;
      i != unions.end() ; ++i) {
    on_wm += string((*i)->getID()) + " ";
    ids_on_wm.insert(string((*i)->getID()));
  }
  
  for(map<string,BindingData::BindingScore>::map::const_iterator i = proxyScores.m_unionScore.begin();
      i != proxyScores.m_unionScore.end() ; 
      ++i) {
    scored+= i->first + " ";
    ids_on_wm.erase(i->first);    
  }
  debug("on WM: " + on_wm);
  debug("scored: " + scored);

  string not_scored;
  for(set<string>::const_iterator i = ids_on_wm.begin();
      i != ids_on_wm.end() ; 
      ++i)
    not_scored+= *i + " ";  
  debug("not scored: " + not_scored);
#endif //NDEBUG
  
  // if all unions are scored, then generate a set of the best ones
  // for this proxy and store the result onto WM
  if(proxyScores.m_unionScore.size() ==
     unions.size() + 1) { // the + 1 is for the comparison to BindingData::NO_UNION
    
    const vector<string> last_best_unions(proxyScores.m_bestUnions);
    if(!last_best_unions.empty())
      assert(last_best_unions[0] != "");

    // check if the proxy is already bound to a union (this is
    // tricky since there is no guarantee that the proxy "knows"
    // yet what it is bound to)
    string bound_to = BindingData::NO_UNION;
    map<string,string>::const_iterator p2u = m_prox2uni.find(_proxyID);
    if(p2u != m_prox2uni.end()) {
      bound_to = p2u->second;
    }
    
    vector<string> nonmatching_unions = _nonMatchingUnions(_proxyID, proxyScores.m_unionScore);
    BindingData::NonMatchingUnions* 
      new_nonmatch(new BindingData::NonMatchingUnions());
    new_nonmatch->m_proxyID = CORBA::string_dup(_proxyID.c_str());
    new_nonmatch->m_nonMatchingUnionIDs.length(nonmatching_unions.size());
    vector<string>::const_iterator nonmatch_itr = nonmatching_unions.begin(); 
    for(unsigned int i = 0;
	i < new_nonmatch->m_nonMatchingUnionIDs.length() ; 
	++i,++nonmatch_itr) {
      new_nonmatch->m_nonMatchingUnionIDs[i] = CORBA::string_dup(nonmatch_itr->c_str());
    }
    const string nonmatchID(proxy->m_nonMatchingUnionID);
    // to pass the consistency check, the data must be loaded before
    // it's written. It's not strictly necessary for any other purpose
    // in this particular place.
//    getWorkingMemoryEntry<BindingData::NonMatchingUnions>(nonmatchID);
    overwriteWorkingMemory(nonmatchID,
			   //BindingLocalOntology::NON_MATCHING_UNIONS_TYPE, 
			   new_nonmatch,
			   cdl::BLOCKING);
    
    log("proxy " + _proxyID + " is already bound to " + bound_to +
	" which will be given preference in the best list generation");
    
    BindingData::BindingScore score;
    
    
    BindingData::BindingScore thresholdScore = defaultThresholdScore();
    thresholdScore.m_proxyUpdatesWhenThisComputed = m_proxyScores[_proxyID].m_bestBindingScore.m_proxyUpdatesWhenThisComputed;
    
    if(proxyScores.m_unionScore.find(string(BindingData::NO_UNION)) == proxyScores.m_unionScore.end()) {
      log("oddly enough, the NIL union score was not found. This may happen during rapid deletion of proxies. Skipping this proxy.");
      BindingData::ProxyProcessingFinished* p = new BindingData::ProxyProcessingFinished;
      p->m_proxyID = CORBA::string_dup(proxy.id().c_str());
      addToWorkingMemory(newDataID(),p, cast::cdl::BLOCKING);
      return;
    }
    vector<string> best_unions = _bestUnions(proxyScores.m_unionScore, 
					     score,
					     thresholdScore,
					     bound_to,
					     _proxyID);

    
    log("     BEST for proxy: " + _proxyID);
    log("     BEST for this proxy: " + ttoString(best_unions) + " (" + scoreToString(score) + ")");
    log("LAST BEST for this proxy: " + ttoString(last_best_unions) + " (" + scoreToString(proxyScores.m_bestBindingScore) + ")");
    
    if(!last_best_unions.empty())
      assert(last_best_unions[0] != "");
    
    // if there is no difference to the bestunions already stored,
    // then there is no point in storing anything
    if(best_unions == last_best_unions) {

      assert(proxy->m_proxyState != BindingData::NEW);

      bool alreadyBound = false;

      if(proxy->m_proxyState == BindingData::BOUND) {
	log("state is bound already");
	alreadyBound = true;
      }
      
      if(proxy->m_proxyState == BindingData::REPROCESSED) {
	changeProxyState(proxy, BindingData::BOUND);
      }

      if(proxy->m_proxyState == BindingData::UPDATED) {
	changeProxyState(proxy, BindingData::BOUND);
      }


      log("they are equal, no update needed, maybe of the score, though");
      
      if(proxyScores.m_bestBindingScore == score) {
	log("nope... score is the same... even the version numbers..., the proxy is bound and all is fine");	
	BindingData::ProxyProcessingFinished* p = new BindingData::ProxyProcessingFinished;
	p->m_proxyID = CORBA::string_dup(proxy.id().c_str());
	addToWorkingMemory(newDataID(),p, cast::cdl::BLOCKING);
	return;
      }  else if (alreadyBound) {
	log("already bound but score has changed, signalling");
      	BindingData::ProxyProcessingFinished* p = new BindingData::ProxyProcessingFinished;
      	p->m_proxyID = CORBA::string_dup(proxy.id().c_str());
      	addToWorkingMemory(newDataID(),p, cast::cdl::BLOCKING);
      	return;
      }
	//score_not_equal = true;
      
#warning altered nick fix
      
    }
    // store locally
    proxyScores.m_bestUnions       = best_unions;
    proxyScores.m_bestBindingScore = score;
    
    if(best_unions.size() == 1 && 
       *best_unions.begin() == BindingData::NO_UNION &&
       !last_best_unions.empty()) {
      //      assert(proxyScores.m_unionScore.find(*last_best_unions.begin()) !=
      //	     proxyScores.m_unionScore.end()); // this may be a faulty assumption... hmmm
      log("!!!!!!!!!!!!!!!!!!! a situation occurred");
      log("Score for NIL         : " + scoreToString(score));
      log("Score for last binding: " + scoreToString(proxyScores.m_unionScore[*last_best_unions.begin()]));
    }    
    
    BindingData::BestUnionsForProxy* 
      new_best(new BindingData::BestUnionsForProxy());
    new_best->m_proxyID = CORBA::string_dup(_proxyID.c_str());
    new_best->m_proxyFeatureSignature = CORBA::string_dup(proxy->m_featureSignature);
    new_best->m_unionIDs.length(best_unions.size());
    new_best->m_proxyUpdatesWhenThisComputed = proxy->m_updates;
    
    vector<string>::const_iterator best_itr = best_unions.begin(); 
    for(unsigned int i = 0;
	i < new_best->m_unionIDs.length() ; 
	++i,++best_itr) {
      new_best->m_unionIDs[i] = CORBA::string_dup(best_itr->c_str());
    }
    new_best->m_score = score;
    
    const string bestID(proxy->m_bestUnionsForProxyID);
    // to pass the consistency check, the data must be loaded before
    // it's written. It's not strictly necessary for any other purpose
    // in this particular place.
    //    getWorkingMemoryEntry<BindingData::BestUnionsForProxy>(bestID); 

    overwriteWorkingMemory(bestID, 
			   new_best,
			   cast::cdl::BLOCKING);
    log(string("updated best union list: ") + bestID  + " for proxy "+ _proxyID);
    

    if(string(new_best->m_proxyFeatureSignature) != 
       string(proxyScores.m_bestBindingScore.m_proxyFeatureSignature) || 
       (
	best_unions != last_best_unions && 
	(last_best_unions.empty() || 
	 last_best_unions[0] != string(BindingData::NO_UNION)
	 )
	)
       ) {
      //      if(m_bindingTask) { // there is already a task, then just enque the proxy ID
      //	m_proxyIDQueue.push_back(_proxyID);
      //	string tmp;
      //	for(deque<string>::const_iterator itr = m_proxyIDQueue.begin(); itr != m_proxyIDQueue.end() ; ++itr) {
      //	  tmp += *itr + ", ";
      //	}
      //	log("proxy queue just grew: {" + tmp + "}");
      //      } else {	
      log("need a new binding task for " + _proxyID);
      BindingData::BindingTask* task = new BindingData::BindingTask();
      task->m_bestUnionsForProxyID = CORBA::string_dup(bestID.c_str());
      m_bindingTask = true;
      addToWorkingMemory(newDataID(), 
			 //BindingLocalOntology::BINDING_TASK_TYPE,
			 task//, cdl::BLOCKING
			 );
      //      }
    } 
    } else {
    log("not enough scores collected yet, do nothing...");
    BindingData::ProxyProcessingFinished* p = new BindingData::ProxyProcessingFinished;
    p->m_proxyID = CORBA::string_dup(proxy.id().c_str());
    addToWorkingMemory(newDataID(),p, cast::cdl::BLOCKING);
  }
}

void 
BindingJudge::bindingTaskDeleted(const cast::cdl::WorkingMemoryChange & _wmc)
{
  m_bindingTask = false;  
  while(!m_bindingTask && !m_proxyIDQueue.empty()) {
    const string id(m_proxyIDQueue.dequeue());
    _calculateBestUnionsAndStoreIfNecessary(id);
  }
  string tmp;
  for(deque<string>::const_iterator itr = m_proxyIDQueue.begin(); itr != m_proxyIDQueue.end() ; ++itr) {
    tmp += *itr + ", ";
  }
  log("proxy queue just shrinked: {" + tmp + "}");
}

} // namespace Binding 


