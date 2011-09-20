/**
 * @author Alen Vrecko
 * @date May 2011
 *
 * Markov logic network engine listener.
 */

#ifndef MLN_EVD_PROVIDER_H
#define MLN_EVD_PROVIDER_H

#include <vector>
#include <string>
#include <queue>
#include <map>
#include <algorithm>

#include <boost/interprocess/sync/named_semaphore.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <MLNUtils.h>

#include <cast/architecture/ManagedComponent.hpp>

#include <binder.hpp>
#include <beliefs_cogx.hpp>

namespace cast
{

using namespace org::cognitivesystems::binder::mln;
using namespace eu::cogx::mln::slice;
using namespace std;

class MLNEvdProvider :  public ManagedComponent
{
 private:
 
  /// List of MLN engine components we are providing evidence to
  vector<string> m_engIds;
  
  /// Name of the binder subarchitecture
  string m_bindingSA; 
  
  /// 
  bool m_changedEvd;
  
  vector<MLNFact> m_rawFacts;
//  map<string,MLNFact> m_filtFacts;
//  map<string,MLNFact> m_oldFacts;
  
  string m_beliefType;
  string m_epiStatus;
  set<string> m_relevantKeys;
  string m_instKey;
  size_t m_maxinst;
  
  /**
   * callback function called whenever there is a new InferredResult
   */
  void newEvdSt(const cdl::WorkingMemoryChange & _wmc)
  {
	log("A new MLNState WM entry ID %s ", _wmc.address.id.c_str());
	
	MLNStatePtr st; 
	
	try {
	  st = getMemoryEntry<MLNState>(_wmc.address);
	}
	catch (DoesNotExistOnWMException e) {
	  log("WARNING: the entry MLNState ID %s not in WM.", _wmc.address.id.c_str());
	  return;
	}
  
	m_rawFacts = st->facts;
	m_changedEvd = true;
  }
  
 protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config)
  {
  //  BindingWorkingMemoryWriter::configure(_config);
	ManagedComponent::configure(_config);
	
	map<string,string>::const_iterator it;
	
	
	if ((it = _config.find("--bsa")) != _config.end()) {
	  m_bindingSA = it->second;
	} else {
	 m_bindingSA="binder.sa";
	}
	
	if ((it = _config.find("--eids")) != _config.end()) {
		stringstream ss(it->second);
		string token;
	
	  while(getline(ss, token, ',')) {
	      m_engIds.push_back(token);
	  }
	}
	
	if ((it = _config.find("--keys")) != _config.end()) {
		stringstream ss(it->second);
		string token;
	
	  while(getline(ss, token, ',')) {
	      m_relevantKeys.insert(token);
	  }
	}
	
	if ((it = _config.find("--type")) != _config.end()) {
	  m_beliefType = it->second;
	} else {
	 m_beliefType="";
	}
	
	if ((it = _config.find("--inst")) != _config.end()) {
	  m_instKey = it->second;
	} else {
	  m_instKey="percept";
	}
	
	if ((it = _config.find("--estatus")) != _config.end()) {
	  m_epiStatus = it->second;
	} else {
	 m_epiStatus="private";
	}
  }
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start()
  {
	m_changedEvd = true;
     // filters for belief updates
	addChangeFilter(createGlobalTypeFilter<MLNState>(cdl::OVERWRITE),
		new MemberFunctionChangeReceiver<MLNEvdProvider>(this,
		  &MLNEvdProvider::newEvdSt));
		  
	addChangeFilter(createGlobalTypeFilter<MLNState>(cdl::ADD),
		new MemberFunctionChangeReceiver<MLNEvdProvider>(this,
		  &MLNEvdProvider::newEvdSt));
			
	log("An instance of MLNEvdProvider initialized");
  }
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent() = 0;
  
  string getBindingSA()
  {
	return m_bindingSA;
  }
  
  map<string, MLNFact> filterFacts(vector<MLNFact> rawEvd)
  {
	map<string, MLNFact> filtEvd;
	
	vector<MLNFact>::iterator it;
	for (it=rawEvd.begin() ; it != rawEvd.end(); it++ ) {
	  if(it->type == m_beliefType && it->estatus == m_epiStatus
		  && (m_relevantKeys.count(it->key) || m_instKey == it->key)) {
		filtEvd.insert(pair<string,MLNFact>(it->atom,*it));
	  }
	}
	return filtEvd;
  }
  
  vector<MLNFact> getRawFacts() {
	return m_rawFacts;
  }
  
  bool pendingEvdChanges() {
	return m_changedEvd;
  } 
  
  void resetEvdChanges() {
	m_changedEvd=false;
  }
  
  bool getEvdChanges(map<string, MLNFact> facts, map<string, MLNFact> oldFacts, EvidencePtr evd)
  {
//	EvidencePtr evd = new Evidence();
	bool evdChanged = false;
	
	map<string,MLNFact>::iterator it;
	/// Remove obsolete evidence
	for(it=oldFacts.begin(); it != oldFacts.end(); it++)
	{
	  if(facts.find(it->first) == facts.end()) {
		evdChanged = true;
		if(it->second.prob == 1 || it->second.prob == 0) {
		  if(it->second.key == m_instKey) {
			Instance inst;
			inst.name=it->second.id;
			inst.type="perc";
			evd->removeInstances.push_back(inst);
		  } else  
			evd->noEvidence.push_back(it->first);
		}  
		else {
		  evd->resetPriors.push_back(it->first);
		}
	  }
	}
	  
	///  Add new evidence
	for(it=facts.begin(); it != facts.end(); it++)
	{
	  string atom = it->first;
	  map<string,MLNFact>::iterator old = oldFacts.find(atom);
	  double prob = it->second.prob;
	  double oldProb = old->second.prob;  
	  
	  if(old == oldFacts.end()) {
		evdChanged = true;
		if(prob >= 1)
		  if(it->second.key == m_instKey) {
			Instance inst;
			inst.name=it->second.id;
			inst.type="perc";
			evd->newInstances.push_back(inst);
		  } else
			evd->trueEvidence.push_back(atom);
		else if (it->second.prob <= 0)
		  evd->falseEvidence.push_back(atom);
		else {
		  evd->extPriors.push_back(atom);
		  evd->priorWts.push_back(prob);
		}
	  }
	  else
	  ///  Modify existing evidence
	  {
	    if(prob >= 1 && oldProb < 1) {
		  evdChanged = true;
		  evd->trueEvidence.push_back(atom);
		  if(oldProb > 0)
			evd->resetPriors.push_back(atom);
			
		} else if(prob <= 0 && oldProb > 0) {
		  evdChanged = true;
		  evd->falseEvidence.push_back(atom);	  
		  if(oldProb < 1)
			evd->resetPriors.push_back(atom);
			
		} else if(prob > 0 && prob < 1 && (oldProb == 0 || oldProb >= 1) ) {
		  evdChanged = true;
		  evd->extPriors.push_back(atom);
		  evd->priorWts.push_back(prob);
		  evd->noEvidence.push_back(atom);
		}
	  }	
	}
	return evdChanged;
  }
	
  void distributeEvd(EvidencePtr evd, int initSteps = 400,
		int burnInSteps = 100, int prevSteps = 0)
  {
	evd->initInfSteps = initSteps;
	evd->burnInSteps = burnInSteps;
	evd->prevInfSteps = prevSteps;
  	
  	vector<string>::iterator id;
  	for(id=m_engIds.begin(); id!=m_engIds.end(); id++)
  	{
	  evd->engId = *id;
	  addToWorkingMemory(newDataID(), getBindingSA(), evd);
	  log("Provided new evidence to MLN engine id %s", evd->engId.c_str());
	}
  }
  
  void distributeQuery(string query)
  {
	QueryPtr q = new Query();
	q->atoms.push_back(query);
  	
  	vector<string>::iterator id;
  	for(id=m_engIds.begin(); id!=m_engIds.end(); id++)
  	{
	  q->engId = *id;
	  addToWorkingMemory(newDataID(), getBindingSA(), q);
	  log("Provided new query '%s' to MLN engine id %s", query.c_str(), q->engId.c_str());
	}
  }
  
  
  string adaptAtom(string atom, string grd)
  {
	return atom.replace(atom.find("__"), 2, grd);
  } 
  
 public:
  virtual ~MLNEvdProvider() {}
};

}

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/
