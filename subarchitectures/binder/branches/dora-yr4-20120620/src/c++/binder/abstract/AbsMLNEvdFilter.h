/**
 * @author Alen Vrecko
 * @date September 2011
 *
 * A component that filters beliefs and packs the changes into evidence fot Markov logic network engine.
 */

#ifndef ABST_MLN_EVD_FILTER_H
#define ABST_MLN_EVD_FILTER_H

#define DEFAULT_EVD_LOW_THRESHOLD 0.15
#define DEFAULT_EVD_HIGH_THRESHOLD 0.85

#include <AbsMLNClient.h>
#include <math.h>

namespace cast
{

using namespace org::cognitivesystems::binder::mln;
using namespace eu::cogx::mln::slice;
using namespace std;

const string DEFAULT_INSTANCE_KEY = "belief";
class AbsMLNEvdFilter :  public AbsMLNClient
{
 private:
  
  /// Flag that signals there is a change in the fact collection 
  bool m_changedFacts;
  
  vector<MLNFact> m_rawFacts;
  
  string m_beliefType;
  set<string> m_epiStatuses;
  set<string> m_relevantKeys;
  set<string> m_instKeys;
  
  double m_0_thr, m_1_thr;
  
  /**
   * callback function called whenever there is a change in fact collection
   */
  void newFactSt(const cdl::WorkingMemoryChange & _wmc)
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
	m_changedFacts = true;
  }
  
 protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config)
  {
  //  BindingWorkingMemoryWriter::configure(_config);
	AbsMLNClient::configure(_config);
	
	m_epiStatuses.clear();
	m_relevantKeys.clear();
	m_instKeys.clear();
	
	map<string,string>::const_iterator it;
	
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
	  stringstream ss(it->second);
	  string token;
	
	  while(getline(ss, token, ',')) {
	      m_instKeys.insert(token);
	  }
	} else {
	  m_instKeys.insert(DEFAULT_INSTANCE_KEY);
	}
	
	if ((it = _config.find("--epstatus")) != _config.end()) {
	  stringstream ss(it->second);
	  string token;
	  
	  while(getline(ss, token, ',')) {
	      m_epiStatuses.insert(token);
	  }
	}
	
	if ((it = _config.find("--thr0")) != _config.end()) {
	  m_0_thr = atof(it->second.c_str());
	} else {
	 m_0_thr = DEFAULT_EVD_LOW_THRESHOLD;
	}
	
	if ((it = _config.find("--thr1")) != _config.end()) {
	   m_1_thr = atof(it->second.c_str());
	} else {
	 m_1_thr = DEFAULT_EVD_HIGH_THRESHOLD;
	}
  }
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start()
  {
	m_changedFacts = true;
     // filters for belief updates
	addChangeFilter(createGlobalTypeFilter<MLNState>(cdl::OVERWRITE),
		new MemberFunctionChangeReceiver<AbsMLNEvdFilter>(this,
		  &AbsMLNEvdFilter::newFactSt));
		  
	addChangeFilter(createGlobalTypeFilter<MLNState>(cdl::ADD),
		new MemberFunctionChangeReceiver<AbsMLNEvdFilter>(this,
		  &AbsMLNEvdFilter::newFactSt));
			
	log("An instance of MLNEvdFilter initialized");
  }
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent() = 0;
  
  map<string, MLNFact> filterFacts(vector<MLNFact> rawFact)
  {
	map<string, MLNFact> filtFact;
	
	vector<MLNFact>::iterator it;
	for (it=rawFact.begin() ; it != rawFact.end(); it++ ) {
	  if(it->type == m_beliefType
		&& (m_epiStatuses.empty() || m_epiStatuses.count(it->estatus))
		&& (m_relevantKeys.count(it->key) || m_instKeys.count(it->key))) {
		filtFact.insert(pair<string,MLNFact>(it->atom,*it));
	  }
	}
	return filtFact;
  }
  
  vector<MLNFact> getRawFacts() {
	return m_rawFacts;
  }
  
  bool pendingFactChanges() {
	return m_changedFacts;
  } 
  
  void resetFactChanges() {
	m_changedFacts=false;
  }
  
  bool getEvdChanges(map<string, MLNFact> facts, map<string, MLNFact> oldFacts, EvidencePtr evd)
  {
	bool evdChanged = false;
	
	map<string,MLNFact>::iterator it;
	/// Remove obsolete evidence
	for(it=oldFacts.begin(); it != oldFacts.end(); it++)
	{
	  if(facts.find(it->first) == facts.end()) {
		evdChanged = true;
		if(it->second.prob >= m_1_thr || it->second.prob <= m_0_thr) {
		  if(m_instKeys.count(it->second.key)) {
			Instance inst;
			inst.name=it->second.id;
			inst.type=it->second.key;
			evd->removeInstances.push_back(inst);
		  } else if(it->second.key == "epstatus") {
			evd->falseEvidence.push_back(it->first);
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
		if(prob >= m_1_thr)
		  if(m_instKeys.count(it->second.key)) {
			Instance inst;
			inst.name=it->second.id;
			inst.type=it->second.key;
			evd->newInstances.push_back(inst);
		  } else
			evd->trueEvidence.push_back(atom);
		else if (it->second.prob <= m_0_thr)
		  evd->falseEvidence.push_back(atom);
		else {
		  evd->extPriors.push_back(atom);
		  evd->priorWts.push_back(logit(prob));
		}
	  }
	  else
	  ///  Modify existing evidence
	  {
	    if(prob >= m_1_thr && oldProb < m_1_thr) {
		  evdChanged = true;
		  evd->trueEvidence.push_back(atom);
		  if(oldProb > m_0_thr)
			evd->resetPriors.push_back(atom);
			
		} else if(prob <= m_0_thr && oldProb > m_0_thr) {
		  evdChanged = true;
		  evd->falseEvidence.push_back(atom);	  
		  if(oldProb < m_1_thr)
			evd->resetPriors.push_back(atom);
			
		} else if(prob > m_0_thr && prob < m_1_thr && (oldProb == m_0_thr || oldProb >= m_1_thr) ) {
		  evdChanged = true;
		  evd->extPriors.push_back(atom);
		  evd->priorWts.push_back(logit(prob));
		  evd->noEvidence.push_back(atom);
		}
	  }	
	}
	return evdChanged;
  }
  
  double logit(double p) {
	return ::log(p/(1-p));
  }
  
  string adaptAtom(string atom, string grd)
  {
	return atom.replace(atom.find("__"), 2, grd);
  } 
  
 public:
  virtual ~AbsMLNEvdFilter() {}
};

}

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/
