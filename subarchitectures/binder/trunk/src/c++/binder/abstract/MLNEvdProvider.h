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
  vector<string> m_evdEngIds;
  
  /// Name of the binder subarchitecture
  string m_bindingSA; 
  
  /// Queue for WM entries containing serialized belief formulae
  queue<MLNStatePtr> m_evdStQueue;
  
  vector<string> m_evdAtoms;
  vector<double> m_probs;
  vector<string> m_oldEvdAtoms;
  vector<double> m_oldProbs;
  
  map<string,string> m_instances;
  size_t m_maxinst;
  
  /**
   * callback function called whenever there is a new InferredResult
   */
  void newEvdSt(const cdl::WorkingMemoryChange & _wmc)
  {
	debug("A new MLNState WM entry ID %s ", _wmc.address.id.c_str());
	
	MLNStatePtr st; 
	
	try {
	  st = getMemoryEntry<MLNState>(_wmc.address);
	}
	catch (DoesNotExistOnWMException e) {
	  log("WARNING: the entry MLNState ID %s not in WM.", _wmc.address.id.c_str());
	  return;
  }
  
  qEvdStPush(st);

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
	      m_evdEngIds.push_back(token);
	  }
	}
  }
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start()
  {
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
  
  void qEvdStPush(MLNStatePtr st)
  {
	m_evdStQueue.push(st);
  }
  
  void qEvdStPop()
  {
	m_evdStQueue.pop();
  }
  
  MLNStatePtr qInfFront()
  {
	return m_evdStQueue.front();
  }

  bool qEvdStEmpty()
  {
	return m_evdStQueue.empty();
  }
  
  string getBindingSA()
  {
	return m_bindingSA;
  }
  
  void provideEvidence(string eid, MLNStatePtr evdSt)
  {
	m_oldEvdAtoms = m_evdAtoms;
	m_oldProbs = m_probs;
	
	m_evdAtoms = evdSt->facts;
	m_probs = evdSt->probs;
	
	EvidencePtr evd = new Evidence();
	
	evd->engId = eid;
	evd->initInfSteps = 400;
	evd->prevInfSteps = 0;
  	evd->burnInSteps = 100;
	
	/// Remove obsolete evidence
	for(int i=0; i < m_oldEvdAtoms.size(); i++)
	{
	  if(!containsElement<string>(m_evdAtoms, m_oldEvdAtoms[i]) ) {
		if(m_oldProbs[i] == 1 || m_oldProbs[i] == 0) {
		  evd->noEvidence.push_back(m_oldEvdAtoms[i]);
		}  
		else {
		  evd->resetPriors.push_back(m_oldEvdAtoms[i]);
		}
	  }
	}
	  
	///  Add new evidence
	for(int i=0; i < m_evdAtoms.size(); i++)
	{
	  size_t oldidx;
	  if(!containsElement<string>(m_oldEvdAtoms, m_evdAtoms[i], oldidx) ) {
		if(m_probs[i] == 1)
		  evd->trueEvidence.push_back(m_evdAtoms[i]);
		else if (m_probs[i] == 0)
		  evd->falseEvidence.push_back(m_evdAtoms[i]);
		else {
		  evd->extPriors.push_back(m_evdAtoms[i]);
		  evd->priorWts.push_back(m_probs[i]);
		}
	  }
	  else
	  {
		assert(oldidx < m_oldEvdAtoms.size());

	    if(m_probs[i] == 1 && m_oldProbs[oldidx] != 1) {
		  evd->trueEvidence.push_back(m_evdAtoms[i]);
		  if(m_oldProbs[oldidx] != 0)
			evd->resetPriors.push_back(m_evdAtoms[i]);
			
		} else if(m_probs[i] == 0 && m_oldProbs[oldidx] != 0) {
		  evd->falseEvidence.push_back(m_evdAtoms[i]);
		  
		  if(m_oldProbs[oldidx] != 1)
			evd->resetPriors.push_back(m_evdAtoms[i]);
			
		} else if(m_probs[i] != 0 && m_probs[i] != 1 && (m_oldProbs[oldidx] == 0 || m_oldProbs[oldidx] == 1) ) {
		  evd->extPriors.push_back(m_evdAtoms[i]);
		  evd->priorWts.push_back(m_probs[i]);
		  evd->noEvidence.push_back(m_evdAtoms[i]);
		}
	  }
	
	 addToWorkingMemory(newDataID(), getBindingSA(), evd);	
	}
  }
  
  
  void provideEvidence(vector<string> eids, MLNStatePtr evdSt)
  {
	vector<string>::iterator it;
	for(it=eids.begin(); it < eids.end(); it++)
	  provideEvidence(*it, evdSt);
  }
  
  string adaptAtom(string fact, string grd)
  {
	return fact.replace(fact.find("__"), 2, grd);
  } 
  
 public:
  virtual ~MLNEvdProvider() {}
};

}

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/
