/**
 * @author Alen Vrecko
 * @date May 2011
 *
 * Markov logic network engine listener.
 */

#ifndef ABS_MLN_CLIENT_H
#define ABS_MLN_CLIENT_H

#include <vector>
#include <string>
#include <map>
#include <algorithm>

#include <cast/architecture/ManagedComponent.hpp>

#include <binder.hpp>
#include <beliefs_cogx.hpp>

namespace cast
{

using namespace org::cognitivesystems::binder::mln;
using namespace eu::cogx::mln::slice;
using namespace std;
using namespace cdl;

class AbsMLNClient :  public ManagedComponent
{
 private:
 
  /// List of MLN engine components we are providing evidence to
  vector<string> m_pEvdIds;
  
  /// MLN engines we are listening to
//  vector<string> m_lInfIds;
  string m_lInfId;
  
  /// Name of the binder subarchitecture
  string m_bindingSA;
  
  /// True if a new result has been overwritten
  bool m_infResultReady;
  
  /// WM address of the InferredResult ice structure
  WorkingMemoryAddress m_infResultAddr;  
//  vector<bool> m_infResultReady;
//  vector<string> m_infResultAddr;

  
  
 protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config)
  {
  //  BindingWorkingMemoryWriter::configure(_config);
	ManagedComponent::configure(_config);
	
	map<string,string>::const_iterator it;
	
	m_pEvdIds.clear();
//	m_lInfIds.clear();
//	m_infResultReady.clear();
//	m_infResultAddr.clear();
	
	if ((it = _config.find("--sa")) != _config.end()) {
	  m_bindingSA = it->second;
	} else {
	 m_bindingSA="binder.sa";
	}
	
	if ((it = _config.find("--inf-id")) != _config.end()) {
	  m_lInfId = it->second;
	} else {
	 m_lInfId = "";
	}
/*	
	if ((it = _config.find("--inf-ids")) != _config.end()) {
		stringstream ss(it->second);
		string token;
	
	  while(getline(ss, token, ',')) {
	      m_lInfIds.push_back(token);
	      m_infResultReady.push_back(false);
	      m_infResultAddr.push_back("");
	  }
	}
*/	
	if ((it = _config.find("--evd-ids")) != _config.end()) {
		stringstream ss(it->second);
		string token;
	
	  while(getline(ss, token, ',')) {
	      m_pEvdIds.push_back(token);
	  }
	}
  }
  
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start()
  {
	m_infResultReady=false;
	
	// filters inference updates
/*	if (!m_fInfId.empty())
	  addChangeFilter(createGlobalTypeFilter<InferredResult>(cdl::OVERWRITE),
		new MemberFunctionChangeReceiver<AbsMLNClient>(this,
		&AbsMLNClient::newInferredResult)); */
		
	if (!m_lInfId.empty())
	  addChangeFilter(createGlobalTypeFilter<InferredResult>(cdl::ADD),
		new MemberFunctionChangeReceiver<AbsMLNClient>(this,
		&AbsMLNClient::firstAddedInfResult));	  
  }
  
  void firstAddedInfResult(const cdl::WorkingMemoryChange & _wmc) {
	log("An added InferredResult WM entry ID %s ", _wmc.address.id.c_str());
	
	InferredResultPtr inf; 
	
	try {
	  inf = getMemoryEntry<InferredResult>(_wmc.address);
	}
	catch (DoesNotExistOnWMException e) {
	  log("WARNING: the entry InferredResult ID %s not in WM.", _wmc.address.id.c_str());
	  return;
	}
	
	if(inf->engId == m_lInfId) {
	  log("Setting address filter for MLN engine ID %s", inf->engId.c_str());
	  m_infResultAddr = _wmc.address;
	  debug("Setting overwrite address filter for inference listening to WMA SA %s ID %s ",
		m_infResultAddr.subarchitecture.c_str(), m_infResultAddr.id.c_str());
		
	  addChangeFilter(createAddressFilter(m_infResultAddr, cdl::OVERWRITE),
		new MemberFunctionChangeReceiver<AbsMLNClient>(this,
		&AbsMLNClient::newInfResult));
	} else {
	  debug("Not listening to to MLN engine ID %s", inf->engId.c_str());
	return;
	}
  }
	
  
  void newInfResult(const cdl::WorkingMemoryChange & _wmc)
  {
	debug("An overwriten InferredResult WM entry from MLN engine '%s'", m_lInfId.c_str());
	m_infResultReady = true;
  }

  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent() = 0;
  
  string getBindingSA()
  {
	return m_bindingSA;
  }
  
  bool newResultReady() {
	return m_infResultReady;
  }
  
  InferredResultPtr getInfResult() {
	InferredResultPtr inf;
	try {
	  inf = getMemoryEntry<InferredResult>(m_infResultAddr);	  
	}
	catch (DoesNotExistOnWMException e) {
	  log("WARNING: the entry InferredResult ID %s not in WM.", m_infResultAddr.id.c_str());
	  exit(-1);
	}
	m_infResultReady = false;
	return inf;
  }
  
  string getInfEngineID() {
	return m_lInfId;
  }
  
  void distributeEvd(EvidencePtr evd)
  {
	vector<string>::iterator id;
  	for(id=m_pEvdIds.begin(); id!=m_pEvdIds.end(); id++)
  	{
	  evd->engId = *id;
	  addToWorkingMemory(newDataID(), getBindingSA(), evd);
	  log("Provided new evidence to MLN engine id %s", evd->engId.c_str());
	}
  }
	
  void distributeEvd(EvidencePtr evd, string setToken, string resetToken= "", int initSteps = 400,
	int burnInSteps = 100, int prevSteps = 0)
  {
	evd->initInfSteps = initSteps;
	evd->burnInSteps = burnInSteps;
	evd->prevInfSteps = prevSteps;
  	evd->setToken = setToken;
  	evd->resetToken = resetToken;
  	
	distributeEvd(evd);
  }

  void distributeQuery(string query)
  {
	QueryPtr q = new Query();
	q->atoms.push_back(query);
	q->engId = m_lInfId;
	
	addToWorkingMemory(newDataID(), getBindingSA(), q);
	log("Provided new query '%s' to MLN engine id %s", query.c_str(), q->engId.c_str());
  }
/* 
  void distributeQuery(string query)
  {
	QueryPtr q = new Query();
	q->atoms.push_back(query);
  	
  	vector<string>::iterator id;
  	for(id=m_lInfIds.begin(); id!=m_lInfIds.end(); id++)
  	{
	  q->engId = *id;
	  addToWorkingMemory(newDataID(), getBindingSA(), q);
	  log("Provided new query '%s' to MLN engine id %s", query.c_str(), q->engId.c_str());
	}
  }
*/  
 public:
  virtual ~AbsMLNClient() {}
};

}

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/
