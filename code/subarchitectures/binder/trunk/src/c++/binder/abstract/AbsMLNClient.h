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

class AbsMLNClient :  public ManagedComponent
{
 private:
 
  /// List of MLN engine components we are providing evidence to
  vector<string> m_pEngIds;
  
  /// Name of the binder subarchitecture
  string m_bindingSA; 
  
  
 protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config)
  {
  //  BindingWorkingMemoryWriter::configure(_config);
	ManagedComponent::configure(_config);
	
	map<string,string>::const_iterator it;
	
	
	if ((it = _config.find("--sa")) != _config.end()) {
	  m_bindingSA = it->second;
	} else {
	 m_bindingSA="binder.sa";
	}
	
	if ((it = _config.find("--evd-ids")) != _config.end()) {
		stringstream ss(it->second);
		string token;
	
	  while(getline(ss, token, ',')) {
	      m_pEngIds.push_back(token);
	  }
	}
  }
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start() = 0;

  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent() = 0;
  
  string getBindingSA()
  {
	return m_bindingSA;
  }
	
  void distributeEvd(EvidencePtr evd, int initSteps = 400,
		int burnInSteps = 100, int prevSteps = 0)
  {
	evd->initInfSteps = initSteps;
	evd->burnInSteps = burnInSteps;
	evd->prevInfSteps = prevSteps;
  	
  	vector<string>::iterator id;
  	for(id=m_pEngIds.begin(); id!=m_pEngIds.end(); id++)
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
  	for(id=m_pEngIds.begin(); id!=m_pEngIds.end(); id++)
  	{
	  q->engId = *id;
	  addToWorkingMemory(newDataID(), getBindingSA(), q);
	  log("Provided new query '%s' to MLN engine id %s", query.c_str(), q->engId.c_str());
	}
  }
  
 public:
  virtual ~AbsMLNClient() {}
};

}

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/
