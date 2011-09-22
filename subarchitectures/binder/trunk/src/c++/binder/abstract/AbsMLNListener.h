/**
 * @author Alen Vrecko
 * @date May 2011
 *
 * Markov logic network engine listener.
 */

#ifndef ABS_MLN_LISTENER_H
#define ABS_MLN_LISTENER_H

#include <AbsMLNClient.h>


namespace cast
{

using namespace org::cognitivesystems::binder::mln;

class AbsMLNListener :  public AbsMLNClient
{
 private:
 
  /// List of MLN engines components we are listening to
  std::vector<std::string> m_lEngIds;
  
  /// Flag that signals there is a change in the inference result 
  bool m_changedInf;
  
  InferredResultPtr m_lastResult;
  
  /**
   * callback function called whenever there is a new InferredResult
   */
  void newInferredResult(const cdl::WorkingMemoryChange & _wmc)
  {
	debug("An overwriten InferredResult WM entry ID %s ", _wmc.address.id.c_str());
	m_changedInf = true;
	
	InferredResultPtr inf; 
	
	try {
	  inf = getMemoryEntry<InferredResult>(_wmc.address);
	}
	catch (DoesNotExistOnWMException e) {
	  log("WARNING: the entry InferredResult ID %s not in WM.", _wmc.address.id.c_str());
	  return;
	}
  		
	debug("Got an inference update for MLN engine ID %s", inf->engId.c_str());
  
   if( containsElement<string>(m_lEngIds, inf->engId)) {
	qInfPush(inf);
  }
  else {
	debug("Not listening to to MLN engine ID %s", inf->engId.c_str());
	return;
  }
  m_changedInf = true; 
}
 protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config)
  {
  //  BindingWorkingMemoryWriter::configure(_config);
	AbsMLNClient::configure(_config);
	
	map<string,string>::const_iterator it;
	
	if ((it = _config.find("--inf-ids")) != _config.end()) {
		stringstream ss(it->second);
		string token;
	
	  while(getline(ss, token, ',')) {
	      m_lEngIds.push_back(token);
	  }
	}
  }
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start()
  {
   // filters for belief updates
	addChangeFilter(createGlobalTypeFilter<InferredResult>(cdl::OVERWRITE),
		new MemberFunctionChangeReceiver<AbsMLNListener>(this,
		  &AbsMLNListener::newInferredResult));
			
	log("AbsMLNListener initialized");
  }
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent() = 0;
  
  void qInfPush(org::cognitivesystems::binder::mln::InferredResultPtr res)
  {
	m_infQueue.push(res);
  }
  
  void qInfPop()
  {
	m_infQueue.pop();
  }
  
  org::cognitivesystems::binder::mln::InferredResultPtr qInfFront()
  {
	return m_infQueue.front();
  }

  bool qInfEmpty()
  {
	return m_infQueue.empty();
  }
  
  string getBindingSA()
  {
	return m_bindingSA;
  }
  
  void setQuery(string eid, string qstr)
  {
	QueryPtr q = new Query();
	q->engId = eid;
	q->atoms.push_back(qstr);
	
    addToWorkingMemory(newDataID(), m_bindingSA, q);
  }
  
  void setQuery(vector<string> eidl, string qstr)
  {
	vector<string>::iterator it;
	
	for(it = eidl.begin(); it < eidl.end(); it++)
	  setQuery(*it, qstr);
  }
  
 public:
  virtual ~AbsMLNListener() {}
};

}

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/
