/**
 * @author Alen Vrecko
 * @date March 2011
 *
 * Markov logic network engine.
 */

#ifndef MLN_ENGINE_H
#define MLN_ENGINE_H

#include <vector>
#include <string>
#include <queue>
#include <map>
#include <algorithm>

#include <boost/interprocess/sync/named_semaphore.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <cast/architecture/ManagedComponent.hpp>

#include <beliefs_cast.hpp>
#include <binder.hpp>
#include <onlineengine.h>

using namespace de::dfki::lt::tr::beliefs::slice;

namespace cast
{

class MLNEngine :  public ManagedComponent

{
 private:
  bool doDisplay;
  
  OnlineEngine* m_oe;
  string m_inferenceString;
  string m_id;
  string m_resultWMId;
  /**
   * Name of the binder subarchitecture
   */
  std::string m_bindingSA;

  enum EvidenceStatus {
   NEW,
   SAMPLED};
  /** 
   * VisualObject data, contains also data used to evaluate VisualObject persistency
   */	
  struct EvidenceData {
   cdl::WorkingMemoryAddress addr;
   EvidenceStatus status;
   std::string attrBeliefId;
   cdl::CASTTime addedTime;
   cdl::CASTTime deleteTime;
  };

  std::map<std::string, std::string>EvidenceMap;
  
  std::queue<org::cognitivesystems::binder::mln::EvidencePtr> m_evidenceQueue;

  boost::interprocess::named_semaphore* m_queuesNotEmpty;

  /**
   * callback function called whenever a Belief changes
   */
  void newBelief(const cdl::WorkingMemoryChange & _wmc);
  
  /**
   * callback function called whenever there is new evidence
   */
  void newEvidence(const cdl::WorkingMemoryChange & _wmc);
  
  /**
   * checks if the epistemic status is T
   */
  template <class T>		    
  bool epistemicStatus(epobject::EpistemicObject obj)
  {
	epstatus::EpistemicStatus *eps = &(obj.estatus);
	debug("The epistemic status class is %s", typeid(*eps).name());

	if(typeid(*eps) == typeid(T))
	  return true;
	else
	  return false;
  };
  
  void queueNewEvidence(org::cognitivesystems::binder::mln::EvidencePtr evd);
  

 protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start();
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent();

 public:
  virtual ~MLNEngine() {}
};

}

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/
