/**
 * @author Alen Vrecko
 * @date September 2010
 *
 * A learner for reference resolution.
 */

#ifndef RR_LEARNER_H
#define RR_LEARNER_H

#include <vector>
#include <string>
#include <queue>
#include <map>
#include <algorithm>

#include <boost/interprocess/sync/named_semaphore.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <cast/architecture/ManagedComponent.hpp>
#include <../../VisionUtils.h>

#include <beliefs_cast.hpp>
#include <beliefs_cogx.hpp>

using namespace de::dfki::lt::tr::beliefs::slice;

namespace cast
{

class RRLearner :  public ManagedComponent

{
 private:
  bool doDisplay;
  
  /**
   * Name of the binder subarchitecture
   */
  std::string m_bindingSA;
  
  /**
   * Name of the binder subarchitecture
   */
  std::string m_salientObjID;
  bool first; 

  enum learnTaskStatus {
   SAMPLE,
   LEARNED};
  /** 
   * VisualObject data, contains also data used to evaluate VisualObject persistency
   */	
  struct learnTaskData {
   cdl::WorkingMemoryAddress addr;
   learnTaskStatus status;
   std::string attrBeliefId;
   cdl::CASTTime addedTime;
   cdl::CASTTime deleteTime;
  };

  std::map<std::string, std::string>learnTaskMap;
  
  std::queue<std::string> learnTaskQueue;

  boost::interprocess::named_semaphore* queuesNotEmpty;

  /**
   * callback function called whenever a Belief changes
   */
  void newBelief(const cdl::WorkingMemoryChange & _wmc);
  
  /**
   * find a labeled reference CAST WM address
   */
  bool pointedCASTAddr(std::string key, std::map<string,ProbDistribution> distibutions, cast::cdl::WorkingMemoryAddress &refWmaAddr);
  
  bool pointedCASTAddr(std::string key, distribs::ProbDistribution distribution, cast::cdl::WorkingMemoryAddress &refWmaAddr);
  
  bool pointedCASTAddr(std::string key, dBelief sitbeliefs::belief, cast::cdl::WorkingMemoryAddress &refWmaAddr);
  
  /**
   * checks if the epistemic status is T
   */
  template <class T>		    
  bool epistemicStatus(epobject::EpistemicObject obj)
  {
	EpistemicStatus *eps = &(obj.epistemicStatus);
	debug("The epistemic status class is %s", typeid(*eps).name());

	if(typeid(*eps) == typeid(T))
	  return true;
	else
	  return false;
  };
  
  /**
   * 
   */
  bool findAsserted(beliefmodels::adl::FormulaPtr fp, 
					std::vector<beliefmodels::domainmodel::cogx::Color> &colors,
					std::vector<beliefmodels::domainmodel::cogx::Shape> &shapes,
					std::vector<float> &colorDist, std::vector<float> &shapeDist);


  
  void addFeatureListToProxy(binder::autogen::core::ProxyPtr proxy, VisionData::IntSeq labels,
							 VisionData::DoubleSeq distribution);
							 
  void compileAndSendLearnTask(std::string visualObjID,  const std::string beliefID,
				std::vector<beliefmodels::domainmodel::cogx::Color> &colors,
				std::vector<beliefmodels::domainmodel::cogx::Shape> &shapes,
				std::vector<float> &colorDist, std::vector<float> &shapeDist);
				
  void removeLearnedAssertions(beliefmodels::adl::FormulaPtr fp, VisionData::VisualLearnerLearningTaskPtr task);
  
  void checkDistribution4Clarification(std::string proxyID, VisionData::IntSeq labels,
							 VisionData::DoubleSeq distribution);

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
  virtual ~RRLearner() {}
};

}

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/
