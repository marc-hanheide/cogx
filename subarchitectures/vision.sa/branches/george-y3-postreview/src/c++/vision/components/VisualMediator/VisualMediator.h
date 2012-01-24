/**
 * @author Alen Vrecko
 * @date July 2009
 *
 * A component the filters out persistent VisualObjects.
 */

#ifndef OBJECT_ANALYZER_H
#define OBJECT_ANALYZER_H

#define UPD_THR_DEFAULT 5

#include <vector>
#include <string>
#include <queue>
#include <map>
#include <algorithm>

#include <boost/interprocess/sync/named_semaphore.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <cast/architecture/ManagedComponent.hpp>
#include <../../VisionUtils.h>

#include <VisionData.hpp>
#include <BindingWorkingMemoryWriter.hpp>
#include <BindingWorkingMemoryReader.hpp>
#include <BinderEssentials.hpp>
#include <BeliefModels.hpp>
#include <DomainModel.hpp>

namespace cast
{

class VisualMediator :  
  public binder::BindingWorkingMemoryWriter,
  public binder::BindingWorkingMemoryReader

{
 private:

  /**
   * Time and update thresholds
   *(part of the ROI persistency criteria)
   */
  int updateThr;
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

  /**
   * status of VisualObject persistency
   */
  enum VisualObjectStatus {
   OBJECT,
   PROXY,
   DELETED };

  /** 
   * VisualObject data, contains also data used to evaluate VisualObject persistency
   */	
  struct VisualObjectData {
   cdl::WorkingMemoryAddress addr;
   VisualObjectStatus status;
   std::string proxyId;
   cdl::CASTTime addedTime;
   cdl::CASTTime lastUpdateTime;
   cdl::CASTTime deleteTime;
  };

  std::map<std::string, VisualObjectData>VisualObjectMap;
  
  std::map<std::string, cast::WorkingMemoryChangeReceiver*>TaskFilterMap;

  std::queue<std::string> proxyToAdd;
  std::queue<std::string> proxyToDelete;
  std::queue<std::string> proxyToUpdate;

  boost::interprocess::named_semaphore* queuesNotEmpty;

  /**
   * callback function called whenever a new VisualObject appears
   */
  void newVisualObject(const cdl::WorkingMemoryChange & _wmc);

  /**
   * callback function called whenever a VisualObject changes
   */
  void updatedVisualObject(const cdl::WorkingMemoryChange & _wmc);

  /**
   * callback function called whenever a VisualObject is deleted
   */
  void deletedVisualObject(const cdl::WorkingMemoryChange & _wmc);

  /**
   * callback function called whenever a Belief changes
   */
  void updatedBelief(const cdl::WorkingMemoryChange & _wmc);
  
    /**
   * callback function called whenever a learning task is overwritten
   */
  void updatedLearningTask(const cdl::WorkingMemoryChange & _wmc);

//  bool unionRef(beliefmodels::domainmodel::cogx::SuperFormulaPtr f, std::string &unionID);

  bool unionRef(beliefmodels::adl::FormulaPtr f, std::string &unionID);
  
  bool findAsserted(beliefmodels::adl::FormulaPtr fp, 
					std::vector<beliefmodels::domainmodel::cogx::Color> &colors,
					std::vector<beliefmodels::domainmodel::cogx::Shape> &shapes,
					std::vector<float> &colorDist, std::vector<float> &shapeDist);
					
  bool AttrAgent(beliefmodels::adl::AgentStatusPtr ags);
  
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
  virtual ~VisualMediator() {}
};

}

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/
