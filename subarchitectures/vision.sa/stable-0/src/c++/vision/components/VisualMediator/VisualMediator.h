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
#include <autogen/BinderEssentials.hpp>

namespace cast
{

class VisualMediator : public binder::BindingWorkingMemoryWriter
{
private:

  /**
  * Time and update thresholds
  *(part of the ROI persistency criteria)
  */
  int updateThr;
  bool doDisplay;
  
  /**
   * status of VisualObject persistency
   */
  enum VisualObjectStatus {
  	STABLE,
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
  
  std::queue<std::string> proxyToAdd;
  std::queue<std::string> proxyToDelete;

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

