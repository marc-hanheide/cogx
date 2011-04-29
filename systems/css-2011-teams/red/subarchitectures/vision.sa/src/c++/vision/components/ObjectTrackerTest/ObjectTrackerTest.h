
#ifndef OBJECT_TRACKER_TEST_H
#define OBJECT_TRACKER_TEST_H

#include <vector>
#include <string>
#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>

#include "ObjectTrackerUtils.hpp"
#include "Tracker.h"
#include "ModelLoader.h"
#include "Timer.h"

namespace cast
{

class ObjectTrackerTest : public ManagedComponent
{
private:

  std::string m_plyfile;
  std::string m_modelID;
  
  /**
   * list of objects we want to have detected
   */
  std::vector<std::string> labels;
  
  /**
   * callback function called whenever a new object appears ore an object
   * changes
   */
  void receiveVisualObject(const cdl::WorkingMemoryChange & _wmc);

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
  virtual ~ObjectTrackerTest() {}
};

}

#endif

