/* Dummy driver for the object tracker component
*
*  @author: Thomas Mörwald
*  @date: April 2009
*
*  This component is an example on how to control the
*  object tracker by loading a model from ply-file
*  to the working memory and calling several tracking commands.
*
*/

#ifndef OBJECT_TRACKER_DRIVER_H
#define OBJECT_TRACKER_DRIVER_H

#include <vector>
#include <string>
#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>

#include "ObjectTrackerUtils.hpp"
#include "PlyModel.h"

namespace cast
{

class ObjectTrackerDriver : public ManagedComponent
{
private:
  /**
   * list of objects we want to have detected
   */
  std::vector<std::string> labels;

  PlyModel m_model;
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
  virtual ~ObjectTrackerDriver() {}
};

}

#endif

