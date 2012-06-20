/**
 * @author Thomas Mörwald
 * @date June 2009
 *
 * Just a dummy component to drive some vision functionality.
 */

#ifndef VISION_DRIVER_H
#define VISION_DRIVER_H

#include <vector>
#include <string>
#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>


namespace cast
{

class VisionDriver : public ManagedComponent
{
private:
  /**
   * list of objects we want to have detected
   */
//  std::vector<std::string> labels;

//  PlyModel m_model;
  /**
   * callback function called whenever a new object appears ore an object
   * changes
   */
   bool tracking;		///< track objects with object tracker
   bool detecting;	///< detecting objects with object detector
   bool pushing;		///< pushing images from video server
	 bool running;
   
   void receiveVisualObject(const cdl::WorkingMemoryChange & _wmc);
   void receiveVisualObjectPoseChange(const cdl::WorkingMemoryChange & _wmc);

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
  virtual ~VisionDriver() {}
};

}

#endif

