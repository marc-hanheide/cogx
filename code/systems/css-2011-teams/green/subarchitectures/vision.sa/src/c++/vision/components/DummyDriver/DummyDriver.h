/**
 * @author Michael Zillich
 * @date February 2009
 *
 * Just a dummy component to drive some vision functionality.
 */

#ifndef DUMMY_DRIVER_H
#define DUMMY_DRIVER_H

#include <vector>
#include <string>
#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include <manipulation.hpp>
#include <SpatialData.hpp>
#include <PTZ.hpp>

namespace cast
{

class DummyDriver : public ManagedComponent
{
private:

  ptz::PTZCompletion m_ptz;
  VisionData::CommandCompletion m_viscomp;
  SpatialData::Completion m_nav;
  /**
   * list of objects we want to have detected
   */
  std::vector<std::string> labels;
  
  std::vector<manipulation::slice::ManipulationPosePtr> m_poses;
  manipulation::slice::ManipulationPosePtr m_best_pose;

  /**
   * callback function called whenever a new object appears ore an object
   * changes
   */
  void receiveVisualObject(const cdl::WorkingMemoryChange & _wmc);
  
  // PTZ functions	
  bool addPTZCommand(double pan, double tilt);
  
  void overwriteSetPTZPoseCommand(const cdl::WorkingMemoryChange & _wmc);
  
    // Look4Obj functions	
  bool addLook4ObjCommand(double pan, double tilt);
  
  void overwriteLook4ObjCommand(const cdl::WorkingMemoryChange & _wmc);
  
    // PTZ functions	
  bool addGraspCommand(std::string label);
  
  void overwriteGraspCommand(const cdl::WorkingMemoryChange & _wmc);
  
  void doExplore();
  bool addNavCommand(long place);
  bool addNavCommand(double x, double y, double angle);
  bool addNavCommand(cogx::Math::Vector3 pose);
  void overwriteNavCommand(const cdl::WorkingMemoryChange & _wmc);
  
  void getGraspPoses(std::vector<manipulation::slice::ManipulationPosePtr> & poses);
  std::vector<manipulation::slice::ManipulationPosePtr> purgePoses(std::string label, std::vector<manipulation::slice::ManipulationPosePtr> poses);
  manipulation::slice::ManipulationPosePtr bestPose(std::vector<manipulation::slice::ManipulationPosePtr> poses);
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
  virtual ~DummyDriver() {}
};

}

#endif

