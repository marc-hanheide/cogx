/**
 * @author Michael Zillich
 * @date April 2011
 *
 * Simulate vision using the player/gazebo simulation interface to directly
 * obtain object poses.
 */

#ifndef GAZEBO_VISION_H
#define GAZEBO_VISION_H

#include <vector>
#include <string>
#include <libplayerc++/playerc++.h>
#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>

namespace cast
{

class GazeboVision : public ManagedComponent
{
private:
  /**
   * host on which player server runs (typically localhost)
   */
  string playerHost;
  /**
   * player port number (typically 6665)
   */
  uint32_t playerPort;
  /** player client object */
  PlayerCc::PlayerClient *robot;
  /** interface to the simulation */
  PlayerCc::SimulationProxy *sim;
  /**
   * list of objects we want to have detected
   */
  std::vector<std::string> labels;
  /**
   * Working memory addresses of objects.
   */
  std::vector<std::string> objAddrs;

  bool newObject(std::string &label, cogx::Math::Pose3 &pose, VisionData::VisualObjectPtr &obj);

protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start();
  virtual void destroy();
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent();
 
public:
  GazeboVision();
};

}

#endif

