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
#include "cogxmath.h"
#include <VisionData.hpp>

namespace cast
{

using namespace cogx::Math;

class GazeboVision : public ManagedComponent
{
private:
  /**
   * Name of robot model in gazebo world.
   */
  static const std::string robotName;
  /**
   * host on which player server runs (typically localhost)
   */
  std::string playerHost;
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
   * sizes of the objects, assuming they are box-shaped 
   */
  std::vector<Vector3> sizes;
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

