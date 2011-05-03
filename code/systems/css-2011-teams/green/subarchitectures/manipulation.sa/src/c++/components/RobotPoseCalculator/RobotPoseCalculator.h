/**
 * @author Michael Zillich
 * @date April 2011
 *
 * Provides a bridge between Golem path planning and a player actarray interface.
 * This is needed to have Golem control an arm simulated in gazebo.
 */

#ifndef ROBOT_POSE_CALCULATOR_H
#define ROBOT_POSE_CALCULATOR_H

#include <vector>

#include <cast/architecture/ManagedComponent.hpp>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

#include "cogxmath.h"
#include <manipulation.hpp>
#include <VisionData.hpp>
#include <NavData.hpp>

namespace cogx
{

using namespace cast;
using namespace manipulation::slice;

/**
 * Provides a bridge between Golem path planning and a player actarray interface.
 * This is needed to have Golem control an arm simulated in gazebo.
 */
class RobotPoseCalculator : public ManagedComponent
{
private:
// #ifdef FEAT_VISUALIZATION
//   class PABDisplayClient : public cogx::display::CDisplayClient
//   {
//   private:
//     PlayerArmBridge *m_comp;
//   public:
//     cogx::display::CFormValues m_frmSettings;

//   public:
//     PABDisplayClient() { m_comp = 0; }
//     void setClientData(PlayerArmBridge *comp) { m_comp = comp; }
//     /\*void handleEvent(const Visualization::TEvent &event);
//     std::string getControlState(const std::string& ctrlId);
//     void handleForm(const std::string& id, const std::string& partId,
//           const std::map<std::string, std::string>& fields);
//     bool getFormData(const std::string& id, const std::string& partId,
//           std::map<std::string, std::string>& fields);*\/
//   };
//   PABDisplayClient display;
// #endif
    
    double m_x;
    double m_y;
    double m_theta;

  void receiveVisualObject(const cdl::WorkingMemoryChange &_wmc);
  void receiveRobotPose(const cdl::WorkingMemoryChange &_wmc);
  /* void movementCommandChanged(const cdl::WorkingMemoryChange &_wmc); */

  Math::Vector3 getObjectDimensions(const VisionData::VisualObject& object);
  void checkGraspPosition(const Math::Vector3& pos, const Math::Vector3& dir, const Math::Vector3& grasp_dir, std::vector<ManipulationPosePtr>& results);
  Math::Vector3 toGlobal(const Math::Vector3& pos);
  void computePoseDistance(ManipulationPose& pose);

protected:
  virtual void configure(const map<string, string> &_config);
  virtual void start();
  virtual void destroy();

public:
  RobotPoseCalculator();
};

}

#endif
