/**
 * @author Alen Vrecko
 * @date September 2011
 *
 * Provides a bridge between Golem path planning and a player actarray interface.
 * This is needed to have Golem control an arm simulated in gazebo.
 */

#ifndef ARM_MANAGER_H
#define ARM_MANAGER_H

#define POINTING_OFFSET_HOR 0.1
#define POINTING_OFFSET_VER 0.1

#include <cast/architecture/ManagedComponent.hpp>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

#include <manipulation.hpp>
#include <manipulation_exe.hpp>

#include <VisionData.hpp>
#include <Pose3.h>

#include <queue>
#include <IceUtil/IceUtil.h>

namespace cogx
{

using namespace std;
using namespace cast;
using namespace manipulation::slice;
using namespace manipulation::execution::slice;

/**
 * Provides a bridge between Golem path planning and a player actarray interface.
 * This is needed to have Golem control an arm simulated in gazebo.
 */
class ArmManager : public ManagedComponent
{
private:
  bool m_halt_arm;
  bool m_repeat_arm_movement;
  bool m_pointing_now;
  double m_pointingOffsetVer;
  double m_pointingOffsetHor;
  double corrAngle;

  cdl::WorkingMemoryAddress m_pointedObjAddr;

  ManipulationTaskStatus m_lastStatus;

#if 0
  struct armAction {
  ManipulationTaskType type;
  cdl::WorkingMemoryAddress objAddr;
  };
#endif
  std::queue<cdl::WorkingMemoryAddress> m_actionQueue;
  IceUtil::Monitor<IceUtil::Mutex> m_queueMonitor;

#ifdef FEAT_VISUALIZATION
  class PABDisplayClient : public cogx::display::CDisplayClient
  {
  private:
    //ArmManager *m_comp;

  public:
    /*PABDisplayClient() { m_comp = 0; }
    void setClientData(ArmManager *comp) { m_comp = comp; }
    void handleEvent(const Visualization::TEvent &event);
    std::string getControlState(const std::string& ctrlId);
    void handleForm(const std::string& id, const std::string& partId,
      const std::map<std::string, std::string>& fields);
    bool getFormData(const std::string& id, const std::string& partId,
      std::map<std::string, std::string>& fields);*/
  };
  PABDisplayClient m_display;
  std::string guiid(const std::string &myid) { return myid + "_" + getComponentID(); }
  void sendPointingTarget(Math::Pose3 &pose, double colR, double colG,
      double colB);
#endif

  ManipulationTaskStatus pointAtObject(cdl::WorkingMemoryAddress addr);
  cogx::Math::Pose3 pointingPose(const cogx::Math::Pose3 objPose);

  bool addFarArmMovementCommand(cast::cdl::WorkingMemoryAddress wma); //, cogx::Math::Vector3 offset);
  ManipulationTaskStatus addMoveToHomeCommand();
  ManipulationTaskStatus addMoveArmToPose(cogx::Math::Pose3 pose);
  bool addOpenGripperCommand();
  bool addCloseGripperCommand();

  void receiveNewCommand(const cdl::WorkingMemoryChange &_wmc);
  void receiveDeletedObject(const cdl::WorkingMemoryChange &_wmc);
  void overwriteFarArmMovementCommand(const cdl::WorkingMemoryChange & _wmc);
  void overwriteMoveToHomeCommand(const cdl::WorkingMemoryChange & _wmc);
  void overwriteMoveToPose(const cdl::WorkingMemoryChange & _wmc);

protected:
  virtual void configure(const map<string, string> &_config);
  virtual void start();
  virtual void destroy();
  virtual void runComponent();

public:
  ArmManager();
};

}
#endif
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et :vim
