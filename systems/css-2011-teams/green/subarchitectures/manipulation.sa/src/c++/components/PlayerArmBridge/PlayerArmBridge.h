/**
 * @author Michael Zillich
 * @date April 2011
 *
 * Provides a bridge between Golem path planning and a player actarray interface.
 * This is needed to have Golem control an arm simulated in gazebo.
 */

#ifndef PLAYER_ARM_BRIDGE_H
#define PLAYER_ARM_BRIDGE_H

#include <cast/architecture/ManagedComponent.hpp>
#include <libplayerc++/playerc++.h>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

#include <manipulation.hpp>

namespace cogx
{

using namespace std;
using namespace cast;
using namespace manipulation::slice;

/**
 * Provides a bridge between Golem path planning and a player actarray interface.
 * This is needed to have Golem control an arm simulated in gazebo.
 */
class PlayerArmBridge : public ManagedComponent
{
private:
#ifdef FEAT_VISUALIZATION
  class PABDisplayClient : public cogx::display::CDisplayClient
  {
  private:
    PlayerArmBridge *m_comp;
  public:
    cogx::display::CFormValues m_frmSettings;

  public:
    PABDisplayClient() { m_comp = 0; }
    void setClientData(PlayerArmBridge *comp) { m_comp = comp; }
    /*void handleEvent(const Visualization::TEvent &event);
    std::string getControlState(const std::string& ctrlId);
    void handleForm(const std::string& id, const std::string& partId,
          const std::map<std::string, std::string>& fields);
    bool getFormData(const std::string& id, const std::string& partId,
          std::map<std::string, std::string>& fields);*/
  };
  PABDisplayClient display;
#endif

  static const double GRIPPER_OPEN_POS = 0.05;
  static const double GRIPPER_CLOSE_POS = 0.00;
  static const int NUM_JOINTS = 5;
  static const int LEFT_FINGER_JOINT = NUM_JOINTS;
  static const int RIGHT_FINGER_JOINT = NUM_JOINTS + 1;

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
  /** interface to the arm */
  PlayerCc::ActArrayProxy *arm;
  /** HACK: sending a close or open when the gripper is alread closed (open)
   * freezes, so remember gripper state and don't sent command if not needed */
  bool gripperClosed;
  bool gripperHolding;

  void receiveSendTrajectory(const cdl::WorkingMemoryChange &_wmc);
  void receiveOpenGripper(const cdl::WorkingMemoryChange &_wmc);
  void receiveCloseGripper(const cdl::WorkingMemoryChange &_wmc);
  /**
   * Sends given trajectory to the player actarray interface, taking into
   * account timestamps of the trajectory.
   * Blocks until trajectory finished.
   * @return true if last point of trajectory could actually be reached, false
   *         otherwise (e.g. if collision occured and the arm is stuck)
   */
  bool sendTrajectory(GenConfigspaceStateSeq &trajectory);
  void openGripper();
  /**
   * Blocks until the gripper is closed or holding an object, or until an
   * internal timeout is reached,
   * @return true if gripper could fully close (with some epsilon), false
   *         otherwise (i.e. if the fingers are blocked as the gripper is
   *         holding something)
   */
  bool closeGripper();
  /**
   * Returns whether two joint configurations are equal, up to some epsilon.
   */
  bool equals(GenConfigspaceCoord &p1, GenConfigspaceCoord &p2);
  /**
   * Blocks until the fingers are no longer moving, or until an
   * internal timeout is reached,
   */
  void waitGripperMoving();

protected:
  virtual void configure(const map<string, string> &_config);
  virtual void start();
  virtual void destroy();

public:
  PlayerArmBridge();
};

}

#endif
