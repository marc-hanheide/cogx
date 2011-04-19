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

  void newCommand(const cdl::WorkingMemoryChange &_wmc);
  void sendTrajectory(manipulation::slice::GenConfigspaceStateSeq &trajectory);
  void openGripper();
  void closeGripper();

protected:
  virtual void configure(const map<string, string> &_config);
  virtual void start();
  virtual void destroy();

public:
  PlayerArmBridge();
};

}

#endif
