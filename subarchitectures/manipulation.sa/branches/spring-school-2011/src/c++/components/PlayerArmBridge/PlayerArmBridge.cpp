/**
 * @author Michael Zillich
 * @date April 2011
*/

#include <iostream>
#include "PlayerArmBridge.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cogx::PlayerArmBridge();
  }
}

namespace cogx
{

using namespace std;
using namespace cast;

PlayerArmBridge::PlayerArmBridge()
{
#ifdef FEAT_VISUALIZATION
  display.setClientData(this);
#endif
  robot = 0;
  arm = 0;
  playerHost = PlayerCc::PLAYER_HOSTNAME;
  playerPort = PlayerCc::PLAYER_PORTNUM;
}

void PlayerArmBridge::configure(const map<string, string> &_config)
{
  map<string,string>::const_iterator it;

  if((it = _config.find("--playerhost")) != _config.end())
  {
    playerHost = it->second;
  }
  if((it = _config.find("--playerport")) != _config.end())
  {
    istringstream str(it->second);
    str >> playerPort;
  }
}

void PlayerArmBridge::start()
{
  robot = new PlayerCc::PlayerClient(playerHost, playerPort);
  arm = new PlayerCc::ActArrayProxy(robot, 0);
  ostringstream s;
  s << "connected to player robot '" << robot << "'\n";
  log(s.str());

  /*addChangeFilter(createLocalTypeFilter<manipulation::somecommand>(cdl::ADD),
    new MemberFunctionChangeReceiver<PlayerArmBridge>(this, &PlayerArmBridge::newCommand));*/
}

void PlayerArmBridge::destroy()
{
  delete arm;
  delete robot;
}

void PlayerArmBridge::newCommand(const cdl::WorkingMemoryChange &_wmc)
{
}

void PlayerArmBridge::sendTrajectory(manipulation::slice::GenConfigspaceStateSeq &trajectory)
{
  for(size_t i = 1; i < trajectory.size(); i++)
  {
    for(int j = 0; j < NUM_JOINTS; j++)
      arm->MoveTo(j, trajectory[i].coord.pos[j]);
    long t0 = (long)(1e6*trajectory[i-1].t);
    long t1 = (long)(1e6*trajectory[i].t);
    usleep(t1 - t0);
  }
}

void PlayerArmBridge::openGripper()
{
  arm->MoveTo(LEFT_FINGER_JOINT, GRIPPER_OPEN_POS);
  arm->MoveTo(RIGHT_FINGER_JOINT, GRIPPER_OPEN_POS);
}

void PlayerArmBridge::closeGripper()
{
  arm->MoveTo(LEFT_FINGER_JOINT, GRIPPER_CLOSE_POS);
  arm->MoveTo(RIGHT_FINGER_JOINT, GRIPPER_CLOSE_POS);
}

}
