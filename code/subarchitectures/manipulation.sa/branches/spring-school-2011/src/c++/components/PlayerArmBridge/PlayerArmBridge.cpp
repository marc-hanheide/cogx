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
  if((it = _config.find("--playerhost")) != _config.end())
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

void PlayerArmBridge::moveTCP()
{
  for(size_t i = 1; i < trajectory.size(); i++)
  {
    std::vector<float> pos(CONFIG_SPACE_DIM);
    printf("%.3lf: ", trajectory[i].t);
    for(int j = 0; j < CONFIG_SPACE_DIM; j++)
    {
      printf("%.3lf ", trajectory[i].pos.c[j]);
      pos[j] = trajectory[i].pos.c[j];
    }
    printf("\n");
    for(int j = 0; j < 5; j++)
    {
      actarray.MoveTo(j, pos[j]);
    }
    usleep(100000);
  }
}

void PlayerArmBridge::getTCP()
{
}

void PlayerArmBridge::openGripper()
{
  arm->MoveTo(5, GRIPPER_OPEN_POS);
  arm->MoveTo(6, GRIPPER_OPEN_POS);
}

void PlayerArmBridge::closeGripper()
{
  arm->MoveTo(5, GRIPPER_CLOSE_POS);
  arm->MoveTo(6, GRIPPER_CLOSE_POS);
}

}
