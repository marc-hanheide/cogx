/**
 * @author Michael Zillich
 * @date April 2011
*/

#include <iostream>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "GazeboTurntable.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cogx::GazeboTurntable();
  }
}

namespace cogx
{

using namespace std;
using namespace cast;

GazeboTurntable::GazeboTurntable()
{
#ifdef FEAT_VISUALIZATION
  display.setClientData(this);
#endif
  robot = 0;
  sim = 0;
  rotSpeed = M_PI/6.;
  timeDeltaMs = 100;
  turning = false;
  playerHost = PlayerCc::PLAYER_HOSTNAME;
  playerPort = PlayerCc::PLAYER_PORTNUM;
}

void GazeboTurntable::configure(const map<string, string> &_config)
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

  if((it = _config.find("--label")) != _config.end())
  {
    objLabel = it->second;
  }
  printf("turning object '%s'\n", objLabel.c_str());
  log("turning object '%s'", objLabel.c_str());
}

void GazeboTurntable::start()
{
  robot = new PlayerCc::PlayerClient(playerHost, playerPort);
  sim = new PlayerCc::SimulationProxy(robot, 0);
  ostringstream s;
  s << "connected to player robot '" << robot << "'\n";
  log(s.str());

#ifdef FEAT_VISUALIZATION
// printf("Feat_Visualization is ON!!!\n");
  display.connectIceClient(*this);
  display.installEventReceiver();
  display.addCheckBox(getComponentID(), "toggle.turntable.turning", "&Turning");
#endif
}

void GazeboTurntable::destroy()
{
  delete sim;
  delete robot;
}

#ifdef FEAT_VISUALIZATION
void GazeboTurntable::GTDisplayClient::handleEvent(const Visualization::TEvent &event)
{
  //debug(event.data + " (received by VideoViewer)");
  if(event.type == Visualization::evCheckBoxChange)
  {
    if(event.sourceId == "toggle.turntable.turning")
    {
      bool newTurn = (event.data != "0");
      if(newTurn != m_comp->turning)
      {
        m_comp->turning = !m_comp->turning;
      }
    }
  }
}

string GazeboTurntable::GTDisplayClient::getControlState(const std::string& ctrlId)
{
  if(ctrlId == "toggle.turntable.turning")
  {
    return m_comp->turning ? "1" : "0";
  }
  return "";
}
#endif

void GazeboTurntable::runComponent()
{
  while(isRunning())
  {
    if(turning)
    {
      double x, y, z, roll, pitch, yaw, time;
      sim->GetPose3d((char*)objLabel.c_str(), x, y, z, roll, pitch, yaw, time);
      yaw += rotSpeed*(double)timeDeltaMs/1000.;
      while(yaw >= 2.*M_PI)
        yaw -= 2.*M_PI;
      while(yaw < 0)
        yaw += 2.*M_PI;
      sim->SetPose3d((char*)objLabel.c_str(), x, y, z, roll, pitch, yaw);
    }
		sleepComponent(timeDeltaMs);
  }
}

}
