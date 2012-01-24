/**
 * @author Michael Zillich
 * @date April 2011
*/

#include <iostream>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <cogxmath.h>
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
using namespace cogx::Math;

static double normalizeAngle(double a)
{
  while(a >= 2.*M_PI)
    a -= 2.*M_PI;
  while(a < 0.)
    a += 2.*M_PI;
  return a;
}

GazeboTurntable::GazeboTurntable()
{
#ifdef FEAT_VISUALIZATION
  display.setClientData(this);
#endif
  robot = 0;
  sim = 0;
  rotSpeed = M_PI/12.;
  timeDeltaMs = 100;
  turningR = false;
  turningP = false;
  turningY = false;
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
  display.connectIceClient(*this);
  display.installEventReceiver();
  display.addCheckBox(getComponentID(), "toggle.turntable.rolling", "rotate x");
  display.addCheckBox(getComponentID(), "toggle.turntable.pitching", "rotate y");
  display.addCheckBox(getComponentID(), "toggle.turntable.yawing", "rotate z");
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
  if(event.type == Visualization::evCheckBoxChange)
  {
    if(event.sourceId == "toggle.turntable.rolling")
    {
      bool newTurn = (event.data != "0");
      if(newTurn != m_comp->turningR)
      {
        m_comp->turningR = !m_comp->turningR;
      }
    }
    else if(event.sourceId == "toggle.turntable.pitching")
    {
      bool newTurn = (event.data != "0");
      if(newTurn != m_comp->turningP)
      {
        m_comp->turningP = !m_comp->turningP;
      }
    }
    else if(event.sourceId == "toggle.turntable.yawing")
    {
      bool newTurn = (event.data != "0");
      if(newTurn != m_comp->turningY)
      {
        m_comp->turningY = !m_comp->turningY;
      }
    }
  }
}

string GazeboTurntable::GTDisplayClient::getControlState(const std::string& ctrlId)
{
  if(ctrlId == "toggle.turntable.rolling")
  {
    return m_comp->turningR ? "1" : "0";
  }
  else if(ctrlId == "toggle.turntable.pitching")
  {
    return m_comp->turningP ? "1" : "0";
  }
  else if(ctrlId == "toggle.turntable.yawing")
  {
    return m_comp->turningY ? "1" : "0";
  }
  return "";
}
#endif

void GazeboTurntable::runComponent()
{
  while(isRunning())
  {
    if(turningR || turningP || turningY)
    {
      double x, y, z, roll, pitch, yaw, time;
      Pose3 pose;

      sim->GetPose3d((char*)objLabel.c_str(), x, y, z, roll, pitch, yaw, time);
      pose.pos = vector3(x, y, z);
      fromRPY(pose.rot, roll, pitch, yaw);
      if(turningY)
      {
        Matrix33 R;
        fromRotZ(R, rotSpeed*(double)timeDeltaMs/1000.);
        mult(R, pose.rot, pose.rot);
      }
      if(turningP)
      {
        Matrix33 R;
        fromRotY(R, rotSpeed*(double)timeDeltaMs/1000.);
        mult(R, pose.rot, pose.rot);
      }
      if(turningR)
      {
        Matrix33 R;
        fromRotX(R, rotSpeed*(double)timeDeltaMs/1000.);
        mult(R, pose.rot, pose.rot);
      }
      toRPY(pose.rot, roll, pitch, yaw);
      sim->SetPose3d((char*)objLabel.c_str(), x, y, z, roll, pitch, yaw);
    }
		sleepComponent(timeDeltaMs);
  }
}

}
