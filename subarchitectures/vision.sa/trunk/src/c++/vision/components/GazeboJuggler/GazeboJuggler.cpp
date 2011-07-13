/**
 * @author Marko Mahnič
 * @date July 2011
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
//#include <../../VisionUtils.h>
#include "GazeboJuggler.h"

extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cogx::GazeboJuggler();
  }
}

namespace cogx
{

#include "res/juggler_gen.inc"

using namespace std;
using namespace PlayerCc;
//using namespace VisionData;
using namespace cogx::Math;

//const string GazeboJuggler::robotName = "robot";

GazeboJuggler::GazeboJuggler()
{
  pRobot = 0;
  pSim = 0;
  m_playerHost = PlayerCc::PLAYER_HOSTNAME;
  m_playerPort = PlayerCc::PLAYER_PORTNUM;
}

void GazeboJuggler::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  if((it = _config.find("--player-host")) != _config.end())
  {
    m_playerHost = it->second;
  }
  if((it = _config.find("--player-port")) != _config.end())
  {
    istringstream str(it->second);
    str >> m_playerPort;
  }

  // XXX: TEST TEST TEST
  m_objects.push_back(GObject("manner1"));
  m_objects.push_back(GObject("manner2"));
  m_objects.push_back(GObject("not-there"));

  m_locations.push_back(vector3(0.233, 0.175, 0.6));
  m_locations.push_back(vector3(-0.233, -0.175, 0.6));
}

#ifdef FEAT_VISUALIZATION
#define DLG_JUGGLER "GazeboJuggler"
void GazeboJuggler::CDisplayClient::onDialogValueChanged(const std::string& dialogId,
    const std::string& name, const std::string& value)
{
  if (dialogId == DLG_JUGGLER) {
    if (name == "cbxPlace01" || name == "cbxPlace02") {
      pJuggler->println(" *** cbxPlace *** %s=%s", name.c_str(), value.c_str());
      int placeId;
      if (value == "<empty>")
        placeId = -1;
      else {
        string id = name.substr(8);
        if (id[0] == '0') id = id.substr(1);
        placeId = atoi(id.c_str()) - 1;
      }
      pJuggler->moveObject(value, placeId);
    }
  }
}

void GazeboJuggler::CDisplayClient::handleDialogCommand(const std::string& dialogId,
    const std::string& command, const std::string& params)
{
}
#endif

void GazeboJuggler::start()
{
  pRobot = new PlayerCc::PlayerClient(m_playerHost, m_playerPort);
  pSim = new PlayerCc::SimulationProxy(pRobot, 0);
  log("Connected to Player %x", pRobot);

#ifdef FEAT_VISUALIZATION
  m_display.connectIceClient(*this);
  m_display.setClientData(this);
  m_display.installEventReceiver();

  m_display.addDialog(DLG_JUGGLER, res_juggler_ui, res_juggler_js, "GazeboJuggler juggler");
#endif
}

const double OFF = 121231e99; 
void GazeboJuggler::prepareObjects()
{
  double time;

  // Discover and keep valid objects
  vector<GObject> objs = m_objects;
  vector<GObject> notthere;
  m_objects.clear();
  for(int i = 0; i < objs.size(); i++) {
    GObject& o = objs[i];
    o.loc.x = OFF;
    pSim->GetPose3d((char*)o.label.c_str(), o.loc.x, o.loc.y, o.loc.z, o.pose.x, o.pose.y, o.pose.z, time);
    if (o.loc.x == OFF) {
      println(" *** Object '%s' is not in the scene.", o.label.c_str());
      notthere.push_back(o);
      continue;
    }
    o.loc.x = 100;
    o.loc.y = 100;
    pSim->SetPose3d((char*)o.label.c_str(), o.loc.x, o.loc.y, o.loc.z, o.pose.x, o.pose.y, o.pose.z);
    m_objects.push_back(o);
  }
  println("%ld objects managed in the scene.", m_objects.size());

#ifdef FEAT_VISUALIZATION
  ostringstream ss;
  if (notthere.size()) {
    ostringstream ss;
    ss << "<h3>GazeboJuggler</h3>";
    ss << "Objects not found: ";
    for(int i = 0; i < notthere.size(); i++) {
      GObject& o = notthere[i];
      ss << "'" << o.label << "'";
      if (i < notthere.size() - 1)
        ss << ", ";
    }
    ss << "<br>";
    m_display.setHtml("LOG", "GazeboJuggler", ss.str());
  }

  ss.str("");
  ss.clear();
  ss << "juggler.emptyObject = '<empty>';";
  ss << "juggler.setPlaceCount(" << m_locations.size() << ");";
  if (m_objects.size()) {
    ss << "juggler.setObjectNames([";
    for(int i = 0; i < m_objects.size(); i++) {
      GObject& o = m_objects[i];
      ss << "'" << o.label << "'";
      if (i < m_objects.size() - 1)
        ss << ", ";
    }
    ss << "]);";
  }
  m_display.execInDialog(DLG_JUGGLER, ss.str());
#endif
}

void GazeboJuggler::moveObject(const std::string& label, int placeIndex)
{
  for(int i = 0; i < m_objects.size(); i++) {
    GObject& o = m_objects[i];
    if (o.label != label)
      continue;
    if (placeIndex < 0 || placeIndex >= m_locations.size()) {
      o.loc.x = 100;
      o.loc.y = 100;
    }
    else
      o.loc = m_locations[placeIndex];
    pSim->SetPose3d((char*)o.label.c_str(), o.loc.x, o.loc.y, o.loc.z, o.pose.x, o.pose.y, o.pose.z);
    break;
  }
}

void GazeboJuggler::destroy()
{
  if (pSim) delete pSim;
  pSim = 0;
  if (pRobot) delete pRobot;
  pRobot = 0;
}

void GazeboJuggler::runComponent()
{

  prepareObjects();

  while(isRunning())
  {
    sleepComponent(1000);
  }
}

}

