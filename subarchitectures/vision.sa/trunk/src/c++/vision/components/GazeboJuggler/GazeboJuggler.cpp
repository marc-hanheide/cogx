/**
 * @author Marko Mahnič
 * @date July 2011
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
//#include <../../VisionUtils.h>
#include "GazeboJuggler.h"
#include <fstream>

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
  // Path to a configuration file with objects. The file has the format:
  // [objects]
  // object1-name
  // object2-name
  // [places]
  // x y z
  // x y z
  //
  // Only the object names that are actually present in a Gazebo world file will be used.
  if((it = _config.find("--objects")) != _config.end())
  {
    m_objects.clear();
    m_locations.clear();
    string fname = it->second;
    //println(" GJ **** " + fname);
    ifstream f;
    f.open(fname.c_str());
    if (f.fail()) {
      error("File not found: '" + fname + "'");
    }
    else {
      int mode = 0; // 1 - objects, 2 - places
      while (f.good() && !f.eof()) {
        string line, tok;
        getline(f, line);
        //println(" GJ **** " + line);
        istringstream itok(line);

        int newmode = mode;
        itok >> tok;
        if (tok == "[objects]") newmode = 1;
        else if (tok == "[places]") newmode = 2;
        if (mode != newmode) {
          mode = newmode;
          continue;
        }

        if (mode == 1) {
          if (tok != "")
            // In Gazebo 0.9 top objects didn't have a prefix.
            // In Gazebo 0.10 the prefix is "noname::"
            m_objects.push_back(GObject(tok, "noname::" + tok));
        }
        else if (mode == 2) {
          Vector3 v;
          int n;
          n = sscanf(line.c_str(), "%lf %lf %lf", &v.x, &v.y, &v.z);
          if (n > 1) {
            if (n != 3) v.z = 1.0;
            else v.z += 0.5;
            m_locations.push_back(v);
          }
        }
      }
      f.close();
    }
  }

#if 0
  // XXX: TEST TEST TEST
  m_objects.push_back(GObject("manner1"));
  m_objects.push_back(GObject("manner2"));
  m_objects.push_back(GObject("not-there"));

  m_locations.push_back(vector3(0.233, 0.175, 0.6));
  m_locations.push_back(vector3(-0.233, -0.175, 0.6));
#endif
}

#ifdef FEAT_VISUALIZATION
#define DLG_JUGGLER "GazeboJuggler"
void GazeboJuggler::CDisplayClient::onDialogValueChanged(const std::string& dialogId,
    const std::string& name, const std::string& value)
{
  if (dialogId == DLG_JUGGLER) {
    if (name.substr(0, 8) == "cbxPlace") {
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
    pSim->GetPose3d((char*)o.gazeboName.c_str(), o.loc.x, o.loc.y, o.loc.z, o.pose.x, o.pose.y, o.pose.z, time);
    if (o.loc.x == OFF) {
      println(" *** Object '%s' is not in the scene.", o.label.c_str());
      notthere.push_back(o);
      continue;
    }
    o.loc.x = 100 + i * 10;
    o.loc.y = 100 + i * 10;
    pSim->SetPose3d((char*)o.gazeboName.c_str(), o.loc.x, o.loc.y, o.loc.z, o.pose.x, o.pose.y, o.pose.z);
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
    m_display.setHtml("INFO", "GazeboJuggler", ss.str());
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
      o.loc.x = 100 + i * 10;
      o.loc.y = 100 + i * 10;
    }
    else
      o.loc = m_locations[placeIndex];
    pSim->SetPose3d((char*)o.gazeboName.c_str(), o.loc.x, o.loc.y, o.loc.z, o.pose.x, o.pose.y, o.pose.z);
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

