/**
 * @author Marko Mahnič
 * @date July 2011
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
//#include <../../VisionUtils.h>
#include "GazeboJuggler.h"
#include <fstream>
#include <cstdlib>
#include <cstring>

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
  mGazeboVersion = 100;

  auto sp = strdup(getenv("GAZEBO_RESOURCE_PATH"));
  // Split into individual paths.
  std::vector<std::string> paths;
  auto pcp = strtok(sp, ":"); // strtok is destructive!
  while (pcp) {
    std::string p(pcp);
    if (p.length() > 1) {
      mGazeboResourcePath.push_back(p);
    }
    pcp = strtok(nullptr, ":");
  }
  delete sp;
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
  if((it = _config.find("--gazebo-version")) != _config.end())
  {
     istringstream iss(it->second);
     iss >> mGazeboVersion;
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
    //println(" GzJ **** " + fname);
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
        //println(" GzJ **** " + line);
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
            if (mGazeboVersion < 100) {
              // In Gazebo 0.9 top objects didn't have a prefix.
              // In Gazebo 0.10 the prefix is "noname::"
              m_objects.push_back(GObject(tok, "noname::" + tok));
            }
            else {
              // In Gazebo 1.0 the prefix was removed again
              m_objects.push_back(GObject(tok, tok));
            }
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
#define STR_EMPTY "<empty>"
void GazeboJuggler::CDisplayClient::onDialogValueChanged(const std::string& dialogId,
    const std::string& name, const std::string& value)
{
  if (dialogId == DLG_JUGGLER) {
    if (name.substr(0, 8) == "cbxPlace") {
      pJuggler->println(" *** cbxPlace *** %s=%s", name.c_str(), value.c_str());

      int placeId;
      string id = name.substr(8);
      if (id[0] == '0') id = id.substr(1);
      placeId = atoi(id.c_str()) - 1;

      int toPlaceId = placeId;
      if (value == STR_EMPTY)
        toPlaceId = -1;
      
#if 0 // It is impossible to retrieve information from Gazebo 1.0 (rev2707)
      if (placeId >= 0 && pJuggler->mGazeboVersion >= 100 ) {
        pJuggler->println(" ****** isObjectLoaded ***** ");
        if (!pJuggler->isObjectLoaded(value))
            pJuggler->tryLoadObject(value);
      }
#endif
      if (pJuggler->mGazeboVersion >= 100 && toPlaceId >= 0) {
        auto ito = pJuggler->mKnownObjects.find(value);
        if (ito == pJuggler->mKnownObjects.end()) {
          bool rv = pJuggler->tryLoadObject(value);
          pJuggler->mKnownObjects[value] = true;
        }
        //if (placeId >= 0) {
        //  std::string pc = pJuggler->mPlaceContent[placeId];
        //  if (pc != "" && pc != STR_EMPTY)
        //    pJuggler->tryRemoveObject(pc);
        //  if (value != pc)
        //    pJuggler->tryRemoveObject(value);
        //  pJuggler->tryLoadObject(value);

        //  if (value != STR_EMPTY) {
        //    for (auto p : pJuggler->mPlaceContent) {
        //      if (p.second == value) {
        //        p.second = STR_EMPTY;
        //      }
        //    }
        //  }
        //}
        //pJuggler->mPlaceContent[placeId] = value;
      }
      pJuggler->moveObject(value, toPlaceId);
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

void GazeboJuggler::prepareObjects()
{
  double time;
  println(" *** Preparing objects");

  // Discover and keep valid objects
  vector<GObject> objs = m_objects;
  vector<GObject> notthere;
  m_objects.clear();
  for(int i = 0; i < objs.size(); i++) {
    GObject& o = objs[i];
    if (mGazeboVersion < 100 && !isObjectLoaded(o.label)) {
      println(" *** Object '%s' is not in the scene.", o.label.c_str());
      notthere.push_back(o);
      continue;
    }
    o.loc.x = 100 + i * 10;
    o.loc.y = 100 + i * 10;
    pSim->SetPose3d((char*)o.gazeboName.c_str(), o.loc.x, o.loc.y, o.loc.z, o.pose.x, o.pose.y, o.pose.z);
    m_objects.push_back(o);
  }

  // In Gazebo 1.0 we can load objects later. The various GetXXX functions
  // don't work in rev 2707.
  if (mGazeboVersion >= 100) {
    for (auto o : notthere) {
      m_objects.push_back(o);
    }
    notthere.clear();
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
  ss << "juggler.emptyObject = '" << STR_EMPTY << "';";
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

bool GazeboJuggler::isObjectLoaded(const std::string& label)
{
  if (mGazeboVersion < 100) {
    const double OFF = 121231e99; 
    double time;
    GObject o("", "");
    o.loc.x = OFF;
    pSim->GetPose3d((char*)label.c_str(), o.loc.x, o.loc.y, o.loc.z, o.pose.x, o.pose.y, o.pose.z, time);

    return o.loc.x != OFF;
  }

  return 0;
#if 0 // It is impossible to retrieve information from Gazebo 1.0 (rev2707)
  long exists = 0;
  pSim->GetProperty((char*)label.c_str(), "exists", &exists, sizeof(exists));
  log(" isObjectLoaded %s -> %ld ", label.c_str(), exists);

  return exists != 0;
#endif
}

bool GazeboJuggler::tryRemoveObject(const std::string& label)
{
  std::ostringstream cmd;
  cmd << "gzfactory delete -m " << label;
  log("Executing w. system: %s", cmd.str().c_str());
  int rv = system(cmd.str().c_str());

  return rv == 0;
}

bool GazeboJuggler::tryLoadObject(const std::string& label)
{
#if 0
  log ("***** 0 *****");
  if (isObjectLoaded(label))
    return true;
#endif

  if (mGazeboResourcePath.size() < 1) {
    return false;
  }

  auto joinPath = [](const std::vector<std::string>& parts) -> std::string {
    ostringstream ss;
    int count = 0;
    for (unsigned int i = 0; i < parts.size(); i++) {
      auto p = parts[i];
      auto j = p.length();
      while (j > 0 && p[j-1] == '/') {
        --j;
      }
      if (j < 1) continue;
      p = p.substr(0, j);
      if (count > 0) ss << '/';
      ss << p;
      ++count;
    }
    return ss.str();
  };

  auto fileExists = [](const std::string &fname) -> bool {
    ifstream ifile(fname);
    return ifile;
  };

  std::string modelFilename = "";
  for (auto p : mGazeboResourcePath) {
    // Try to find the model-file with various prefixes used in CogX
    for (std::string pfx : { "", "gen-", "cereals-" }) {
      auto fn = joinPath({ p, "models", pfx + label + ".model" });
      //log("Trying: %s", fn.c_str());
      if (fileExists(fn)) {
        modelFilename = fn;
        break;
      }
    }
    if (modelFilename != "") 
      break;
  }

  if (modelFilename == "") {
    return false;
  }

  static long modelId = 0;
  ++modelId;
  std::ostringstream cmd;
  cmd << "gzfactory spawn -f " << modelFilename << " -m " << label;
  cmd << " -x " << ( 1 + (modelId / 10) * 3);
  cmd << " -y " << (-1 - (modelId % 10) * 3);
  cmd << " -z " << 1.5;
  //log("Executing w. system: %s", cmd.str().c_str());
  int rv = system(cmd.str().c_str());

  return rv == 0; //  && isObjectLoaded(label);
}

void GazeboJuggler::moveObject(const std::string& label, int placeIndex)
{
  for(int i = 0; i < m_objects.size(); i++) {
    GObject& o = m_objects[i];
    if (o.label != label)
      continue;
    if (placeIndex < 0 || placeIndex >= m_locations.size()) {
      o.loc.x = 100 + i / 20;
      o.loc.y = 100 + i % 20;
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
  sleepComponent(1000);

  prepareObjects();

  while(isRunning())
  {
    sleepComponent(1000);
  }
}

}
// vim: set sw=2 sts=4 ts=8 et :vim
