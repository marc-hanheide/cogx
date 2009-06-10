//
// = FILENAME
//    PeekabotControl.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2008 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "PeekabotControl.hpp"

#include <AddressBank/ConfigFileReader.hh>

using namespace std;
using namespace navsa;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new PeekabotControl();
  }
}

PeekabotControl::PeekabotControl() 
{
  m_CtrlAction = 0;
}

PeekabotControl::~PeekabotControl() 
{}

void PeekabotControl::configure(const map<string,string>& config) 
{
  println("configure entered");

  m_RetryDelay = 10;
  if(config.find("--retry-interval") != config.end()){
    std::istringstream str(config.find("--retry-interval")->second);
    str >> m_RetryDelay;
  }

  Cure::ConfigFileReader *cfg = 0;

  map<string,string>::const_iterator it;

  it = config.find("-c");
  if (it != config.end()) {
    cfg = new Cure::ConfigFileReader;
    log("About to try to open the config file");
    if (cfg->init(it->second) != 0) {
      delete cfg;
      cfg = 0;
      log("Could not init Cure::ConfigFileReader with -c argument");
    } else {
      log("Managed to open the Cure config file");
    }
  }

  it = config.find("--action");
  if (it != config.end()) {
    std::string action(it->second);
    if (action == "object") {
      m_CtrlAction = 1;
    }
  }

  m_PbPort = 5050;
  m_PbHost = "localhost";

  if (cfg) {

    // To be backward compatible with config files that specify the
    // RoboLook host and really mean peekabot we read that first and
    // overwrite it below if both are specified
    cfg->getRoboLookHost(m_PbHost);

    std::string usedCfgFile, tmp;
    if (cfg && cfg->getString("PEEKABOT_HOST", true, tmp, usedCfgFile) == 0) {
      m_PbHost = tmp;
    }

  }

  connectPeekabot();  

  println("configure done");
}

void PeekabotControl::start() 
{
  println("start entered");
  
}

void PeekabotControl::runComponent() {

  println("runComponent");

  while(!m_PeekabotClient.is_connected() && (m_RetryDelay > -1)){
    sleep(m_RetryDelay);
    connectPeekabot();
  }

  println("Connected to peekabot, ready to go");

  double radius = 0.75;
  double xNoA = 10, yNoA = 10;

  peekabot::GroupProxy root;
  root.assign(m_PeekabotClient, "root");

  peekabot::GroupProxy controlmodule;
  controlmodule.add(root, "ctrl", peekabot::REPLACE_ON_CONFLICT);

  // Create an "icon" to drag around and show where you want the robot
  // to go
  peekabot::CylinderProxy target;
  target.add(controlmodule, "target", peekabot::REPLACE_ON_CONFLICT);
  target.set_scale(0.5*radius,0.5*radius,0.1);
  target.set_position(xNoA - 2*radius,yNoA,0.05);
  target.set_color(0.5,0.5,0.5);

  // Create an "icon" to mark if you want to control according to the marker
  peekabot::SphereProxy control;
  control.add(controlmodule, "control", peekabot::REPLACE_ON_CONFLICT);
  control.set_scale(0.5*radius,0.5*radius,0.1);
  control.set_position(xNoA,yNoA+2*radius,0.05);
  control.set_color(0,1,0);

  peekabot::CylinderProxy actionZone;
  actionZone.add(controlmodule, "action-zone", 
                     peekabot::REPLACE_ON_CONFLICT);
  actionZone.set_scale(radius,radius,0);
  actionZone.set_position(xNoA,yNoA,0);
  actionZone.set_color(0,1,0);

  bool wasInCtrl = false;
  float xT, yT;
  
  if (m_PeekabotClient.is_connected()) {

    while (isRunning()) {

      peekabot::Result<peekabot::Vector3ru> r;

      // Check if the control marker is inside or outside the control
      // zone. If it is 
      r = actionZone.get_position(peekabot::WORLD_COORDINATES);

      if (r.succeeded()) {
        xNoA = r.get_result().m_c[0];
        yNoA = r.get_result().m_c[1];
      }

      // Get the position of the control marker
      r = control.get_position(peekabot::WORLD_COORDINATES);

      if (r.succeeded()) {

        // Check if the control marker is inside the action zone
        if (hypot(yNoA - r.get_result().m_c[1],
                  xNoA - r.get_result().m_c[0]) < radius) {

          // We should listen to the control marker, since it is
          // inside the action zone

          // Get position of the target marker
          r = target.get_position(peekabot::WORLD_COORDINATES);

          if (r.succeeded()) {

            bool sendCmd = true;

            // Check if we already were in control
            if (wasInCtrl &&               
                (xT == r.get_result().m_c[0]) &&
                (yT == r.get_result().m_c[1])) {
              sendCmd = false;
            }

            xT = r.get_result().m_c[0];
            yT = r.get_result().m_c[1];

            if (sendCmd) {

              if (m_CtrlAction == 1) {
                NavData::ObjObsPtr oo = new NavData::ObjObs;

                // First object with default values
                oo = new NavData::ObjObs;
                oo->category = "musli";
                oo->time = getCASTTime();              
                std::string id = "objobs-" + newDataID();
                addToWorkingMemory<NavData::ObjObs>(id, oo);
                println("Added object obs of objct \"%s\", task id:%s", 
                      oo->category.c_str(), id.c_str());

                // Second object with some values set
                oo = new NavData::ObjObs;
                oo->category = "rice";
                oo->time = getCASTTime();              

                // We can specify a certain OPTIONAL (def=1m) distance
                // to the obkject
                oo->dist.resize(1);
                oo->dist[0] = 2;

                // Specify OPTIONAL pan,tilt angles (def 0deg)
                oo->angles.resize(2);
                double f = 320; //pixels for a 640x320 image
                double u = 500;  double v = 20;  // upper right corner
                double u0 = 320; double v0 = 240;
                oo->angles[0] = -atan((u-u0) / f); // "pan"
                oo->angles[1] = -atan((v-v0) / f);  // "tilt"

                id = "objobs-" + newDataID();
                addToWorkingMemory<NavData::ObjObs>(id, oo);
                println("Added object obs of objct \"%s\", task id:%s", 
                      oo->category.c_str(), id.c_str());
              } else {

                NavData::NavCommandPtr cmd = new NavData::NavCommand;
                
                cmd->prio = NavData::NORMAL;
                cmd->pose.resize(2);
                cmd->pose[0] = xT;
                cmd->pose[1] = yT;
                cmd->cmd = NavData::GOTOPOSITION;
                //cmd->tolerance.resize(1);
                //cmd->tolerance[0] = 0.1;
                std::string id = "gotoxy-" + newDataID();
                log("Sending robot to new target position (%.2f, %.2f) task id: %s", xT, yT, id.c_str());
                addToWorkingMemory<NavData::NavCommand>(id, cmd);
                
              }
            }

          }

          wasInCtrl = true;

        } else {

          // No longer in control
          wasInCtrl = false;

        }
      }


      // Sleep for a second and check again
      sleepComponent(1000);

    }

  }
}

void PeekabotControl::connectPeekabot()
{
  try {
    log("Trying to connect to Peekabot (again) on host %s and port %d",
        m_PbHost.c_str(), m_PbPort);

    m_PeekabotClient.connect(m_PbHost, m_PbPort, true);

  } catch(std::exception &e) {
    log("Caught exception when connecting to peekabot (%s)",
        e.what());
    return;
  }
}

