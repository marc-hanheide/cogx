//
// = FILENAME
//    LaserStop.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "LaserStop.hpp"

#include <cast/core/CASTUtils.hpp>

#include <cmath>

using namespace Robotbase;
using namespace Laser;
using namespace cast;

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new LaserStop();
  }
}

LaserStop::LaserStop()
{
  m_RobotServerName = "RobotbaseServer";
  m_RobotServerHost = "localhost";
  m_RobotServerPort = cast::cdl::CPPSERVERPORT;

  m_LaserServerName = "LaserServer";
  m_LaserServerHost = "localhost";
  m_LaserServerPort = cast::cdl::CPPSERVERPORT;

  m_MoveRobot = false;
  m_Inited = false;
}

LaserStop::~LaserStop()
{
}

void 
LaserStop::configure(const std::map<std::string,std::string> & config)
{
  println("configure");

  std::map<std::string,std::string>::const_iterator it;

  if ((it = config.find("--laser-server-name")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_LaserServerName;
  }
  log("Using m_LaserServerName=%s", m_LaserServerName.c_str());

  if ((it = config.find("--laser-server-host")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_LaserServerHost;
  }
  log("Using m_LaserServerHost=%s", m_LaserServerHost.c_str());

  if ((it = config.find("--laser-server-port")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_LaserServerPort;
  }
  log("Using m_LaserServerPort=%d", m_LaserServerPort);

  if ((it = config.find("--robot-server-name")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_RobotServerName;
  }
  log("Using m_RobotServerName=%s", m_RobotServerName.c_str());

  if ((it = config.find("--robot-server-host")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_RobotServerHost;
  }
  log("Using m_RobotServerHost=%s", m_RobotServerHost.c_str());

  if ((it = config.find("--robot-server-port")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_RobotServerPort;
  }
  log("Using m_RobotServerPort=%d", m_RobotServerPort);

  if ((it = config.find("--move")) != config.end()) {
    m_MoveRobot = true;
  }
  log("Using m_MoveRobot=%d", (int)m_MoveRobot);
}

void 
LaserStop::start()
{
  Ice::CommunicatorPtr ic = getCommunicator();
  
  //
  Ice::Identity id;
  id.name = m_RobotServerName;
  id.category = "RobotbaseServer";

  std::ostringstream str;
  str << ic->identityToString(id) 
      << ":default"
      << " -h " << m_RobotServerHost
      << " -p " << m_RobotServerPort; 

  println(str.str());

  Ice::ObjectPrx base = ic->stringToProxy(str.str());    
  m_Robot = Robotbase::RobotbaseServerPrx::uncheckedCast(base);
}

void LaserStop::runComponent() 
{
  setupPushOdometry(*this, -1, m_RobotServerHost);
  setupPushScan2d(*this, -1, m_LaserServerHost);
}

void 
LaserStop::receiveOdometry(const Odometry &odom)
{
  if (!odom.odompose.empty()) {

    m_Odom = odom.odompose[0];

    if (!m_Inited) {
      m_InitOdom = m_Odom;
      m_Inited = true;
    }

    std::cerr << "LaserStop: Was pushed odom with time " << odom.time 
              << " x=" << m_Odom.x
              << " y=" << m_Odom.y
              << " theta=" << m_Odom.theta
              << std::endl;

    // Do the control 
    if (m_MoveRobot) {
      Robotbase::MotionCommand cmd;
      
      // Let the controller switch between two goal points. The
      // initial position and one

      double dist = 1;
      double xGoal = m_InitOdom.x + dist * cos(m_InitOdom.theta);
      double yGoal = m_InitOdom.y + dist * sin(m_InitOdom.theta);
      if ((odom.time.s / 20) % 2) {
        xGoal = m_InitOdom.x;
        yGoal = m_InitOdom.y;
      }

      // We define a controller that tries to drive the robot to 0,0
      double dErr = hypot(yGoal - odom.odompose[0].y, 
                          xGoal - odom.odompose[0].x);
      double aErr = (atan2(yGoal - odom.odompose[0].y, 
                           xGoal - odom.odompose[0].x) -
                     odom.odompose[0].theta);
      if (aErr > M_PI) aErr -= 2.0*M_PI;
      if (aErr <-M_PI) aErr += 2.0*M_PI;
      
      cmd.rotspeed = 0.5 * aErr;
      cmd.speed = 0.5 * dErr * exp(-aErr*aErr/(0.2*0.2));
      
      // Check if something is too close in which case we stop
      for (unsigned int i = 0; i < m_Scan.ranges.size(); i++) {
        if (m_Scan.ranges[i] > 0.05 && m_Scan.ranges[i] < 0.4) {
          println("Stopping");
          cmd.rotspeed = 0;
          cmd.speed = 0;
          break;
        }
      }

      m_Robot->execMotionCommand(cmd);
    }
  }
}

void 
LaserStop::receiveScan2d(const Scan2d &scan)
{
  m_Scan = scan;

  if (!scan.ranges.empty()) {
    std::cerr << "LaserStop: Was pushed scan with time " << scan.time 
              << " n=" << scan.ranges.size()
              << " r0=" << scan.ranges[0]
              << std::endl;
  }
}

