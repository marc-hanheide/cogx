//
// = FILENAME
//    PullClientOdometry.cpp
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

#include "PullClientOdometry.hpp"
#include <cast/core/CASTUtils.hpp>

#include "RobotbaseClientUtils.hpp"

#include <cmath>

using namespace Robotbase;
using namespace cast;

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new PullClientOdometry();
  }
}

PullClientOdometry::PullClientOdometry()
{
  m_IceServerName = "RobotbaseServer";
  m_IceServerHost = "localhost";
  m_IceServerPort = cast::cdl::CPPSERVERPORT;

  m_MoveRobot = false;
}

PullClientOdometry::~PullClientOdometry()
{}

void 
PullClientOdometry::configure(const std::map<std::string,std::string> & config)
{
  println("configure");

  std::map<std::string,std::string>::const_iterator it;

  if ((it = config.find("--laser-server-name")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_IceServerName;
  }
  log("Using m_IceServerName=%s", m_IceServerName.c_str());

  if ((it = config.find("--laser-server-host")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_IceServerHost;
  }
  log("Using m_IceServerHost=%s", m_IceServerHost.c_str());

  if ((it = config.find("--laser-server-port")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_IceServerPort;
  }
  log("Using m_IceServerPort=%d", m_IceServerPort);

  if ((it = config.find("--move")) != config.end()) {
    m_MoveRobot = true;
  }
  log("Using m_Move=%d", (int)m_MoveRobot);
}

void 
PullClientOdometry::start()
{
  m_Server = RobotbaseClientUtils::getServerPrx(*this,
                                                m_IceServerHost,
                                                m_IceServerPort,
                                                m_IceServerName);
}

void 
PullClientOdometry::runComponent()
{
  println("runComponent started");

  
  while (isRunning()) {

    std::cout << "About to pull odom\n";
    Robotbase::Odometry odom = m_Server->pullOdometry();

    if (!odom.odompose.empty()) {
      std::cout << "PullClientOdometry: Pulled odom with time " << odom.time 
                << " x=" << odom.odompose[0].x
                << " y=" << odom.odompose[0].y
                << " theta=" << odom.odompose[0].theta
                << std::endl;
      
      if (m_MoveRobot) {
        Robotbase::MotionCommand cmd;
        
        cast::cdl::CASTTime tmp;
        tmp.s = 10;
        tmp.us = 0;
        if (odom.time > tmp) {
          // We define a controller that tries to drive the robot to 0,0
          double dErr = hypot(odom.odompose[0].y, odom.odompose[0].x);
          double aErr = (atan2(0-odom.odompose[0].y, 0-odom.odompose[0].x) -
                         odom.odompose[0].theta);
          if (aErr > M_PI) aErr -= 2.0*M_PI;
          if (aErr <-M_PI) aErr += 2.0*M_PI;
          
          cmd.rotspeed = 0.5 * aErr;
          cmd.speed = 0.5 * dErr * exp(-aErr*aErr/(0.2*0.2));

        } else {
          cmd.speed = 0.05;
          cmd.rotspeed = 0;
        }
   
        m_Server->execMotionCommand(cmd);
        
      }
    }

    sleepComponent(100);

  }
}
