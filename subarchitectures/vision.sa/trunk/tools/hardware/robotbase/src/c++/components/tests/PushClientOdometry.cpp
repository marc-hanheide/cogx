//
// = FILENAME
//    PushClientOdometry.cpp
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

#include "PushClientOdometry.hpp"
#include <cast/core/CASTUtils.hpp>

using namespace Robotbase;
using namespace cast;

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new PushClientOdometry();
  }
}

PushClientOdometry::PushClientOdometry()
{
  m_IceServerName = "RobotbaseServer";
  m_IceServerHost = "localhost";
  m_IceServerPort = cast::cdl::CPPSERVERPORT;
}

PushClientOdometry::~PushClientOdometry()
{
}

void 
PushClientOdometry::configure(const std::map<std::string,std::string> & config)
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
}

void 
PushClientOdometry::start()
{
  log("start");
}

void PushClientOdometry::runComponent() 
{
  setupPushOdometry(*this, -1, m_IceServerHost);
}

void 
PushClientOdometry::receiveOdometry(const Odometry &odom)
{
  log("receiveOdometry");

  if (!odom.odompose.empty()) {
    std::cerr << "PushClientOdometry: Was pushed odom with time " << odom.time 
              << " x=" << odom.odompose[0].x
              << " y=" << odom.odompose[0].y
              << " theta=" << odom.odompose[0].theta
              << std::endl;
  }
}

