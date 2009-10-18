//
// = FILENAME
//    PushClientLaser.cpp
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

#include "PushClientLaser.hpp"
#include <cast/core/CASTUtils.hpp>

using namespace Laser;
using namespace cast;

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new PushClientLaser();
  }
}

PushClientLaser::PushClientLaser()
{
  m_IceServerName = "LaserServer";
  m_IceServerHost = "localhost";
  m_IceServerPort = cast::cdl::CPPSERVERPORT;
}

PushClientLaser::~PushClientLaser()
{
}

void 
PushClientLaser::configure(const std::map<std::string,std::string> & config)
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
PushClientLaser::start()
{
  log("start");
}

void PushClientLaser::runComponent() 
{
  setupPushScan2d(*this, -1, m_IceServerHost);
}

void 
PushClientLaser::receiveScan2d(const Scan2d &scan)
{
  log("receiveScan2d");
  std::cout << "Was pushed a scan with time " << scan.time 
            << " and " << scan.ranges.size() << " pts and "
            << " r0=" << scan.ranges[0] << std::endl;
}

