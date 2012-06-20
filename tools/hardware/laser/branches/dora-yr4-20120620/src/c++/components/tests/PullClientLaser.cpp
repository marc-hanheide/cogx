//
// = FILENAME
//    PullClientLaser.cpp
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

#include "PullClientLaser.hpp"
#include <cast/core/CASTUtils.hpp>

#include "LaserClientUtils.hpp"

using namespace Laser;
using namespace cast;

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new PullClientLaser();
  }
}

PullClientLaser::PullClientLaser()
{
  m_IceServerName = "LaserServer";
  m_IceServerHost = "localhost";
  m_IceServerPort = cast::cdl::CPPSERVERPORTb;
}

PullClientLaser::~PullClientLaser()
{}

void 
PullClientLaser::configure(const std::map<std::string,std::string> & config)
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
PullClientLaser::start()
{
  m_Server = LaserClientUtils::getServerPrx(*this,
                                            m_IceServerHost,
                                            m_IceServerPort,
                                            m_IceServerName);
}

void 
PullClientLaser::runComponent()
{
  println("runComponent started");

  
  while (isRunning()) {

    std::cout << "About to pull a scan\n";
    Laser::Scan2d scan = m_Server->pullScan2d();
    
    if (!scan.ranges.empty()) {
      std::cout << "Pulled a scan with time " << scan.time 
                << " n=" << scan.ranges.size()
                << " r0=" << scan.ranges[0]
                << std::endl;
    } else {
      std::cout << "Pulled EMPTY scan with time " << scan.time 
                << std::endl;
    }
    
    sleepComponent(100);

  }
}
