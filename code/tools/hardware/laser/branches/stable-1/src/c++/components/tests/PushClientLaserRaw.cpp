//
// = FILENAME
//    PushClientLaserRaw.cpp
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

#include "PushClientLaserRaw.hpp"
#include <cast/core/CASTUtils.hpp>

using namespace Laser;
using namespace cast;

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new PushClientLaserRaw();
  }
}

PushClientLaserRaw::PushClientLaserRaw()
{
  m_IceServerName = "LaserServer";
  m_IceServerHost = "localhost";
  //  m_IceServerPort = 12345;
  m_IceServerPort = cast::cdl::CPPSERVERPORT;
}

PushClientLaserRaw::~PushClientLaserRaw()
{
}

void 
PushClientLaserRaw::configure(const std::map<std::string,std::string> & config)
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
PushClientLaserRaw::start()
{
  log("start");
  
  Ice::Identity id;
  id.name = m_IceServerName;
  id.category = "LaserServer";

  std::ostringstream str;
  str << getCommunicator()->identityToString(id) 
      << ":default"
      << " -h " << m_IceServerHost
      << " -p " << m_IceServerPort; 

  println(str.str());

  Ice::ObjectPrx base = getCommunicator()->stringToProxy(str.str());    
  m_Server = Laser::LaserServerPrx::uncheckedCast(base);
  if (!m_Server) throw "Invalid proxy";
}

void PushClientLaserRaw::runComponent() {
  //add new facet for this object... client is this, ice id is the
  //unique id of the object in the ice runtime, and PushClientLaserRaw is
  //an arbitrary strings that must be unique for facets within this
  //object
  Ice::ObjectPrx basePrx(getObjectAdapter()->addFacet(new Scan2dReceiverHelper(this), 
                                                      getIceIdentity(), 
                                                      "PushClientLaserRaw")); 
  
  //TODO error handling?
  if(!basePrx) {
    println("couldn't get basePrx");
    return;
  }
  
  Laser::Scan2dPushClientPrx clientPrx = 
    Laser::Scan2dPushClientPrx::uncheckedCast(basePrx);
  
  //TODO error handling?
  if(!clientPrx) {
    println("couldn't cast back down to LaserPushClientPrx");
  } else {
    m_Server->registerScan2dPushClient(clientPrx, 1);    
  } 
}

void 
PushClientLaserRaw::receiveScan2d(const Scan2d &scan, const Ice::Current&)
{
  log("receiveScan2d");
  std::cout << "Was pushed a scan with time " << scan.time << std::endl;
}

