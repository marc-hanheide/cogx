//
// = FILENAME
//    Scan2dReceiver.cpp
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

#include "Scan2dReceiver.hpp"

#include <cast/core/CASTUtils.hpp>

using namespace Laser;
using namespace cast;

Scan2dReceiver::Scan2dReceiver()
{}

Scan2dReceiver::~Scan2dReceiver()
{
}

bool
Scan2dReceiver::setupPushScan2d(cast::CASTComponent &owner,
                                double interval,
                                const std::string &serverName)
{
  if (!owner.isRunning()) {
    owner.println("You must not call Scan2dReceiver::setupPush before the component is running");
    return false;
  }

  LaserServerPrx server(owner.getIceServer<LaserServer>(serverName));
  

//  Ice::Identity id;
//  id.name = iceServerName;
//  id.category = "LaserServer";
//
//  std::ostringstream str;
//  str << owner.getCommunicator()->identityToString(id) 
//      << ":default"
//      << " -h " << iceServerHost
//      << " -p " << iceServerPort; 
//  
//  owner.println(str.str());
//
//  Ice::ObjectPrx base = owner.getCommunicator()->stringToProxy(str.str());    
//  Laser::LaserServerPrx server = Laser::LaserServerPrx::uncheckedCast(base);
//  if (!server) throw "Invalid proxy";

  //add new facet for this object... client is this, ice id is the
  //unique id of the object in the ice runtime, and Scan2dReceiver is
  //an arbitrary strings that must be unique for facets within this
  //object
  Ice::ObjectPrx basePrx(owner.getObjectAdapter()->addFacet(new Scan2dReceiverHelper(this), 
                                                             owner.getIceIdentity(), 
                                                      "Scan2dReceiver")); 

  //TODO error handling?
  if(!basePrx) {
    owner.println("couldn't get basePrx");
    return false;
  }
  
  Laser::Scan2dPushClientPrx clientPrx = 
    Laser::Scan2dPushClientPrx::uncheckedCast(basePrx);

  //TODO error handling?
  if(!clientPrx) {
    owner.println("couldn't cast back down to LaserPushClientPrx");
    return false;
  } else {
    owner.log("Registering scan push client");
    server->registerScan2dPushClient(clientPrx, interval);    
  } 

  return true;
}

