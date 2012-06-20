//
// = FILENAME
//    OdometryReceiver.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2009 Patric Jensfelt, Nick Hawes
//
/*----------------------------------------------------------------------*/

#include "OdometryReceiver.hpp"

#include <cast/core/CASTUtils.hpp>

using namespace Robotbase;
using namespace cast;

OdometryReceiver::OdometryReceiver()
{}

OdometryReceiver::~OdometryReceiver()
{
}

bool
OdometryReceiver::setupPushOdometry(cast::CASTComponent &owner, 
                                    double interval,
                                    const std::string &serverName)
{
  if (!owner.isRunning()) {
    owner.println("You must not call OdometryReceiver::setupPush before the component is running");
    return false;
  }

  RobotbaseServerPrx server(owner.getIceServer<RobotbaseServer>(serverName));

//  Ice::Identity id;
//  id.name = iceServerName;
//  id.category = "RobotbaseServer";
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
//  Robotbase::RobotbaseServerPrx server = Robotbase::RobotbaseServerPrx::uncheckedCast(base);
//  if (!server) throw "Invalid proxy";

  //add new facet for this object... client is this, ice id is the
  //unique id of the object in the ice runtime, and OdometryReceiver is
  //an arbitrary strings that must be unique for facets within this
  //object
  Ice::ObjectPrx basePrx(owner.getObjectAdapter()->addFacet(new OdometryReceiverHelper(this), 
                                                      owner.getIceIdentity(), 
                                                      "OdometryReceiver")); 
  
  //TODO error handling?
  if(!basePrx) {
    owner.println("couldn't get basePrx");
    return false;
  }
  
  Robotbase::OdometryPushClientPrx clientPrx = 
    Robotbase::OdometryPushClientPrx::uncheckedCast(basePrx);
  
  //TODO error handling?
  if(!clientPrx) {
    owner.println("couldn't cast back down to OdometryPushClientPrx");
    return false;
  } else {
    owner.log("Registering odometry push client");
    server->registerOdometryPushClient(clientPrx, interval);    
  } 

  return true;
}

