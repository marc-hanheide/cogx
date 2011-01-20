/*
 * KinectDataReceiver.cpp
 *
 *  Created on: Jan 20, 2011
 *      Author: Alper Aydemir
 */
#include <cast/core/CASTUtils.hpp>
#include "KinectDataReceiver.h"
using namespace cast;
KinectDataReceiver::KinectDataReceiver() {
	// TODO Auto-generated constructor stub

}

KinectDataReceiver::~KinectDataReceiver() {
	// TODO Auto-generated destructor stub
}
bool KinectDataReceiver::setupPushKinectData(cast::CASTComponent &owner,
	                                 double interval,
	                                 const std::string &iceServerHost,
	                                 int iceServerPort,
	                                 const std::string &iceServerName){

	 if (!owner.isRunning()) {
	    owner.println("You must not KinectDataReceiver::setupPush before the component is running");
	    return false;
	  }

	  Ice::Identity id;
	  id.name = iceServerName;
	  id.category = "RGBDServer";

	  std::ostringstream str;
	  str << owner.getCommunicator()->identityToString(id)
	      << ":default"
	      << " -h " << iceServerHost
	      << " -p " << iceServerPort;

	  owner.println(str.str());

	  Ice::ObjectPrx base = owner.getCommunicator()->stringToProxy(str.str());
	  RGBD::RGBDPushServerPrx server = RGBD::RGBDPushServerPrx::uncheckedCast(base);
	  if (!server) throw "Invalid proxy";

	  //add new facet for this object... client is this, ice id is the
	  //unique id of the object in the ice runtime, and OdometryReceiver is
	  //an arbitrary strings that must be unique for facets within this
	  //object
	  Ice::ObjectPrx basePrx(owner.getObjectAdapter()->addFacet(new KinectDataReceiverHelper(this),
	                                                      owner.getIceIdentity(),
	                                                      "KinectDataReceiver"));

	  //TODO error handling?
	  if(!basePrx) {
	    owner.println("couldn't get basePrx");
	    return false;
	  }

	  RGBD::KinectPushClientPrx clientPrx =
	    RGBD::KinectPushClientPrx::uncheckedCast(basePrx);

	  //TODO error handling?
	  if(!clientPrx) {
	    owner.println("couldn't cast back down to KinectPushClientPrx");
	    return false;
	  } else {
	    server->registerKinectPushClient(clientPrx, interval);
	  }

	  return true;

}
