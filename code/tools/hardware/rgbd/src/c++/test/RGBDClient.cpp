/*
 * RGBDClient.cpp
 *
 *  Created on: Jan 20, 2011
 *      Author: Alper Aydemir
 */

#include "RGBDClient.h"

extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new RGBDClient();
  }
}
RGBDClient::RGBDClient() {
	// TODO Auto-generated constructor stub

}

RGBDClient::~RGBDClient() {
	// TODO Auto-generated destructor stub
}

void RGBDClient::receiveKinectData(RGBD::KinectData data){
	log("got data: %d", data.depth[0]);
}

void RGBDClient::configure(const std::map<std::string,std::string> & config){

}

void RGBDClient::runComponent(){
	debug("RGBDClient running.");
	setupPushKinectData(*this,0.5);

}
