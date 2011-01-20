/*
 * RGBDServer.hpp
 *
 *  Created on: Jan 19, 2011
 *      Author: Alper Aydemir
 */

#ifndef RGBDSERVER_HPP_
#define RGBDSERVER_HPP_

#include <cast/core.hpp>
#include <RGBD.hpp>
#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>

class RGBDServer: public cast::CASTComponent,
				  public RGBD::RGBDPushServer{

private:
std::string m_ConfigFilePath;
public:
	RGBDServer();
	~RGBDServer();

	  void registerKinectPushClient(const RGBD::KinectPushClientPrx& client,
	                                  Ice::Double interval,
	                                  const Ice::Current&);
protected:
	virtual void start();
	virtual void runComponent();
	virtual void configure(const std::map<std::string,std::string> & config);
	XnStatus readFromKinect();

	class KinectClient {
	  public:
	    RGBD::KinectPushClientPrx prx;
	    double interval;
	    cast::CASTTimer timer;
	  };
	  std::vector<KinectClient> m_PushClients;


private:
	  RGBD::KinectData m_kdata;
	  std::string m_IceServerName;
	  xn::Context m_Context;
	  XnFPSData   m_xnFPS;
	  xn::DepthGenerator m_depth;
	  bool m_IsConnected;
};

#endif /* RGBDSERVER_HPP_ */
