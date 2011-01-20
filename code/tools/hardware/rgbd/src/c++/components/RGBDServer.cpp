/*
 * RGBDServer.cpp
 *
 *  Created on: Jan 19, 2011
 *      Author: Alper Aydemir
 */

#include "RGBDServer.hh"
#include <cast/core.hpp>

using namespace std;
using namespace Ice;
using namespace cast;
using namespace cast::cdl;
using namespace RGBD;
using namespace xn;

#define CHECK_RC(rc, what) \
        if (rc != XN_STATUS_OK) \
        {\
                printf("%s failed: %s\n", what, xnGetStatusString(rc)); \
        }



/**
 * The function called to create a new instance of our component.
 */
extern "C" {
cast::CASTComponentPtr newComponent() {
	return new RGBDServer();
}
}

void RGBDServer::start() {
		EnumerationErrors errors;
		XnStatus nRetVal = XN_STATUS_OK;
		nRetVal = m_Context.InitFromXmlFile(m_ConfigFilePath.c_str(), &errors);
		if (nRetVal == XN_STATUS_NO_NODE_PRESENT) {
			XnChar strError[1024];
			errors.ToString(strError, 1024);
			log("%s\n", strError);
			m_IsConnected = false;
		} else if (nRetVal != XN_STATUS_OK) {
			log("Open failed: %s\n", xnGetStatusString(nRetVal));
			m_IsConnected = false;
		}
		if (nRetVal == XN_STATUS_OK){
			m_IsConnected = true;
		}

		if (!m_IsConnected){
			log("Could not connect to device. %s",xnGetStatusString(nRetVal));
			exit(1);
		}

			nRetVal = m_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, m_depth);
			CHECK_RC(nRetVal, "Find depth generator");

			nRetVal = xnFPSInit(&m_xnFPS, 180);
			CHECK_RC(nRetVal, "FPS Init");

}

XnStatus RGBDServer::readFromKinect(){
	debug("Reading from Kinect.");
	XnStatus nRetVal = XN_STATUS_OK;
	DepthMetaData depthMD;
	nRetVal = m_Context.WaitOneUpdateAll(m_depth);
	if (nRetVal != XN_STATUS_OK) {
		log("UpdateData failed: %s", xnGetStatusString(nRetVal));
		nRetVal = XN_STATUS_NO_NODE_PRESENT;
		return nRetVal;
	}

	xnFPSMarkFrame(&m_xnFPS);

	m_depth.GetMetaData(depthMD);
	const XnDepthPixel* pDepthMap = depthMD.Data();
	//Convert to Ice

	m_kdata.XRes = depthMD.XRes();
	m_kdata.YRes = depthMD.YRes();
	m_kdata.frameid = depthMD.FrameID();
	m_kdata.depth.resize(m_kdata.XRes*m_kdata.YRes);
	debug("Frame metadata, XRes: %d, YRes: %d, Frameid: %d", depthMD.XRes(), depthMD.YRes(), depthMD.FrameID());
	debug("allocated %d", m_kdata.depth.size());
	for (unsigned int i =0; i<depthMD.XRes();i++){
		for (unsigned int j=0; j<depthMD.YRes();j++){
			m_kdata.depth[i*j + j] = depthMD(i,j);
		}
		}
	return nRetVal;
}
void RGBDServer::runComponent() {
	// get RGBD data and push to clients...
	XnStatus nRetVal;
	log("RGBDServer running");
	while (isRunning() && m_IsConnected) {
		 nRetVal = readFromKinect();
		 debug("read from kinect");
		if (nRetVal == XN_STATUS_OK) {
			debug("pushing to clients");
		for (unsigned int i = 0; i < m_PushClients.size(); i++) {
			if (isRunning() && (!m_PushClients[i].timer.isRunning()
					|| (m_PushClients[i].timer.split()
							>= m_PushClients[i].interval))) {
				debug("pushed depth data to client %d", i);
				m_PushClients[i].timer.restart();
				m_PushClients[i].prx->receiveKinectData(m_kdata);
			}
		}
		}
		else{
			log("Something went wrong: %s",  xnGetStatusString(nRetVal));
			log("exiting...");
			break;
		}
	}
}

RGBDServer::RGBDServer() {

	m_IceServerName = "RGBDServer";
	m_ConfigFilePath = "./instantiations/kinectconfig/SamplesConfig.xml";
}
RGBDServer::~RGBDServer() {
	m_Context.Shutdown();
}

void RGBDServer::configure(const std::map<std::string, std::string> & config) {
	log("Configure");
	m_IsConnected= false;
	std::map<std::string, std::string>::const_iterator it;

	if ((it = config.find("--configfile-path")) != config.end()) {
		std::istringstream str(it->second);
		str >> m_ConfigFilePath;
	}
	// start the interface
	try {
		Ice::Identity id;
		id.name = m_IceServerName;
		id.category = "RGBDServer";

		//add as a separate object
		getObjectAdapter()->add(this, id);

		println("server registered");
	} catch (const Ice::Exception& e) {
		std::cerr << e << std::endl;
	} catch (const char* msg) {
		std::cerr << msg << std::endl;
	}
}
void RGBDServer::registerKinectPushClient(
		const RGBD::KinectPushClientPrx& client, Ice::Double interval,
		const Ice::Current&) {
	println("registerKinectPushClient");

	KinectClient c;
	c.prx = client;
	c.interval = interval;
	m_PushClients.push_back(c);
}

