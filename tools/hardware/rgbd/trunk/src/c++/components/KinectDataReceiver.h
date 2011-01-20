/*
 * KinectDataReceiver.h
 *
 *  Created on: Jan 20, 2011
 *      Author: Alper Aydemir
 */
#include <Ice/Communicator.h>
#include <cast/core/CASTComponent.hpp>
#include <RGBD.hpp>

#ifndef KINECTDATARECEIVER_H_
#define KINECTDATARECEIVER_H_

class KinectDataReceiver {
public:
	KinectDataReceiver();
	virtual ~KinectDataReceiver();
	virtual void receiveKinectData(const RGBD::KinectData)=0;
	virtual bool setupPushKinectData(cast::CASTComponent &owner,
	                                 double interval = -1,
	                                 const std::string &iceServerHost = "localhost",
	                                 int iceServerPort = cast::cdl::CPPSERVERPORT,
	                                 const std::string &iceServerName = "RGBDServer");
protected:
	  class KinectDataReceiverHelper : virtual public RGBD::KinectPushClient {
	    KinectDataReceiver * m_receiver;
	  public:
	    KinectDataReceiverHelper(KinectDataReceiver * _receiver) :
	      m_receiver(_receiver)
	    {}

	    virtual ~KinectDataReceiverHelper(){}

	    virtual void receiveKinectData(const RGBD::KinectData &data,
	                                 const Ice::Current&) {
	      m_receiver->receiveKinectData(data);
	    }

	  };
};

#endif /* KINECTDATARECEIVER_H_ */
