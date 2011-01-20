/*
 * RGBDClient.h
 *
 *  Created on: Jan 20, 2011
 *      Author: Alper Aydemir
 */

#ifndef RGBDCLIENT_H_
#define RGBDCLIENT_H_


#include <cast/core.hpp>
#include <RGBD.hpp>
#include <KinectDataReceiver.h>
class RGBDClient : public KinectDataReceiver,
				   public cast::CASTComponent {
public:
	RGBDClient();
	virtual ~RGBDClient();
	virtual void receiveKinectData(RGBD::KinectData data);
protected:
	void configure(const std::map<std::string,std::string> & config);
	void runComponent();
};

#endif /* RGBDCLIENT_H_ */
