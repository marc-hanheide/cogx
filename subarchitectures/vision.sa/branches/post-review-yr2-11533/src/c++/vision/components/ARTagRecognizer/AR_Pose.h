/*
 * AR_Pose.h
 *
 *  Created on: Mar 10, 2011
 *      Author: alper
 */

#ifndef AR_POSE_H_
#define AR_POSE_H_

#include <cast/architecture/ManagedComponent.hpp>
#include <Video.hpp>
#include <VisionData.hpp>
#include <VideoClient.h>
#include "ConvertImage.h"


#include <Matrix33.h>
#include "ModelLoader.h"

/* ARToolkit includes */
#include <AR/ar.h>
#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <AR/arMulti.h>
#include "object.h"

namespace cast{
class AR_Pose : public cast::ManagedComponent,
				public cast::VideoClient {

public:
	AR_Pose();
	virtual ~AR_Pose();
protected:
	void newARTagCommand(const cast::cdl::WorkingMemoryChange &objID);
	void runComponent();
	void configure(const std::map<std::string,std::string> & _config);
	void start();
private:

	/* Value to hold ARMarker result */
	struct ARMarker{
		int id;
		unsigned int confidence;
		cogx::Math::Pose3 trans;
	};
	std::vector<ARMarker> ARMarkers;

	/* Object to hold Marker data */
	ar_object::ObjectData_T * object;


	/* Ice proxe name */
	std::string m_videoServerName;

	/* our ICE proxy to the video server */
	Video::VideoInterfacePrx m_videoServer;


	/* Camera Id */
	int m_camId;

	/* Captured Image */
	Video::Image m_image;

	/* Threshold for recognition */
	int m_threshold;

	/* Number of markers known */
	int m_objectnum;

	/* Read-in objects and init values*/
	void init();
	void detectMarkers();

};
};

#endif /* AR_POSE_H_ */
