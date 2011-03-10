/*
 * AR_Pose.cpp
 *
 *  Created on: Mar 10, 2011
 *      Author: alper
 */

#include "AR_Pose.h"
#include <VisionData.hpp>
#include <VideoClient.h>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <map>
extern "C" {
cast::CASTComponentPtr newComponent() {
	return new cast::AR_Pose();
}
}
const double CM_TO_M = 0.001;

using namespace std;

namespace cast {

AR_Pose::AR_Pose() {
	// TODO Auto-generated constructor stub

}

void AR_Pose::init() {

}

void AR_Pose::runComponent() {
	log("I am running.");
while(isRunning()){
	detectMarkers();
	sleepComponent(500);
}
}

AR_Pose::~AR_Pose() {
	// TODO Auto-generated destructor stub
}

void AR_Pose::start() {
	m_videoServer = getIceServer<Video::VideoInterface> (m_videoServerName);
	// register our client interface to allow the video server pushing images
	Video::VideoClientInterfacePtr servant = new VideoClientI(this);
	registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface> (
			servant);
	addChangeFilter(createLocalTypeFilter<VisionData::ARTagCommand> (cdl::ADD),
			new MemberFunctionChangeReceiver<AR_Pose> (this,
					&AR_Pose::newARTagCommand));

}

void AR_Pose::newARTagCommand(const cast::cdl::WorkingMemoryChange &objID) {

}
void AR_Pose::detectMarkers() {
	ARMarkerInfo *marker_info;
	int marker_num;
	int i, k, j;

	/* Get the image from ROSTOPIC
	 * NOTE: the dataPtr format is BGR because the ARToolKit library was
	 * build with V4L, dataPtr format change according to the
	 * ARToolKit configure option (see config.h).*/
	m_videoServer->getImage(m_camId, m_image);
	log("Image received .");
	IplImage *iplImage = convertImageToIpl(m_image);
	ARUint8 *dataPtr = (unsigned char*) iplImage->imageData;
	// detect the markers in the video frame
	if (arDetectMarker(dataPtr, m_threshold, &marker_info, &m_objectnum) < 0) {
		log("Warning: Something bad happened.");
	}
	log("found %d markers", m_objectnum);

	ARMarkers.clear();
	// check for known patterns
	// check for known patterns
	for (i = 0; i < m_objectnum; i++) {
		log("FOUND MARKER!! ");
		k = -1;
		for (j = 0; j < marker_num; j++) {
			if (object[i].id == marker_info[j].id) {
				if (k == -1)
					k = j;
				else // make sure you have the best pattern (highest confidence factor)
				if (marker_info[k].cf < marker_info[j].cf)
					k = j;
			}
		}
		if (k == -1) {
			object[i].visible = 0;
			continue;
		}

	// calculate the transform for each marker
	if (object[i].visible == 0) {
		arGetTransMat(&marker_info[k], object[i].marker_center,
				object[i].marker_width, object[i].trans);
	} else {
		arGetTransMatCont(&marker_info[k], object[i].trans,
				object[i].marker_center, object[i].marker_width,
				object[i].trans);
	}
	object[i].visible = 1;

	double arQuat[4], arPos[3];

	//arUtilMatInv (object[i].trans, cam_trans);
	arUtilMat2QuatPos(object[i].trans, arQuat, arPos);

    double quat[4], pos[3];

    pos[0] = arPos[0] * CM_TO_M;
    pos[1] = arPos[1] * CM_TO_M;
    pos[2] = arPos[2] * CM_TO_M;
	}


}
void AR_Pose::configure(const map<string, string> & _config) {
	log("configuring..");
	map<string, string>::const_iterator it;

	std::string tagfile;
	if ((it = _config.find("--tagfile")) != _config.end()) {
		tagfile = it->second;
	}
	log("Tagfile: %s", tagfile.c_str());


	m_videoServerName = "VideoServer";
		if ((it = _config.find("--videoserver")) != _config.end()) {
			m_videoServerName = it->second;
		}

	m_threshold = 100;
	 it = _config.find("--threshold");
	  if (it != _config.end()) {
		  m_threshold = (atof(it->second.c_str()));
	      log("Threshold set to: %d", m_threshold);
	   }

	// load in the object data - trained markers and associated bitmap files
	char buf[1024];
	sprintf(buf, "%s", tagfile.c_str());
	int objectnum;
	if ((object = ar_object::read_ObjData(buf, &objectnum))
			== NULL) {
		log("Warning: Could not read tag file, I'll probably crash");
	}
	log("Read %d tags.", objectnum);
}

} // namespace
