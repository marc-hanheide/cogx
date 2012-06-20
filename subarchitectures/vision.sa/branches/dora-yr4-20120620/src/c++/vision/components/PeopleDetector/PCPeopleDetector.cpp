#include <cast/core/CASTUtils.hpp>
#include <cast/architecture.hpp>
#include "LaserClientUtils.hpp"

#include <VideoUtils.h>
#include <VisionData.hpp>

#include <highgui.h>
#include <cvaux.h>

#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <sstream>

#include "PCPeopleDetector.hpp"

#define DONT_SAVE_FRAMES

using namespace std;
using namespace cast;
using namespace VisionData;
using namespace PointCloud;
using namespace cogx::Math;

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
cast::CASTComponentPtr newComponent() {
	return new PCPeopleDetector();
}
}

PCPeopleDetector::PCPeopleDetector() :
	m_numDetectionAttempts(1), m_runContinuously(false), maxDist(1e10) {
}

void PCPeopleDetector::configure(
		const std::map<std::string, std::string> & config) {
	//println("configure");
	// first let the base classes configure themselves
	configureServerCommunication(config);

	m_display.configureDisplayClient(config);

	std::map<std::string, std::string>::const_iterator it;

	if ((it = config.find("--faceCascade")) != config.end()) {
		face_cascade = (CvHaarClassifierCascade*) cvLoad(it->second.c_str(), 0,
				0, 0);
	} else {
		face_cascade = NULL;
	}

	if ((it = config.find("--numattempts")) != config.end()) {
		istringstream istr(it->second);
		istr >> m_numDetectionAttempts;
	}

	if ((it = config.find("--max-dist")) != config.end()) {
		istringstream istr(it->second);
		istr >> maxDist;
	}

	if ((it = config.find("--continuous")) != config.end()) {
		m_runContinuously = true;
	}

	if (m_runContinuously) {
		log("running continuously");
	} else {
		log("running on demand");
	}

}

void PCPeopleDetector::start() {

	// we want to receive PeopleDetectionCommand
	addChangeFilter(createGlobalTypeFilter<PeopleDetectionCommand> (cdl::ADD),
			new MemberFunctionChangeReceiver<PCPeopleDetector> (this,
					&PCPeopleDetector::receiveDetectionCommand));

	face_storage = cvCreateMemStorage(0);

	startPCCServerCommunication(*this);

	m_display.connectIceClient(*this);

}

void PCPeopleDetector::receiveDetectionCommand(
		const cdl::WorkingMemoryChange & _wmc) {
	try {
		PeopleDetectionCommandPtr cmd =
				getMemoryEntry<PeopleDetectionCommand> (_wmc.address);

		if (m_runContinuously) {
			log("Ignoring people detection command as running continuously");
		} else {
			runDetection();
		}
		// executed the command, results (if any) are on working memory,
		// now delete command as not needed anymore
		deleteFromWorkingMemory(_wmc.address);
	} catch (const cast::CASTException& e) {
		error("CASTException in receiveDetectionCommand: %s", e.message.c_str());
	}
}

std::string IntToStr(float i) {
	std::ostringstream ss;
	ss << i;
	return ss.str();
}

template<class Tp, class Tp2, class Tp3>
inline Tp limit(Tp a, Tp2 left, Tp3 right) {
	if (a < left)
		a = left;
	if (a > right)
		a = right;
	return a;
}

void PCPeopleDetector::runComponent() {
	if (m_runContinuously) {
		while (isRunning()) {
			lockComponent();
			runDetection();
			sleepComponent(100);
			unlockComponent();
		}
	}
}

void PCPeopleDetector::runDetection() {
	log("running detection");

	// Capture an image from the camera
	bool validDetection = false;
	PersonPtr person = new Person();
	person->existProb = 0.0;
	for (unsigned int detection = 0; !validDetection && detection < m_numDetectionAttempts; detection++) {
		Video::Image image;
		getRectImage(0, 640, image);

		IplImage * imgt = convertImageToIpl(image);
		CvSize img_sz = cvGetSize(imgt);
		IplImage * img = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);
		cvCvtColor(imgt, img, CV_RGB2GRAY);
		CvSeq * face = NULL;

		if (face_cascade) {
			face = cvHaarDetectObjects(img, face_cascade, face_storage, 1.5, 3,
					CV_HAAR_DO_CANNY_PRUNING, cvSize(60, 60));
		}

		for (int i = 0; !validDetection && face && i < face->total; i++) {
			CvRect* rect = (CvRect*) cvGetSeqElem(face, i);
			CvPoint tl;
			tl.x = rect->x;
			tl.y = rect->y;
			CvPoint br;

			br.x = rect->x + rect->width;
			br.y = rect->y + rect->height;
			log("detected a face at %d %d (%dx%d)", rect->x, rect->y,
					rect->width, rect->height);
			SurfacePointSeq points;
			getPoints(true, 640, points);
			double avg_distance = 0.0;
			int count = 0;
			Vector3 centerOfMass = vector3(0.0, 0.0, 0.0);
			for (unsigned int i = 0; !validDetection && i < points.size(); i++) {
				Vector3 v = points.at(i).p;
				Vector2 projected(projectPoint(image.camPars, v));
				if (projected.x > tl.x + rect->width
						* DETECTWINDOW_PADDING_PROPORTION && projected.x < br.x
						- rect->width * DETECTWINDOW_PADDING_PROPORTION
						&& projected.y > tl.y + rect->height
								* DETECTWINDOW_PADDING_PROPORTION
						&& projected.y < br.y - rect->height
								* DETECTWINDOW_PADDING_PROPORTION) {
					avg_distance += sqrt(v.y * v.y + v.x * v.x);
					centerOfMass.x += v.x;
					centerOfMass.y += v.y;
					centerOfMass.z += v.z;
					count++;
					//log("point: %f %f %f", v.x, v.y, v.z);
				}
			}
			if (count > 0) {
				avg_distance /= count;
				centerOfMass.x /= count;
				centerOfMass.y /= count;
				centerOfMass.z /= count;
				double size = projectSize(image.camPars, centerOfMass,
						FACE_DIAMETER);
				double sizeRatio = size / rect->width;
				log("count=%d, dist=%f, x=%f, y=%f, z=%f, size ratio=%f",
						count, avg_distance, centerOfMass.x, centerOfMass.y,
						centerOfMass.z, sizeRatio);
				if (sizeRatio > MIN_RATIO && sizeRatio < MAX_RATIO && avg_distance
						<= maxDist) {
					person->distance = avg_distance;
					person->angle = atan2(centerOfMass.y, centerOfMass.x);
					person->existProb = 1.0;
					println("person at distance=%f, angle=%f", person->distance,
							person->angle * 180.0 / M_PI);
					validDetection = true;
				}
			}
			if (validDetection) {
				cvRectangle(imgt, tl, br, cvScalar(0.0, 255.0, 0.0, 255.0), 2);
				CvFont font = cvFont(1.0, 2);
				char buf[255];
				sprintf(buf, "distance=%.2f,angle=%.2f", person->distance,
						person->angle * 180.0 / M_PI);
				cvPutText(imgt, buf, cvPoint(tl.x + 1, (tl.y + br.y) / 2),
						&font, cvScalar(0.0, 255.0, 0.0, 255.0));
			} else {
				cvRectangle(imgt, tl, br, cvScalar(255.0, 0.0, 0.0, 255.0), 2);
			}
		}
		m_display.setImage(getComponentID(), imgt);
		cvReleaseImage(&imgt);
		cvReleaseImage(&img);
	}
	// display in viewer
	addToWorkingMemory(newDataID(), person);

	sleepComponent(100);
}

