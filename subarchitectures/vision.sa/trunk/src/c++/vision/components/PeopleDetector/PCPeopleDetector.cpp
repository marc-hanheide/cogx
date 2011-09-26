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
#define SUB_WINDOW 0.2
#define FACE_DIAMETER 0.18

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
	deinterlacing(false), m_numDetectionAttempts(1), m_runContinuously(false) {
}

PCPeopleDetector::~PCPeopleDetector() {
}

void PCPeopleDetector::configure(
		const std::map<std::string, std::string> & config) {
	//println("configure");
	// first let the base classes configure themselves
	configureServerCommunication(config);

	m_display.configureDisplayClient(config);

	std::map<std::string, std::string>::const_iterator it;

	if ((it = config.find("--trackerThreshold")) != config.end()) {
		std::istringstream str(it->second);
		str >> trackerThreshold;
	} else
		trackerThreshold = 1.0;

	if ((it = config.find("--faceCascade")) != config.end()) {
		face_cascade = (CvHaarClassifierCascade*) cvLoad(it->second.c_str(), 0,
				0, 0);
	} else {
		face_cascade = NULL;
	}

	if ((it = config.find("--fullbodyCascade")) != config.end()) {
		fullbody_cascade = (CvHaarClassifierCascade*) cvLoad(
				it->second.c_str(), 0, 0, 0);
	} else {
		fullbody_cascade = NULL;
	}

	if ((it = config.find("--numattempts")) != config.end()) {
		istringstream istr(it->second);
		istr >> m_numDetectionAttempts;
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

	upper_storage = cvCreateMemStorage(0);
	face_storage = cvCreateMemStorage(0);
	fullbody_storage = cvCreateMemStorage(0);

	startPCCServerCommunication(*this);

	//cvNamedWindow(getComponentID().c_str(), 1);
	m_display.connectIceClient(*this);

}

void PCPeopleDetector::receiveDetectionCommand(
		const cdl::WorkingMemoryChange & _wmc) {
	PeopleDetectionCommandPtr cmd = getMemoryEntry<PeopleDetectionCommand> (
			_wmc.address);

	if (m_runContinuously) {
		log("Ignoring people detection command as running continuously");
	} else {
		log("People detecting for %d attempts", m_numDetectionAttempts);
		for (unsigned int i = 0; i < m_numDetectionAttempts; ++i) {
			runDetection();
		}
	}
	// executed the command, results (if any) are on working memory,
	// now delete command as not needed anymore
	deleteFromWorkingMemory(_wmc.address);
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
//
//// Deinterlaces an Opencv IplImage. It assumes the image is in colour RGB format.
//// Done in-place, using a fast blending method.
//void deinterlaceImg(IplImage * img) {
//	for (int y = 0; y < img->height - 2; y += 2) {
//		for (int x = 0; x < img->width; x++) {
//			CvScalar s1, s2;
//			s1 = cvGet2D(img, y, x);
//			s2 = cvGet2D(img, y + 2, x);
//
//			s1.val[0] = (s1.val[0] + s2.val[0]) / 2;
//			s1.val[1] = (s1.val[1] + s2.val[1]) / 2;
//			s1.val[2] = (s1.val[2] + s2.val[2]) / 2;
//
//			cvSet2D(img, y + 1, x, s1);
//		}
//	}
//}

struct PersonRecord {
	int scanIndex;
	double laserX, laserZ;
	double deltaX, deltaZ;
	double distanceMoved;
	int distanceMovedCount;
	std::string castID;

	PersonRecord() {
		distanceMoved = 0;
		distanceMovedCount = 0;
		deltaX = deltaZ = numeric_limits<double>::min();
		castID = "NULL";
	}
};

void PCPeopleDetector::runComponent() {
	if (m_runContinuously) {
		while (isRunning()) {
			lockComponent();
			runDetection();
			unlockComponent();
		}
	}
}

void PCPeopleDetector::runDetection() {
	log("running detection");

	static int cnt = 0;

	static vector<PersonRecord> detections;

	//log("Frame %d",cnt);

	// Capture an image from the camera
	Video::Image image;
	getRectImage(0, 640, image);

	log("got image");

	IplImage * imgt = convertImageToIpl(image);
	log("converted");

	CvSize img_sz = cvGetSize(imgt);
	IplImage * img = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);
	cvCvtColor(imgt, img, CV_RGB2GRAY);

	log("converted to greyscale");

	CvSeq * face = NULL;

	if (face_cascade) {
		face = cvHaarDetectObjects(img, face_cascade, face_storage, 1.5, 3,
				CV_HAAR_DO_CANNY_PRUNING, cvSize(60, 60));
	}
	// Loop through full-body detections and find the corresponding Z value from the laser
	// data. It is assumed that the shape closest to the camera within the detection window
	// is in fact the person to be tracked.
	bool validDetection = false;
	PersonPtr person;
	for (int i = 0; !validDetection && face && i < face->total; i++) {
		CvRect* rect = (CvRect*) cvGetSeqElem(face, i);
		CvPoint tl;
		tl.x = rect->x;
		tl.y = rect->y;
		CvPoint br;

		br.x = rect->x + rect->width;
		br.y = rect->y + rect->height;
		println("detected a face at %d %d (%dx%d)", rect->x, rect->y,
				rect->width, rect->height);
		SurfacePointSeq points;
		getPoints(true, 640, points);
		println("got points %d", points.size());
		double avg_distance = 0.0;
		int count = 0;
		Vector3 centerOfMass = vector3(0.0, 0.0, 0.0);
		for (int i = 0; i < points.size(); i++) {
			Vector3 v = points.at(i).p;
			Vector2 projected(projectPoint(image.camPars, v));
			if (projected.x > tl.x + rect->width * SUB_WINDOW && projected.x
					< br.x - rect->width * SUB_WINDOW && projected.y > tl.y
					+ rect->height * SUB_WINDOW && projected.y < br.y
					- rect->height * SUB_WINDOW) {
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
			println("count=%d, dist=%f, x=%f, y=%f, z=%f, size ratio=%f",
					count, avg_distance, centerOfMass.x, centerOfMass.y,
					centerOfMass.z, sizeRatio);
			if (sizeRatio > 0.8 && sizeRatio < 1.2) {
				person = new Person();
				person->distance = avg_distance;
				person->angle = atan2(centerOfMass.y, centerOfMass.x);
				println("person distance=%f, angle=%f", person->distance,
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
			cvPutText(imgt, buf, cvPoint(tl.x + 1, (tl.y + br.y) / 2), &font,
					cvScalar(0.0, 255.0, 0.0, 255.0));
		} else {
			cvRectangle(imgt, tl, br, cvScalar(255.0, 0.0, 0.0, 255.0), 2);
		}
	}

	m_display.setImage(getComponentID(), imgt);
	//cvShowImage(getComponentID().c_str(), imgt);

	if (validDetection) {
	}

	//cvSaveImage(("./out/" + IntToStr((float)count++) +".bmp").c_str(), iplImage);

	// needed to make the window appear
	// (an odd behaviour of OpenCV windows!)
	cvWaitKey(10);

	//	if (fullbody_cascade) {
	//		full = cvHaarDetectObjects(img, fullbody_cascade, fullbody_storage,
	//				1.5, 2, CV_HAAR_DO_CANNY_PRUNING, cvSize(60, 120));
	//	}
	//
	//	println("full body cascade done");

	cvReleaseImage(&imgt);
	cvReleaseImage(&img);
	sleepComponent(100);
}

