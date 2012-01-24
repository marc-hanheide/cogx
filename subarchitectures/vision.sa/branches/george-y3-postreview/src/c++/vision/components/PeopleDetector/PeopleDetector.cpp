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

#include "PeopleDetector.hpp"

#define DONT_SAVE_FRAMES

using namespace std;
using namespace Laser;
using namespace cast;
using namespace VisionData;

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
cast::CASTComponentPtr newComponent() {
	return new PeopleDetector();
}
}

PeopleDetector::PeopleDetector() :
	deinterlacing(false), m_numDetectionAttempts(1), m_runContinuously(false) {
}

PeopleDetector::~PeopleDetector() {
}

void PeopleDetector::configure(
		const std::map<std::string, std::string> & config) {
	//println("configure");

	std::map<std::string, std::string>::const_iterator it;

	if ((it = config.find("--videoname")) != config.end()) {
		videoServerName = it->second;
	}

	if ((it = config.find("--camid")) != config.end()) {
		std::istringstream str(it->second);
		str >> camId;
	}

	if ((it = config.find("--deinterlace")) != config.end()) {
		std::istringstream str(it->second);
		str >> deinterlacing;
	} else
		deinterlacing = false;

	if ((it = config.find("--sleepForSync")) != config.end()) {
		std::istringstream str(it->second);
		str >> sleepForSync;
	} else
		sleepForSync = 0;

	if ((it = config.find("--trackerThreshold")) != config.end()) {
		std::istringstream str(it->second);
		str >> trackerThreshold;
	} else
		trackerThreshold = 1.0;

	if ((it = config.find("--removeAfterFrames")) != config.end()) {
		std::istringstream str(it->second);
		str >> removeAfterFrames;
	} else
		removeAfterFrames = 15;

	if ((it = config.find("--removeAfterDistance")) != config.end()) {
		std::istringstream str(it->second);
		str >> removeAfterDistance;
	} else
		removeAfterDistance = 1.0;

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

	if ((it = config.find("--laser-server-name")) != config.end()) {
		std::istringstream str(it->second);
		//str >> m_IceServerName;
	}
	//log("Using m_IceServerName=%s", m_IceServerName.c_str());

	if ((it = config.find("--laser-server-host")) != config.end()) {
		std::istringstream str(it->second);
		//str >> m_IceServerHost;
	}
	//log("Using m_IceServerHost=%s", m_IceServerHost.c_str());

	if ((it = config.find("--laser-server-port")) != config.end()) {
		std::istringstream str(it->second);
		//str >> m_IceServerPort;
	}
	//log("Using m_IceServerPort=%d", m_IceServerPort);


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

	// sanity checks: Have all important things be configured? Is the
	// configuration consistent?
	if (videoServerName.empty()) {
		throw runtime_error(exceptionMessage(__HERE__, "no video server name given"));
	}

}

void PeopleDetector::start() {

	// we want to receive PeopleDetectionCommand
	addChangeFilter(createGlobalTypeFilter<PeopleDetectionCommand> (cdl::ADD),
			new MemberFunctionChangeReceiver<PeopleDetector> (this,
					&PeopleDetector::receiveDetectionCommand));

	// get connection to the video server
	m_videoServer = getIceServer<Video::VideoInterface> (videoServerName);

	// and the same for the laser server
	m_laserServer = LaserClientUtils::getServerPrx(*this, "localhost",
			cast::cdl::CPPSERVERPORT, "LaserServer");

	upper_storage = cvCreateMemStorage(0);
	face_storage = cvCreateMemStorage(0);
	fullbody_storage = cvCreateMemStorage(0);
}

void PeopleDetector::receiveDetectionCommand(
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

// Deinterlaces an Opencv IplImage. It assumes the image is in colour RGB format.
// Done in-place, using a fast blending method.
void deinterlaceImg(IplImage * img) {
	for (int y = 0; y < img->height - 2; y += 2) {
		for (int x = 0; x < img->width; x++) {
			CvScalar s1, s2;
			s1 = cvGet2D(img, y, x);
			s2 = cvGet2D(img, y + 2, x);

			s1.val[0] = (s1.val[0] + s2.val[0]) / 2;
			s1.val[1] = (s1.val[1] + s2.val[1]) / 2;
			s1.val[2] = (s1.val[2] + s2.val[2]) / 2;

			cvSet2D(img, y + 1, x, s1);
		}
	}
}

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

void PeopleDetector::runComponent() {
	if (m_runContinuously) {
		while (isRunning()) {
			lockComponent();
			runDetection();
			unlockComponent();
		}
	}
}

void PeopleDetector::runDetection() {
	log("running detection");

	static int cnt = 0;

	static vector<PersonRecord> detections;

	//log("Frame %d",cnt);

	// Capture an image from the camera
	Video::Image image;
	m_videoServer->getImage(camId, image);
	//log("got image");

	IplImage * imgt = convertImageToIpl(image);
	//log("converted");

	// If the image is interlaced, this removes the horizontal
	// stripes fast blending. This is needed on the b21 because
	// interlacing is messing with the detectors.
	if (deinterlacing) {
		deinterlaceImg(imgt);
	}

	//log("deinterlaced");

	// If this component is running on a different machine from Player,
	// network latencies can desync the two so the detection/tracking
	// can be messed up. Use this with care, however, because the longer
	// you sit around waiting the lower your total fps.
	sleepComponent(sleepForSync);

	// Pull laser scan.
	Laser::Scan2d scan = m_laserServer->pullScan2d();

	// Convert image to greyscale for processing.
	CvSize img_sz = cvGetSize(imgt);
	IplImage * img = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);
	cvCvtColor(imgt, img, CV_RGB2GRAY);
	cvReleaseImage(&imgt);

	//log("converted to greysalce");

	CvSeq * face = NULL, *full = NULL;

	// Run the detectors.
	// The fullbody detector looks for people who are further away and their full body is visible, so it
	// can't find things that are very close. The face detector compensates for that by looking only for
	// large faces that are close by.
	// NB: This code is meant to work with the latest stable of OpenCV. Their latest svn code also supports
	// a flag CV_HAAR_DO_ROUGH_SEARCH which should speed up processing in this step, which you are free to
	// add when that code is out and about (you know, | it with the canny flag).
	if (face_cascade) {
		face = cvHaarDetectObjects(img, face_cascade, face_storage, 1.5, 3,
				CV_HAAR_DO_CANNY_PRUNING, cvSize(60, 60));
	}

	//println("face cascade done");


	if (fullbody_cascade) {
		full = cvHaarDetectObjects(img, fullbody_cascade, fullbody_storage,
				1.5, 2, CV_HAAR_DO_CANNY_PRUNING, cvSize(60, 120));
	}

	//println("full body cascade done");

	if (!scan.ranges.empty()) {
		// Original coordinates from top-down view from laser.
		vector<double> originalX;
		vector<double> originalY;

		// Transformed coordinates, with X corresponding to a position on screen and Z being a depth value.
		vector<double> seqX;
		vector<double> seqZ;

		// Projection parameters.
		int screenWid = img->width;
		double xscale = screenWid * 1.0;
		int xcentre = int(screenWid * 0.525);

		for (size_t i = 0; i < scan.ranges.size(); i++) {
			// Angle that corresponds to position in scan array i.
			double angle = scan.startAngle + scan.angleStep * i;
			// Transform from polar (laser) to cartesian (3d) coordinate system.
			double y = sin(angle), x = cos(angle);
			// Keep in mind that range data is reversed for some reason.
			double z = scan.ranges[scan.ranges.size() - i - 1];

			// Project data onto 2d plane corresponding to the camera FOV.
			double sx = (y * z * xscale) / (x * z) + xcentre;

			seqX.push_back(sx);
			seqZ.push_back(z);

			originalX.push_back(y * z);
			originalY.push_back(x * z);
		}

		// zbuffer with a value for each vertical scanline in the image, with corresponding
		// bit vector indicating which values have been set and the index of the original
		// values before projection.
		vector<double> zbuffer;
		vector<bool> zbufferBits;
		vector<int> zbufferOrigin;
		for (int i = 0; i < img->width; i++) {
			zbuffer.push_back(0.0);
			zbufferBits.push_back(false);
			zbufferOrigin.push_back(0);
		}

		// Paint the depth values into the zbuffer.
		for (size_t i = 0; i < seqX.size(); i++) {
			// Watch out for things that project outside the FOV!
			if (seqX[i] > 0 && seqX[i] < img->width) {
				// Only update the zbuffer with the things closest to the laser
				// to account for occlusions (unlikely but possible).
				if ((!zbufferBits[int(seqX[i])]) || seqZ[i]
						< zbuffer[int(seqX[i])]) {
					zbufferBits[int(seqX[i])] = true;
					zbuffer[int(seqX[i])] = seqZ[i];
					zbufferOrigin[int(seqX[i])] = i;
				}
			}
		}

		// Propagate known data into spaces where nothing was projected to get
		// a contiguous zbuffer.
		double d = 0.0;
		int o = 0;
		for (int i = 0; i < img->width; i++) {
			if (zbufferBits[i]) {
				d = zbuffer[i];
				o = zbufferOrigin[i];
			} else {
				zbuffer[i] = d;
				zbufferOrigin[i] = o;
			}
		}

#ifndef DONT_SAVE_FRAMES
		// This just paints the laser visualisation into the image.
		for (int x = 0; x < img->width; x++)
		{
			// Truncating after 15m to actually make things visible
			// to the human eye (assuming greyscale).
			if (zbuffer[x] > 15) d = 0;
			else d = 15 - zbuffer[x];
			d /= 15 / 1.5;

			for (int y = 0; y < img->height; y++)
			{
				CvScalar s, s2;
				s = cvGet2D(img, y, x);

				s2.val[0] = limit(d * s.val[0], 0, 255);

				cvSet2D(img, y, x, s2);
			}
		}
#endif

		// This will contain PersonRecords for all the new inputs from the detectors, minus those
		// that are obviously already being tracked.
		vector<PersonRecord> newDetections;

		// Loop through full-body detections and find the corresponding Z value from the laser
		// data. It is assumed that the shape closest to the camera within the detection window
		// is in fact the person to be tracked.
		for (int i = 0; full && i < full->total; i++) {
			PersonRecord r;

			CvRect* rect = (CvRect*) cvGetSeqElem(full, i);

			double closest = numeric_limits<double>::max();
			int index = 0;

			// Ok, the detection window needs to be made narrower by a constant factor to avoid sticking onto
			// things around the person (assuming the detector mostly finds fullbody regions this should work
			// pretty well most of the time).
			for (int j = int(rect->x + rect->width / 2.5); j < int(rect->x
					+ rect->width - rect->width / 2.5); j++) {
				if (closest > zbuffer[j]) {
					closest = zbuffer[j];
					index = j;
				}
			}

			// Refer to the original (cartesian) values from the laser instead of the projected values.
			r.laserZ = originalX[zbufferOrigin[index]];
			r.laserX = originalY[zbufferOrigin[index]];
			r.scanIndex = zbufferOrigin[index];

			// To avoid slowing processing later, check if this detection is a duplicate of something else
			// already being tracked. Only add this to the things to track if it really is new.
			// Using a constant (1) because this is a rough check -- doing it again later.
			bool found = false;
			for (size_t j = 0; j < newDetections.size(); j++) {
				double diffX = r.laserX - newDetections[j].laserX;
				double diffZ = r.laserZ - newDetections[j].laserZ;

				if (sqrt(diffX * diffX + diffZ * diffZ < 1)) {
					found = true;
				}
			}

			if (!found)
				newDetections.push_back(r);
		}

		// Same exact thing for face detections. It is assumed that a face must also have a body to
		// go with it, which is more or less vertical from the face position.
		for (int i = 0; face && i < face->total; i++) {
			PersonRecord r;

			CvRect* rect = (CvRect*) cvGetSeqElem(face, i);

			double closest = numeric_limits<double>::max();
			int index = 0;

			// Stretch the face by a constant amount to make sure we don't get stuck on the background
			// between the person's legs.
			for (int j = int(rect->x - rect->width / 2.5); j < int(rect->x
					+ rect->width + rect->width / 2.5); j++) {
				if (closest > zbuffer[j]) {
					closest = zbuffer[j];
					index = j;
				}
			}

			r.laserZ = originalX[zbufferOrigin[index]];
			r.laserX = originalY[zbufferOrigin[index]];
			r.scanIndex = zbufferOrigin[index];

			bool found = false;
			for (size_t j = 0; j < newDetections.size(); j++) {
				double diffX = r.laserX - newDetections[j].laserX;
				double diffZ = r.laserZ - newDetections[j].laserZ;

				if (sqrt(diffX * diffX + diffZ * diffZ < 1)) {
					found = true;
				}
			}

			if (!found)
				newDetections.push_back(r);
		}

		// This will contain all the objects being tracked that are still active
		// and is a temp store for stuff in detections.
		vector<PersonRecord> detectionsCopy;

		// Loop through the old detections, do tracking or update using detector data.
		for (size_t i = 0; i < detections.size(); i++) {
			// HACK: If something being tracked strays and gets stuck someplace, remove
			// it if it doesn't move more than 1m in some number of frames.
			// We could work around this by learning the background (the zbuffer).
			PersonRecord temp = detections[i];
			if (temp.distanceMovedCount > removeAfterFrames) {
				double d = temp.distanceMoved;
				temp.distanceMovedCount = 0;
				temp.distanceMoved = 0;
				detections[i] = temp;
				if (d < removeAfterDistance)
					continue;
			}

			bool isBeingTracked = false;
			double minVal = numeric_limits<double>::max();
			int minJ = -1;

			// Use the latest laser data to track an old detection and update its position
			// so it's easier to match to the new detections if needed.
			for (size_t j = 0; j < originalX.size(); j++) {
				double tempX = originalY[j] - detections[i].laserX;
				double tempZ = originalX[j] - detections[i].laserZ;

				// Estimate new position using older direction and speed.
				if (detections[i].deltaX > numeric_limits<double>::min()) {
					tempX -= detections[i].deltaX / 2;
					tempZ -= detections[i].deltaZ / 2;
				}

				// Looking for the nearest point.
				if (sqrt(tempX * tempX + tempZ * tempZ) < minVal) {
					minVal = sqrt(tempX * tempX + tempZ * tempZ);
					minJ = j;
				}
			}

			double tempX = originalY[minJ] - detections[i].laserX;
			double tempZ = originalX[minJ] - detections[i].laserZ;

			if (detections[i].deltaX > numeric_limits<double>::min()) {
				tempX -= detections[i].deltaX;
				tempZ -= detections[i].deltaZ;
			}

			// Actually do the updating.
			if (sqrt(tempX * tempX + tempZ * tempZ) < trackerThreshold) {
				PersonRecord r = detections[i];

				r.deltaX = originalY[minJ] - r.laserX;
				r.deltaZ = originalX[minJ] - r.laserZ;

				r.laserX = originalY[minJ];
				r.laserZ = originalX[minJ];
				r.scanIndex = minJ;

				r.distanceMoved += sqrt(r.deltaX * r.deltaX + r.deltaZ
						* r.deltaZ);
				r.distanceMovedCount++;

				detections[i] = r;
				isBeingTracked = true;
			}

			bool hasNewDetection = false;
			// Using the latest tracking positions, try to match detections to known objects.
			for (size_t j = 0; j < newDetections.size(); j++) {
				double tempX = newDetections[j].laserX - detections[i].laserX;
				double tempZ = newDetections[j].laserZ - detections[i].laserZ;

				// Ignoring direction and speed info as it seems to make things worse (?)
				/*if (detections[i].deltaX > numeric_limits<double>::min())
				 {
				 tempX -= detections[i].deltaX / 2;
				 tempZ -= detections[i].deltaZ / 2;
				 }*/

				// The first matching detection within a radius is associated with the tracked object
				// and eliminated so it can't be used again.
				if (sqrt(tempX * tempX + tempZ * tempZ) < trackerThreshold) {
					PersonRecord r = detections[i];

					r.deltaX = newDetections[j].laserX - r.laserX;
					r.deltaZ = newDetections[j].laserZ - r.laserZ;

					r.laserX = newDetections[j].laserX;
					r.laserZ = newDetections[j].laserZ;
					r.scanIndex = newDetections[j].scanIndex;

					r.distanceMoved += sqrt(r.deltaX * r.deltaX + r.deltaZ
							* r.deltaZ);
					r.distanceMovedCount++;

					detectionsCopy.push_back(r);
					hasNewDetection = true;

					// Update blackboard with new position.
					VisionData::PersonPtr data = new VisionData::Person(
							scan.startAngle + scan.angleStep * r.scanIndex,
							scan.ranges[scan.ranges.size() - r.scanIndex - 1],
							r.laserX, r.laserZ, r.deltaX, r.deltaZ, 1.0);
					overwriteWorkingMemory(r.castID, data);

					newDetections.erase(newDetections.begin() + j);
					break;
				}
			}

			// To avoid duplicates, only add the updated tracking info if that object has not been
			// re-detected. If it has not been re-detected and the tracker didn't find a match
			// this is not added to the new list of known detections.
			if ((!hasNewDetection) && isBeingTracked) {
				PersonRecord r = detections[i];

				// Update blackboard with new position.
				VisionData::PersonPtr data = new VisionData::Person(
						scan.startAngle + scan.angleStep * r.scanIndex,
						scan.ranges[scan.ranges.size() - r.scanIndex - 1],
						r.laserX, r.laserZ, r.deltaX, r.deltaZ, 1.0);
				overwriteWorkingMemory(r.castID, data);

				detectionsCopy.push_back(r);
			}
		}

		// Necessary evil: loop through old detections and weed out stuff that needs to be deleted
		// from the blackboard (i.e. things that are no longer present in the tracker).
		for (size_t i = 0; i < detections.size(); i++) {
			PersonRecord r = detections[i];
			bool found = false;

			for (size_t j = 0; j < detectionsCopy.size(); j++) {
				if (r.castID == detectionsCopy[j].castID) {
					found = true;
					break;
				}
			}

			if (!found) {
				deleteFromWorkingMemory(r.castID);
			}
		}

		detections = detectionsCopy;

		// Any new detections that did not seem to correspond to existing objects are added to the
		// detections list.
		for (size_t j = 0; j < newDetections.size(); j++) {
			// New detections that are valid get a new ID from cast.
			PersonRecord r = newDetections[j];
			r.castID = newDataID();
			detections.push_back(r);

			VisionData::PersonPtr data = new VisionData::Person(scan.startAngle
					+ scan.angleStep * r.scanIndex,
					scan.ranges[scan.ranges.size() - r.scanIndex - 1],
					r.laserX, r.laserZ, r.deltaX, r.deltaZ, 1.0);
			addToWorkingMemory(r.castID, data);
		}

#ifndef DONT_SAVE_FRAMES
		// Updating visualisation with objects being tracked.
		for (size_t i = 0; i < detections.size(); i++)
		{
			int pos = int((detections[i].laserZ * xscale) / (detections[i].laserX) + xcentre);

			if (pos > 0 && pos < img->width)
			for (int y = 0; y < img->height; y++)
			{
				CvScalar s2;

				s2.val[0] = 255;

				cvSet2D(img, y, pos, s2);
			}
		}

		// Draw faces on visualisation if they exist.
		for (int i = 0; face && i < face->total; i++)
		{
			CvRect* r = (CvRect*)cvGetSeqElem(face, i);

			CvPoint pt1, pt2;
			pt1.x = r->x;
			pt2.x = r->x + r->width;
			pt1.y = r->y;
			pt2.y = r->y + r->height;

			cvRectangle(img, pt1, pt2, CV_RGB(0,255,0), 3, 8, 0);
		}

		// Draw full bodies on visualisation if they exist.
		for (int i = 0; full && i < full->total; i++)
		{
			CvRect* r = (CvRect*)cvGetSeqElem(full, i);

			CvPoint pt1, pt2;
			pt1.x = r->x;
			pt2.x = r->x + r->width;
			pt1.y = r->y;
			pt2.y = r->y + r->height;

			cvRectangle(img, pt1, pt2, CV_RGB(0,0,255), 3, 8, 0);
		}

		// Save images to disk so they can be viewed later.
		cvSaveImage(("./out/" + IntToStr((float)cnt) +".bmp").c_str(), img);
#endif

		// update frame counter (not really necessary)
		cnt++;
	} else {
		std::cout << "Pulled EMPTY scan with time " << scan.time << std::endl;
	}

	cvReleaseImage(&img);

}

