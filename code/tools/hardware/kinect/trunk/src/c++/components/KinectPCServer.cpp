/**
 * @file KinectPCServer.cpp
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Point cloud server for the kinect sensor.
 */

#define EIGEN2_SUPPORT

#include <cmath>
#include <algorithm>
#include <Eigen/LeastSquares>
#include <Eigen/Geometry>
#include <cast/core/CASTUtils.hpp>
#include "KinectPCServer.h"
#include <highgui.h>
/**
 * The function called to create a new instance of our component.
 */
extern "C" {
cast::CASTComponentPtr newComponent() {
	return new cast::KinectPCServer();
}
}

namespace cast {

using namespace std;
using namespace cogx;
using namespace cogx::Math;
using namespace cast::cdl;

KinectPCServer::KinectPCServer() {
  for (int i = 0; i < N_PLANES; i++) {
    fovPlanes[i] = NULL;
    senses[i] = 0;
  }
  personDetectServer=new PersonDetectServerI(this);
}

KinectPCServer::~KinectPCServer() {
	delete kinect;

  /* Delete the view cone planes */
  for (int i = 0; i < N_PLANES; i++)
    if (fovPlanes[i])
      delete fovPlanes[i];
}

/**
 * @brief Get video resolution for given camera.
 * @param camIdx which camera
 * @param size video resolution
 */
void KinectPCServer::getResolution(int camIdx, CvSize &size) {
	kinect->GetColorVideoSize(size);
}

/**
 * @brief Set resolution for a given camera
 * @param camIdx which camera
 * @param size requested video resolution, on exit contains the actually
 *        set resolution, which might differ dependig on the cameras capabilities
 * @return true if the requested resolution could be set, false if another
 *        reslution was chosen
 */
bool KinectPCServer::setResolution(int camIdx, CvSize &size) {
	log("setResolution: warning: Not yet implemented! Defined by kinect camera calibration file!\n");
	return false;
}

/**
 * @brief Configure the component
 */
void KinectPCServer::configure(const map<string, string> & _config)
		throw (runtime_error) {
	// configure the point cloud server
	PointCloudServer::configure(_config);

	map<string, string>::const_iterator it;

  m_createViewCone = false;
  if (_config.find("--create-viewcone") != _config.end()) {
    m_createViewCone = true;
  }


  m_detectPersons = false;
  if (_config.find("--detect-persons") != _config.end()) {
	  m_detectPersons = true;
  }

	if ((it = _config.find("--kconfig")) != _config.end()) {
		istringstream str(it->second);
		str >> kinectConfig;
	} else{
		throw runtime_error(exceptionMessage(__HERE__, "no kinect config file (kconfig) specified."));
	}
		// init kinect hardware driver
	 CvSize size;
	 const char* name = kinectConfig.c_str();
	 kinect = new Kinect::Kinect(name);
	 kinect->GetColorVideoSize(size);
	 captureSize.width = size.width;
	 captureSize.height = size.height;
     kinect->StartCapture(0); 	// start capturing
     depthGenerator = kinect::getDepthGenerator();
     imageGenerator = kinect::getImageGenerator();
     userGenerator = kinect::getUserGenerator();
     m_saveToFile = false;
     if ((it = _config.find("--save-to-file")) != _config.end()) {
       m_saveToFile = true;

       if ((it = _config.find("--save-directory")) != _config.end()) {
       	std::istringstream str(it->second);
        str >> m_saveDirectory;
    	//check if the last char is / if yes remove
        if (m_saveDirectory[m_saveDirectory.size()-1] == '/'){
       	 m_saveDirectory.erase(m_saveDirectory.size()-1);
        }
        log("Will be saving scans to %s", m_saveDirectory.c_str());
       }
       else
       {
       	log("You haven't specified a save directory! (usage: --save-directory)");
       	abort();
       }
     }
 
     m_displayImage = false;
     if ((it = _config.find("--display-rgb")) != _config.end()) {
     log("Will display Kinect RGB image \n");
       m_displayImage= true;
 }
     m_lastframe = -1;
  log("Capturing from kinect sensor started.");


  registerIceServer<kinect::slice::PersonDetectorInterface, PersonDetectServerI>(personDetectServer);
  println("PersonDetectServer registered");
}

kinect::slice::PersonsDict PersonDetectServerI::getPersons(const Ice::Current& cur) {
	kinect::slice::PersonsDict persons;
	return pcSrv->detectPersons();
}

/**
* @brief Configure the component
*/
void KinectPCServer::start()
{
  PointCloudServer::start();
}

void KinectPCServer::runComponent() {
	log("I am running ");
	if(m_displayImage){
		log("Displaying window");
		cvNamedWindow("Kinect RGB",CV_WINDOW_AUTOSIZE);
	}
	cvWaitKey(100);

	while(isRunning()) {
		if (m_detectPersons) {
			detectPersons();
		}
		if (m_saveToFile) {
			saveNextFrameToFile();
		}
		sleepComponent(50);
	}
}

::kinect::slice::PersonsDict KinectPCServer::detectPersons() {

	::kinect::slice::PersonsDict persons;
	if (userGenerator==NULL || !userGenerator->IsValid()) {
		println("we don't have a valid userGenerator");
		return persons;
	}
  if (!suspendReading) {
    kinect->NextFrame();
  }
	int count=userGenerator->GetNumberOfUsers();
	XnUserID aUsers[count];
	XnUInt16 nUsers=count;

	userGenerator->GetUsers(aUsers, nUsers);
	for (int i=0; i<count; i++) {
		xn::SceneMetaData smd;
		userGenerator->GetUserPixels(aUsers[i], smd);

		int xRes=smd.LabelMap().XRes();
		int yRes=smd.LabelMap().YRes();
		long pixelCount=0;
		for (int y=0;y<yRes; y++) {
			for (int x=0;x<xRes; x++) {
				int v=smd.LabelMap()(x,y);
				if (v==aUsers[i])
					pixelCount++;
			}
		}
		double pixelRatio = ((double) pixelCount)/(xRes*yRes);
		log("user %d xRes=%d, yRes=%d pixelCount=%f", aUsers[i], xRes, yRes, pixelRatio);
		if (pixelRatio > RELATIVE_MINIMUM_PERSON_AREA) {
			kinect::slice::KinectPersonPtr person = new kinect::slice::KinectPerson;
			person->size=pixelCount;
			persons[aUsers[i]] = person;

		}
	}
	log("number of users in image: %d", persons.size());
	return persons;
}

void KinectPCServer::saveNextFrameToFile() {
  kinect->NextFrame();
  if(kinect->frameNumber == m_lastframe){
    return;
  }
  IplImage* rgb_data = new IplImage(kinect->rgbImage);
  // Doing new IplImage(kinect->depImage); actually causes the depth map stored as a binary image for some reason
  if(m_displayImage){
    cvShowImage("Kinect RGB",rgb_data);
    cvWaitKey(5);
  }
  IplImage* depth_data = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
  char buf[256];
  CASTTime timeNow = getCASTTime();
  sprintf(buf, "%s/frame_%d_rgb_%ld_%ld.bmp", m_saveDirectory.c_str(), kinect->frameNumber,
      (long int)timeNow.s, (long int)timeNow.us);

  cvSaveImage(buf, rgb_data);

  short*d = kinect->depImage.ptr<short>(0);	
  for(int i = 0; i < kinect->depImage.rows*kinect->depImage.cols; i++)
  {
    short value = d[i]/16;
    char value_pt1 = d[i]>>8;
    char value_pt2 = d[i]&0xFF;
    depth_data->imageData[3*i+0]=(char)value_pt1;
    depth_data->imageData[3*i+1]=(char)value_pt2;
    depth_data->imageData[3*i+2]=(char)value;
  }
  char buf2[256];
  sprintf(buf2,"%s/frame_%d_depth_%ld_%ld.bmp", m_saveDirectory.c_str(), kinect->frameNumber,
      (long int)timeNow.s, (long int)timeNow.us);
  log("Saving Kinect frame # %d",kinect->frameNumber);
  cvSaveImage(buf2, depth_data);
  m_lastframe = kinect->frameNumber;
  delete rgb_data;
  cvReleaseImage(&depth_data);
}

bool KinectPCServer::createViewCone()
{
  kinect::changeRegistration(0);
  if (!suspendReading) {
    kinect->NextFrame();
  }
  
  cv::Mat_<cv::Point3f> cloud;
  cv::Mat_<cv::Point3f> colCloud;
  kinect->Get3dWorldPointCloud(cloud, colCloud);

  Pose3 global_kinect_pose, zeroPose;
  setIdentity(zeroPose);
  setIdentity(global_kinect_pose);
  transform(lastValidCamPose, zeroPose, global_kinect_pose);

  for (int i = 0; i < N_PLANES; i++) {
    if (fovPlanes[i])
      continue;
    cv::Mat_<cv::Point3f> colOrRow;
    Eigen::Vector3d axis(0, 0, 0);
    switch (i) {
    case PLANE_LEFT:
      /* Grab left column */
       colOrRow = cloud.col(0);
       axis = Eigen::Vector3d(0, 1, 0);
      break;
    case PLANE_TOP:
      /* Grab top row */
      colOrRow = cloud.row(0);
      axis = Eigen::Vector3d(0, 0, 1);
      break;
    case PLANE_RIGHT:
      /* Grab right column (NOTE: ignore deadzone of 8 pixels) */
      colOrRow = cloud.col(cloud.size().width - 9);
      axis = Eigen::Vector3d(0, 1, 0);
      break;
    case PLANE_BOTTOM:
      /* Grab bottom row */
      colOrRow = cloud.row(cloud.size().height - 1);
       axis = Eigen::Vector3d(0, 0, 1);
      break;
    }
    std::vector<cv::Point3f> cvPoints(colOrRow.begin(), colOrRow.end());
    fovPlanes[i] = createPlane(cvPoints, global_kinect_pose);
    if (fovPlanes[i] != NULL) {
      /* Compensate for the variable direction of the plane normals */
      senses[i] = 2 * (std::acos(axis.dot(fovPlanes[i]->normal())) > M_PI/2) - 1;
    }
  }

  return fovPlanes[0] && fovPlanes[1] && fovPlanes[2] && fovPlanes[3];
}

// ########################## Point Cloud Server Implementations ########################## //
void KinectPCServer::getPoints(bool transformToGlobal, int imgWidth, vector<PointCloud::SurfacePoint> &points, bool complete)
{
  lockComponent();

  if (!suspendReading) {
    kinect->NextFrame();
    lastValidCamPose=camPars[0].pose;
  }
  
  cv::Mat_<cv::Point3f> cloud;
  cv::Mat_<cv::Point3f> colCloud;
  kinect->Get3dWorldPointCloud(cloud, colCloud);

  Pose3 global_kinect_pose, zeroPose;
  setIdentity(zeroPose);
  setIdentity(global_kinect_pose);
  if(transformToGlobal)
    transform(lastValidCamPose, zeroPose, global_kinect_pose);

  // copy clouds to points-vector (dense!)
  int scale = imgWidth == 0 ? 1 : cloud.size().width / imgWidth;
  for (int row=0; row < cloud.size().height; row += scale)   /// SLOW conversion
  {
    for (int col=0; col < cloud.size().width; col += scale)
    {
      PointCloud::SurfacePoint pt;
      pt.p.x = cloud.at<cv::Point3f>(row, col).x;
      pt.p.y = cloud.at<cv::Point3f>(row, col).y;
      pt.p.z = cloud.at<cv::Point3f>(row, col).z;

      /* Check point for validity */
      if (pt.p.x == FLT_MAX && pt.p.y == FLT_MAX && pt.p.z == FLT_MAX)
      {
        /* If no data is available add (0,0,0) as specified in PointCloudServer */
        if (complete) {
          pt.p.x = 0;
          pt.p.y = 0;
          pt.p.z = 0;
        }
        /* Or if we don't care about missing data ignore the point */
        else
          continue;
      }

      pt.c.r = colCloud.at<cv::Point3f>(row, col).z;
      pt.c.g = colCloud.at<cv::Point3f>(row, col).y;
      pt.c.b = colCloud.at<cv::Point3f>(row, col).x;

      if(transformToGlobal)
        pt.p = transform(global_kinect_pose, pt.p);   // now get from kinect cam coord sys to global coord sys

      points.push_back(pt);
    }
  }
  unlockComponent();
}


void KinectPCServer::getRectImage(int side, int imgWidth, Video::Image& image)
{
  lockComponent();

  double scaleFactor = 1.;
  IplImage *rgbImage;
  kinect->GetColorImage(&rgbImage);

  if(imgWidth != rgbImage->width)
  {
    IplImage *resized;
    scaleFactor = (double) rgbImage->width / (double) imgWidth;
    resized = cvCreateImage(cvSize(imgWidth, (int) (rgbImage->height/scaleFactor)), IPL_DEPTH_8U, 3);
    cvResize(rgbImage, resized);
    convertImageFromIpl(resized, image);
    cvReleaseImage(&rgbImage);
    cvReleaseImage(&resized);
  }
  else
  {
    convertImageFromIpl(rgbImage, image);
    cvReleaseImage(&rgbImage);
  }
  
  initCameraParameters(image.camPars);
  image.camPars.id = camIds[0];
  image.camPars.width = imgWidth;
  image.camPars.height = imgWidth*3/4;
  image.camPars.fx = camPars[0].fx/scaleFactor;
  image.camPars.fy = camPars[0].fy/scaleFactor;
  image.camPars.cx = camPars[0].cx/scaleFactor;
  image.camPars.cy = camPars[0].cy/scaleFactor;
 
  Pose3 global_pose, zeroPose;
  setIdentity(zeroPose);
  transform(camPars[0].pose, zeroPose, global_pose);
  image.camPars.pose = global_pose;
  image.camPars.time = getCASTTime();

  unlockComponent();
}

bool KinectPCServer::getCameraParameters(Ice::Int side /*not used*/, Video::CameraParameters& _camPars)
{

  lockComponent(); // TODO: CASTComponent::Lock lock(this);

  // TODO: we don't need the image! This is an expensive way to obtain the width
  IplImage *rgbImage;
  kinect->GetColorImage(&rgbImage);
  int imgWidth = rgbImage->width;
  double scaleFactor = camPars[0].width / imgWidth;
  cvReleaseImage(&rgbImage);

  initCameraParameters(_camPars);
  _camPars.id = camIds[0];
  _camPars.width  = imgWidth;
  _camPars.height = imgWidth * 3/4;
  _camPars.fx = camPars[0].fx/scaleFactor;
  _camPars.fy = camPars[0].fy/scaleFactor;
  _camPars.cx = camPars[0].cx/scaleFactor;
  _camPars.cy = camPars[0].cy/scaleFactor;

  Pose3 global_pose, zeroPose;
  setIdentity(zeroPose);
  transform(camPars[0].pose, zeroPose, global_pose);
  _camPars.pose = global_pose;
  _camPars.time = getCASTTime();

  unlockComponent(); // TODO: remove

  return true;
}

void KinectPCServer::getDisparityImage(int imgWidth, Video::Image& image)
{
  printf("KinectPCServer::getDisparityImage: Warning: Not yet implemented!\n");
/*
	lockComponent();
	convertImageFromIpl(disparityImg, image);
	unlockComponent();
*/
}

void KinectPCServer::getRangePoints(Laser::Scan2d &KRdata)
{
  printf("KinectPCServer::getRangePoints: Warning: Not yet implemented!\n");
}

void KinectPCServer::getDepthMap(cast::cdl::CASTTime &time, vector<int>& depth)
{
  lockComponent();
  //kinect->NextFrame();
  //cv::Mat RGB;
  //cv::Mat DEPTH;
  //kinect->GetImages(RGB, DEPTH);
  //kinect::readFrame();  // read next frame
  kinect::changeRegistration(0);
  const DepthMetaData* pDepthMD = kinect->getNextDepthMD();
  time = getCASTTime();

  //for(int row=0;row < DEPTH.rows;row++)
  for(size_t row=0;row < pDepthMD->YRes();row++)
  {
    for(size_t col=0;col < pDepthMD->XRes();col++)
    {
      //depth.push_back( int ( DEPTH.ptr<short>(row)[(640-col-1)] ) );
      //depth.push_back( int ( DEPTH.ptr<short>(row)[col] ) );
      depth.push_back( (*pDepthMD)(col,row) );
    }
  }
  unlockComponent();
}

void KinectPCServer::receiveImages(const vector<Video::Image>& images)
{
//   lockComponent();
  printf("KinectPCServer::receiveImages: Warning: Not yet implemented!\n");
  //stereoProcessing();
//   unlockComponent();
}

void KinectPCServer::receiveCameraParameters(const cdl::WorkingMemoryChange & _wmc)
{
	PointCloudServer::receiveCameraParameters(_wmc);
	
	if (m_createViewCone)
	{
		/* Delete old view cone planes */
		for (int i = 0; i < N_PLANES; i++) {
			senses[i] = 0;
			if (fovPlanes[i]) {
				delete fovPlanes[i];
				fovPlanes[i] = NULL;
			}
		}
		bool hasAllPlanes = createViewCone();
		if (!hasAllPlanes)
			log("Failed to get a complete view cone!");
	}
}

bool KinectPCServer::isPointInViewCone(const Vector3& point)
{
  lockComponent();
  /* Make sure we have a complete view cone */
  for (int i = 0; i < N_PLANES; i++) {
    if (fovPlanes[i] == NULL) {
      unlockComponent();
      return false;
    }
  }

  Eigen::Vector3d p(point.x, point.y, point.z);

  bool inView = senses[PLANE_LEFT] * fovPlanes[PLANE_LEFT]->signedDistance(p) >= 0 &&
    senses[PLANE_TOP] * fovPlanes[PLANE_TOP]->signedDistance(p) >= 0 &&
    senses[PLANE_RIGHT] * fovPlanes[PLANE_RIGHT]->signedDistance(p) <= 0 &&
    senses[PLANE_BOTTOM] * fovPlanes[PLANE_BOTTOM]->signedDistance(p) <= 0;
  unlockComponent();

  return inView;
}

class CvToGlobal {
  Pose3& global_pose;
  public:
    CvToGlobal(Pose3& pose) : global_pose(pose) {}
    Vector3 operator()(const cv::Point3f& p) {
      Vector3 v;
      v.x = p.x;
      v.y = p.y;
      v.z = p.z;
      v = transform(global_pose, v);
      return v;
    }
};
class Vector3ToVector3d {
  public:
    Eigen::Vector3d operator()(const Vector3& p) {
      return Eigen::Vector3d(p.x, p.y, p.z);
    }
};
class InvalidPointFilter {
  public:
  bool operator()(const cv::Point3f& p) const {
    return p.x == FLT_MAX || p.y == FLT_MAX || p.z == FLT_MAX;
  }
};

Eigen::Hyperplane<double, 3>* KinectPCServer::createPlane(std::vector<cv::Point3f>& cvPoints, Pose3& global_kinect_pose) {
  Eigen::Hyperplane<double, 3>* plane = NULL;

  /* Filter invalid points */
  cvPoints.erase(std::remove_if(cvPoints.begin(), cvPoints.end(), InvalidPointFilter()), cvPoints.end());
  /* Transform to global coordinates */
  std::vector<Vector3> globalPoints(cvPoints.size());
  std::transform(cvPoints.begin(), cvPoints.end(), globalPoints.begin(), CvToGlobal(global_kinect_pose));
  /* Include the kinect position */
  cvPoints.push_back(cv::Point3f(global_kinect_pose.pos.x, global_kinect_pose.pos.y, global_kinect_pose.pos.z));
  /* Transform to Eigen::Vector3d */
  std::vector<Eigen::Vector3d> points(globalPoints.size());
  std::transform(globalPoints.begin(), globalPoints.end(), points.begin(), Vector3ToVector3d());

  if (points.size() >= 3) {
    plane = new Eigen::Hyperplane<double, 3>(3);
    // create a vector of pointers to the points (weird Eigen API...)
    std::vector<Eigen::Vector3d*> points_ptrs(points.size());
    for(unsigned int k=0; k<points.size(); ++k) points_ptrs[k] = &points[k];

    /* Fit a plane to the points */
    double soundness;
    Eigen::fitHyperplane(points.size(), &(points_ptrs[0]), plane, &soundness);
    /* FIXME: Check soundness! */
  }
  return plane;
}

}

