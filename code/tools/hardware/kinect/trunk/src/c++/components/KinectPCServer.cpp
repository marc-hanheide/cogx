/**
 * @file KinectPCServer.cpp
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Point cloud server for the kinect sensor.
 */

#include <cmath>
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
	log("I am created.");
}

KinectPCServer::~KinectPCServer() {
	delete kinect;
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
	log(
			"setResolution: warning: Not yet implemented! Defined by kinect camera calibration file!\n");
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
       m_displayImage= true;
 }
     m_lastframe = -1;
  log("Capturing from kinect sensor started.");
}

/**
* @brief Configure the component
*/
void KinectPCServer::start()
{
  PointCloudServer::start();
  if(m_displayImage){
    cvNamedWindow(getComponentID().c_str(),1);
  }
}

void KinectPCServer::runComponent() {
	log("I am running");
		while(isRunning()) {
			if (m_saveToFile) {
				saveNextFrameToFile();
				usleep(50000);
			}
		}
	}

void KinectPCServer::saveNextFrameToFile() {
  kinect->NextFrame();
  if(kinect->frameNumber == m_lastframe){
    return;
  }
  IplImage* rgb_data = new IplImage(kinect->rgbImage);
  // Doing new IplImage(kinect->depImage); actually causes the depth map stored as a binary image for some reason
  if(m_displayImage){
    cvShowImage(getComponentID().c_str(),rgb_data);
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
  cvSaveImage(buf2, depth_data);
m_lastframe = kinect->frameNumber;
}

// ########################## Point Cloud Server Implementations ########################## //
void KinectPCServer::getPoints(bool transformToGlobal, int imgWidth, vector<PointCloud::SurfacePoint> &points, bool complete)
{
  lockComponent();
  kinect->NextFrame();
  
  cv::Mat_<cv::Point3f> cloud;
  cv::Mat_<cv::Point3f> colCloud;
  kinect->Get3dWorldPointCloud(cloud, colCloud);

  Pose3 global_kinect_pose, zeroPose;
  setIdentity(zeroPose);
  setIdentity(global_kinect_pose);
  if(transformToGlobal)
    transform(camPars[0].pose, zeroPose, global_kinect_pose);

  // copy clouds to points-vector (dense!)
  for (int row=0; row < cloud.size().height; row++)   /// TODO SLOW!!!
  {
    for (int col=0; col < cloud.size().width; col++)
    {
      PointCloud::SurfacePoint pt;
      pt.p.x = cloud.at<cv::Point3f>(row, col).x;
      pt.p.y = cloud.at<cv::Point3f>(row, col).y;
      pt.p.z = cloud.at<cv::Point3f>(row, col).z;
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

}

