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

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::KinectPCServer();
  }
}

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;


KinectPCServer::KinectPCServer()
{
}

KinectPCServer::~KinectPCServer()
{
  delete kinect;
}

/**
 * @brief Get video resolution for given camera.
 * @param camIdx which camera
 * @param size video resolution
 */
void KinectPCServer::getResolution(int camIdx, CvSize &size)
{
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
bool KinectPCServer::setResolution(int camIdx, CvSize &size)
{
  log("setResolution: warning: Not yet implemented! Defined by kinect camera calibration file!\n");
  return false;
}

/**
 * @brief Configure the component
 */
void KinectPCServer::configure(const map<string,string> & _config) throw(runtime_error)
{
  // configure the point cloud server
  PointCloudServer::configure(_config);

  map<string,string>::const_iterator it;
    
  if((it = _config.find("--kconfig")) != _config.end())
  {  
    istringstream str(it->second);
    str >> kinectConfig;
  }
  else throw runtime_error(exceptionMessage(__HERE__, "no kinect config file (kconfig) specified."));

  // init kinect hardware driver
  CvSize size;
  const char* name = kinectConfig.c_str();
  kinect = new Kinect::Kinect(name);
  kinect->GetColorVideoSize(size);
  captureSize.width = size.width;
  captureSize.height = size.height;
  
  kinect->StartCapture(0); 	// start capturing
  log("Capturing from kinect sensor started.");
}


/**
 * @brief Configure the component
 */
void KinectPCServer::start()
{
  PointCloudServer::start(); 
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
  if(transformToGlobal)
    transform(camPars[0].pose, zeroPose, global_kinect_pose);

  // copy clouds to points-vector (dense!)
  for(unsigned row=0; row< cloud.size().height; row++)   /// TODO SLOW!!!
  {
    for(unsigned col=0; col < cloud.size().width; col++)
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
	/*lockComponent();
	convertImageFromIpl(disparityImg, image);
	unlockComponent();*/
}

void KinectPCServer::receiveImages(const vector<Video::Image>& images)
{
//   lockComponent();
  printf("KinectPCServer::receiveImages: Warning: Not yet implemented!\n");
  //stereoProcessing();
//   unlockComponent();
}

}

