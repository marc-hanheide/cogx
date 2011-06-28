/**
 * @file StereoDetectorKinectLines.h
 * @author Andreas Richtsfeld
 * @date 2011
 * @version 0.1
 * @brief Calculate 3D lines with kinect sensor.
 */


#ifndef STEREO_DETECTOR_KINECT_LINES_H
#define STEREO_DETECTOR_KINECT_LINES_H

#include <vector>
#include <stdexcept>

#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include <VideoClient.h>
#include <PointCloudClient.h>
#include <VideoUtils.h>
#include <../../VisionUtils.h>

#include "KinectCore.h"
#include "StereoCore.h"

#include "Pose3.h"
#include "StereoBase.h"
#include "Gestalt.hh"
#include "Reasoner.h"
#include "Array.hh"

#include "ObjRep.h"
#include "TomGineThread.hh"

#include "StereoCamera.h"

#include "Learner.h"

namespace cast
{

/**
 * @class StereoDetectorKinectLines
 */
class StereoDetectorKinectLines : public ManagedComponent,
                                  public PointCloudClient
{
private:
  TGThread::TomGineThread *tgRenderer;                      ///< 3D render engine
  std::vector<PointCloud::SurfacePoint> points;             ///< 3D points from kinect view
//  std::vector<PointCloud::SurfacePoint> points_fromLeft;    ///< 3D point vector from view of left stereo camera!
//   IplImage *backProjPointCloud;                            ///< Back projected point cloud
  cast::StereoCamera *stereo_cam;                           ///< stereo camera parameters and functions
 
  Z::VisionCore *vcore;                                     ///< VisionCore
  Z::StereoCore *score;                                     ///< Stereo core
  Z::KinectCore *kcore;                                     ///< Kinect core
  Z::Learner *learner;                                      ///< Learner

  int runtime;                                              ///< Overall processing runtime for one image (pair)
  float cannyAlpha, cannyOmega;                             ///< Alpha and omega value of the canny edge detector											/// TODO muss hier nicht sein?
  std::string stereoconfig;                                 ///< Config name of stereo camera config file
  std::vector<int> camIds;                                  ///< Which cameras to get images from
  std::vector<Video::CameraParameters> camPars;             ///< Camera parameters for each camera

  int kinectImageWidth, kinectImageHeight;                  ///< width and height of the kinect color image
  int pointCloudWidth, pointCloudHeight;                    ///< width and height of the kinect point cloud
  Video::Image image_l, image_r, image_k;                   ///< Left and right stereo image and kinect image
  IplImage *iplImage_l, *iplImage_r, *iplImage_k;           ///< Converted left and right stereo images (openCV ipl-images)
  IplImage *iplImage_depthMap;                              ///< iplImage with depth map of kinect
  
  cv::Mat_<cv::Point3f> kinect_point_cloud;                 ///< point cloud with kinect 3d points                                  /// TODO delete later => change to cv::Vec4f
  cv::Mat_<cv::Point3f> kinect_color_point_cloud;           ///< point cloud with kinect color information
  
  bool single;                                              ///< Single shot mode for the stereo detector learner
  bool showImages;                                          ///< Show images in openCV windows

  void Points2DepthMap(cast::StereoCamera *sc, cv::Mat_<cv::Point3f> c, cv::Mat_<cv::Point3f> cc, cv::Mat_<cv::Point3f> &depthImage, cv::Mat_<cv::Point3f> &depthMap);
  void GetImageData();
  
  void processImage();
  void SingleShotMode();

protected:
  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

public:
  StereoDetectorKinectLines() {}
  virtual ~StereoDetectorKinectLines() {delete vcore;}
};

}

#endif



