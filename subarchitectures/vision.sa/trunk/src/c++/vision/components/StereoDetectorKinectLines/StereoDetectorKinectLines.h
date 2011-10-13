/**
 * @file StereoDetectorKinectLines.h
 * @author Andreas Richtsfeld
 * @date 2011
 * @version 0.1
 * @brief Do everything with Kinect!
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
#include "StereoCamera.h"

#include "StereoCore.h"
#include "KinectCore.h"
#include "CalculateRelations.h"
//#include "Learner.h"
#include "GraphCut.h"
#include "SVMPredictor.h"

#include "Pose3.h"
#include "StereoBase.h"
#include "Gestalt.hh"
#include "Array.hh"

#include "ObjRep.h"

#include "v4r/PCLAddOns/PlanePopout.hh"
#include "v4r/PCLAddOns/utils/PCLUtils.h"
#include "v4r/PCLAddOns/functions/PCLFunctions.h"
#include "v4r/TomGine/tgTomGineThread.h"


namespace cast
{

/**
 * @class StereoDetectorKinectLines
 */
class StereoDetectorKinectLines : public ManagedComponent,
                                  public PointCloudClient
{
private:
  TomGine::tgTomGineThread *tgRenderer;                     ///< 3D render engine
  std::vector<PointCloud::SurfacePoint> points;             ///< 3D points from kinect view
//  std::vector<PointCloud::SurfacePoint> points_fromLeft;    ///< 3D point vector from view of left stereo camera!
//   IplImage *backProjPointCloud;                            ///< Back projected point cloud
  cast::StereoCamera *stereo_cam;                           ///< stereo camera parameters and functions
 
  Z::VisionCore *vcore;                                     ///< VisionCore
  Z::StereoCore *score;                                     ///< Stereo core
  Z::KinectCore *kcore;                                     ///< Kinect core
  pclA::PlanePopout *planePopout;                           ///< Plane popout
//  Z::CalculateRelations *relations;                         ///< Calculate relations between features.
//  Z::Learner *learner;                                      ///< Learner
//  Z::SVMPredictor *svmPredictor;                            ///< SVM predictor
//  Z::GraphCut *graphCutter;                                 ///< Graph cutter
  
  int runtime;                                              ///< Overall processing runtime for one image (pair)
  float cannyAlpha, cannyOmega;                             ///< Alpha and omega value of the canny edge detector											/// TODO muss hier nicht sein?
  std::string stereoconfig;                                 ///< Config name of stereo camera config file
  std::vector<int> camIds;                                  ///< Which cameras to get images from
  std::vector<Video::CameraParameters> camPars;             ///< Camera parameters for each camera (left/right/kinect)

  int kinectImageWidth, kinectImageHeight;                  ///< width and height of the kinect color image
  int pointCloudWidth, pointCloudHeight;                    ///< width and height of the kinect point cloud
  Video::Image image_l, image_r, image_k;                   ///< Left and right stereo image and kinect image
  IplImage *iplImage_l, *iplImage_r, *iplImage_k;           ///< Converted left and right stereo images (openCV ipl-images)
//  IplImage *iplImage_depthMap;                              ///< iplImage with depth map of kinect
  
  cv::Mat_<cv::Vec4f> kinect_point_cloud;                   ///< Point cloud from the kinect
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud ;        ///< PCL point cloud
  
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > sois; ///< Estimated sois from the PlanePopout

  cv::Mat_<cv::Vec3b> patches;                              ///< 3D patches on 2D image

  bool single;                                              ///< Single shot mode for the stereo detector learner
  bool showImages;                                          ///< Show images in openCV windows

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
  
  void ProjectPoint(double X, double Y, double Z, double &u, double &v, int imgWidth);

};

}

#endif



