/**
 * @file SegTester.h
 * @author Andreas Richtsfeld
 * @date 2011
 * @version 0.1
 * @brief Get properties to learn how to segment.
 */


#ifndef SEG_TESTER_H
#define SEG_TESTER_H

#include <vector>
#include <stdexcept>

#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include <VideoClient.h>
#include <PointCloudClient.h>
#include <VideoUtils.h>
#include <../../VisionUtils.h>

#include "StereoCore.h"
#include "KinectCore.h"
#include "CalculateRelations.h"
#include "GraphCut.h"
#include "SVMPredictor.h"

#include "Pose3.h"
#include "StereoBase.h"
#include "StereoCamera.h"
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
 * @class SegTester
 */
class SegTester : public ManagedComponent,
                   public PointCloudClient
{
private:
  TomGine::tgTomGineThread *tgRenderer;                      ///< 3D render engine
  std::vector<PointCloud::SurfacePoint> points;             ///< 3D points from kinect view
  cast::StereoCamera *stereo_cam;                           ///< stereo camera parameters and functions
 
  Z::VisionCore *vcore;                                     ///< VisionCore
  Z::StereoCore *score;                                     ///< Stereo core
  Z::KinectCore *kcore;                                     ///< Kinect core
  pclA::PlanePopout *planePopout;                           ///< PlanePopout for SOI calculation (ground truth data)
//  Z::SVMFileCreator *svmFileCreator;                        ///< SVM training file creator
  Z::CalculateRelations *relations;                         ///< Calculate relations between features.
  Z::SVMPredictor *svmPredictor;                            ///< SVM predictor
  Z::GraphCut *graphCutter;                                 ///< Graph cutter

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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;         ///< PCL point cloud
  
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > sois; ///< Estimated sois from the PlanePopout

  cv::Mat_<cv::Vec3b> patch_image;                          ///< 3D patches on 2D image
  cv::Mat_<cv::Vec3b> line_image;                           ///< 3D lines on 2D image

//  cv::Mat_<cv::Point3f> kinect_point_cloud;                 ///< point cloud with kinect 3d points                                  /// TODO delete later => change to cv::Vec4f
//  cv::Mat_<cv::Point3f> kinect_color_point_cloud;           ///< point cloud with kinect color information
  
  bool single;                                              ///< Single shot mode for the stereo detector learner
  bool showImages;                                          ///< Show images in openCV windows

//   void Points2DepthMap(cast::StereoCamera *sc, cv::Mat_<cv::Point3f> c, cv::Mat_<cv::Point3f> cc, cv::Mat_<cv::Point3f> &depthImage, cv::Mat_<cv::Point3f> &depthMap);
  void GetImageData();
  
  void processImage();
  void SingleShotMode();

protected:
  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

public:
  SegTester() {}
  virtual ~SegTester() {delete vcore;}
  
  void ProjectPoint(double X, double Y, double Z, double &u, double &v, int imgWidth);

};

}

#endif



