/**
 * @file SegLearner.h
 * @author Andreas Richtsfeld
 * @date 2011
 * @version 0.1
 * @brief Get properties to learn how to segment.
 */


#ifndef SEG_LEARNER_H
#define SEG_LEARNER_H

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
#include "Learner.h"              /// TODO Braucht man hier?
#include "SVMFileCreator.h"

#include "Pose3.h"
#include "StereoBase.h"
#include "StereoCamera.h"
#include "Gestalt.hh"
#include "Array.hh"

#include "ObjRep.h"

#include "v4r/PCLAddOns/PlanePopout.hh"
#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/PCLAddOns/PCLFunctions.h"
#include "v4r/PCLAddOns/Annotation.h"
#include "v4r/PCLAddOns/ModelFitter.h"
#include "v4r/PCLAddOns/Planes.h"
#include "v4r/PCLAddOns/Patches.h"
#include "v4r/PCLAddOns/Relation.h"
#include "v4r/TomGine/tgTomGineThread.h"
#include "v4r/svm/SVMFileCreator.h"
#include "v4r/GraphCut/GraphCut.h"
#include "v4r/SurfaceModeling/SurfaceModeling.hh"
#include "v4r/PCLAddOns/BilateralFilter.hh"
#include "v4r/PCLAddOns/SubsamplePointCloud.hh"
#include "v4r/PCLAddOns/NormalsEstimationNR.hh"

#include "SegUtilsFunctions.h"

namespace cast
{

/**
 * @class SegLearner
 */
class SegLearner : public ManagedComponent,
                   public PointCloudClient
{
private:
  TomGine::tgTomGineThread *tgRenderer;                     ///< 3D render engine
  std::vector<PointCloud::SurfacePoint> points;             ///< 3D points from kinect view
  cast::StereoCamera *stereo_cam;                           ///< stereo camera parameters and functions
 
   /// TODO new ones
  pclA::BilateralFilter *bilateral;                         ///< Bilateral filter for point clouds
  pclA::SubsamplePointCloud *subsample;                     ///< Subsample point cloud
  pclA::ModelFitter *model_fitter;                          ///< Fit multiple models to point cloud
  surface::SurfaceModeling *modeling;                       ///< Nurbs-fitting and model-selection
  pa::Annotation *annotation;                               ///< Annotation from file
  pclA::Patches *patches;                                   ///< Patch tool for calculation of relations between surface patches
  svm::SVMFileCreator *svm;                                 ///< SVM-predictor

  pcl::PointCloud<pcl::Normal>::Ptr pcl_normals;            ///< Normals of the point cloud
  pcl::PointCloud<pcl::Normal>::Ptr pcl_normals_new;            ///< Normals of the point cloud
  pcl::PointCloud<pcl::Normal>::Ptr pcl_normals_repro;      ///< Reprocessed normals after fitting => TODO Remove later
  std::vector<pcl::PointIndices::Ptr> pcl_model_indices_old;    ///< indices of the surface patches (from fitter for debugging => TODO Remove later)
  std::vector<pcl::PointIndices::Ptr> pcl_model_indices;    ///< indices of the surface patches

  int nr_anno;                                              ///< Number of annotated objects
  std::vector<int> anno;                                    ///< Annotation of all pcl_model_indices
  std::vector<bool> texture;                                ///< Texture on 2D image space

  std::vector<pcl::PointIndices::Ptr> plane_indices;        ///< Plane indices after model selection  /// TODO REMOVE
  std::vector<pcl::PointIndices::Ptr> nurbs_indices;        ///< NURBS indices after model selection  /// TODO REMOVE
  std::vector<cv::Ptr<surface::SurfaceModel> > surfaces;    ///< Surfaces container (for Planes, NURBS)
  /// TODO end new ones

 
  Z::VisionCore *vcore;                                     ///< VisionCore
  Z::StereoCore *score;                                     ///< Stereo core
  Z::KinectCore *kcore;                                     ///< Kinect core
//  Z::Learner *learner;                                      ///< Learner
  Z::SVMFileCreator *svmFileCreator;                        ///< SVM training file creator
//  Z::GraphCut *graphCutter;                                 ///< Graph cutter
  pclA::PlanePopout *planePopout;                           ///< PlanePopout for SOI calculation (ground truth data)

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
  std::vector<unsigned> soi_labels;                           ///< Labels of the estimated sois

  cv::Mat_<cv::Vec3b> patch_image;                          ///< 3D patches on 2D image
  cv::Mat_<cv::Vec3b> line_image;                           ///< 3D lines on 2D image

  bool single;                                              ///< Single shot mode for the stereo detector learner
  bool showImages;                                          ///< Show images in openCV windows

  void GetImageData();
  
  void processImage();
  void processImageNew();

  void SingleShotMode();

protected:
  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

public:
  SegLearner() {}
  virtual ~SegLearner() {/*delete vcore;*/printf("TODO: SegLearner.h: Delete all the stuff!!!\n");}
  
  void ProjectPoint(double X, double Y, double Z, double &u, double &v, int imgWidth);

};

}

#endif



