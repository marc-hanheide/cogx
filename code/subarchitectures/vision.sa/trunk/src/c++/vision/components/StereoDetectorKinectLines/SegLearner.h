/**
 * @file SegLearner.h
 * @author Andreas Richtsfeld
 * @date 2011
 * @version 0.1
 * @brief Get properties to learn how to segment.
 */


#ifndef SEG_LEARNER_H
#define SEG_LEARNER_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include <stdexcept>

#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include <VideoClient.h>
#include <PointCloudClient.h>
#include <VideoUtils.h>
#include <../../VisionUtils.h>

#include "VisionCore.hh"
#include "Gestalt.hh"
#include "Segment.hh"
#include "Edgel.hh"

#include "Pose3.h"
#include "StereoBase.h"
#include "StereoCamera.h"

#include "v4r/Annotation/Annotation.h"
#include "v4r/PCLAddOns/PlanePopout.hh"
#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/PCLAddOns/PCLFunctions.h"
#include "v4r/PCLAddOns/ModelFitter.h"
#include "v4r/PCLAddOns/BilateralFilter.hh"
#include "v4r/PCLAddOns/SubsamplePointCloud.hh"
#include "v4r/PCLAddOns/NormalsEstimationNR.hh"
#include "v4r/PCLAddOns/Relation.h"
#include "v4r/TomGine/tgTomGineThread.h"
#include "v4r/GraphCut/GraphCut.h"
#include "v4r/SurfaceModeling/CreateMeshModel.hh"
#include "v4r/SurfaceModeling/SurfaceModeling.hh"
#include "v4r/SurfaceModeling/Patches.h"
#include "v4r/SurfaceModeling/MoSPlanes3D.hh"
#include "v4r/svm/SVMFileCreator.h"
#include "v4r/svm/Statistics.h"

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
  
  bool stopComponent;                                       ///< Set true to stop component
  
  bool deb;                                                 ///< debug flag for output
  bool single;                                              ///< Single shot mode for the stereo detector learner
  bool showImages;                                          ///< Show images in openCV windows
  bool labels;                                              ///< draw labels into TomGine
  
  TomGine::tgTomGineThread *tgRenderer;                     ///< 3D render engine
//   cast::StereoCamera *stereo_cam;                           ///< stereo camera parameters and functions
  std::vector<int> camIds;                                  ///< Which cameras to get images from
  std::vector<Video::CameraParameters> camPars;             ///< Camera parameters for each camera (left/right/kinect)
  int runtime;                                              ///< Overall processing runtime for one image (pair)
  float cannyAlpha, cannyOmega;                             ///< Alpha and omega value of the canny edge detector

  std::vector<PointCloud::SurfacePoint> points;             ///< 3D points from kinect view
  int kinectImageWidth, kinectImageHeight;                  ///< width and height of the kinect color image
  int pointCloudWidth, pointCloudHeight;                    ///< width and height of the kinect point cloud
  Video::Image image_k;                                     ///< Left and right stereo image and kinect image
  IplImage *iplImage_k;                                     ///< Converted left and right stereo images (openCV ipl-images)
  cv::Mat_<cv::Vec4f> kinect_point_cloud;                   ///< Point cloud from the kinect  TODO nur zur Anzeige?
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;         ///< PCL point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_filtered;///< PCL point cloud (z-)filtered
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_model_cloud;   ///< PCL point cloud with points projected to model
  
  int nr_anno;                                              ///< Number of annotated objects
  std::vector<int> anno;                                    ///< Annotation of all pcl_model_indices
  std::vector<bool> texture;                                ///< Texture on 2D image space

  std::vector<surface::SurfaceModel::Ptr > surfaces;        ///< Surfaces container (for Planes, NURBS)
  std::vector<int> postProcessIndices;                      ///< TODO Indices of points eliminated by postProcessing
  std::vector<int> checkPCLines;                            ///< TODO Indices of points found by checkPCLines
  
  /// TODO new ones
  Z::VisionCore *vcore;                                     ///< VisionCore
  pclA::BilateralFilter *bilateral;                         ///< Bilateral filter for point clouds
  pclA::SubsamplePointCloud *subsample;                     ///< Subsample point cloud
  pclA::ModelFitter *model_fitter;                          ///< Fit multiple models to point cloud
  surface::MoSPlanes3D *planeFitter;                        ///< mosPlane fitter
  surface::SurfaceModeling *modeling;                       ///< Nurbs-fitting and model-selection
  anno::Annotation *annotation;                             ///< Annotation from file
  surface::Patches *patches;                                ///< Patch tool for calculation of relations between surface patches
  svm::SVMFileCreator *svm;                                 ///< SVM-predictor

  pcl::PointCloud<pcl::Normal>::Ptr pcl_normals;            ///< Normals of the point cloud
  pcl::PointCloud<pcl::Normal>::Ptr pcl_normals_repro;      ///< Reprocessed normals after fitting => TODO Remove later
  std::vector<pcl::PointIndices::Ptr> pcl_model_indices_planes; ///< indices of the plane patches (For debugging)
  std::vector<pcl::PointIndices::Ptr> pcl_model_indices;    ///< indices of the surface patches
  /// TODO end new ones

  void GetImageData();
  void processImageNew();
  void SingleShotMode();

protected:
  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

public:
  SegLearner() {}
  virtual ~SegLearner() {printf("TODO: SegLearner.h: Delete all the stuff!!!\n");}
  
  void ProjectPoint(double X, double Y, double Z, double &u, double &v, int imgWidth);

};

}

#endif



