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
#include <ostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

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
#include "v4r/svm/SVMPredictor.h"
#include "v4r/svm/SVMFileCreator.h"
#include "v4r/GraphCut/GraphCut.h"
#include "v4r/SurfaceModeling/CreateMeshModel.hh"
#include "v4r/SurfaceModeling/SurfaceModeling.hh"
#include "v4r/SurfaceModeling/Patches.h"
#include "v4r/SurfaceModeling/FileSystem.hh"
#include "v4r/SurfaceModeling/MoSPlanes3D.hh"


#include "SegUtilsFunctions.h"

namespace cast
{

/**
 * @class SegTester
 */
class SegTester : public ManagedComponent,
                  public PointCloudClient
{
private:
  
  bool deb;                                                 ///< Debug flag
  
  surface::SaveFileSequence save_models;                    ///< Save surface models
  bool save_results;                                        ///< Save surface models
  char *save_filename;                                      ///< filename for surface models
  
  TomGine::tgTomGineThread *tgRenderer;                     ///< 3D render engine
  cast::StereoCamera *stereo_cam;                           ///< stereo camera parameters and functions

  /// TODO new ones
  Z::VisionCore *vcore;                                     ///< VisionCore
  pclA::ModelFitter *model_fitter;                          ///< Fit multiple models to point cloud
  surface::MoSPlanes3D *planeFitter;                        ///< mosPlane fitter
  surface::SurfaceModeling *modeling;                       ///< Nurbs-fitting and model-selection
  anno::Annotation *annotation;                             ///< Annotation from file
  surface::Patches *patches;                                ///< Patch tool for calculation of relations between surface patches
  svm::SVMPredictor *svm;                                   ///< SVM-predictor
  svm::SVMFileCreator *svmFile;                             ///< SVM-file creator
  gc::GraphCut *graphCut;                                   ///< Graph cut

  int nr_anno;                                              ///< Number of annotated objects
  std::vector<int> anno;                                    ///< Annotation of all pcl_model_indices
  std::vector<bool> texture;                                ///< Texture on 2D image space

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;         ///< PCL point cloud (dilation)
  pcl::PointCloud<pcl::Normal>::Ptr pcl_normals;            ///< Normals of the point cloud
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_vis;     ///< PCL point cloud (dilation)
//  pcl::PointCloud<pcl::Normal>::Ptr pcl_normals_vis;        ///< Normals of the point cloud
  std::vector<pcl::PointIndices::Ptr> pcl_model_indices_old;///< indices of the surface patches (from fitter for debugging => TODO Remove later)
  std::vector<pcl::PointIndices::Ptr> pcl_model_indices;    ///< indices of the surface patches
  std::vector< std::vector<int> > preProcessIndices;        ///< TODO Indices of points eliminated by postProcessing
  std::vector<pcl::PointIndices::Ptr> pcl_model_indices_planes; ///< TODO indices of the plane patches (For debugging)

  std::vector<surface::SurfaceModel::Ptr> surfaces;         ///< Surfaces container (for Planes, NURBS)
  std::vector< std::vector<unsigned> > graphCutGroups;      ///< Graph cut groups of surface patch models

  pcl::PointCloud<pcl::Normal>::Ptr pcl_normals_repro;      ///< Reprocessed normals after fitting => TODO Remove later
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_model_cloud;   ///< TODO PCL point cloud with points projected to model

//   std::vector<pcl::PointIndices::Ptr> pcl_model_cloud_indices;  ///< indices of the plane patches
  /// TODO end new ones
 
//   Z::StereoCore *score;                                     ///< Stereo core
//   Z::KinectCore *kcore;                                     ///< Kinect core
//   pclA::PlanePopout *planePopout;                           ///< PlanePopout for SOI calculation (ground truth data)
//   Z::CalculateRelations *relations;                         ///< Calculate relations between features.
//   Z::SVMPredictor *svmPredictor;                            ///< SVM predictor
//   Z::GraphCut *graphCutter;                                 ///< Graph cutter

  int runtime;                                              ///< Overall processing runtime for one image (pair)
  float cannyAlpha, cannyOmega;                             ///< Alpha and omega value of the canny edge detector											/// TODO muss hier nicht sein?
  std::string stereoconfig;                                 ///< Config name of stereo camera config file
  std::vector<int> camIds;                                  ///< Which cameras to get images from
  std::vector<Video::CameraParameters> camPars;             ///< Camera parameters for each camera (left/right/kinect)

  int rgbWidth, rgbHeight;                                  ///< width and height of the kinect color image
  int pointCloudWidth, pointCloudHeight;                    ///< width and height of the kinect point cloud
  Video::Image /*image_l, image_r,*/ image_k;                   ///< Left and right stereo image and kinect image
  IplImage /*iplImage_l, *iplImage_r,*/ *iplImage_k;           ///< Converted left and right stereo images (openCV ipl-images)
//  IplImage *iplImage_depthMap;                              ///< iplImage with depth map of kinect
  
  std::vector<PointCloud::SurfacePoint> points;             ///< 3D points from kinect sensor
  cv::Mat_<cv::Vec4f> kinect_point_cloud;                   ///< Point cloud from the kinect
  cv::Mat_<cv::Vec3b> kinect_point_cloud_image;             ///< Image of the kinect point cloud
  
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > sois; ///< Estimated sois from the PlanePopout
  std::vector<unsigned> soi_labels;                           ///< Labels of the estimated sois

  cv::Mat_<cv::Vec3b> patch_image;                          ///< 3D patches on 2D image
  cv::Mat_<cv::Vec3b> line_image;                           ///< 3D lines on 2D image

//  cv::Mat_<cv::Point3f> kinect_point_cloud;                 ///< point cloud with kinect 3d points                                  /// TODO delete later => change to cv::Vec4f
//  cv::Mat_<cv::Point3f> kinect_color_point_cloud;           ///< point cloud with kinect color information
  
  bool single;                                              ///< Single shot mode for the stereo detector learner
  bool showImages;                                          ///< Show images in openCV windows

//   void Points2DepthMap(cast::StereoCamera *sc, cv::Mat_<cv::Point3f> c, cv::Mat_<cv::Point3f> cc, cv::Mat_<cv::Point3f> &depthImage, cv::Mat_<cv::Point3f> &depthMap);
  void GetImageData();
  
  void processImageNew();
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



