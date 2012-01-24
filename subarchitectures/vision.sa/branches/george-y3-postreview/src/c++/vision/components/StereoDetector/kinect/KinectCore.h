/**
 * @file KinectCore.h
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Managment of processing kinect data.
 */

#ifndef Z_KINECT_CORE_H
#define Z_KINECT_CORE_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdexcept>
#include <set>

#include "KinectBase.h"
#include "Gestalt3D.h"

#include "Vector.hh"

#include "v4r/PCLAddOns/PlanePopout.hh"
#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/PCLAddOns/PCLFunctions.h"
#include "v4r/TomGine/tgTomGineThread.h"

namespace Z
{

/**
 * @brief Class StereoCore: Management of calculations in stereo-core and both vision-cores.
 */
class KinectCore
{
public:
private:
  bool initialized;                                                 ///< Set true after first initialization
  bool drawNodeID;                                                  ///< Draw with Gestalts the NodeID
  
  Z::VisionCore *vcore;                                             ///< left and right vision core

  IplImage *iplImg;                                                 ///< current ipl-image of kinect camera
  cv::Mat_<cv::Vec4f> points;                                       ///< point cloud of the kinect camera
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;                 ///< point cloud of the kinect as pcl-cloud               /// TODO remove at one point and use const-version
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr const_pcl_cloud;      ///< constant point cloud of the kinect as pcl-cloud

  double fx,fy,cx,cy;                                               ///< Camera parameters for kinect (NOT scaled!!!)
  bool valid_normals;                                               ///< Flag for valid point cloud normals
  pcl::PointCloud<pcl::Normal>::Ptr pcl_normals;                    ///< calculated point cloud normals
  cv::Mat_<cv::Vec4f> cv_normals;                                   ///< calculated point cloud normals
  
  int nr_processed_images;                                          ///< Number of processed images
  std::vector<double> sum_res_multi;                                ///< results of the object check (anno to real)
  std::vector<double> sum_res_single;                               ///< results of the single object check (anno to real)
  std::vector<double> sum_res_backcheck;                            ///< backcheck (real to anno)

  std::set<unsigned> graphCutGroups;                                ///< group numbers of graph-cut result.

  KinectBase* kinectPrinciples[KinectBase::MAX_TYPE];               ///< Kinect 3D Gestalt principle list.
  Z::Array<Z::Gestalt3D*> kinectGestalts[Gestalt3D::MAX_TYPE];      ///< Kinect 3D Gestalt list 

  void SetNodeIDs();                                                ///< Give all 3D Gestalts a unique node ID
  void InitKinectPrinciples();
  
public:

  KinectCore(Z::VisionCore *vc, double _fx, double _fy, double _cx, double _cy) throw(std::runtime_error);
  ~KinectCore();
  
  void ClearResults();
  void InitKinectPrinciplesForSegmenter();

  Array<Gestalt3D*>* Gestalts3D() {return kinectGestalts;}                                              ///< Return Gestalt array
  Array<Gestalt3D*>& Gestalts3D(Gestalt3D::Type type) {return kinectGestalts[type];}                    ///< Returns Gestalt array of "type"
  Gestalt3D* Gestalts3D(Gestalt3D::Type type, unsigned id) {return kinectGestalts[type][id];}           ///< Returns Gestalt of "type" and "id"
  unsigned NumGestalts3D(Gestalt3D::Type type) {return kinectGestalts[type].Size();}                    ///< Ruturns number of Gestalts of "type"
  void NewGestalt3D(Gestalt3D* g);
  
  void DrawNodeID(bool draw) {drawNodeID = draw;}
  void DrawGestalts3D(TomGine::tgTomGineThread *tgRenderer, 
                      Gestalt3D::Type type, 
                      bool random_color = true, 
                      bool use_color = false, 
                      float color = 0.0);                                                               ///< Draw Gestalt to 3d render engine
  void DrawGestalts3DToImage(cv::Mat_<cv::Vec3b> &image,
                             Gestalt3D::Type type,
                             Video::CameraParameters camPars);                                          ///< Draw Gestalt to image

  void DrawObjects3D(TomGine::tgTomGineThread *tgRenderer);                                             ///< Draw colored objects-features (from planePopout)
  void DrawGraphCut3D(TomGine::tgTomGineThread *tgRenderer);                                            ///< Draw Graph-cutted segments
  void PrintGestalts3D(Gestalt3D::Type type);
  
  const char* GetKinectTypeName(KinectBase::Type type);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetPclCloud() {return pcl_cloud;}
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr GetConstPclCloud() {return const_pcl_cloud;}
  cv::Mat_<cv::Vec4f> GetPointCloud() {return points;}
  unsigned GetImageWidth() {return iplImg->width;}
  unsigned GetPointCloudWidth() {return points.cols;}
  unsigned GetPointCloudHeight() {return points.rows;}
  double GetScale() {return ((double) GetImageWidth() / (double) GetPointCloudWidth());}                /// Scale between rgb and depth image
  cv::Mat_<cv::Vec4f> GetCvNormals();
  pcl::PointCloud<pcl::Normal>::Ptr GetPclNormals();
  void GetCameraParameters(double &_fx, double &_fy, double &_cx, double &_cy) 
       { _fx = fx; _fy = fy; _cx = cx; _cy = cy; }
  void GetScaledCameraParameters(double &_fx, double &_fy, double &_cx, double &_cy) 
       { _fx = fx/GetScale(); _fy = fy/GetScale(); _cx = cx/GetScale(); _cy = cy/GetScale(); }
  
  void Process(IplImage *_iplImg, cv::Mat_<cv::Vec4f> &_points, 
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_pcl_cloud);
  
  const char* GetGestaltListInfo();
  const char* GetGestaltTypeName(Z::Gestalt3D::Type type);
  int NumMonoGestalts(Gestalt::Type type) {return vcore->Gestalts(type).Size();}
  void PrintVCoreStatistics();
  
  void SetObjectLabels(pclA::PlanePopout *pp);
  void SetGraphCutGroups(std::set<unsigned> &gcg) {graphCutGroups = gcg;}

  double CheckAnnotation(std::vector<int> &anno);
  void GetAnnotationResults(double &m, double &s, double &b);       
  void SetAnnotation(std::vector<int> &anno);
  
  void PrintNodeIDs();                                                ///< TODO Remove

};

}

#endif

