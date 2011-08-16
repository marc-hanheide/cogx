/**
 * @file KinectCore.h
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Managment of processing kinect data.
 */

#ifndef Z_KINECT_CORE_H
#define Z_KINECT_CORE_H

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <stdexcept>

#include "TomGineThread.hh"

#include "KinectBase.h"
#include "Gestalt3D.h"

#include "Vector.hh"

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
  
  Z::VisionCore *vcore;                                             ///< left and right vision core
  IplImage *iplImg;                                                 ///< current left and right image
  cv::Mat_<cv::Vec4f> points;                                       ///< point cloud of the kinect camera

  KinectBase* kinectPrinciples[KinectBase::MAX_TYPE];               ///< Kinect 3D Gestalt principle list.
  Z::Array<Z::Gestalt3D*> kinectGestalts[Gestalt3D::MAX_TYPE];      ///< Kinect 3D Gestalt list 
  void SetNodeIDs();                                                ///< Give all 3D Gestalts a unique node ID

//   void SetActiveDrawAreaSide(int side);
  void InitKinectPrinciples();

public:
  KinectCore(/*const string &calibfile, VisionCore *vc*/) throw(std::runtime_error);
  ~KinectCore();
  
  void ClearResults();
  
  Array<Gestalt3D*>* Gestalts3D() {return kinectGestalts;}                                              ///< Return Gestalt array
  Array<Gestalt3D*>& Gestalts3D(Gestalt3D::Type type) {return kinectGestalts[type];}                    ///< Returns Gestalt array of "type"
  Gestalt3D* Gestalts3D(Gestalt3D::Type type, unsigned id) {return kinectGestalts[type][id];}           ///< Returns Gestalt of "type" and "id"
  unsigned NumGestalts3D(Gestalt3D::Type type) {return kinectGestalts[type].Size();}                    ///< Ruturns number of Gestalts of "type"
  void NewGestalt3D(Gestalt3D* g);
  
  void DrawGestalts3D(TGThread::TomGineThread *tgRenderer, Gestalt3D::Type type);                       ///< Draw Gestalt to 3d render engine
  void DrawGestalts3DToImage(cv::Mat_<cv::Vec3b> &image,
                             Gestalt3D::Type type,
                             Video::CameraParameters camPars);                                          ///< Draw Gestalt to image

  void PrintGestalts3D(Gestalt3D::Type type);
  const char* GetKinectTypeName(KinectBase::Type type);

  void ProcessKinectData(VisionCore *_vcore, IplImage *_iplImg, cv::Mat_<cv::Vec4f> &_points);
  
  const char* GetGestaltListInfo();
  const char* GetGestaltTypeName(Z::Gestalt3D::Type type);
  int NumMonoGestalts(Gestalt::Type type) {return vcore->Gestalts(type).Size();}
  void PrintVCoreStatistics();
};

}

#endif

