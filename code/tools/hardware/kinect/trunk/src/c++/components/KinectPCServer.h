/**
 * @file KinectPCServer.h
 * @author Richtsfeld Andreas; Hanheide, Marc
 * @date April 2011
 * @version 0.1
 * @brief Point cloud server for the kinect sensor.
 */ 

#ifndef KINECT_PC_SERVER_H
#define KINECT_PC_SERVER_H

#include <vector>
#include <sys/time.h>

#include <Eigen/Geometry>

#include "PointCloudServer.h"
#include "Kinect.h"
#include "VideoUtils.h"
#include <cast/core/CASTTimer.hpp>
#include "cv.h"

#ifdef KINECT_USER_DETECTOR
#include "../autogen/KinectPersonDetect.hpp"
#endif

namespace cast
{

class KinectPCServer;

#ifdef KINECT_USER_DETECTOR
class PersonDetectServerI : public kinect::slice::PersonDetectorInterface
{
private:
  KinectPCServer* pcSrv;

public:
  PersonDetectServerI(KinectPCServer *_ptCloud) : pcSrv(_ptCloud) {}

  kinect::slice::PersonsDict getPersons(const Ice::Current&);

};
#endif

/**
 * @brief Video device simply wrapping the Kinect API.
 */
class KinectPCServer : public PointCloudServer
{
private:
#ifdef KINECT_USER_DETECTOR
  kinect::slice::PersonDetectorInterfacePtr personDetectServer;
#endif
  std::string kinectConfig;                     ///< Kinect configuration file
  CvSize captureSize;                           ///< Size of captured images from kinect
  Kinect::Kinect *kinect;                       ///< The kinect hardware interface.
  cogx::Math::Pose3 lastValidCamPose; 

  void getResolution(int camIdx, CvSize &size);
  bool setResolution(int camIdx, CvSize &size);

  DepthGenerator* depthGenerator;
  ImageGenerator* imageGenerator;
#ifdef KINECT_USER_DETECTOR
  UserGenerator* userGenerator;
#endif
  DepthMetaData depthMD;
  ImageMetaData imageMD;

  bool m_saveToFile;
#ifdef KINECT_USER_DETECTOR
  bool m_detectPersons;
#endif
  bool m_displayImage;
  std::string m_saveDirectory;
  int m_lastframe;
  bool m_createViewCone;
  int m_subSampleScale;

  enum {
    PLANE_LEFT,
    PLANE_TOP,
    PLANE_RIGHT,
    PLANE_BOTTOM,
    N_PLANES
  };
  /* The planes defining the view cone */ 
  Eigen::Hyperplane<double, 3>* fovPlanes[N_PLANES];
  /* Sense of the normals for the planes above */
  int senses[N_PLANES];

  bool createViewCone();
  Eigen::Hyperplane<double, 3>* createPlane(std::vector<cv::Point3f>&, cogx::Math::Pose3& pose);


protected:
  virtual void configure(const std::map<std::string,std::string> & _config) throw(std::runtime_error);
  virtual void start();
  virtual void runComponent();

  virtual void receiveCameraParameters(const cdl::WorkingMemoryChange & _wmc);


public:
  KinectPCServer();
  virtual ~KinectPCServer();

  // *********************************** Point Cloud Server *********************************** //
  void getPoints(bool transformToGlobal, int imgWidth, std::vector<PointCloud::SurfacePoint> &points, bool complete);
  void getRectImage(int side, int imgWidth, Video::Image& image);
  void getDisparityImage(int imgWidth, Video::Image& image);
  void getDepthMap(cast::cdl::CASTTime &time, vector<int>& depth);
  void getRangePoints(Laser::Scan2d &KRdata);
  void receiveImages(const std::vector<Video::Image>& images);
  bool getCameraParameters(Ice::Int side, Video::CameraParameters& camPars);;
  bool isPointInViewCone(const cogx::Math::Vector3& point);
#ifdef KINECT_USER_DETECTOR
  kinect::slice::PersonsDict detectPersons();
#endif
  void saveNextFrameToFile();
#ifdef KINECT_USER_DETECTOR
  static const float RELATIVE_MINIMUM_PERSON_AREA = 0.10;
#endif
};


}

#endif

