/**
 * @file KinectPCServer.h
 * @author Richtsfeld Andreas; Hanheide, Marc
 * @date April 2011
 * @version 0.1
 * @brief Point cloud server for the kinect sensor.
 */ 

#ifndef KINECT_PC_SERVER_H
#define KINECT_PC_SERVER_H


#include "Kinect.h"
#include "PointCloudServer.h"
#include <VideoUtils.h>
#include <castutils/Timers.hpp>
#include <cast/core/CASTTimer.hpp>

#include <IceUtil/IceUtil.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>

#include <vector>
#include <sys/time.h>


#ifdef KINECT_USER_DETECTOR
#include "../autogen/KinectPersonDetect.hpp"
#endif

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
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
  IceUtil::Mutex m_kinectMutex;
  cogx::Math::Pose3 lastValidCamPose; 

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
  bool m_viewConeNeedsUpdate;
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
  castutils::CMilliTimer m_tmUpdateViewCone;

private:
  class KinectLock
    : private IceUtil::Mutex::Lock
  {
  public:
    KinectLock(KinectPCServer *pServer) : IceUtil::Mutex::Lock(pServer->m_kinectMutex)
    {
    }
  };
  void lockKinect();
  void unlockKinect();

private:
  void getResolution(int camIdx, CvSize &size);
  bool setResolution(int camIdx, CvSize &size);

  void deleteViewConePlanes();
  bool createViewCone();
  void checkUpdateViewCone();
  Eigen::Hyperplane<double, 3>* createPlane(std::vector<cv::Point3f>&, cogx::Math::Pose3& pose);

#ifdef FEAT_VISUALIZATION
private:
  class DisplayClient: public cogx::display::CDisplayClient
  {
    //KinectPCServer* pPcServer;
  public:
    DisplayClient() { /*pViewer = NULL;*/ }
    //void setClientData(VideoViewer* pVideoViewer) { pViewer = pVideoViewer; }
    //void handleEvent(const Visualization::TEvent &event); [>override<]
    //std::string getControlState(const std::string& ctrlId); [>override<]
  };
  DisplayClient m_display;
  bool m_bUseV11n;
#endif

protected:
  virtual void configure(const std::map<std::string,std::string> & _config) throw(std::runtime_error);
  virtual void start();
  virtual void runComponent();

  virtual void receiveCameraParameters(const cdl::WorkingMemoryChange & _wmc);


public:
  KinectPCServer();
  virtual ~KinectPCServer();
  using CASTComponent::sleepComponent;

  // *********************************** Point Cloud Server *********************************** //
  void getPoints(bool transformToGlobal, int imgWidth, std::vector<PointCloud::SurfacePoint> &points, bool complete);
  // Kinect hack: side<0 --> return RGB depth image
  void getRectImage(int side, int imgWidth, Video::Image& image);
  void getDisparityImage(int imgWidth, Video::Image& image);
  void getDepthMap(cast::cdl::CASTTime &time, vector<int>& depth);
  void getRangePoints(Laser::Scan2d &KRdata);
  void receiveImages(const std::vector<Video::Image>& images);
  bool getCameraParameters(Ice::Int side, Video::CameraParameters& camPars);;
  bool isPointInViewCone(const cogx::Math::Vector3& point);
  bool isPointVisible(const cogx::Math::Vector3& point);
#ifdef KINECT_USER_DETECTOR
  kinect::slice::PersonsDict detectPersons();
#endif
  void saveNextFrameToFile();
#ifdef KINECT_USER_DETECTOR
  static const float RELATIVE_MINIMUM_PERSON_AREA = 0.10;
#endif
};

inline
void KinectPCServer::lockKinect()
{
  m_kinectMutex.lock();
}

inline
void KinectPCServer::unlockKinect()
{
  m_kinectMutex.unlock();
}


}

#endif

