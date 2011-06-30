/**
 * @file KinectPCServer.h
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Point cloud server for the kinect sensor.
 */

#ifndef KINECT_PC_SERVER_H
#define KINECT_PC_SERVER_H

#include <vector>
#include <sys/time.h>

#include "PointCloudServer.h"
#include "Kinect.h"
#include "VideoUtils.h"
#include <cast/core/CASTTimer.hpp>
#include "cv.h"

namespace cast
{

/**
 * @brief Video device simply wrapping the Kinect API.
 */
class KinectPCServer : public PointCloudServer
{
private:

  std::string kinectConfig;                     ///< Kinect configuration file
  CvSize captureSize;                           ///< Size of captured images from kinect
  Kinect::Kinect *kinect;                       ///< The kinect hardware interface.

  void getResolution(int camIdx, CvSize &size);
  bool setResolution(int camIdx, CvSize &size);

  DepthGenerator* depthGenerator;
  ImageGenerator* imageGenerator;
  DepthMetaData depthMD;
  ImageMetaData imageMD;

  bool m_saveToFile;
  bool m_displayImage;
  std::string m_saveDirectory;

protected:
  virtual void configure(const std::map<std::string,std::string> & _config) throw(std::runtime_error);
  virtual void start();
  virtual void runComponent();
  
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
  void saveNextFrameToFile();
};

}

#endif

