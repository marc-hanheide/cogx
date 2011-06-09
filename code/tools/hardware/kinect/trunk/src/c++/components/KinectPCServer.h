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
  bool saveDepth;

  void getResolution(int camIdx, CvSize &size);
  bool setResolution(int camIdx, CvSize &size);

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
  void receiveImages(const std::vector<Video::Image>& images);
  bool getCameraParameters(Ice::Int side, Video::CameraParameters& camPars);;
  void saveDepthToFile();
};

}

#endif

