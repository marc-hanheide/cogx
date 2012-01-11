/**
 * @file PointCloudServer.h
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Point cloud server for the cast-framework
 */

#ifndef POINT_CLOUD_SERVER_H
#define POINT_CLOUD_SERVER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <stdexcept>
#include <vector>
#include <map>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <VideoClient.h>

#include "StereoCamera.h"
#include "Math.hpp"
#include "PointCloud.hpp"

#ifdef HAVE_GPU_STEREO
#include "gpustereo/CensusGPU.h"
#endif

namespace cast
{

class PointCloudServer;

/**
 * @brief Ice interface to a point cloud server.
 */
class PointCloudServerI : public PointCloud::PointCloudInterface
{
private:
  PointCloudServer *ptCloudSrv;

public:
  PointCloudServerI(PointCloudServer *_ptCloud) : ptCloudSrv(_ptCloud) {}

  virtual void getPoints(bool transformToGlobal, int imgWidth, PointCloud::SurfacePointSeq& points, const Ice::Current&);
  virtual void getCompletePoints(bool transformToGlobal, int imgWidth, PointCloud::SurfacePointSeq& points, const Ice::Current&);
  virtual void getRectImage(Ice::Int camId, int imgWidth, Video::Image& image, const Ice::Current&);
  virtual void getDisparityImage(int imgWidth, Video::Image& image, const Ice::Current&);
  virtual void getDepthMap(cast::cdl::CASTTime &time, vector<int>& data, const Ice::Current&);
  virtual void getRangePoints(Laser::Scan2d &KRdata, const Ice::Current&);
  virtual bool getCameraParameters(Ice::Int camId, Video::CameraParameters& camPars, const Ice::Current&);
  virtual bool isPointInViewCone(const cogx::Math::Vector3&, const Ice::Current&);
  virtual bool isPointVisible(const cogx::Math::Vector3&, const Ice::Current&);
};


/**
 * @brief Point cloud server implementation as CAST component.
 */
class PointCloudServer : virtual public ManagedComponent
{
private:
  /**
   * @brief The ICE point cloud server instance
   */
  PointCloud::PointCloudInterfacePtr hPointCloudServer;
  
protected:
  
  /**
   * @brief Camera IDs camIds[sourceNum]
   * e.g. camIds[0] = 3; camIds[1] = 4;
   */
  std::vector<int> camIds;

  /**
   * @brief Camera parameters for the video sources
   */
  std::vector<Video::CameraParameters> camPars;
  
  
  virtual void configure(const std::map<std::string,std::string> & _config) throw(std::runtime_error);
  virtual void runComponent() {}
  virtual void start();
  virtual void receiveCameraParameters(const cdl::WorkingMemoryChange & _wmc);
  virtual void receivePTZCommand(const cdl::WorkingMemoryChange & _wmc);
  bool suspendReading;


public:
  PointCloudServer();
  virtual ~PointCloudServer();

  /**
   * @brief Returns the 3D point cloud.
   * @param transformToGlobal If true use the camera's pose to return points in global coordinates.
   *        Otherwise return in left camera coordinates.
   * @param imgWidth Specifies at which image resolution stereo matching should be performed.
   *        The nearest available resolution will be used.
   * @param points The 3D points with their color.
   * @param complete If false pixels with no available disparity are discarded. So there is
   *        no correspondence of the resulting point cloud and the rectangular image pixel grid.
   *        If true pixels with no available disparity will result in points with (0, 0, 0).
   *        The rectangular grid structure of points is thus maintained, which can help in finding
   *        nearest neighours etc.
   */
  virtual void getPoints(bool transformToGlobal, int imgWidth, std::vector<PointCloud::SurfacePoint> &points, bool complete) {}
  
  /**
   * @brief Get rectified image
   * @param camID Camera id.
   * @param imgWidth Image width
   * @param image Rectified image
   */
  virtual void getRectImage(int camID, int imgWidth, Video::Image& image) {}
  
  /**                                                                                                         /// TODO TODO
   * @brief Get disparity image of ....
   * @param imgWidth Image width
   * @param image Rectified image
   */  
  virtual void getDisparityImage(int imgWidth, Video::Image& image) {}

  virtual void getDepthMap(cast::cdl::CASTTime &time, vector<int>& data) {}
  virtual bool isPointInViewCone(const cogx::Math::Vector3& point) { return true; }

  // @brief Check if the point is visible by all cameras that generate the point cloud.
  virtual bool isPointVisible(const cogx::Math::Vector3&);

  /**
   * @brief Get camera parameters of one camera
   * @param camID Camera id.
   * @param camPars Camera parameters.
   */    
  virtual bool getCameraParameters(int camID, Video::CameraParameters& camPars) {return false;}

  /**
   * The callback function for 2D points extracted from the Kinect depth data
   * @author Rasoul Mojtahedzadeh
   */
  virtual void getRangePoints(Laser::Scan2d &KRdata) {}
  /**
   * The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   * @param images Images to be received.
   */
  virtual void receiveImages(const std::vector<Video::Image>& images) {}
};

}

#endif

