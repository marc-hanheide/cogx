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
#include <opencv/cv.h>
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
  virtual void getRectImage(Ice::Int side, int imgWidth, Video::Image& image, const Ice::Current&);
  virtual void getDisparityImage(int imgWidth, Video::Image& image, const Ice::Current&);
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
  void receiveCameraParameters(const cdl::WorkingMemoryChange & _wmc);


public:
  PointCloudServer();
  virtual ~PointCloudServer();

  /**
   * Returns the 3D point cloud.
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
  virtual void getRectImage(int side, int imgWidth, Video::Image& image) {}
  virtual void getDisparityImage(int imgWidth, Video::Image& image) {}

  /**
   * The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   */
  virtual void receiveImages(const std::vector<Video::Image>& images) {}
};

}

#endif

