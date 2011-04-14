/**
 * @file PointCloudClient.h
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Point cloud client for the cast-framework.
 */

#ifndef POINT_CLOUD_CLIENT_H
#define POINT_CLOUD_CLIENT_H

#include <stdexcept>
#include <vector>
#include <string>
#include <map>
#include <cast/core/CASTComponent.hpp>
#include "PointCloud.hpp"

namespace cast
{

/**
 * @brief Client to a PointCloudServer.
 * Inherit from this class if You want to connect to point cloud servers.
 * You will have to call configureServerCommunication() and
 * startServerCommunication() in that order from somewhere in Your code,
 * probably from the configure() and start() methods of Your CAST component.
 */
class PointCloudClient
{
private:
  //std::string stereoServerHost;											/// TODO Unused?
  std::string pointCloudServerName;
//   int stereoServerPort;												/// TODO Unused?
  PointCloud::PointCloudInterfacePrx pointCloudServer;

protected:
  void configureServerCommunication(const std::map<std::string,std::string> & _config) throw(std::runtime_error);
  void startPCCServerCommunication(CASTComponent &owner) throw(std::runtime_error);

public:
  PointCloudClient();

  /**
   * Get data from the point cloud client
   */
  void getPoints(bool transformToGlobal, int imgWidth, PointCloud::SurfacePointSeq& points);
  void getCompletePoints(bool transformToGlobal, int imgWidth, PointCloud::SurfacePointSeq& points);
  void getRectImage(int side, int imgWidth, Video::Image& image);
  void getDisparityImage(int imgWidth, Video::Image& image);
};

}

#endif


