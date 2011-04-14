/**
 * @file PointCloudClient.cpp
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Point cloud client for the cast-framework.
 */

#include <sstream>
#include <Ice/Ice.h>
#include <cast/core/CASTUtils.hpp>
#include "PointCloudClient.h"

namespace cast
{

using namespace std;

PointCloudClient::PointCloudClient()
{
//   stereoServerHost = "localhost";
  pointCloudServerName = "";
//  stereoServerPort = cdl::CPPSERVERPORT;
}

void PointCloudClient::configureServerCommunication(const map<string,string> & _config) throw(runtime_error)
{
  map<string,string>::const_iterator it;

  //if((it = _config.find("--stereohost")) != _config.end())
  //{
  //  stereoServerHost = it->second;
  //}
  
  if((it = _config.find("--servername")) != _config.end())
  {
    pointCloudServerName = it->second;
  }

  // sanity checks: Have all important things be configured? Is the
  // configuration consistent?
  //if(stereoServerHost.empty())
  //  throw runtime_error(exceptionMessage(__HERE__, "no stereo server host given"));
  
  if(pointCloudServerName.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no stereo server name given"));
}

void PointCloudClient::startPCCServerCommunication(CASTComponent &owner) throw(runtime_error)
{
  try {
    pointCloudServer = owner.getIceServer<PointCloud::PointCloudInterface>(pointCloudServerName);
    owner.debug("PointCloudClient Connected.");
  }
  catch (...) {
    owner.println(" *** PointCloudClient could not connect to '%s'.", pointCloudServerName.c_str());
    throw runtime_error(exceptionMessage(__HERE__, "failed to connect to point cloud server: %s", pointCloudServerName.c_str()));
  }
}

void PointCloudClient::getPoints(bool transformToGlobal, int imgWidth, PointCloud::SurfacePointSeq& points)
{
  pointCloudServer->getPoints(transformToGlobal, imgWidth, points);
}

void PointCloudClient::getCompletePoints(bool transformToGlobal, int imgWidth, PointCloud::SurfacePointSeq& points)
{
  pointCloudServer->getCompletePoints(transformToGlobal, imgWidth, points);
}

void PointCloudClient::getRectImage(int side, int imgWidth, Video::Image& image)
{
  pointCloudServer->getRectImage(side, imgWidth, image);
}

void PointCloudClient::getDisparityImage(int imgWidth, Video::Image& image)
{
  pointCloudServer->getDisparityImage(imgWidth, image);
}

}
