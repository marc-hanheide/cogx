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
  pointCloudServerName = "";
}

void PointCloudClient::configureServerCommunication(const map<string,string> & _config) throw(runtime_error)
{
  map<string,string>::const_iterator it;
  
  if((it = _config.find("--pcserver")) != _config.end())
  {
    pointCloudServerName = it->second;
  } else throw runtime_error(exceptionMessage(__HERE__, "no point cloud server name given"));
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

void PointCloudClient::getDepthMap(cast::cdl::CASTTime &time, vector<int>& depth)
{
  pointCloudServer->getDepthMap(time, depth);
}

void PointCloudClient::getRangePoints(Laser::Scan2d &KRdata)
{
  pointCloudServer->getRangePoints(KRdata);
}

void PointCloudClient::getDisparityImage(int imgWidth, Video::Image& image)
{
  pointCloudServer->getDisparityImage(imgWidth, image);
}

bool PointCloudClient::getCameraParameters(Ice::Int side, Video::CameraParameters& camPars)
{
  return pointCloudServer->getCameraParameters(side, camPars);
}

bool PointCloudClient::isPointInViewCone(const cogx::Math::Vector3& point)
{
  return pointCloudServer->isPointInViewCone(point);
}

bool PointCloudClient::isPointVisible(const cogx::Math::Vector3& point)
{
  return pointCloudServer->isPointVisible(point);
}

}
