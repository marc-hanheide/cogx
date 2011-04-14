/**
 * @author Michael Zillich
 * @date June 2009
 */

#include <sstream>
#include <Ice/Ice.h>
#include <cast/core/CASTUtils.hpp>
#include "StereoClient.h"

namespace cast
{

using namespace std;

StereoClient::StereoClient()
{
  stereoServerHost = "localhost";
  stereoServerName = "";
  stereoServerPort = cdl::CPPSERVERPORT;
}

void StereoClient::startStereoCommunication(CASTComponent &owner)
  throw(runtime_error)
{
  try {
    stereoServer = owner.getIceServer<Stereo::StereoInterface>(stereoServerName);
    owner.debug("StereoClient Connected.");
  }
  catch (...) {
    owner.println(" *** StereoClient could not connect to '%s'.", stereoServerName.c_str());
    throw runtime_error(exceptionMessage(__HERE__,
          "failed to connect to stereo server: %s",
          stereoServerName.c_str()));
  }
}

void StereoClient::configureStereoCommunication(const map<string,string> & _config)
  throw(runtime_error)
{
  map<string,string>::const_iterator it;

  //if((it = _config.find("--stereohost")) != _config.end())
  //{
  //  stereoServerHost = it->second;
  //}
  if((it = _config.find("--stereoname")) != _config.end())
  {
    stereoServerName = it->second;
  }

  // sanity checks: Have all important things be configured? Is the
  // configuration consistent?
  //if(stereoServerHost.empty())
  //  throw runtime_error(exceptionMessage(__HERE__, "no stereo server host given"));
  if(stereoServerName.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no stereo server name given"));
}

void StereoClient::getPoints(bool transformToGlobal, int imgWidth, VisionData::SurfacePointSeq& points)
{
  stereoServer->getPoints(transformToGlobal, imgWidth, points);
}

void StereoClient::getCompletePoints(bool transformToGlobal, int imgWidth, VisionData::SurfacePointSeq& points)
{
  stereoServer->getCompletePoints(transformToGlobal, imgWidth, points);
}

void StereoClient::getRectImage(int side, int imgWidth, Video::Image& image)
{
  stereoServer->getRectImage(side, imgWidth, image);
}

void StereoClient::getDisparityImage(int imgWidth, Video::Image& image)
{
  stereoServer->getDisparityImage(imgWidth, image);
}

}
