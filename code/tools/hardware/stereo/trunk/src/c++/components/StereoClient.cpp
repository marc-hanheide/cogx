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
  ostringstream stereoServerAddr;
  Ice::Identity id;

  id.name = stereoServerName;
  id.category = "StereoServer";
  stereoServerAddr << owner.getCommunicator()->identityToString(id)
    << ":default -h " << stereoServerHost << " -p " << stereoServerPort;

  Ice::ObjectPrx base = owner.getCommunicator()->stringToProxy(stereoServerAddr.str());
  // doing a checkedCast here freezes the server
  // stereoServer = Stereo::StereoInterfacePrx::checkedCast(base);
  stereoServer = Stereo::StereoInterfacePrx::uncheckedCast(base);
  if(!stereoServer)
    throw runtime_error(exceptionMessage(__HERE__,
          "failed to connect to stereo server: %s",
          stereoServerAddr.str().c_str()));
}

void StereoClient::configureStereoCommunication(const map<string,string> & _config)
  throw(runtime_error)
{
  map<string,string>::const_iterator it;

  if((it = _config.find("--stereohost")) != _config.end())
  {
    stereoServerHost = it->second;
  }
  if((it = _config.find("--stereoname")) != _config.end())
  {
    stereoServerName = it->second;
  }

  // sanity checks: Have all important things be configured? Is the
  // configuration consistent?
  if(stereoServerHost.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no stereo server host given"));
  if(stereoServerName.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no stereo server name given"));
}

void StereoClient::getPoints(bool transformToGlobal, VisionData::SurfacePointSeq& points)
{
  stereoServer->getPoints(transformToGlobal, points);
}

void StereoClient::getPointsInSOI(bool transformToGlobal, const VisionData::SOI &soi,
    VisionData::SurfacePointSeq& points)
{
  VisionData::SOIPtr soiPtr = new VisionData::SOI;
  *soiPtr = soi;
  stereoServer->getPointsInSOI(transformToGlobal, soiPtr, points);
}

void StereoClient::getRectImage(int side, Video::Image& image)
{
  stereoServer->getRectImage(side, image);
}

void StereoClient::getDisparityImage(Video::Image& image)
{
  stereoServer->getDisparityImage(image);
}

}

