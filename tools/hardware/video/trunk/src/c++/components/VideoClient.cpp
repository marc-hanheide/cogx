/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <sstream>
#include <Ice/Ice.h>
#include <cast/core/CASTUtils.hpp>
#include "VideoClient.h"

namespace cast
{

using namespace std;

VideoClient::VideoClient()
{
  videoServerHost = "localhost";
  videoServerName = "";
  videoServerPort = cdl::CPPSERVERPORT;
}

void VideoClient::startVideoCommunication(CASTComponent &owner)
  throw(runtime_error)
{
  ostringstream videoServerAddr;
  Ice::Identity id;

  id.name = videoServerName;
  id.category = "VideoServer";
  videoServerAddr << owner.getCommunicator()->identityToString(id)
    << ":default -h " << videoServerHost << " -p " << videoServerPort;

  Ice::ObjectPrx base = owner.getCommunicator()->stringToProxy(videoServerAddr.str());
  // doing a checkedCast here freezes the server
  // videoServer = Video::VideoInterfacePrx::checkedCast(base);
  videoServer = Video::VideoInterfacePrx::uncheckedCast(base);
  if(!videoServer)
    throw runtime_error(exceptionMessage(__HERE__,
          "failed to connect to video server: %s",
          videoServerAddr.str().c_str()));
}

void VideoClient::configureVideoCommunication(const map<string,string> & _config)
  throw(runtime_error)
{
  map<string,string>::const_iterator it;

  if((it = _config.find("--videohost")) != _config.end())
  {
    videoServerHost = it->second;
  }
  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }
  /*if((it = _config.find("--videoport")) != _config.end())
  {
    istringstream str(it->second);
    str >> videoServerPort;
  }*/

  // sanity checks: Have all important things be configured? Is the
  // configuration consistent?
  if(videoServerHost.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no video server host given"));
  if(videoServerName.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no video server name given"));
}

int VideoClient::getNumCameras()
{
  return videoServer->getNumCameras();
}

void VideoClient::getImageSize(int& width, int& height)
{
  videoServer->getImageSize(width, height);
}

int VideoClient::getFramerateMilliSeconds()
{
  return videoServer->getFramerateMilliSeconds();
}

void VideoClient::getImage(int camId, Video::Image& image)
{
  videoServer->getImage(camId, image);
}

void VideoClient::getImages(Video::ImageSeq& images)
{
  videoServer->getImages(images);
}

void VideoClient::getScaledImages(int width, int height, Video::ImageSeq& images)
{
  videoServer->getScaledImages(width, height, images);
}

}

