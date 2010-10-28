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
using namespace Video;

VideoClient::VideoClient()
{
  videoServerHost = "localhost";
  videoServerName = "";
}

void VideoClient::startVideoCommunication(CASTComponent &owner)
  throw(runtime_error)
{
  videoServer = getIceServer<VideoInterface>(videoServerName);
//  VideoInterfacePrx vidk(getIceServer<VideoInterface>(videoServerName));
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

void VideoClient::getImage(int camId, Image& image)
{
  videoServer->getImage(camId, image);
}

void VideoClient::getImages(ImageSeq& images)
{
  videoServer->getImages(images);
}

void VideoClient::getScaledImages(int width, int height, ImageSeq& images)
{
  videoServer->getScaledImages(width, height, images);
}

}

