/**
 * @author Michael Zillich
 * @date February 2009
 */

#ifndef VIDEO_CLIENT_H
#define VIDEO_CLIENT_H

#include <stdexcept>
#include <vector>
#include <string>
#include <map>
#include <cast/core/CASTComponent.hpp>
#include "Video.hpp"

namespace cast
{

/**
 * Client to a VideoServer.
 * Inherit from this class if You want to connect to video servers.
 */
class VideoClient
{
protected:
  class VideoClientI : virtual public Video::VideoClientInterface
  {
  private:
    VideoClient *vidClt;

  public:
    VideoClientI(VideoClient *clt) : vidClt(clt) {}
    virtual void receiveImages(const Video::ImageSeq &images, const Ice::Current&)
    {
      vidClt->receiveImages(images);
    }
    virtual void receiveImages2(const std::string& serverName, const Video::ImageSeq &images, const Ice::Current&)
    {
      vidClt->receiveImages(serverName, images);
    }
  };

public:
  /**
   * The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   */
  virtual void receiveImages(const std::vector<Video::Image>& images) {}

  virtual void receiveImages(const std::string& serverName, const std::vector<Video::Image>& images)
  {
    receiveImages(images);
  }
};

}

#endif


