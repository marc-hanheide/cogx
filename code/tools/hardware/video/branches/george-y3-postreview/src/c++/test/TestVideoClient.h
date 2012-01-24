/**
 * @author Michael Zillich
 * @date February 2009
 */

#ifndef TEST_VIDEO_CLIENT_H
#define TEST_VIDEO_CLIENT_H

#include <cast/core/CASTComponent.hpp>
#include <VideoClient.h>

namespace cast
{

class TestVideoClient : public CASTComponent,
                        public VideoClient
{
private:
  /**
   * Which camera to get images from
   */
  std::vector<int> camIds;
  /**
   * component ID of the video server to connect to
   */
  std::string videoServerName;
  /**
   * our ICE proxy to the video server
   */
  Video::VideoInterfacePrx videoServer;

protected:
  virtual void start();
  virtual void configure(const std::map<std::string,std::string> & _config)
    throw(std::runtime_error);

public:
  virtual ~TestVideoClient() {}
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif


