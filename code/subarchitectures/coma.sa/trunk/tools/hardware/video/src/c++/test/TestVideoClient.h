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

class TestVideoClient : public VideoClient,
                        public CASTComponent
{
public:
  virtual ~TestVideoClient() {}
  virtual void start();
  virtual void configure(const std::map<std::string,std::string> & _config)
    throw(std::runtime_error);
  virtual void runComponent();
};

}

#endif


