/**
 * @author Michael Zillich
 * @date February 2009
 *
 * Just receives images and displays them.
 * A dummy component showing how to get images.
 */

#ifndef VIDEO_VIEWER_H
#define VIDEO_VIEWER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>

namespace cast
{

class VideoViewer : public VideoClient,
                    public ManagedComponent
{
private:
  /**
   * Which camera to get images from
   */
  int camId;

protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start();
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent();

public:
  VideoViewer() : camId(0) {}
  virtual ~VideoViewer() {}
};

}

#endif


