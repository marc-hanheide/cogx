/**
 * @author Michael Zillich
 * @date June 2009
 *
 * Just receives stereo point clouds and displays them.
 */

#ifndef STEREO_VIEWER_H
#define STEREO_VIEWER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <StereoClient.h>

namespace cast
{

class StereoViewer : public StereoClient,
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
  StereoViewer() : camId(0) {}
  virtual ~StereoViewer() {}
};

}

#endif



