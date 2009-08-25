/**
 * @file VisionSystem3.h
 * @author Andreas Richtsfeld
 * @date April 2008
 * @version 0.1
 * @brief Management component for running simple object detector (vs3)
 */

#ifndef VISION_SYSTEM_3_H
#define VISION_SYSTEM_3_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>
#include <Vs3Interface.h>
#include <VisionUtils.h>
// #include <Image.hh>

namespace cast
{

class VisionSystem3 : public VideoClient,
                      public ManagedComponent
{
private:
  /**
   * Which camera to get images from
   */
  int camId;

	/**
	 *	Interface for working with vision system 3 (vs3)
	 */
	Vs3Interface *vs3Interface;

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
  VisionSystem3() : camId(0) {}
  virtual ~VisionSystem3() {}
};

}

#endif



