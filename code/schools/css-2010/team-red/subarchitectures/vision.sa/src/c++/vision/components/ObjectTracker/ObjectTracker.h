/**
 * @author Thomas Mörwald
 * @date April 2009
 *
 * Tracks Objects using either edge or texture features
 */

#ifndef OBJECT_TRACKER_H
#define OBJECT_TRACKER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>
#include <vector>
#include <string>

#include "Tracker.h"
#include "EdgeTracker.h"
#include "TextureTracker.h"
#include "Timer.h"
#include "ObjectTrackerUtils.hpp"

namespace cast
{

class ObjectTracker : public VideoClient, public ManagedComponent
{
private:
  Tracker* m_tracker;
  Camera* m_camera;
  Timer m_timer;
  Video::Image m_image;
  //Particle m_trackpose;
  
  /**
   * Which camera to get images from
   */
  int camId;
  /**
   * component ID of the video server to connect to
   */
  std::string videoServerName;
  /**
   * our ICE proxy to the video server
   */
  Video::VideoInterfacePrx videoServer;
	int m_maxModels;
  bool track;
  bool running;
  bool testmode;
  bool bfc;						// backface culling (disable for not closed surfaces, like polyflaps)
  bool trackTexture;	// use texture to track object (slower but more accurate [only for hi-end systems])
  
  struct IDList{
  	int resources_ID;
  	cdl::WorkingMemoryAddress cast_AD;
  	Particle trackpose;
  };
  
  std::vector<IDList> m_modelID_list;
  
  void initTracker(const Video::Image &image);
  void runTracker(const Video::Image &image);
  
  void receiveVisualObject(const cdl::WorkingMemoryChange & _wmc);
  void receiveTrackingCommand(const cdl::WorkingMemoryChange & _wmc);

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
   * called by the framework upon deletion of the component
   */
  virtual void destroy();
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent();

public:
  ObjectTracker();
  virtual ~ObjectTracker();
  /**
   * The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   */
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif



