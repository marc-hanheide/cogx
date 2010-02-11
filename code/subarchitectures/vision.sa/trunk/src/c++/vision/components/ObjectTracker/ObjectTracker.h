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
#include <semaphore.h>

#include "Tracker.h"
#include "EdgeTracker.h"
#include "TextureTracker.h"
#include "Timer.h"
#include "ObjectTrackerUtils.hpp"
#include "TrackingEntry.h"

namespace cast
{



class ObjectTracker : public VideoClient, public ManagedComponent
{
private:
	Tracking::Tracker* m_tracker;
  Tracking::Camera m_camera;
  Timer m_timer;
  Video::Image m_image;
  Parameters m_params;
  int m_ImageWidth;
  int m_ImageHeight;
  float fTimeTracker;
  cdl::CASTTime last_image_time;
  std::string m_ini_file;
  
  /** Which camera to get images from */
  int m_camId;
  /** component ID of the video server to connect to */
  std::string m_videoServerName;
  /** our ICE proxy to the video server */
  Video::VideoInterfacePrx m_videoServer;
	int m_maxModels;
  bool m_track;
  bool m_running;
  bool m_testmode;
  bool m_textured;
  bool m_bfc;						// backface culling (disable for not closed surfaces, like polyflaps)
  
  typedef std::vector<TrackingEntry*> TrackingEntryList;
  TrackingEntryList m_trackinglist;
  
  std::vector<VisionData::TrackingCommandPtr> m_trackingCommandList;
  std::vector<std::string> m_trackingCommandWMID;
  
  // Functions with GL commands allowed
  void initTracker();
  void runTracker();
  void applyTrackingCommand();
  
  // Do not use GL commands in this functions (different thread with no GL context)
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



