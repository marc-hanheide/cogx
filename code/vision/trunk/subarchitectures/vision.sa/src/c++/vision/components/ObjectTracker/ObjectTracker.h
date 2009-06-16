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

#include "EdgeTracker.h"
#include "Timer.h"
#include "ObjectTrackerUtils.hpp"
#include "mxCameraModel.h"

namespace cast
{

class ObjectTracker : public VideoClient, public ManagedComponent
{
private:
  /**
   * Which camera to get images from
   */
  Tracker* m_tracker;
  Camera* m_camera;
  Video::Image m_image;
  Timer m_timer;
  Particle m_trackpose;
  
  mat4 m_cv2gl;
  mat4 m_extrinsic;
  mat4 m_intrinsic;
  float zFar, zNear;
  vec4 v1;
  vec4 v2;
  vec4 v3;
  vec4 v4;
  vec4 v5;
  
  int camId;  
  bool track;
  bool running;
  bool testmode;
  
  struct IDList{
  	int resources_ID;
  	cdl::WorkingMemoryAddress cast_AD;
  };
  
  std::vector<IDList> m_modelID_list;
  
  void initTracker();
  void runTracker();
  
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
   * called by the framework to start compnent run loop
   */
  virtual void runComponent();

public:
  ObjectTracker();
  virtual ~ObjectTracker();
};

}

#endif



