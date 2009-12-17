/**
 * @author Thomas Mörwald
 * @date April 2009
 *
 * Tracks Objects using either edge or texture features
 */

#ifndef VIRTUAL_SCENE_H
#define VIRTUAL_SCENE_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>
#include <vector>
#include <string>

#include "tgEngine.h"
#include "VirtualSceneUtils.hpp"

namespace cast
{

class VirtualScene : public VideoClient, public ManagedComponent
{
private:
  Video::Image m_image;
  int m_width;
  int m_height;
  
  tgEngine m_engine;
  tgCamera* m_camera;
  vector<tgModel> m_modellist;

  /**
   * Which camera to get images from
   */
  int m_camId;
  /**
   * component ID of the video server to connect to
   */
  std::string m_videoServerName;
  /**
   * our ICE proxy to the video server
   */
  Video::VideoInterfacePrx m_videoServer;
	int m_maxModels;
  bool m_render;
  bool m_running;
 
  // Functions with GL commands allowed
  void initScene(const Video::Image &image);
  void runScene();
  
  // Do not use GL commands in this functions (different thread with no GL context)
  void receiveVisualObject(const cdl::WorkingMemoryChange & _wmc);
  void changeVisualObject(const cdl::WorkingMemoryChange & _wmc);
  void removeVisualObject(const cdl::WorkingMemoryChange & _wmc);

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
  VirtualScene();
  virtual ~VirtualScene();
  /**
   * The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   */
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif



