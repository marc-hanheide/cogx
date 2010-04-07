 /**
 * @file VirtualScene.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Component for rendering all VisualObjects in the Working Memory.
 */

#ifndef VIRTUAL_SCENE_H
#define VIRTUAL_SCENE_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>
#include <vector>
#include <string>
#include <cogxmath.h>

#include "tgEngine.h"
#include "tgFont.h"
#include "ModelEntry.h"
#include "tgModelLoader.h"

namespace cast
{

enum VirtualSceneCmd{ 
	ADDVISUALOBJECT, OVERWRITEVISUALOBJECT, DELETEVISUALOBJECT,
	ADDCONVEXHULL, OVERWRITECONVEXHULL, DELETECONVEXHULL,
	ADDSOI, OVERWRITESOI, DELETESOI
};

struct VirtualSceneChange{
	cdl::WorkingMemoryChange wmc;
	VirtualSceneCmd cmd;
};


class VirtualScene : public VideoClient, public ManagedComponent
{
private:
  Video::Image m_image;
  int m_width;
  int m_height;
  
  TomGine::tgEngine* m_engine;
  TomGine::tgCamera m_camera0, m_camera;
	TomGine::tgTimer m_timer;
  cogx::Math::Pose3 m_cam_pose;
  TomGine::tgFont* m_font;
  std::string m_fontfilename;
  std::vector<TomGine::tgEvent> m_eventlist;
  std::vector<VirtualSceneChange> m_vsc;
  
  // Models
  ModelEntry m_camModel;
  std::vector<ModelEntry*> m_VisualObjectList;
  std::vector<ModelEntry*> m_ConvexHullList;
  std::vector<ModelEntry*> m_SOIList;
  
  cogx::Math::Vector3 m_cor;	///< center of rotation of the scene
  cogx::Math::Vector3 m_coo;	///< center of rotation of the scene
  int	m_cor_num;
  

  /** Which camera to get images from */
  int m_camId;
  
  /** component ID of the video server to connect to */
  std::string m_videoServerName;
  
  /** our ICE proxy to the video server */
  Video::VideoInterfacePrx m_videoServer;
	int m_maxModels;
	int m_objectType;
  bool m_render;
  bool m_running;
  bool m_lock;
  bool m_wireframe;
  bool m_normals;
  bool m_labels;
  bool m_drawcamera;
  float m_fTime;
 
  // Functions with GL commands allowed
  void initScene(const Video::Image &image);
  void updateCameraViews();
  void updateGL();
  void applyVirtualSceneCmd();
  
  void drawCamera();
  void drawVisualObjects();
  void drawConvexHulls();
  void drawSOIs();
  void inputControl();
  TomGine::tgRenderModel::Material getRandomMaterial(float alpha=1.0);
  TomGine::vec4 getRandomColor(float alpha=1.0);
  
  // Do not use GL commands in this functions (different thread with no GL context)
  void addVisualObject(const cdl::WorkingMemoryChange & _wmc);
  void overwriteVisualObject(const cdl::WorkingMemoryChange & _wmc);
  void deleteVisualObject(const cdl::WorkingMemoryChange & _wmc);
  
  void addConvexHull(const cdl::WorkingMemoryChange & _wmc);
  void overwriteConvexHull(const cdl::WorkingMemoryChange & _wmc);
  void deleteConvexHull(const cdl::WorkingMemoryChange & _wmc);
  
  void addSOI(const cdl::WorkingMemoryChange & _wmc);
  void overwriteSOI(const cdl::WorkingMemoryChange & _wmc);
  void deleteSOI(const cdl::WorkingMemoryChange & _wmc);

protected:
  /** called by the framework to configure our component */
  virtual void configure(const std::map<std::string,std::string> & _config);
  
  /** called by the framework after configuration, before run loop */
  virtual void start();
  
  /** called by the framework upon deletion of the component */
  virtual void destroy();
  
  /** called by the framework to start compnent run loop */
  virtual void runComponent();

public:
  VirtualScene();
  virtual ~VirtualScene();
  
  /** The callback function for images pushed by the image server.
   * To be overwritten by derived classes. */
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif



