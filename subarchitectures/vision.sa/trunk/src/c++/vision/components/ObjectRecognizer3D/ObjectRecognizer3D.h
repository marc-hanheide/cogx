 /**
 * @file ObjectRecognizer3D.h
 * @author Hannes Prankl, Thomas Mörwald, Michael Zillich (zillich@acin.tuwien.ac.at)
 * @date Januray 2010
 * @version 0.1
 * @brief Learning and recognizing pose of Object using SIFT features (cooperates with ObjetTracker)
 * @namespace Tracking
 */
#ifndef OBJECT_RECOGNIZER_3D_H
#define OBJECT_RECOGNIZER_3D_H

#include <vector>
#include <string>
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>
#include "DetectGPUSIFT.hh"
#include "ODetect3D.hh"
#include "Object3D.hh"
#include "ModelObject3D.hh"
#include "KeypointDescriptor.hh"
#include "matrix.h"

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>

#include "ObjectTrackerUtils.hpp"
#include "Tracker.h"
#include "ModelLoader.h"
#include "tgEngine.h"
#include "VirtualSceneUtils.hpp"

namespace cast
{

class ObjectRecognizer3D : public VideoClient, public ManagedComponent
{
private:
	P::Array<P::KeypointDescriptor*> keys;
	P::DetectGPUSIFT sift;
	P::ModelObject3D model;
  P::ODetect3D detect;
  
  Video::Image m_image;
  int m_width;
  int m_height;
  int camId;
  std::vector<int> camIds;								///< Which cameras to get images from
  std::string videoServerName;
  Video::VideoInterfacePrx videoServer;
  
  TomGine::tgEngine m_engine;
  TomGine::tgCamera m_camera;
  float m_fTime;
    
  std::string m_plyfile;
  
  /** @brief list of objects we want to have detected */
  std::vector<std::string> labels;
  
  /** @brief callback function called whenever a new object appears ore an object changes */
  void receiveVisualObject(const cdl::WorkingMemoryChange & _wmc);
  
  void initSzene(const Video::Image &image);

protected:

  /** @brief called by the framework to configure our component */
  virtual void configure(const std::map<std::string,std::string> & _config);
  
  /** @brief called by the framework after configuration, before run loop */
  virtual void start();
  
  /** @brief called by the framework to start compnent run loop */
  virtual void runComponent();
 
public:
	ObjectRecognizer3D() : camId(0) {}
  virtual ~ObjectRecognizer3D() {}
  
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif

