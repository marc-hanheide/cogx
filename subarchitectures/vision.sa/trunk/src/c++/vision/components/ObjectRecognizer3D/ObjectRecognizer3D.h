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
#include "SDraw.hh"
#include "ModelObject3D.hh"
#include "KeypointDescriptor.hh"
#include "matrix.h"

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>

#include "ObjectRecognizer3DUtils.hpp"
#include "ObjectTrackerUtils.hpp"
#include "Tracker.h"
#include "ModelLoader.h"

namespace cast
{

class ObjectRecognizer3D : public VideoClient, public ManagedComponent
{
private:
	P::Array<P::KeypointDescriptor*> image_keys;
	P::Array<P::KeypointDescriptor*> temp_keys;
	P::ModelObject3D	sift_model_learner;
	P::Object3D				object;
  
  Video::Image m_image;
  int m_width;
  int m_height;
  int camId;
  std::vector<int> camIds;								///< Which cameras to get images from
  std::string videoServerName;
  Video::VideoInterfacePrx videoServer;
  
  std::string m_modelID;
  std::string m_plyfile;
  std::string m_siftfile;
  
  bool m_learnmode;
  
  /** @brief list of objects we want to have detected */
  std::vector<std::string> labels;
  
  void loadVisualModelToWM(std::string filename, std::string& modelID, cogx::Math::Pose3 pose);
  void startTracker();
  void addTrackerModel(std::string& modelID);
  void lockTrackerModel(std::string& modelID);
  void unlockTrackerModel(std::string& modelID);
  void get3DPointFromTrackerModel(std::string& modelID, VisionData::VertexSeq& vertexlist);
  
  void receiveTrackingCommand(const cdl::WorkingMemoryChange & _wmc);
  
  void learnSiftModel();
  void recognizeSiftModel();

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

