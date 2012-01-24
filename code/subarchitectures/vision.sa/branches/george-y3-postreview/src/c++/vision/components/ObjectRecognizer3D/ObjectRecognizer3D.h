 /**
 * @file ObjectRecognizer3D.h
 * @author Thomas Mörwald, Hannes Prankl
 * @date Januray 2010
 * @version 0.1
 * @brief Learning and recognizing pose of Object using SIFT features (cooperates with ObjetTracker)
 * @namespace Tracking
 */
#ifndef OBJECT_RECOGNIZER_3D_H
#define OBJECT_RECOGNIZER_3D_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <cxcore.h>
#include "DetectGPUSIFT.hh"
#include "ODetect3D.hh"
#include "Object3D.hh"
#include "SDraw.hh"
#include "ModelObject3D.hh"
#include "KeypointDescriptor.hh"
#include "matrix.h"

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <PointCloudClient.h>
#include <VisionData.hpp>
#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

#include "ObjectRecognizer3DUtils.hpp"
#include "ObjectTrackerUtils.hpp"
#include "Tracker.h"

namespace cast
{

class ObjectRecognizer3D : public VideoClient, public PointCloudClient, public ManagedComponent
{
private:
	VisionData::Recognizer3DCommandType m_task;

	P::Array<P::KeypointDescriptor*> m_image_keys;
	P::Array<P::KeypointDescriptor*> m_temp_keys;
	P::ModelObject3D	sift_model_learner;
	P::ODetect3D*			m_detect;

  IplImage *m_iplImage;
  IplImage *m_iplGray;

  std::vector<VisionData::Recognizer3DCommandPtr> m_recCommandList;
  std::vector<std::string> m_recCommandID;
  VisionData::Recognizer3DCommandPtr m_rec_cmd;
  std::string m_rec_cmd_id;

  Video::Image m_image;
  int m_width;
  int m_height;
  int camId;
  std::string videoServerName;
  std::string pcServerName;
  Video::VideoInterfacePrx videoServer;
  /**
   * initial pose for tracking when learning a new object
   */
  cogx::Math::Pose3 initPose;

  struct RecEntry{
  	std::string siftfile;
  	std::string plyfile;
  	P::Object3D* object;
  	std::string visualObjectID;
  	bool learn;
  	RecEntry(){
  		object = 0;
  		learn = false;
  	}
  };
	std::map<std::string,RecEntry> m_recEntries;
	std::string m_label;

	float m_confidence;

  bool m_simulationOnly;
  bool m_starttask;
  bool m_wait4data;
  bool m_delete_command_from_wm;
  bool m_showCV;
  bool m_noLearning;

#ifdef FEAT_VISUALIZATION
  class CDisplayClient: public cogx::display::CDisplayClient
	{
		ObjectRecognizer3D* pRec;
	public:
		CDisplayClient() { pRec = NULL; }
		void setClientData(ObjectRecognizer3D* _pRec) { pRec = _pRec; }
    void handleEvent(const Visualization::TEvent &event);
	};
	CDisplayClient m_display;
#endif
  void getImage();
  void loadVisualModelToWM(RecEntry &rec_entry, cogx::Math::Pose3 &pose,
    std::string &label, bool forceNewObject = false);
  std::string loadEmptyVisualModelToWM(std::string &label);
  void addTrackerCommand(VisionData::TrackingCommandType cmd, std::string& modelID);
  void get3DPointFromTrackerModel(std::string& modelID, VisionData::VertexSeq& vertexlist);

  void receiveTrackingCommand(const cdl::WorkingMemoryChange & _wmc);
  void receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc);
  void receiveRecognitionCommand(const cdl::WorkingMemoryChange & _wmc);
  void receiveRecognizer3DCommand(const cdl::WorkingMemoryChange & _wmc);
  void PostFake3DObject(const cdl::WorkingMemoryChange & _wmc);

  void initInStart();
  void initInRun();
  void learnSiftModel(std::string& modelID, std::string& label);
  void learnSiftModel(P::DetectGPUSIFT &sift);
  void recognizeSiftModel(P::DetectGPUSIFT &sift);

protected:

  /** @brief called by the framework to configure our component */
  virtual void configure(const std::map<std::string,std::string> & _config);

  /** @brief called by the framework after configuration, before run loop */
  virtual void start();

  /** @brief called by the framework upon deletion of the component */
  virtual void destroy();

  /** @brief called by the framework to start compnent run loop */
  virtual void runComponent();

public:
	ObjectRecognizer3D();
  virtual ~ObjectRecognizer3D();

  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif

