 /**
 * @file ObjectRecognizer3D2.h
 * @author Thomas Mörwald, Hannes Prankl
 * @date Januray 2010
 * @version 0.1
 * @brief Learning and recognizing pose of Object using SIFT features (cooperates with ObjetTracker)
 * @namespace Tracking
 */
#ifndef OBJECT_RECOGNIZER_3D2_H
#define OBJECT_RECOGNIZER_3D2_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <cxcore.h>

#include <cast/architecture/ManagedComponent.hpp>
#include <StereoClient.h>
#include <VisionData.hpp>
#include <ObjectRecognizerSrv.hpp>
#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

#include "DetectGPUSIFT.hh"
#include "ODetect3D.hh"
#include "Object3D.hh"
#include "SDraw.hh"
#include "ModelObject3D.hh"
#include "KeypointDescriptor.hh"
#include "matrix.h"
#include "ObjectRecognizer3D2Utils.hpp"

namespace cast
{

class ObjectRecognizer3D2 : public StereoClient, public ManagedComponent
{
private:

	P::Array<P::KeypointDescriptor*> m_image_keys;
	P::Array<P::KeypointDescriptor*> m_temp_keys;
	P::ModelObject3D	sift_model_learner;
	P::ODetect3D*			m_detect;
	float m_min_confidence;
	cogx::Math::Pose3 m_camPose;
  bool m_haveCameraParameters;
  std::string m_recTaskId;
  ObjectRecognizerIce::ObjectRecognitionTaskPtr m_recTask;

  struct RecEntry{
  	std::string siftfile;
  	std::string plyfile;
  	P::Object3D* object;
  	RecEntry(){
  		object = 0;
  	}
  };
	std::map<std::string,RecEntry> m_recEntries;


#ifdef FEAT_VISUALIZATION
  class CDisplayClient: public cogx::display::CDisplayClient
	{
		ObjectRecognizer3D2* pRec;
	public:
		CDisplayClient() { pRec = NULL; }
		void setClientData(ObjectRecognizer3D2* _pRec) { pRec = _pRec; }
	};
	CDisplayClient m_display;
#endif

  void initRecognizer();
  void loadModels();
  void setCameraParameters(int imgWidth, int igHeight);
  void receiveRecognitionTask(const cdl::WorkingMemoryChange & _wmc);
  void fillImages(VisionData::ProtoObjectPtr pobj, IplImage **img, IplImage **grey);
  void releaseImages(IplImage **img, IplImage **grey);
  void recognizeAllObjects(P::DetectGPUSIFT &sift, IplImage *img, IplImage *grey,
    VisionData::VisualObjectPtr obj, ObjectRecognizerIce::ObjectRecognitionTaskPtr recTask);
  void finalizeObject(VisionData::VisualObjectPtr obj);
  void recognizeSiftModel(P::DetectGPUSIFT &sift, VisionData::VisualObjectPtr,
    ObjectRecognizerIce::ObjectRecognitionTaskPtr recTask);

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
	ObjectRecognizer3D2();
  virtual ~ObjectRecognizer3D2();

  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif

