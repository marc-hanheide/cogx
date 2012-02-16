/**
 * @author Michael Zillich
 * @date August 2011
 * @brief Recognise trained 3D models
 */

#ifndef OBJECT_RECOGNIZER_3D3_H
#define OBJECT_RECOGNIZER_3D3_H

#include <map>
#include <iostream>
#include <stdexcept>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "v4r/PCore/PMath.hh"
#include "v4r/CModelRecogniser/CModelHandler.hh"
#include "v4r/CModelRecogniser/KeypointDetectorSURF.hh"
#include "v4r/CModelRecogniser/KeypointDetector.hh"
#include "v4r/CModelRecogniser/ObjectLocation.hh"
#include "v4r/CModelRecogniser/RecogniserCore.hh"
#include <v4r/CModelRecogniser/RecogniserThread.hh>
#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <PointCloudClient.h>
#include <VisionData.hpp>
#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

namespace cast
{

using namespace VisionData;
using namespace cogx;
using namespace Math;
using namespace std;

class ObjectRecognizer3D : public VideoClient, public ManagedComponent
{
private:
  int camId;
  string videoServerName;
  Video::VideoInterfacePrx videoServer;
  map<string, string> modelFiles;
  map<string, string> objectWMIds;
  vector<cv::Ptr<P::CModel> > models;
  cv::Ptr<P::RecogniserThread> recogniser;

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

  void recognize(vector<string> &labels, cv::Mat &colImg,
    Video::CameraParameters &camPars, cv::Mat &mask,
    vector<P::ObjectLocation> &objects);
  void receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc);
  void receiveRecognitionCommand(const cdl::WorkingMemoryChange & _wmc);
  void objectLoationToVisualObject(P::ObjectLocation &objLoc,
      VisualObjectPtr &visObj);

protected:

  /** @brief called by the framework to configure our component */
  virtual void configure(const map<string,string> & _config);

  /** @brief called by the framework after configuration, before run loop */
  virtual void start();

public:
  ObjectRecognizer3D();
  virtual ~ObjectRecognizer3D();

  virtual void receiveImages(const vector<Video::Image>& images) {}
};

}

#endif

