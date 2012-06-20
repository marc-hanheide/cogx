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
#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/PCLAddOns/PCLFunctions.h"
#include "v4r/PCore/PMath.hh"
#include "v4r/PCore/Pose.hh"
#include "v4r/CModel/CModelThread.hh"
#include "v4r/CModel/ObjectModel.hh"
#include "v4r/CModel/PSiftGPU.hh"
#include "v4r/CModel/CModelHandler.hh"
#include "v4r/CModel/ObjectLocation.hh"
#include "v4r/CModel/VoxelGrid.hh"
#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <PointCloudClient.h>
#include <VisionData.hpp>
#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

namespace cast
{

using namespace std;
using namespace cogx;
using namespace Math;
using namespace Video;
using namespace VisionData;

class ObjectRecognizer3D : public VideoClient, public ManagedComponent
{
private:
  int camId;
  string videoServerName;
  Video::VideoInterfacePrx videoServer;
  cv::Ptr<P::CModelThread> recogniser;
  //map<string, string> objectWMIds;
  map<string, P::ObjectModel::Ptr> models;
  int modelNameCnt;

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
  cv::Mat generateMaskImage(VisualObjectPtr visObj,
      const CameraParameters &camPars, bool boundingBoxOnly);
  void recognize(const Image &image,
    const vector<PointCloud::SurfacePoint> &points,
    vector<P::ObjectLocation> &objects);
  void learn(const string &label, const Image &image,
    const vector<PointCloud::SurfacePoint> &points);
  void receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc);
  void receiveRecognitionCommand(const cdl::WorkingMemoryChange & _wmc);
  void receiveLearnObjectViewCommand(const cdl::WorkingMemoryChange & _wmc);
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

