/**
 * @author Marko Mahnič
 * @date May 2011
 *
 */

#ifndef SOI_FILTER_H
#define SOI_FILTER_H

#include "GraphCutSegmenter.h"
#include "Snapper.h"
#include "../../VisionUtils.h"
#include "TaskBase.h"

#include <VideoClient.h>
#include <PointCloudClient.h>
#include <ConvertImage.h>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>

# if defined(HAS_LIBPLOT)
#  include <CSvgPlotter.hpp>
   // The same ID as in VirtualScene2D, part of the VirtualScene2D View
#  define OBJ_VISUAL_OBJECTS "scene2d.VisualObjects"
#  if 1
#   define YY(y) -(y)
#  else
#   define YY(y) y
#  endif
# endif
#else
# include <opencv/cv.h>
# include <opencv/highgui.h>
#endif

#include <VisionData.hpp>
#include <NavData.hpp>
#include <PTZServer.hpp>
#include <cast/architecture/ManagedComponent.hpp>

#include <IceUtil/IceUtil.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>
#include <vector>
#include <string>
#include <queue>
#include <map>
#include <stdexcept>


namespace cast
{

template<typename TK, typename TV>
class mmap: public std::map<TK, TV>
{
public:
  bool has_key(const TK& key)
  {
    return find(key) != this->end();
  }
  TV& get(const TK& key)
  {
    if (this->find(key) == this->end())
      throw std::range_error("Key does not exist in map.");
    return std::map<TK,TV>::operator[](key);
  }
};

/** 
 * Cached SOI data
 */	
struct SoiRecord: public IceUtil::SimpleShared
{
  cdl::WorkingMemoryAddress addr;
  cdl::WorkingMemoryAddress protoObjectAddr; // The associated PO
  VisionData::SOIPtr psoi;
};
typedef IceUtil::Handle<SoiRecord> SoiRecordPtr;

struct ProtoObjectRecord : public IceUtil::SimpleShared
{
  cdl::WorkingMemoryAddress addr;
  VisionData::ProtoObjectPtr pobj;
  long long tmDisappeared;
};
typedef IceUtil::Handle<ProtoObjectRecord> ProtoObjectRecordPtr;

struct VisualObjectRecord : public IceUtil::SimpleShared
{
  cdl::WorkingMemoryAddress addr;
  VisionData::VisualObjectPtr pobj;
};
typedef IceUtil::Handle<VisualObjectRecord> VisualObjectRecordPtr;

class SOIPointCloudClient: public PointCloudClient
{
  public:
    using PointCloudClient::configureServerCommunication;
    using PointCloudClient::startPCCServerCommunication;
};

class SOIFilter : public ManagedComponent,
  public VideoClient
{
private:

  /**
   * Which camera to get images from
   */
  int camId;
  /**
   * component ID of the video server to connect to
   */
  std::string videoServerName;
  std::string stereoServerName; /* PointCloudServer! */
  std::string ptzServerName;

public:
  std::string m_coarsePcServer;  // periferial vision (eg. Kinect)
  std::string m_finePcServer;    // detailed vision (eg. stereo gear)

  /**
   * Identifiers of SOI sources. (plane pop-out)
   */
#define SOURCE_FAKE_SOI   "--fake.soi"
  std::string m_coarseSource;  // periferial vision (eg. Kinect)
  std::string m_fineSource;    // detailed vision (eg. stereo gear)

  SOIPointCloudClient m_coarsePointCloud;
  SOIPointCloudClient m_finePointCloud;

private:
  bool m_bSameSource;

  /**
   * our ICE proxy to the video server
   */
  Video::VideoInterfacePrx videoServer;
  ptz::PTZInterfacePrx ptzServer;

  bool doDisplay;

public:
  /**
   * Segmentation tolerances for distance and hsl
   *(gaussian dispersion)
   */
  GraphCutSegmenter m_segmenter;
  Snapper m_snapper;

private:
  std::deque<WmEvent*> m_EventQueue;
  IceUtil::Monitor<IceUtil::Mutex> m_EventQueueMonitor;

private:
#ifdef FEAT_VISUALIZATION
  class CSfDisplayClient: public cogx::display::CDisplayClient
  {
    SOIFilter* pFilter;
  public:
    CSfDisplayClient() { pFilter = NULL; }
    void setClientData(SOIFilter* pSoiFilter) { pFilter = pSoiFilter; }
    void handleEvent(const Visualization::TEvent &event); /*override*/
    std::string getControlState(const std::string& ctrlId);
    void onDialogValueChanged(const std::string& dialogId, const std::string& name, const std::string& value);
    void handleDialogCommand(const std::string& dialogId, const std::string& command, const std::string& params);
  };
  bool m_bShowProtoObjects;
  std::string m_sProtoObjectView;
  void sendSyncAllProtoObjects();
  void sendProtoObject(const cdl::WorkingMemoryAddress& addr, const VisionData::ProtoObjectPtr& pobj);
  void sendRemoveProtoObject(const cdl::WorkingMemoryAddress& addr);
public:
  CSfDisplayClient m_display;
#endif

private:
  void onAdd_SOI(const cdl::WorkingMemoryChange & _wmc);
  void onUpdate_SOI(const cdl::WorkingMemoryChange & _wmc);
  void onDelete_SOI(const cdl::WorkingMemoryChange & _wmc);

public:
  // The proto objects are kept locally so that we can match them by position with SOIs.
  // We don't keep all PO data locally! (see saveProtoObjectData)
  mmap<cdl::WorkingMemoryAddress, ProtoObjectRecordPtr> m_protoObjects;

  // The visual objects are kept locally so that we can match them to POs.
  // We don't keep all VO data locally! (see saveVisualObjectData)
  mmap<cdl::WorkingMemoryAddress, VisualObjectRecordPtr> m_visualObjects;

  // The sois are kept locally so register the SOI-PO connections.
  mmap<cdl::WorkingMemoryAddress, SoiRecordPtr> m_sois;

public:
  ProtoObjectRecordPtr findProtoObjectAt(const VisionData::SOIPtr &psoi);
  ProtoObjectRecordPtr findProtoObjectAt(const cogx::Math::Vector3 &pos);
  VisualObjectRecordPtr findVisualObjectFor(const cdl::WorkingMemoryAddress& protoAddr);
  VisualObjectRecordPtr findHiddenVisualObjectFor(const cdl::WorkingMemoryAddress& protoAddr);
  void updateRobotPosePtz();
  bool hasPtz() { return (ptzServer.get() != 0); }
  bool movePtz(double pan, double tilt, double zoom=0);
  void saveProtoObjectData(VisionData::ProtoObjectPtr& poOrig, VisionData::ProtoObjectPtr& poCopy);
  void saveVisualObjectData(VisionData::VisualObjectPtr& voOrig, VisionData::VisualObjectPtr& voCopy);
  void saveSoiData(VisionData::SOIPtr& soiOrig, VisionData::SOIPtr& soiCopy);

private:
  void onAdd_ProtoObject(const cdl::WorkingMemoryChange & _wmc);
  void onUpdate_ProtoObject(const cdl::WorkingMemoryChange & _wmc);
  void onDelete_ProtoObject(const cdl::WorkingMemoryChange & _wmc);
  void onAdd_VisualObject(const cdl::WorkingMemoryChange & _wmc);
  void onUpdate_VisualObject(const cdl::WorkingMemoryChange & _wmc);
  void onDelete_VisualObject(const cdl::WorkingMemoryChange & _wmc);

  void onAdd_MoveToVcCommand(const cdl::WorkingMemoryChange & _wmc);
  void onAdd_LookAroundCommand(const cdl::WorkingMemoryChange & _wmc);
  void onAdd_AnalyzeProtoObjectCommand(const cdl::WorkingMemoryChange & _wmc);

  void onChange_RobotPose(const cdl::WorkingMemoryChange & _wmc);
  void connectPtz();

  IceUtil::Monitor<IceUtil::Mutex> m_FilterMonitor;

  // Use this as an anchor to define target view cones
public:
  struct _RobotPose
  {
    double x;
    double y;
    double theta;
    double pan;
    double tilt;
    _RobotPose()
    {
      x = y = theta = 0;
      pan = tilt = 0;
    }
  };
  _RobotPose m_RobotPose;

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
  SOIFilter();
  virtual ~SOIFilter() {}
};

} // namespace

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */
