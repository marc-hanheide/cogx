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

#include <castutils/Timers.hpp>
#include <VisionData.hpp>
#include <NavData.hpp>
#include <PTZServer.hpp>
#include <cast/architecture/ManagedComponent.hpp>

#ifdef FEAT_TRACK_ARM
#include <execution/manipulation_exe.hpp>
#endif

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
  bool has_key(const TK& key) const
  {
    return find(key) != this->end();
  }
  TV& get(const TK& key) // not const because of []
  {
    if (this->find(key) == this->end())
      throw std::range_error("Key does not exist in map.");
    return std::map<TK,TV>::operator[](key);
  }
};

/** 
 * Cached data
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
  castutils::CMilliTimer tmDisappeared;
  ProtoObjectRecord() : tmDisappeared(true) {}
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

template<typename TItem>
class CMointoredQueue
{
  typedef IceUtil::Monitor<IceUtil::Mutex>::Lock Lock;
  mutable IceUtil::Monitor<IceUtil::Mutex> monitor;
  std::deque<TItem> data;
public:
  void addItem(TItem pEvent)
  {
    Lock my(monitor);
    data.push_back(pEvent);
    monitor.notify(); // works only if the monitor is locked here
  };
  bool waitForItem(long milliseconds) const
  {
    Lock my(monitor);
    if (data.size() > 0) return true;
    monitor.timedWait(IceUtil::Time::milliSeconds(milliseconds));
    return data.size() > 0;
  }
  std::deque<TItem> getItems(int maxItems = -1 /*all*/)
  {
    Lock my(monitor);
    std::deque<TItem> evts;
    if (maxItems < 0 || (unsigned int)maxItems >= data.size()) {
      //std::move(data.begin(), data.end(), std::back_inserter(evts));
      evts = std::move(data);
      data.clear();
    }
    else {
      while (data.size() > 0 && evts.size() < (unsigned int)maxItems) {
        evts.push_back(std::move(data.front()));
        data.pop_front();
      }
    }
    return evts;
  }
  int countIf(bool (*testFunc)(const TItem&)) const
  {
    Lock my(monitor);
    int count = 0;
    // g++ 4.5.2 doesn't eat range-for;
    // g++ 4.6.1 thinks item is a (cast::WmEvent) when it is actually a (cast::WmEvent*);
    //for (auto item : data) {
    for (auto item = data.begin(); item != data.end(); item++) {
      if (testFunc(*item)) count++;
    }
    return count;
  }
  bool empty() const
  {
    Lock my(monitor);
    return data.empty();
  }
  size_t size() const
  {
    Lock my(monitor);
    return data.size();
  }
};

class SOIFilter : public ManagedComponent,
  public VideoClient
{
private:

  /**
   * Which camera to get images from
   */
  int camId;
  bool m_bCameraMoving;
  castutils::CMilliTimer m_endMoveTimeout;
  Video::CameraParameters m_cameraParams;

  /**
   * component ID of the video server to connect to
   */
  std::string videoServerName;
  std::string stereoServerName; /* PointCloudServer! */
  std::string ptzServerName;

public:
  std::string m_coarsePcServer;  // periferial vision (eg. Kinect)
  std::string m_finePcServer;    // detailed vision (eg. stereo gear)
  int m_identityRecognizerVersion;  // 2 - prepared for review 2010, 3 - prepared for review 2011

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
  CMointoredQueue<WmEvent*> m_EventQueue;

private:
  // time-based queue
  std::vector<WmEvent*> m_EventRetryQueue;
  IceUtil::RWRecMutex m_retryQueueMutex;

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
    void sendPtuStateToDialog();
  };
  bool m_bShowProtoObjects;
  std::string m_sProtoObjectView;
  void sendSyncAllProtoObjects();
  void sendProtoObject(const cdl::WorkingMemoryAddress& addr, const VisionData::ProtoObjectPtr& pobj);
  void sendRemoveProtoObject(const cdl::WorkingMemoryAddress& addr);
public:
  CSfDisplayClient m_display;
#endif

public:
  // The proto objects are kept locally so that we can match them by position with SOIs.
  // We don't keep all PO data locally! (see saveProtoObjectData)
  mmap<cdl::WorkingMemoryAddress, ProtoObjectRecordPtr> m_protoObjects;
  IceUtil::RWRecMutex m_protoObjectMapMutex;

  // The visual objects are kept locally so that we can match them to POs.
  // We don't keep all VO data locally! (see saveVisualObjectData)
  mmap<cdl::WorkingMemoryAddress, VisualObjectRecordPtr> m_visualObjects;

  // The sois are kept locally so register the SOI-PO connections.
  mmap<cdl::WorkingMemoryAddress, SoiRecordPtr> m_sois;

  // The SOIs are ignored if distance from 0 to SOI center is outside of these bounds.
  double m_minSoiDistance;
  double m_maxSoiDistance;
#ifdef FEAT_TRACK_ARM
  // Distance from the katana arm
  double m_minArmDistance;
  manipulation::execution::slice::ArmStatusPtr m_pArmStatus;
#endif

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
  bool isCameraStable(unsigned long milliSeconds = 0);
  bool isPointVisible(const cogx::Math::Vector3 &pos);

public:
  bool retryEvent(WmEvent* pEvent, long milliSeconds, long nRetries=1);
private:
  void checkRetryEvents();
  long getMillisToRetryEvent(long defaultMs);
  bool isRetryEvent(WmEvent* pEvent);

private:
  void onAdd_SOI(const cdl::WorkingMemoryChange & _wmc);
  void onUpdate_SOI(const cdl::WorkingMemoryChange & _wmc);
  void onDelete_SOI(const cdl::WorkingMemoryChange & _wmc);

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
  void onChange_CameraParameters(const cdl::WorkingMemoryChange & _wmc);
#ifdef FEAT_TRACK_ARM
  void onChange_ArmPosition(const cdl::WorkingMemoryChange & _wmc);
#endif
  void connectPtz();
  void checkInvisibleObjects();

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
  void queueCheckVisibilityOf_PO(const cdl::WorkingMemoryAddress& protoObjectAddr);
  using CASTComponent::sleepComponent;
};

} // namespace

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */
