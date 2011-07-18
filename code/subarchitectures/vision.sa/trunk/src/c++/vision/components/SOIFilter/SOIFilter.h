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

#include <VideoClient.h>
#include <PointCloudClient.h>
#include <ConvertImage.h>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

#include <VisionData.hpp>
#include <NavData.hpp>
#include <PTZServer.hpp>
#include <cast/architecture/ManagedComponent.hpp>

#include <IceUtil/IceUtil.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>
#include <vector>
#include <string>
#include <queue>
#include <map>

#define FEAT_GENERATE_FAKE_SOIS

namespace cast
{

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

  std::string m_coarsePcServer;  // periferial vision (eg. Kinect)
  std::string m_finePcServer;    // detailed vision (eg. stereo gear)
  SOIPointCloudClient m_coarsePointCloud;
  SOIPointCloudClient m_finePointCloud;

  /**
   * Identifiers of SOI sources. (plane pop-out)
   */
  std::string m_coarseSource;  // periferial vision (eg. Kinect)
  std::string m_fineSource;    // detailed vision (eg. stereo gear)
  bool m_bSameSource;

  /**
   * our ICE proxy to the video server
   */
  Video::VideoInterfacePrx videoServer;
  ptz::PTZInterfacePrx ptzServer;

  bool doDisplay;

  /**
   * Segmentation tolerances for distance and hsl
   *(gaussian dispersion)
   */
  GraphCutSegmenter m_segmenter;
  Snapper m_snapper;

#if 0
  UNUSED
  /**
   * Time and update thresholds
   *(part of the ROI persistency criteria)
   */
  unsigned timeThr;
  int updateThr;
#endif

  /**
   * status of SOI persistency
   */
  enum SOIStatus {
    CANDIDATE, // Used only in Y1
    STABLE,
    PROTOOBJECT,
    OBJECT,
    DELETED
  };

  /** 
   * SOI data, contains also data used to evaluate SOI persistency
   */	
  struct SOIData {
    cdl::WorkingMemoryAddress addr;
    SOIStatus status;
    // 	VisionData::SurfacePointsSeq points;
    int updCount;
    std::string objId;
    std::string sourceId;
    cdl::CASTTime addTime;
    cdl::CASTTime stableTime;
    cdl::CASTTime objectTime;
    cdl::CASTTime deleteTime;
  };

  struct colorHLS {
    int h;
    float l;
    float s;
  };

  std::map<std::string, SOIData> SOIMap;

  static long long g_order;
  static IceUtil::Monitor<IceUtil::Mutex> g_OrderMonitor;
  static long long getEventOrder()
  {
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(g_OrderMonitor);
    ++g_order;
    return g_order;
  }

  enum {
    TYPE_SOI = 1, TYPE_CMD_LOOK = 2, TYPE_CMD_ANALYZE = 3
  };
  struct WmEvent
  {
    int  objectType;
    int  change; // One of cdl ADD, OVERWRITE, DELETE
    long long order;
    cdl::WorkingMemoryChange wmc;
    WmEvent(int wmType, int changeType, const cdl::WorkingMemoryChange& _wmc)
    {
      objectType = wmType;
      change = changeType;
      wmc = _wmc;
      order = SOIFilter::getEventOrder();
    }
  };
  std::deque<WmEvent*> m_EventQueue;
  IceUtil::Monitor<IceUtil::Mutex> m_EventQueueMonitor;

  class WmTaskExecutor
  {
  protected:
    SOIFilter* pSoiFilter;
  public:
    WmTaskExecutor(SOIFilter* soif)
    {
      pSoiFilter = soif;
    }
    virtual void handle(WmEvent *pEvent) = 0;
  };

  class WmTaskExecutor_Soi: public WmTaskExecutor
  {
  protected:
    virtual void handle_add_soi(WmEvent *pEvent);
    virtual void handle_delete_soi(WmEvent *pEvent);
  public:
    WmTaskExecutor_Soi(SOIFilter* soif) : WmTaskExecutor(soif) {}

    virtual void handle(WmEvent *pEvent)
    {
      if (pEvent->change == cdl::ADD) handle_add_soi(pEvent);
      else if (pEvent->change == cdl::DELETE) handle_delete_soi(pEvent);
    }
  };

  // The planner/executor is responsible for the correct ordering of Analyze
  // and MoveToViewCone tasks. While an Analyze task is active, a
  // MoveToViewCone should not be executed.

  class WmTaskExecutor_Analyze: public WmTaskExecutor
  {
  protected:
    virtual void handle_add_task(WmEvent *pEvent);
  public:
    WmTaskExecutor_Analyze(SOIFilter* soif) : WmTaskExecutor(soif) {}

    virtual void handle(WmEvent *pEvent)
    {
      if (pEvent->change == cdl::ADD) handle_add_task(pEvent);
    }
  };

  class WmTaskExecutor_MoveToViewCone: public WmTaskExecutor
  {
  protected:
    virtual void handle_add_task(WmEvent *pEvent);
  public:
    WmTaskExecutor_MoveToViewCone(SOIFilter* soif) : WmTaskExecutor(soif) {}

    virtual void handle(WmEvent *pEvent)
    {
      if (pEvent->change == cdl::ADD) handle_add_task(pEvent);
    }
  };

  // An implementation of a RPC call through WM that completes on WM-overwrite.
  /*
   * A WorkingMemoryChangeReceiver should be created on the heap. We wait for
   * it to complete then we remove the it from the change filter list with
   * removeChangeFilter(*, cdl::DELETERECEIVER), but we DON'T DELETE it. It
   * will be moved to a deletion queue by the framework and deleted later. */
  class GetSoisCommandRcv:
    public cast::WorkingMemoryChangeReceiver
  {
  protected:
    SOIFilter* pSoiFilter;
    bool m_complete;
    IceUtil::Monitor<IceUtil::Mutex> m_CompletionMonitor;
  public:
    VisionData::GetStableSoisCommandPtr m_pcmd;
  public:
    GetSoisCommandRcv(SOIFilter* psoif, std::string component_id);
    void workingMemoryChanged(const cast::cdl::WorkingMemoryChange &_wmc);
    bool waitForCompletion(double milliSeconds);
    void getSois(std::vector<VisionData::SOIPtr>& sois);
  };
 
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
  CSfDisplayClient m_display;
  bool m_bShowProtoObjects;
  std::string m_sProtoObjectView;
  void sendSyncAllProtoObjects();
  void sendProtoObject(const cdl::WorkingMemoryAddress& addr, const VisionData::ProtoObjectPtr& pobj);
  void sendRemoveProtoObject(const cdl::WorkingMemoryAddress& addr);
#endif
#ifdef FEAT_GENERATE_FAKE_SOIS
  void addFakeSoi();
#endif

private:
  void onAdd_SOI(const cdl::WorkingMemoryChange & _wmc);
  void onUpdate_SOI(const cdl::WorkingMemoryChange & _wmc);
  void onDelete_SOI(const cdl::WorkingMemoryChange & _wmc);

public:
  // The proto objects are kept locally so that we can match them by position with SOIs.
  // We don't keep all PO data locally! (see saveProtoObjectData)
  std::map<cdl::WorkingMemoryAddress, VisionData::ProtoObjectPtr> m_protoObjects;

  // The visual objects are kept locally so that we can match them to POs.
  // We don't keep all VO data locally! (see saveVisualObjectData)
  std::map<cdl::WorkingMemoryAddress, VisionData::VisualObjectPtr> m_visualObjects;

public:
  VisionData::ProtoObjectPtr findProtoObjectAt(VisionData::SOIPtr psoi);
  cdl::WorkingMemoryAddress findVisualObjectFor(const cdl::WorkingMemoryAddress& protoAddr);

private:
  void saveProtoObjectData(VisionData::ProtoObjectPtr& poOrig, VisionData::ProtoObjectPtr& poCopy);
  void saveVisualObjectData(VisionData::VisualObjectPtr& voOrig, VisionData::VisualObjectPtr& voCopy);

  void onAdd_ProtoObject(const cdl::WorkingMemoryChange & _wmc);
  void onUpdate_ProtoObject(const cdl::WorkingMemoryChange & _wmc);
  void onDelete_ProtoObject(const cdl::WorkingMemoryChange & _wmc);
  void onAdd_VisualObject(const cdl::WorkingMemoryChange & _wmc);
  void onUpdate_VisualObject(const cdl::WorkingMemoryChange & _wmc);
  void onDelete_VisualObject(const cdl::WorkingMemoryChange & _wmc);

  void onAdd_MoveToVcCommand(const cdl::WorkingMemoryChange & _wmc);
  void onAdd_AnalyzeProtoObjectCommand(const cdl::WorkingMemoryChange & _wmc);

  IceUtil::Monitor<IceUtil::Mutex> m_FilterMonitor;
  // Use this as an anchor to define target view cones
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
  void onChange_RobotPose(const cdl::WorkingMemoryChange & _wmc);
  void connectPtz();

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
