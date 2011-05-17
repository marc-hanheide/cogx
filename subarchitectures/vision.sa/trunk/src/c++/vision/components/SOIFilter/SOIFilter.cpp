/**
 * @author Marko Mahnič
 * @date May 2011
 *
 */

#include "SOIFilter.h"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <fstream>
#include <cmath>

#define CAM_ID_DEFAULT 0

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::SOIFilter();
  }
}

#if defined(FEAT_VISUALIZATION) && HAS_LIBPLOT
#include <CSvgPlotter.hpp>

// The same ID as in VirtualScene2D, part of the VirtualScene2D View
#define OBJ_VISUAL_OBJECTS "scene2d.VisualObjects"
#endif

namespace cast
{

using namespace std;
using namespace cdl;
using namespace VisionData;
using namespace Video;

using namespace boost::posix_time;

long long SOIFilter::g_order = 0;
IceUtil::Monitor<IceUtil::Mutex> SOIFilter::g_OrderMonitor;

SOIFilter::SOIFilter()
{
  m_segmenter.pPcClient = &m_finePointCloud;
  m_snapper.logger = this;
  m_snapper.videoServer = videoServer;

#ifdef FEAT_VISUALIZATION
  m_segmenter.pDisplay = &m_display;
#endif

}

void SOIFilter::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  // first let the base classes configure themselves
  m_segmenter.configure(_config);

  m_coarseSource = "";
  m_fineSource = "";

  if((it = _config.find("--coarse-pointcloud")) != _config.end()) {
    m_coarseSource = it->second;
  }
  if((it = _config.find("--fine-pointcloud")) != _config.end()) {
    m_fineSource = it->second;
  }

  string serverParam("--servername");
  if (m_coarseSource == "") {
    if (m_fineSource != "")
      m_coarseSource = m_fineSource;
    else if((it = _config.find(serverParam)) != _config.end())
      m_coarseSource = it->second;
  }
  if (m_fineSource == "") {
    if (m_coarseSource != "")
      m_fineSource = m_coarseSource;
    else if((it = _config.find(serverParam)) != _config.end())
      m_fineSource = it->second;
  }
  if (m_coarseSource == "" || m_fineSource == "") {
    throw runtime_error(exceptionMessage(__HERE__, "No point cloud server name given"));
  }

  map<string, string> configPcc;
  configPcc[serverParam] = m_coarseSource;
  m_coarsePointCloud.configureServerCommunication(configPcc); 
  configPcc[serverParam] = m_fineSource;
  m_finePointCloud.configureServerCommunication(configPcc); 

  camId = CAM_ID_DEFAULT;
  doDisplay = false;

  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream str(it->second);
    str >> camId;
  }

  if((it = _config.find("--display")) != _config.end())
  {
    doDisplay = true;
  }

  if((it = _config.find("--coarseSource")) != _config.end())
  {
    m_coarseSource = it->second;
  }

  if((it = _config.find("--fineSource")) != _config.end())
  {
    m_fineSource = it->second;
  }

  if (m_fineSource.size() == 0 && m_coarseSource.size() != 0)
    m_fineSource = m_coarseSource;

  if (m_coarseSource.size() == 0 && m_fineSource.size() != 0)
    m_coarseSource = m_fineSource;

  m_bSameSource = m_coarseSource == m_fineSource;

#ifdef FEAT_VISUALIZATION
  m_display.configureDisplayClient(_config);
#endif
}

void SOIFilter::connectPtz()
{
  Ice::CommunicatorPtr ic = getCommunicator();

  Ice::Identity id;
  id.name = "PTZServer";
  id.category = "PTZServer";

  std::ostringstream str;
  str << ic->identityToString(id) 
    << ":default"
    << " -h localhost"
    << " -p " << cast::cdl::CPPSERVERPORT;

  Ice::ObjectPrx base = ic->stringToProxy(str.str());    
  ptzServer = ptz::PTZInterfacePrx::uncheckedCast(base);
}

void SOIFilter::start()
{
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);
  m_snapper.videoServer = videoServer;

  m_coarsePointCloud.startPCCServerCommunication(*this);
  m_finePointCloud.startPCCServerCommunication(*this);

#ifdef FEAT_VISUALIZATION
  m_display.connectIceClient(*this);
  m_display.setClientData(this);
  m_display.installEventReceiver();
  m_display.addButton(ID_OBJ_LAST_SEGMENTATION, "take.snapshot", "&Snapshot");
#else
  if (doDisplay)
  {
    cvNamedWindow("Full image", 1);
    cvNamedWindow("Last ROI Segmentation", 1);
    cvNamedWindow("Color Filtering", 1);
  }
#endif

  // we want to receive detected SOIs
  addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::ADD),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onAdd_SOI));
  // .., when they are updated
  //	addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::OVERWRITE),
  //		new MemberFunctionChangeReceiver<SOIFilter>(this,
  //		  &SOIFilter::updatedSOI));
  // .. and when they are deleted
  addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::DELETE),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onDelete_SOI));

  // XXX: added to save SurfacePatches with saveSnapshot
  if (m_snapper.m_bAutoSnapshot) {
    log("AUTOSNAP is ON; Triggered on ProtoObject OVERWRITE");
    addChangeFilter(createLocalTypeFilter<VisionData::ProtoObject>(cdl::OVERWRITE),
        new MemberFunctionChangeReceiver<SOIFilter>(this,
          &SOIFilter::onUpdate_ProtoObject));
  }

  // Hook up changes to the robot pose to a callback function.
  // We will use the robot position as an anchor for VC commands.
  addChangeFilter(createGlobalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onChange_RobotPose));  
  addChangeFilter(createGlobalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onChange_RobotPose));  
}

#ifdef FEAT_VISUALIZATION
void SOIFilter::CSfDisplayClient::handleEvent(const Visualization::TEvent &event)
{
  if (event.type == Visualization::evButtonClick) {
    if (event.sourceId == "take.snapshot") {
      pFilter->m_snapper.saveSnapshot();
    }
  }
}
#endif

void SOIFilter::onAdd_SOI(const cdl::WorkingMemoryChange & _wmc)
{
  {
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);
    m_EventQueue.push_back(new WmEvent(cdl::ADD, _wmc));
  }
  m_EventQueueMonitor.notify();
}

void SOIFilter::onDelete_SOI(const cdl::WorkingMemoryChange & _wmc)
{
  {
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);
    m_EventQueue.push_back(new WmEvent(cdl::DELETE, _wmc));
  }
  m_EventQueueMonitor.notify();
}

void SOIFilter::onUpdate_SOI(const cdl::WorkingMemoryChange & _wmc)
{
  {
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);
    m_EventQueue.push_back(new WmEvent(cdl::OVERWRITE, _wmc));
  }
  m_EventQueueMonitor.notify();
}

// XXX: Added to saveSnapshot with SurfacePatches
void SOIFilter::onUpdate_ProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  // XXX: ASSUME it's the same protoobject
  m_snapper.m_LastProtoObject = getMemoryEntry<VisionData::ProtoObject>(_wmc.address);
  m_snapper.saveSnapshot();
}

void SOIFilter::onChange_RobotPose(const cdl::WorkingMemoryChange & _wmc)
{
  NavData::RobotPose2dPtr ppose =
    getMemoryEntry<NavData::RobotPose2d>(_wmc.address);

  ptz::PTZReading ptup;
  if (ptzServer.get())
    ptup = ptzServer->getPose();

  {
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_FilterMonitor);
    m_RobotPose.x = ppose->x;
    m_RobotPose.y = ppose->y;
    m_RobotPose.theta = ppose->theta;
    if (ptzServer.get())
    {
      m_RobotPose.pan = ptup.pose.pan;
      m_RobotPose.tilt = ptup.pose.tilt;
    }
  }
}

void SOIFilter::runComponent()
{
  WmTaskExecutor worker(this);

  while(isRunning())
  {
    std::deque<WmEvent*> tasks;
    tasks.clear();
    {
      // SYNC: Lock the monitor
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);

      // SYNC: If queue is empty, unlock the monitor and wait for notify() or timeout
      if (m_EventQueue.size() < 1)
        m_EventQueueMonitor.timedWait(IceUtil::Time::seconds(2));

      // SYNC: Continue with a locked monitor

#if 0
      // Process whole queue at once
      tasks = m_EventQueue;
      m_EventQueue.clear();
#else
      // Process queued items one by one
      if (m_EventQueue.size() > 0) {
        tasks.push_back(m_EventQueue.front());
        m_EventQueue.pop_front();
      }
#endif
      // SYNC: unlock the monitor when going out of scope
    }

    // if (tasks.empty()) debug("Timeout");

    while (!tasks.empty())
    { 
      WmEvent* pevent = tasks.front();
      tasks.pop_front();
      worker.handle(pevent);
      delete pevent;
    }
  }

#ifndef FEAT_VISUALIZATION
  if (doDisplay)
  {
    log("Destroying OpenCV windows..");
    cvDestroyWindow("Full image");
    cvDestroyWindow("Last ROI Segmentation");
    cvDestroyWindow("Color Filtering");
  }
#endif
}

/*
 * TODO: 2 step VisualObject generation.
 *   1. If the SOI comes from coarseSource, create a proto-object with the
 *      desired ViewCones
 *   2. If the SOI comes from fineSource, we need to know which PO it belongs
 *      to (SOI matching by position), then we update the PO and create the VO.
 * TODO: Decide how to process stuff when there is only a single source ...
 */
void SOIFilter::WmTaskExecutor::handle_add_soi(WmEvent* pEvent)
{
  SOIPtr psoi =
    pSoiFilter->getMemoryEntry<VisionData::SOI>(pEvent->wmc.address);

  SOIData soi;
  soi.addr = pEvent->wmc.address;
  soi.addTime = psoi->time;
  soi.status = STABLE;
  soi.stableTime = pSoiFilter->getCASTTime();
  soi.updCount = 0;
  pSoiFilter->SOIMap.insert(make_pair(soi.addr.id, soi));

  if (psoi->sourceId == pSoiFilter->m_coarseSource)
  {
    ProtoObjectPtr pobj = new ProtoObject();
    pSoiFilter->m_snapper.m_LastProtoObject = pobj;
    string objId = pSoiFilter->newDataID();

    // Calculate the current and the desired View Cone
    //    We don't (shouldn't?) know an absolute position of the robot so we
    //    can only use relative coordinates. We should keep track of the robot
    //    movements from the time we calculate the VCs to the time the desired
    //    VC is reached. We could use anchors for that, but are they currently
    //    implemented?
    //    Current VC is at anchor. Desired VC(s) are relative to the anchor.
    ViewConePtr pCurVc = new ViewCone();
    pCurVc->anchor.x = pSoiFilter->m_RobotPose.x;
    pCurVc->anchor.y = pSoiFilter->m_RobotPose.y;
    pCurVc->anchor.z = pSoiFilter->m_RobotPose.theta;
    pCurVc->x = 0;
    pCurVc->y = 0;
    pCurVc->viewDirection = pSoiFilter->m_RobotPose.pan;
    pCurVc->tilt = pSoiFilter->m_RobotPose.tilt;
    pobj->cameraLocation = pCurVc;

    double dirDelta = 0;
    double tiltDelta = 0;
    CameraParameters camPars;
    if (pSoiFilter->m_coarsePointCloud.getCameraParameters(LEFT, camPars)) {
      ROIPtr roiPtr = projectSOI(camPars, *psoi);

      // Center of the projected SOI
      double rcx = roiPtr->rect.pos.x + roiPtr->rect.width  / 2;
      double rcy = roiPtr->rect.pos.y + roiPtr->rect.height / 2;

      // how far from the center of the LEFT image is the SOI
      dirDelta  = atan( (rcx - camPars.cx) / camPars.fx);
      tiltDelta = atan( (rcy - camPars.cy) / camPars.fy);

#if defined(FEAT_VISUALIZATION) && HAS_LIBPLOT
      // draw the thing
      std::ostringstream ssvg;
      cogx::display::CSvgStringPlotter p(ssvg);
      p.openpl();
      p.flinewidth (2.0);        // line thickness in user coordinates
      p.pencolorname ("red");    // path will be drawn in red

      p.line(camPars.cx, camPars.cy, rcx, rcy);
      p.line(camPars.cx, camPars.cy, camPars.cx + dirDelta * 10, camPars.cy);
      p.line(camPars.cx, camPars.cy, camPars.cx, camPars.cy + tiltDelta * 10);

      p.closepl();

      string s = p.getScreenSvg();

      pSoiFilter->m_display.setObject(OBJ_VISUAL_OBJECTS, "soif-last-move", s);
#endif
    }

    ViewConePtr pBetterVc = new ViewCone();
    pBetterVc->anchor = pCurVc->anchor;
    pBetterVc->x = 0;
    pBetterVc->y = 0;
    pBetterVc->viewDirection = pCurVc->viewDirection + dirDelta;
    pBetterVc->tilt = pCurVc->tilt + tiltDelta;
    pobj->desiredLocations.push_back(pBetterVc);

    pSoiFilter->addToWorkingMemory(objId, pobj);

    // Now it is up to the planner to create a plan to move the robot
    // The task contains: PO, target VC

    // Simulation of the plan:
    // TODO: Post MovementTask(PO, VC)
    //    class MoveToViewCone {
    //      ViewCone target;
    //      string reason;   // look-at-object; maybe enum instd of string
    //      string objectId; // callers reference
    //    }
    //    - only control the PTU; use the difference of VC-thetas to get the angle
    // onMovementTask_Complete (overwrite):
    //    TODO: Post VisualAnalysisTask(PO)
    //
    // Actual work performed here:
    //    onVisualAnalysisTask_Accepted: -- this is implemented in SOIF
    //       TODO: Remember the PO, the desired VC
    //       TODO: calculate the position of actual VC in relation to desired VC
    //       on_handle_FineSoi:
    //          TODO: wait for SOI(s) at the location of the *desired* VC center
    //          TODO: update PO with data from SOI(s)
    //
    // When the PO is updated, another component may create a VO
  }
  else if (psoi->sourceId == pSoiFilter->m_fineSource)
  {
    // If the SOI is in center of scene, find the PO we want to analyze and 
    // 'promote' the PO. Then somebody will have to generate a VO.
    //   - check if we have a pending recognition task
    //   - check if the robot is in the desired position
    //   - assume the PO is in the center of the scene <-- the desired position is reached
  }
}

void SOIFilter::WmTaskExecutor::handle_delete_soi(WmEvent* pEvent)
{
  const WorkingMemoryChange &wmc = pEvent->wmc;
  SOIData &soi = pSoiFilter->SOIMap[wmc.address.id];
  soi.status = DELETED;

  if (soi.objId.size() > 0)
  {
    // TODO: We have to load the PO and save it again. A lot of transfer !!!
  }
}


} // namespace
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */

