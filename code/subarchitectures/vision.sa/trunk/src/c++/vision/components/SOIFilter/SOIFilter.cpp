/**
 * @author Marko Mahnič
 * @date May 2011
 *
 */

#include "SOIFilter.h"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <fstream>
#include <cmath>

#include "../VisionUtils.h"

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

#if defined(FEAT_VISUALIZATION) && defined(HAS_LIBPLOT)
#include <CSvgPlotter.hpp>

// The same ID as in VirtualScene2D, part of the VirtualScene2D View
#define OBJ_VISUAL_OBJECTS "scene2d.VisualObjects"
#endif

namespace cast
{

#include "res/ptuctrl.inc"

using namespace std;
using namespace cdl;
using namespace VisionData;
using namespace Video;

using namespace boost::posix_time;

SOIFilter::SOIFilter()
{
  m_segmenter.pPcClient = &m_finePointCloud;
  m_snapper.logger = this;
  m_snapper.videoServer = videoServer;

#ifdef FEAT_VISUALIZATION
  m_segmenter.pDisplay = &m_display;
  m_sProtoObjectView = "";
  m_bShowProtoObjects = false;
#endif

}

void SOIFilter::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  // first let the base classes configure themselves
  m_segmenter.configure(_config);

  m_coarseSource = "";
  m_fineSource = "";

  // TODO: we only need the image source from the fine PointCloudServer/StereoServer
  // TODO: we need the identifier of the Fine SOI
  if((it = _config.find("--coarse-pcserver")) != _config.end()) {
    m_coarsePcServer = it->second;
  }
  if((it = _config.find("--fine-pcserver")) != _config.end()) {
    m_finePcServer = it->second;
  }

  string serverParam("--pcserver");
  if (m_coarsePcServer == "") {
    if (m_finePcServer != "")
      m_coarsePcServer = m_finePcServer;
    else if((it = _config.find(serverParam)) != _config.end())
      m_coarsePcServer = it->second;
  }
  if (m_finePcServer == "") {
    if (m_coarsePcServer != "")
      m_finePcServer = m_coarsePcServer;
    else if((it = _config.find(serverParam)) != _config.end())
      m_finePcServer = it->second;
  }
  if (m_coarsePcServer == "" || m_finePcServer == "") {
    throw runtime_error(exceptionMessage(__HERE__, "No point cloud server name given"));
  }

  map<string, string> configPcc;
  configPcc[serverParam] = m_coarsePcServer;
  m_coarsePointCloud.configureServerCommunication(configPcc); 
  configPcc[serverParam] = m_finePcServer;
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

#ifdef FEAT_VISUALIZATION
  m_bShowProtoObjects = false;
  // The name of the display server scene where 3D objects (proto-objects) should be displayed
  if((it = _config.find("--scene3d")) != _config.end())
  {
    m_sProtoObjectView = it->second;
    if (m_sProtoObjectView == "")
      m_sProtoObjectView = "soif.objects.3d";
    m_bShowProtoObjects = true;
  }
#else
  if((it = _config.find("--display")) != _config.end())
  {
    doDisplay = true;
  }
#endif

  if((it = _config.find("--coarse-source")) != _config.end())
  {
    m_coarseSource = it->second;
  }

  if((it = _config.find("--fine-source")) != _config.end())
  {
    m_fineSource = it->second;
  }

  if (m_fineSource.size() == 0 && m_coarseSource.size() != 0)
    m_fineSource = m_coarseSource;

  if (m_coarseSource.size() == 0 && m_fineSource.size() != 0)
    m_coarseSource = m_fineSource;

  m_bSameSource = m_coarseSource == m_fineSource;

  ptzServerName = "";
  if((it = _config.find("--ptzserver")) != _config.end())
  {
    ptzServerName = it->second;
  }

#ifdef FEAT_VISUALIZATION
  m_display.configureDisplayClient(_config);
#endif
}

void SOIFilter::connectPtz()
{
  std::string ptzServerHost = "localhost";
  int ptzServerPort = cast::cdl::CPPSERVERPORT;

  if (ptzServerName.length() > 0) {
    cast::cdl::ComponentDescription desc =
      getComponentManager()->getComponentDescription(ptzServerName);

    ptzServerHost = desc.hostName;
    ptzServerPort = cast::languageToPort(desc.language);
  }

  Ice::CommunicatorPtr ic = getCommunicator();

  Ice::Identity id;
  id.name = "PTZServer";
  id.category = "PTZServer";

  std::ostringstream str;
  str << ic->identityToString(id) 
    << ":default"
    << " -h " << ptzServerHost
    << " -p " << ptzServerPort;

  Ice::ObjectPrx base = ic->stringToProxy(str.str());    
  ptzServer = ptz::PTZInterfacePrx::uncheckedCast(base);
}

#define IDC_SOIF_PROTOOBJECTS "popout.show.protoobjects"
#define ID_PART_3D_PO     "PO:"

void SOIFilter::start()
{
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);
  m_snapper.videoServer = videoServer;

  m_coarsePointCloud.startPCCServerCommunication(*this);
  m_finePointCloud.startPCCServerCommunication(*this);

  connectPtz();

#ifdef FEAT_VISUALIZATION
  m_display.connectIceClient(*this);
  m_display.setClientData(this);
  m_display.installEventReceiver();
  m_display.addButton(ID_OBJ_LAST_SEGMENTATION, "take.snapshot", "&Snapshot");

  if (m_bShowProtoObjects) {
    Visualization::ActionInfo act;
    act.id = IDC_SOIF_PROTOOBJECTS;
    act.label = "Toggle Update Proto Objects";
    act.iconLabel = "ProtoObjects";
    act.iconSvg = "text:PO";
    act.checkable = true;
    m_display.addAction(m_sProtoObjectView, act);
  }

#ifdef FEAT_GENERATE_FAKE_SOIS
  m_display.addButton(ID_OBJ_LAST_SEGMENTATION, "fake.soi", "&Fake SOI");
#endif

  m_display.addDialog("PtuCtrl", res_ptucontroller_ui, res_ptucontroller_js, "PtuController ptuctrl");

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

  addChangeFilter(createLocalTypeFilter<VisionData::MoveToViewConeCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onAdd_MoveToVcCommand));

  addChangeFilter(createLocalTypeFilter<VisionData::AnalyzeProtoObjectCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onAdd_AnalyzeProtoObjectCommand));

  addChangeFilter(createLocalTypeFilter<VisionData::ProtoObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onAdd_ProtoObject));
  addChangeFilter(createLocalTypeFilter<VisionData::ProtoObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onUpdate_ProtoObject));
  addChangeFilter(createLocalTypeFilter<VisionData::ProtoObject>(cdl::DELETE),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onDelete_ProtoObject));
  if (m_snapper.m_bAutoSnapshot) {
    log("AUTOSNAP is ON; Triggered on ProtoObject OVERWRITE");
  }

  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onAdd_VisualObject));
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onUpdate_VisualObject));
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::DELETE),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onDelete_VisualObject));

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
  if (!pFilter) return;
  if (event.type == Visualization::evButtonClick) {
    if (event.sourceId == "take.snapshot") {
      pFilter->m_snapper.saveSnapshot();
    }
#ifdef FEAT_GENERATE_FAKE_SOIS
    else if (event.sourceId == "fake.soi") {
      pFilter->addFakeSoi();
    }
#endif
  }
  if (event.sourceId == IDC_SOIF_PROTOOBJECTS) {
    if (event.data == "0" || event.data=="") pFilter->m_bShowProtoObjects = false;
    else pFilter->m_bShowProtoObjects = true;
    pFilter->sendSyncAllProtoObjects();
  }
}

std::string SOIFilter::CSfDisplayClient::getControlState(const std::string& ctrlId)
{
    if (!pFilter) return "";
    pFilter->println("Get control state: %s", ctrlId.c_str());
    if (ctrlId == IDC_SOIF_PROTOOBJECTS) {
	if (pFilter->m_bShowProtoObjects) return "2";
	else return "0";
    }
    return "";
}

void SOIFilter::sendSyncAllProtoObjects()
{
  // copy to minimize race conditions
  std::map<cdl::WorkingMemoryAddress, VisionData::ProtoObjectPtr> objs = m_protoObjects;
  typeof(objs.begin()) it = objs.begin();
  for (; it != objs.end(); ++it) {
    ProtoObjectPtr& pobj = it->second;
    if (m_bShowProtoObjects) sendProtoObject(it->first, pobj);
    else sendRemoveProtoObject(it->first);
  }
}

void SOIFilter::sendProtoObject(const cdl::WorkingMemoryAddress& addr, const VisionData::ProtoObjectPtr& pobj)
{
  cdl::WorkingMemoryAddress voaddr = findVisualObjectFor(addr);
  bool bHasVo = voaddr.id != "";

  ostringstream ss;
  ss << "function render()\n";
  ss << "glPushMatrix()\n";
  if (bHasVo)
    ss << "glColor(0.0, 1.0, 0.0, 0.3)\n";
  else
    ss << "glColor(0.2, 0.2, 0.2, 0.3)\n";
  ss << "glTranslate("
    << pobj->position.x << ","
    << pobj->position.y << ","
    << pobj->position.z << ")\n";
  ss << "StdModel:box(0.05,0.05,0.05)\n";
  ss << "glPopMatrix()\n";
  ss << "end\n";
  m_display.setLuaGlObject(m_sProtoObjectView, ID_PART_3D_PO + addr.id, ss.str());
}

void SOIFilter::sendRemoveProtoObject(const cdl::WorkingMemoryAddress& addr)
{
  m_display.setLuaGlObject(m_sProtoObjectView, ID_PART_3D_PO + addr.id, "function render()\nend\n");
  m_display.removePart(m_sProtoObjectView, ID_PART_3D_PO + addr.id);
}

void SOIFilter::CSfDisplayClient::onDialogValueChanged(const std::string& dialogId,
    const std::string& name, const std::string& value)
{
  if (dialogId == "PtuCtrl" && pFilter->ptzServer.get()) {
    if (name == "PTZ") {
      pFilter->println(" *** PTZ *** ");
      double pan, tilt, zoom;
      int nf = sscanf(value.c_str(), "%lf, %lf, %lf", &pan, &tilt, &zoom);
      if (nf == 3) {
        ptz::PTZPose p;
        p.pan = pan * M_PI / 180;
        p.tilt = tilt * M_PI / 180;
        p.zoom = zoom;
        pFilter->ptzServer->setPose(p);
      }
    }
  }
}

void SOIFilter::CSfDisplayClient::handleDialogCommand(const std::string& dialogId,
    const std::string& command, const std::string& params)
{
  if (dialogId == "PtuCtrl" && pFilter->ptzServer.get()) {
    if (command == "sendStateToDialog") {
      pFilter->println(" *** sendStateToDialog *** ");
      ptz::PTZReading ptup;
      if (pFilter->ptzServer.get())
        ptup = pFilter->ptzServer->getPose();
      ostringstream ss;
      ss << "ptuctrl.ui.wctrls.spinPan.value=" << ptup.pose.pan * 180 / M_PI << ";";
      ss << "ptuctrl.ui.wctrls.spinTilt.value=" << ptup.pose.tilt * 180 / M_PI << ";";
      ss << "ptuctrl.ui.wctrls.spinZoom.value=" << ptup.pose.zoom << ";";

      execInDialog(dialogId, ss.str());
    }
  }
}
#endif

#ifdef FEAT_GENERATE_FAKE_SOIS
#define SOURCE_FAKE_SOI   "--fake.soi"
void SOIFilter::addFakeSoi()
{
  VisionData::SOIPtr psoi = new VisionData::SOI;
  psoi->sourceId = SOURCE_FAKE_SOI;
  psoi->status = 0;
  double x, y, z, r;
  // SOURCE_FAKE_SOI coordinates are in % of image size (top-left = 0.0) , (bottom-right = 1.0)
  x = 0.7; /* TODO rand */
  y = 0.7; /* TODO rand */
  z = 0.18;
  r = 0.05;

  psoi->boundingBox.pos.x = x;
  psoi->boundingBox.pos.y = y;
  psoi->boundingBox.pos.z = z;
  psoi->boundingBox.size.x = r;
  psoi->boundingBox.size.y = r;
  psoi->boundingBox.size.z = r;

  psoi->boundingSphere.pos.x = x;
  psoi->boundingSphere.pos.y = y;
  psoi->boundingSphere.pos.z = z;
  psoi->boundingSphere.rad = r;

  psoi->time = getCASTTime();
  //obs->points = psIn1SOI;
  //obs->BGpoints = BGpIn1SOI;
  //obs->EQpoints = EQpIn1SOI;

  addToWorkingMemory(newDataID(), psoi);
}
#endif

void SOIFilter::onAdd_SOI(const cdl::WorkingMemoryChange & _wmc)
{
  {
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);
    m_EventQueue.push_back(new WmEvent(TYPE_SOI, cdl::ADD, _wmc));
  }
  m_EventQueueMonitor.notify();
}

void SOIFilter::onDelete_SOI(const cdl::WorkingMemoryChange & _wmc)
{
  {
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);
    m_EventQueue.push_back(new WmEvent(TYPE_SOI, cdl::DELETE, _wmc));
  }
  m_EventQueueMonitor.notify();
}

void SOIFilter::onUpdate_SOI(const cdl::WorkingMemoryChange & _wmc)
{
  {
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);
    m_EventQueue.push_back(new WmEvent(TYPE_SOI, cdl::OVERWRITE, _wmc));
  }
  m_EventQueueMonitor.notify();
}

void SOIFilter::onAdd_MoveToVcCommand(const cdl::WorkingMemoryChange & _wmc)
{
  {
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);
    m_EventQueue.push_back(new WmEvent(TYPE_CMD_LOOK, cdl::ADD, _wmc));
  }
  m_EventQueueMonitor.notify();
}

void SOIFilter::onAdd_AnalyzeProtoObjectCommand(const cdl::WorkingMemoryChange & _wmc)
{
  {
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);
    m_EventQueue.push_back(new WmEvent(TYPE_CMD_ANALYZE, cdl::ADD, _wmc));
  }
  m_EventQueueMonitor.notify();
}


// Save a part of a ProtoObject to the internal database
void SOIFilter::saveProtoObjectData(VisionData::ProtoObjectPtr& poOrig, VisionData::ProtoObjectPtr& poCopy)
{
  poCopy->cameraLocation = poOrig->cameraLocation;
  poCopy->position = poOrig->position;
}

// Save a part of a VisualObject to the internal database
void SOIFilter::saveVisualObjectData(VisionData::VisualObjectPtr& voOrig, VisionData::VisualObjectPtr& voCopy)
{
  // Tracking visible objects
  voCopy->protoObject = voOrig->protoObject;
  voCopy->lastProtoObject = voOrig->lastProtoObject;
}

void SOIFilter::onAdd_ProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  ProtoObjectPtr pobj = getMemoryEntry<VisionData::ProtoObject>(_wmc.address);
  m_protoObjects[_wmc.address] = new VisionData::ProtoObject();
  saveProtoObjectData(pobj, m_protoObjects[_wmc.address]);
  if (m_bShowProtoObjects) {
    sendProtoObject(_wmc.address, pobj);
  }
}

void SOIFilter::onUpdate_ProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  ProtoObjectPtr pobj = getMemoryEntry<VisionData::ProtoObject>(_wmc.address);
  m_protoObjects[_wmc.address] = new VisionData::ProtoObject();
  saveProtoObjectData(pobj, m_protoObjects[_wmc.address]);

  if (m_snapper.m_bAutoSnapshot) {
    // XXX: Added to saveSnapshot with SurfacePatches
    // XXX: ASSUME it's the same protoobject
    m_snapper.m_LastProtoObject = pobj;
    m_snapper.saveSnapshot();
  }
  if (m_bShowProtoObjects) {
    sendProtoObject(_wmc.address, pobj);
  }
}

void SOIFilter::onDelete_ProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  m_protoObjects.erase(_wmc.address);
  if (m_bShowProtoObjects) {
    sendRemoveProtoObject(_wmc.address);
  }
}

void SOIFilter::onAdd_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisualObjectPtr pobj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
  m_visualObjects[_wmc.address] = new VisionData::VisualObject();
  saveVisualObjectData(pobj, m_visualObjects[_wmc.address]);
}

void SOIFilter::onUpdate_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisualObjectPtr pobj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
  m_visualObjects[_wmc.address] = new VisionData::VisualObject();
  saveVisualObjectData(pobj, m_visualObjects[_wmc.address]);
}

void SOIFilter::onDelete_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  m_visualObjects.erase(_wmc.address);
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
  log("Got robot pose");
}

void SOIFilter::runComponent()
{
  WmTaskExecutor_Soi soiProcessor(this);
  WmTaskExecutor_MoveToViewCone moveProcessor(this);
  WmTaskExecutor_Analyze analysisProcessor(this);

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
      switch (pevent->objectType)
      {
        case TYPE_SOI:
          soiProcessor.handle(pevent);
          break;
        case TYPE_CMD_LOOK:
          moveProcessor.handle(pevent);
          break;
        case TYPE_CMD_ANALYZE:
          analysisProcessor.handle(pevent);
          break;
        default:
          error(" ***** Event with an unknown type of object '%d'", pevent->objectType);
      };
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

#if 1
#define YY(y) -(y)
#else
#define YY(y) y
#endif


// psoi is calculated at pSoiFilter->m_RobotPose
ProtoObjectPtr SOIFilter::findProtoObjectAt(SOIPtr psoi)
{
  if (!psoi.get()) 
    return NULL;

  typeof(m_protoObjects.begin()) itpo = m_protoObjects.begin();
  double dmin = 1e99;
  double d;
  ProtoObjectPtr pBest;
  Vector3& p0 = psoi->boundingSphere.pos;
  log("Find PO near SOI (%.4lf, %.4lf, %.4lf)", p0.x, p0.y, p0.z);

  for(; itpo != m_protoObjects.end(); ++itpo) {
    ProtoObjectPtr& pobj = itpo->second;
    // XXX: We assume that pobj->cameraLocation is at pSoiFilter->m_RobotPose
    //   => we only compare robo-centric positions
    Vector3& p1 = pobj->position;

    d = dist(p0, p1);
    log("PO %s (%.4lf, %.4lf, %.4lf): %.4lf", itpo->first.id.c_str(), p1.x, p1.y, p1.z, d);
    if (d < dmin) {
      dmin = d;
      pBest = pobj;
    }
  }
  if (dmin < 0.12)
    return pBest;

  return NULL;
}

cdl::WorkingMemoryAddress SOIFilter::findVisualObjectFor(const cdl::WorkingMemoryAddress& protoAddr)
{
  if (protoAddr.id == "") 
    return cdl::WorkingMemoryAddress();

  // First check the visible objects
  typeof(m_visualObjects.begin()) itvo = m_visualObjects.begin();
  for(; itvo != m_visualObjects.end(); ++itvo) {
    VisualObjectPtr& pobj = itvo->second;
    if (pobj->protoObject->address == protoAddr)
      return itvo->first;
  }

  // The check the hidden objects
  itvo = m_visualObjects.begin();
  for(; itvo != m_visualObjects.end(); ++itvo) {
    VisualObjectPtr& pobj = itvo->second;
    if (pobj->lastProtoObject->address == protoAddr)
      return itvo->first;
  }

  return cdl::WorkingMemoryAddress();
}

/*
 * TODO: 2 step VisualObject generation.
 *   1. If the SOI comes from coarseSource, create a proto-object with the
 *      desired ViewCones
 *   2. If the SOI comes from fineSource, we need to know which PO it belongs
 *      to (SOI matching by position), then we update the PO and create the VO.
 * TODO: Decide how to process stuff when there is only a single source ...
 */
void SOIFilter::WmTaskExecutor_Soi::handle_add_soi(WmEvent* pEvent)
{
  pSoiFilter->debug("SOIFilter::handle_add_soi");
  SOIPtr psoi;
  try {
    psoi = pSoiFilter->getMemoryEntry<VisionData::SOI>(pEvent->wmc.address);
  }
  catch(cast::DoesNotExistOnWMException){
    pSoiFilter->log("SOIFilter.add_soi: SOI deleted while working...");
    return;
  }

  if (pSoiFilter->ptzServer.get()) {
    // XXX: update the "anchor" to the most recent location.
    // We need this here because onChange_RobotPose doesn't get executed if the robot doesn't move.
    ptz::PTZReading ptup;
    ptup = pSoiFilter->ptzServer->getPose();
    pSoiFilter->m_RobotPose.pan = ptup.pose.pan;
    pSoiFilter->m_RobotPose.tilt = ptup.pose.tilt;
  }

  SOIData soi;
  soi.addr = pEvent->wmc.address;
  soi.addTime = psoi->time;
  soi.status = STABLE;
  soi.stableTime = pSoiFilter->getCASTTime();
  soi.updCount = 0;
  pSoiFilter->SOIMap.insert(make_pair(soi.addr.id, soi));

  if (psoi->sourceId == pSoiFilter->m_coarseSource || psoi->sourceId == SOURCE_FAKE_SOI)
  {
    // Verify all known proto objects if they are at the same location as the SOI;
    // In this case, we are looking at a known PO and don't have to do
    // anything, except if we want to analyze the object in the center.

    ProtoObjectPtr pobj;
    pSoiFilter->log("FIND SOI '%s'", soi.addr.id.c_str());
    pobj = pSoiFilter->findProtoObjectAt(psoi);
    if (pobj.get()) {
      pSoiFilter->log("SOI '%s' belongs to a known ProtoObject", soi.addr.id.c_str());
      // XXX: Do we update the PO with the new SOI?
      // XXX: The object recognizer should verify if this is the same object
      return;
    }

    if (psoi->boundingSphere.pos.x < 0.6 && psoi->boundingSphere.pos.z < 0.5) {
      pSoiFilter->log("SOI '%s' could be the Katana arm.", soi.addr.id.c_str());
      // XXX: may be looking at the katana arm, ignore
      return;
    }

    pobj = new ProtoObject();
    pSoiFilter->m_snapper.m_LastProtoObject = pobj;
    string objId = pSoiFilter->newDataID();

    pobj->position = psoi->boundingSphere.pos;

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
    pCurVc->viewDirection = pSoiFilter->m_RobotPose.theta + pSoiFilter->m_RobotPose.pan;
    pCurVc->tilt = pSoiFilter->m_RobotPose.tilt;
    pobj->cameraLocation = pCurVc;

    double dirDelta = 0;
    double tiltDelta = 0;
    CameraParameters camPars;
    if (! pSoiFilter->m_coarsePointCloud.getCameraParameters(LEFT, camPars)) {
      pSoiFilter->println("FAILED to get the camera parameters from '%s'",
          pSoiFilter->m_coarsePcServer.c_str());
    }
    else {
      pobj->image.camPars = camPars;

      ROIPtr roiPtr = projectSOI(camPars, *psoi);
      if (psoi->sourceId == SOURCE_FAKE_SOI) {
        roiPtr->rect.pos.x = 640 * psoi->boundingBox.pos.x;
        roiPtr->rect.pos.y = 480 * psoi->boundingBox.pos.y;
      }

      // Center of the projected SOI ... Math::Rect2.pos IS the center
      double rcx = roiPtr->rect.pos.x;
      double rcy = roiPtr->rect.pos.y;

      // how far from the center of the LEFT image is the SOI
      dirDelta  = -atan( (rcx - camPars.cx) / camPars.fx); // negative pan is to the right
      tiltDelta = -atan( (rcy - camPars.cy) / camPars.fy); // y is inverted between image and tilt

#if defined(FEAT_VISUALIZATION) && defined(HAS_LIBPLOT)
      // draw the thing
      std::ostringstream ssvg;
      cogx::display::CSvgStringPlotter p(ssvg);
      p.openpl();
      p.flinewidth (2.0);        // line thickness in user coordinates
      p.pencolorname ("red");    // path will be drawn in red

      p.line(camPars.cx, YY(camPars.cy), rcx, YY(rcy));
      p.line(camPars.cx, YY(camPars.cy), camPars.cx - dirDelta * 180 / 3.14, YY(camPars.cy));
      p.line(camPars.cx, YY(camPars.cy), camPars.cx, YY(camPars.cy - tiltDelta * 180 / 3.14));

      p.closepl();

      string s = p.getScreenSvg();

      pSoiFilter->m_display.setObject(OBJ_VISUAL_OBJECTS, "soif-last-move", s);
      //pSoiFilter->m_display.setHtml("KrNeki", "soif-last-move-text", s);
#endif
    }

    // New view cone for turning the head
    ViewConePtr pBetterVc = new ViewCone();
    pBetterVc->anchor = pCurVc->anchor;
    pBetterVc->x = pCurVc->x;
    pBetterVc->y = pCurVc->y;
    pBetterVc->viewDirection = pCurVc->viewDirection + dirDelta;
    pBetterVc->tilt = pCurVc->tilt + tiltDelta;
    pBetterVc->target = new cdl::WorkingMemoryPointer();
    pBetterVc->target->address = cast::makeWorkingMemoryAddress(objId, pSoiFilter->getSubarchitectureID());
    pBetterVc->target->type = cast::typeName<ProtoObject>();

    // Address at which new view cone will be stored
    WorkingMemoryAddress vcAddr = cast::makeWorkingMemoryAddress(pSoiFilter->newDataID(), pSoiFilter->getSubarchitectureID());
    // Write viewcone to memory
    pSoiFilter->addToWorkingMemory(vcAddr, pBetterVc);

    // Create pointer to viewcone on WM
    WorkingMemoryPointerPtr vcPtr = new WorkingMemoryPointer();
    vcPtr->address = vcAddr;
    vcPtr->type = cast::typeName<ViewCone>();
    pobj->desiredLocations.push_back(vcPtr);

    // Add PO to WM
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

void SOIFilter::WmTaskExecutor_Soi::handle_delete_soi(WmEvent* pEvent)
{
  const WorkingMemoryChange &wmc = pEvent->wmc;
  SOIData &soi = pSoiFilter->SOIMap[wmc.address.id];
  soi.status = DELETED;

  if (soi.objId.size() > 0)
  {
    // TODO: We have to load the PO and save it again. A lot of transfer !!!
  }
}

void SOIFilter::WmTaskExecutor_Analyze::handle_add_task(WmEvent* pEvent)
{
  class CCmd:
    public cogx::VisionCommandNotifier<AnalyzeProtoObjectCommand, AnalyzeProtoObjectCommandPtr>
  {
  public:
    CCmd(cast::WorkingMemoryReaderComponent* pReader)
      : cogx::VisionCommandNotifier<AnalyzeProtoObjectCommand, AnalyzeProtoObjectCommandPtr>(pReader) {}
  protected:
    virtual void doFail() { pcmd->status = VisionData::VCFAILED; }
    virtual void doSucceed() { pcmd->status = VisionData::VCSUCCEEDED; }
  } cmd(pSoiFilter);

  pSoiFilter->println("SOIFilter.AnalyzeProtoObjectCommand");

  if (! cmd.read(pEvent->wmc.address)) {
    pSoiFilter->debug("SOIFilter.analyze_task: AnalyzeProtoObjectCommand deleted while working.");
    return;
  }

  // pobj - The proto object to segment
  ProtoObjectPtr pobj;
  try {
    pobj = pSoiFilter->getMemoryEntry<VisionData::ProtoObject>(cmd.pcmd->protoObjectAddr);
  }
  catch(cast::DoesNotExistOnWMException){
    pSoiFilter->debug("SOIFilter.analyze_task: ProtoObject deleted while working. Aborting task.");
    cmd.fail();
    return;
  }
  //catch (Ice::MarshalException){
  //  pSoiFilter->debug("SOIFilter.analyze_task: ProtoObject MarshalException. Aborting task.");
  //  cmd.fail();
  //  return;
  //}

  /* Asynchronous call through a working memory entry.
   *
   * A WorkingMemoryChangeReceiver is created on the heap. We wait for it to
   * complete then we remove it from the change filter list, but we DON'T
   * DELETE it. It will be moved to a deletion queue by the framework and
   * deleted later. */
  GetSoisCommandRcv* pGetSois;
  vector<SOIPtr> sois;
  pGetSois = new GetSoisCommandRcv(pSoiFilter, pSoiFilter->m_fineSource);
  bool bCompleted = pGetSois->waitForCompletion(20e3/*ms*/);
  bCompleted = bCompleted && pGetSois->m_pcmd->status == VisionData::VCSUCCEEDED;
  if (bCompleted)
    pGetSois->getSois(sois);
  pSoiFilter->removeChangeFilter(pGetSois, cdl::DELETERECEIVER);

  if (! bCompleted || sois.size() < 1)
  {
    pSoiFilter->debug("SOIFilter.analyze_task: No Fine SOIs. Aborting task.");
    cmd.fail();
    return;
  }

  pSoiFilter->debug("SOIFilter.analyze_task: Got some Fine SOIs.");

  // Find the SOI that is closesst to the PO
  // psoi - The fine SOI that represents the proto object
  SOIPtr psoi = sois[0];
  double d, dmin;
  dmin = dist(pobj->position, psoi->boundingSphere.pos);
  for (int i = 1; i < sois.size(); ++i) {
    d = dist(pobj->position, sois[i]->boundingSphere.pos);
    if (d < dmin) {
      dmin = d;
      psoi = sois[i];
    }
  }
  if (dmin >= 0.2) {
    pSoiFilter->println("SOIFilter.analyze_task: SOI to be analyzed is far from PO (%.2lfm)", dmin);
  }

  if(pSoiFilter->m_segmenter.segmentObject(psoi, pobj->image, pobj->mask, pobj->points, pobj))
  {
    pSoiFilter->overwriteWorkingMemory(cmd.pcmd->protoObjectAddr, pobj);
  } // TODO: else: do we fail if segmentObject failed?


  // find VO for this PO
  WorkingMemoryAddress voAddr = pSoiFilter->findVisualObjectFor(cmd.pcmd->protoObjectAddr);
  VisualObjectPtr pvo;
  if (voAddr.id == "") {
    pvo = new VisualObject();
    pvo->protoObject = new cdl::WorkingMemoryPointer();
    pvo->protoObject->type = cast::typeName<ProtoObject>();
    pvo->protoObject->address = cmd.pcmd->protoObjectAddr;
    pvo->lastProtoObject = pvo->protoObject;

    voAddr = cast::makeWorkingMemoryAddress(pSoiFilter->newDataID(), pSoiFilter->getSubarchitectureID());
    pSoiFilter->addToWorkingMemory(voAddr, pvo);
  }
  else {
    try {
      pvo = pSoiFilter->getMemoryEntry<VisionData::VisualObject>(voAddr);
    }
    catch(cast::DoesNotExistOnWMException){
      pSoiFilter->debug("SOIFilter.analyze_task: VisualObject deleted while working.");
      return;
    }
  }


  cmd.succeed();
}

SOIFilter::GetSoisCommandRcv::GetSoisCommandRcv(SOIFilter* psoif, std::string component_id)
{
  m_complete = false;
  pSoiFilter = psoif;
  m_pcmd = new GetStableSoisCommand();
  m_pcmd->componentId = component_id;
  string id = pSoiFilter->newDataID();
  pSoiFilter->addChangeFilter(createIDFilter(id, cdl::OVERWRITE), this);  
  pSoiFilter->addToWorkingMemory(id, m_pcmd);  
}

void SOIFilter::GetSoisCommandRcv::workingMemoryChanged(const cast::cdl::WorkingMemoryChange &_wmc)
{
  {
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_CompletionMonitor);
    try {
      GetStableSoisCommandPtr pcmd = pSoiFilter->getMemoryEntry<GetStableSoisCommand>(_wmc.address);
      m_pcmd = pcmd;
    }
    catch (...) {
      pSoiFilter->debug("SOIFilter.GetSoisCommand: Failed to get the results.");
      m_pcmd->status = VisionData::VCFAILED; /* complete, but failed */
    }
    m_complete = true;
  }
  m_CompletionMonitor.notify();
}

void SOIFilter::GetSoisCommandRcv::getSois(std::vector<VisionData::SOIPtr>& sois)
{
  sois = m_pcmd->sois;
}

bool SOIFilter::GetSoisCommandRcv::waitForCompletion(double milliSeconds)
{
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_CompletionMonitor);
  if (m_complete) return true;
  m_CompletionMonitor.timedWait(IceUtil::Time::milliSeconds(milliSeconds));
  return m_complete;
}

void SOIFilter::WmTaskExecutor_MoveToViewCone::handle_add_task(WmEvent *pEvent)
{
  class CCmd:
    public cogx::VisionCommandNotifier<MoveToViewConeCommand, MoveToViewConeCommandPtr>
  {
  public:
    CCmd(cast::WorkingMemoryReaderComponent* pReader)
      : cogx::VisionCommandNotifier<MoveToViewConeCommand, MoveToViewConeCommandPtr>(pReader) {}
  protected:
    virtual void doFail() { pcmd->status = VisionData::VCFAILED; }
    virtual void doSucceed() { pcmd->status = VisionData::VCSUCCEEDED; }
  } cmd(pSoiFilter);

  pSoiFilter->println("SOIFilter.MoveToViewConeCommand");

  if (!cmd.read(pEvent->wmc.address)) {
    pSoiFilter->debug("SOIFilter.move_to: MoveToViewConeCommand deleted while working.");
    return;
  }
  pSoiFilter->debug("SOIFilter.move_to: GOT A MoveToViewConeCommand");

  if (! pSoiFilter->ptzServer.get())
    cmd.fail();
  else {
    // XXX: This command only moves the PTU, but should also move the robot
    ptz::PTZReading ptup;
    ViewConePtr pBetterVc = pSoiFilter->getMemoryEntry<VisionData::ViewCone>(cmd.pcmd->target->address);
    ptup.pose.pan = pBetterVc->viewDirection - pBetterVc->anchor.z;
    ptup.pose.tilt = pBetterVc->tilt;
    ptup.pose.zoom = 0;
    pSoiFilter->log("PTU Command: pan to %.3f, tilt to %.3f", ptup.pose.pan, ptup.pose.tilt);
    pSoiFilter->ptzServer->setPose(ptup.pose);
    cmd.succeed();
  }
}


} // namespace
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */

