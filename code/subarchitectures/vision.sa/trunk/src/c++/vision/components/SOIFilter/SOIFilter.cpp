/**
 * @author Marko Mahnič
 * @date May 2011
 *
 */

#include "SOIFilter.h"
#include "TaskReceiveSoi.h"
#include "TaskMoveToViewCone.h"
#include "TaskAnalyzePo.h"

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

  //default value
  ptzServerName = "ptz.server";
  if((it = _config.find("--ptzserver")) != _config.end())
  {
    ptzServerName = it->second;
  }

#ifdef FEAT_VISUALIZATION
  m_display.configureDisplayClient(_config);
#endif
}

void SOIFilter::connectPtz() {
  ptzServer = getIceServer<ptz::PTZInterface>(ptzServerName);
  if (! ptzServer.get()) {
    println(" *** PTZ SERVER NOT FOUND ***");
  }
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

  addChangeFilter(createLocalTypeFilter<VisionData::LookAroundCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onAdd_LookAroundCommand));

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
  std::map<cdl::WorkingMemoryAddress, ProtoObjectRecordPtr> objs = m_protoObjects;
  typeof(objs.begin()) it = objs.begin();
  for (; it != objs.end(); ++it) {
    ProtoObjectRecordPtr& pporec = it->second;

    if (!pporec->pobj.get() || !m_bShowProtoObjects)
      sendRemoveProtoObject(it->first);
    else sendProtoObject(it->first, pporec->pobj);
  }
}

void SOIFilter::sendProtoObject(const cdl::WorkingMemoryAddress& addr, const VisionData::ProtoObjectPtr& pobj)
{
  // The first time the PO is analyzed, the VO may still not exist in WM, but 
  // PO already references it.
  // (Temporary) fix: send all POs again after every VO change (done in onXXX_VisualObject).
  VisualObjectRecordPtr pvorec = findVisualObjectFor(addr);
  bool bHasVo = pvorec.get() != NULL;

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
  if (bHasVo)
    ss << "glColor(0.0, 1.0, 0.0)\n";
  else
    ss << "glColor(0.2, 0.2, 0.2)\n";
  ss << "showLabel(0, 0, 0.06, 'PO:" << addr.id << "', 12);\n";
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
    //pFilter->println(" *** handleDialogCommand *** " + command);
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

void SOIFilter::onAdd_SOI(const cdl::WorkingMemoryChange & _wmc)
{
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);
  m_EventQueue.push_back(new WmEvent(TYPE_SOI, cdl::ADD, _wmc));
  m_EventQueueMonitor.notify(); // works only if the monitor is locked here
}

void SOIFilter::onDelete_SOI(const cdl::WorkingMemoryChange & _wmc)
{
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);
  m_EventQueue.push_back(new WmEvent(TYPE_SOI, cdl::DELETE, _wmc));
  m_EventQueueMonitor.notify(); // works only if the monitor is locked here
}

void SOIFilter::onUpdate_SOI(const cdl::WorkingMemoryChange & _wmc)
{
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);
  m_EventQueue.push_back(new WmEvent(TYPE_SOI, cdl::OVERWRITE, _wmc));
  m_EventQueueMonitor.notify(); // works only if the monitor is locked here
}

void SOIFilter::onAdd_MoveToVcCommand(const cdl::WorkingMemoryChange & _wmc)
{
  debug("RECEIVED: MoveToViewConeCommand %s", _wmc.address.id.c_str());
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);
  m_EventQueue.push_back(new WmEvent(TYPE_CMD_LOOK, cdl::ADD, _wmc));
  m_EventQueueMonitor.notify(); // works only if the monitor is locked here
}

void SOIFilter::onAdd_AnalyzeProtoObjectCommand(const cdl::WorkingMemoryChange & _wmc)
{
  debug("RECEIVED: AnalyzeProtoObjectCommand %s", _wmc.address.id.c_str());
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);
  m_EventQueue.push_back(new WmEvent(TYPE_CMD_ANALYZE, cdl::ADD, _wmc));
  m_EventQueueMonitor.notify(); // works only if the monitor is locked here
}

void SOIFilter::onAdd_LookAroundCommand(const cdl::WorkingMemoryChange & _wmc)
{
  debug("RECEIVED: LookAroundCommand %s", _wmc.address.id.c_str());
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventQueueMonitor);
  m_EventQueue.push_back(new WmEvent(TYPE_CMD_LOOK_AROUND, cdl::ADD, _wmc));
  m_EventQueueMonitor.notify(); // works only if the monitor is locked here
}

void SOIFilter::saveSoiData(VisionData::SOIPtr& soiOrig, VisionData::SOIPtr& soiCopy)
{
  soiCopy->boundingSphere = soiOrig->boundingSphere;
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
  ProtoObjectRecordPtr pporec = new ProtoObjectRecord();
  pporec->addr = _wmc.address;
  pporec->pobj = new VisionData::ProtoObject();
  saveProtoObjectData(pobj, pporec->pobj);
  m_protoObjects[pporec->addr] = pporec;

  if (m_bShowProtoObjects) {
    sendProtoObject(pporec->addr, pobj);
  }
}

void SOIFilter::onUpdate_ProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  ProtoObjectPtr pobj = getMemoryEntry<VisionData::ProtoObject>(_wmc.address);
  ProtoObjectRecordPtr pporec = new ProtoObjectRecord();
  pporec->addr = _wmc.address;
  pporec->pobj = new VisionData::ProtoObject();
  saveProtoObjectData(pobj, pporec->pobj);
  m_protoObjects[pporec->addr] = pporec;

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
  VisualObjectRecordPtr pvorec = new VisualObjectRecord();
  pvorec->addr = _wmc.address;
  pvorec->pobj = new VisionData::VisualObject();
  saveVisualObjectData(pobj, pvorec->pobj);
  m_visualObjects[pvorec->addr] = pvorec;

  sendSyncAllProtoObjects();
}

void SOIFilter::onUpdate_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisualObjectPtr pobj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
  VisualObjectRecordPtr pvorec = new VisualObjectRecord();
  pvorec->addr = _wmc.address;
  pvorec->pobj = new VisionData::VisualObject();
  saveVisualObjectData(pobj, pvorec->pobj);
  m_visualObjects[pvorec->addr] = pvorec;

  sendSyncAllProtoObjects();
}

void SOIFilter::onDelete_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  m_visualObjects.erase(_wmc.address);
  sendSyncAllProtoObjects();
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

void SOIFilter::updateRobotPosePtz()
{
  if (ptzServer.get()) {
    // XXX: update the "anchor" to the most recent location.
    // We need this here because onChange_RobotPose doesn't get executed if the robot doesn't move.
    ptz::PTZReading ptup;
    ptup = ptzServer->getPose();
    m_RobotPose.pan = ptup.pose.pan;
    m_RobotPose.tilt = ptup.pose.tilt;
  }
}

bool SOIFilter::movePtz(double pan, double tilt, double zoom)
{
  if (!ptzServer.get())
    return false;

  ptz::PTZReading ptup;
  ptup.pose.pan = pan;
  ptup.pose.tilt = tilt;
  ptup.pose.zoom = zoom;
  log("PTU Command: pan to %.3f, tilt to %.3f", ptup.pose.pan, ptup.pose.tilt);
  ptzServer->setPose(ptup.pose);
  return true;
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
        case TYPE_CMD_LOOK_AROUND:
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

// psoi is calculated at pSoiFilter->m_RobotPose
ProtoObjectRecordPtr SOIFilter::findProtoObjectAt(const Vector3 &pos)
{
  typeof(m_protoObjects.begin()) itpo = m_protoObjects.begin();
  double dmin = 1e99;
  double d;
  ProtoObjectRecordPtr pBest;
  const Vector3& p0 = pos;
  log("Find PO near (%.4lf, %.4lf, %.4lf)", p0.x, p0.y, p0.z);

  for(; itpo != m_protoObjects.end(); ++itpo) {
    ProtoObjectRecordPtr& pporec = itpo->second;
    ProtoObjectPtr& pobj = pporec->pobj;
    if (!pobj.get())
      continue;

    // XXX: We assume that pobj->cameraLocation is at pSoiFilter->m_RobotPose
    //   => we only compare robo-centric positions
    Vector3& p1 = pobj->position;

    d = dist(p0, p1);
    log("PO %s (%.4lf, %.4lf, %.4lf): %.4lf", itpo->first.id.c_str(), p1.x, p1.y, p1.z, d);
    if (d < dmin) {
      dmin = d;
      pBest = pporec;
    }
  }
  if (dmin < 0.12)
    return pBest;

  return NULL;
}

ProtoObjectRecordPtr SOIFilter::findProtoObjectAt(const SOIPtr &psoi)
{
  if (!psoi.get()) 
    return NULL;

  return findProtoObjectAt(psoi->boundingSphere.pos);
}

VisualObjectRecordPtr SOIFilter::findVisualObjectFor(const cdl::WorkingMemoryAddress& protoAddr)
{
  if (protoAddr.id == "") 
    return NULL;

  // First check the visible objects
  typeof(m_visualObjects.begin()) itvo = m_visualObjects.begin();
  for(; itvo != m_visualObjects.end(); ++itvo) {
    VisualObjectRecordPtr& pvorec = itvo->second;
    VisualObjectPtr& pobj = pvorec->pobj;
    if (!pobj.get())
      continue;

    if (pobj->protoObject->address == protoAddr)
      return itvo->second;
  }

  // The check the hidden objects
  itvo = m_visualObjects.begin();
  for(; itvo != m_visualObjects.end(); ++itvo) {
    VisualObjectRecordPtr& pvorec = itvo->second;
    VisualObjectPtr& pobj = pvorec->pobj;
    if (!pobj.get())
      continue;

    if (pobj->lastProtoObject->address == protoAddr)
      return itvo->second;
  }

  return NULL;
}

} // namespace
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */

