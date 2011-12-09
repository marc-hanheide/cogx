/**
 * @author Marko Mahnič
 * @date May 2011
 *
 */

#include "SOIFilter.h"
#include "TaskReceiveSoi.h"
#include "TaskMoveToViewCone.h"
#include "TaskAnalyzePo.h"
#include "WmUnlocker.h"

#include <castutils/Timers.hpp>

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <fstream>
#include <cmath>

#ifdef FEAT_TRACK_ARM
namespace ArmIce = manipulation::execution::slice;
#endif

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
  m_bCameraMoving = false;
  m_minSoiDistance = 0.5; // robot body
  m_maxSoiDistance = 3.0;
  m_identityRecognizerVersion = 3;
#ifdef FEAT_TRACK_ARM
  m_minArmDistance = 0.3;
#endif

#ifdef FEAT_VISUALIZATION
  m_segmenter.pDisplay = &m_display;
  m_segmenter.setLoggingComponent(this);
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

  if((it = _config.find("--min-soi-dist")) != _config.end())
  {
    istringstream iss(it->second);
    iss >> m_minSoiDistance;
  }

  if((it = _config.find("--max-soi-dist")) != _config.end()) {
    istringstream iss(it->second);
    iss >> m_maxSoiDistance;
  }

  if (m_minSoiDistance >= m_maxSoiDistance || m_minSoiDistance < 0 || m_maxSoiDistance < 0) {
    error("SOI distance limits are not consistent! 0 <= %.4gm < %.4gm", m_minSoiDistance, m_maxSoiDistance);
  }
  else {
    println("SOI limits are:  (%.4gm,  %.4gm) ", m_minSoiDistance, m_maxSoiDistance);
  }

#ifdef FEAT_TRACK_ARM
  if((it = _config.find("--min-arm-dist")) != _config.end()) {
    istringstream iss(it->second);
    iss >> m_minArmDistance;
  }
  println("Min distance from the arm: %.4gm", m_minArmDistance);
#endif

  if((it = _config.find("--identity-recognizer-version")) != _config.end()) {
    istringstream iss(it->second);
    iss >> m_identityRecognizerVersion;
  }
  println("Using 3D object recognizer version %d", m_identityRecognizerVersion);

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
#define IDDLG_PTUCTRL "PtuCtrl"

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

  m_display.addDialog(IDDLG_PTUCTRL, res_ptucontroller_ui, res_ptucontroller_js, "PtuController ptuctrl");

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

  addChangeFilter(createGlobalTypeFilter<Video::CameraParametersWrapper>(cdl::ADD),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onChange_CameraParameters));  
  addChangeFilter(createGlobalTypeFilter<Video::CameraParametersWrapper>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onChange_CameraParameters));  

#ifdef FEAT_TRACK_ARM
  addChangeFilter(createGlobalTypeFilter<ArmIce::ArmStatus>(cdl::ADD),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onChange_ArmPosition));  
  addChangeFilter(createGlobalTypeFilter<ArmIce::ArmStatus>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::onChange_ArmPosition));  
#endif
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
  IceUtil::RWRecMutex::RLock lock(m_protoObjectMapMutex);

  for (auto it = m_protoObjects.begin(); it != m_protoObjects.end(); ++it) {
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
  bool bHasVo = pvorec.get() != nullptr;

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

  ss << "glColor(0.8, 0.9, 0.1)\n";
  for (auto it = m_sois.begin(); it != m_sois.end(); ++it) {
    if (it->second->protoObjectAddr == addr) {
      Vector3& pos = it->second->psoi->boundingSphere.pos;
      ss << "glBegin(GL_LINES)\n"
        << "glVertex("
        << pobj->position.x << ","
        << pobj->position.y << ","
        << pobj->position.z << ")\n"
        << "glVertex("
        << pos.x << ","
        << pos.y << ","
        << pos.z << ")\n"
        << "glEnd()\n";
    }
  }

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
  if (dialogId == IDDLG_PTUCTRL && pFilter->ptzServer.get()) {
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
  if (dialogId == IDDLG_PTUCTRL && pFilter->ptzServer.get()) {
    //pFilter->println(" *** handleDialogCommand *** " + command);
    if (command == "sendStateToDialog")
      sendPtuStateToDialog();
  }
}

void SOIFilter::CSfDisplayClient::sendPtuStateToDialog()
{
  pFilter->log("PtuCtrl: sendStateToDialog");
  ptz::PTZReading ptup;
  if (pFilter->ptzServer.get())
    ptup = pFilter->ptzServer->getPose();
  ostringstream ss;
  ss << "ptuctrl.ui.wctrls.spinPan.value=" << ptup.pose.pan * 180 / M_PI << ";";
  ss << "ptuctrl.ui.wctrls.spinTilt.value=" << ptup.pose.tilt * 180 / M_PI << ";";
  ss << "ptuctrl.ui.wctrls.spinZoom.value=" << ptup.pose.zoom << ";";

  execInDialog(IDDLG_PTUCTRL, ss.str());
}

#endif

void SOIFilter::onAdd_SOI(const cdl::WorkingMemoryChange & _wmc)
{
  m_EventQueue.addItem(new WmEvent(TYPE_SOI, cdl::ADD, _wmc));
}

void SOIFilter::onDelete_SOI(const cdl::WorkingMemoryChange & _wmc)
{
  m_EventQueue.addItem(new WmEvent(TYPE_SOI, cdl::DELETE, _wmc));
}

void SOIFilter::onUpdate_SOI(const cdl::WorkingMemoryChange & _wmc)
{
  m_EventQueue.addItem(new WmEvent(TYPE_SOI, cdl::OVERWRITE, _wmc));
}

void SOIFilter::onAdd_MoveToVcCommand(const cdl::WorkingMemoryChange & _wmc)
{
  debug("RECEIVED: MoveToViewConeCommand %s", _wmc.address.id.c_str());
  m_EventQueue.addItem(new WmEvent(TYPE_CMD_LOOK, cdl::ADD, _wmc));
}

void SOIFilter::onAdd_AnalyzeProtoObjectCommand(const cdl::WorkingMemoryChange & _wmc)
{
  debug("RECEIVED: AnalyzeProtoObjectCommand %s", _wmc.address.id.c_str());
  m_EventQueue.addItem(new WmEvent(TYPE_CMD_ANALYZE, cdl::ADD, _wmc));
}

void SOIFilter::onAdd_LookAroundCommand(const cdl::WorkingMemoryChange & _wmc)
{
  debug("RECEIVED: LookAroundCommand %s", _wmc.address.id.c_str());
  m_EventQueue.addItem(new WmEvent(TYPE_CMD_LOOK_AROUND, cdl::ADD, _wmc));
}

void SOIFilter::saveSoiData(VisionData::SOIPtr& soiOrig, VisionData::SOIPtr& soiCopy)
{
  soiCopy->boundingSphere = soiOrig->boundingSphere;
  // review 2011: since we don't call getstablesois, we have to remember the points for segmentation
  soiCopy->points = soiOrig->points;
  soiCopy->BGpoints = soiOrig->BGpoints;
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

  {
    IceUtil::RWRecMutex::WLock lock(m_protoObjectMapMutex);
    m_protoObjects[pporec->addr] = pporec;

    if (m_bShowProtoObjects) {
      sendProtoObject(pporec->addr, pobj);
    }
  }
}

void SOIFilter::onUpdate_ProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  ProtoObjectPtr pobj = getMemoryEntry<VisionData::ProtoObject>(_wmc.address);
  ProtoObjectRecordPtr pporec = new ProtoObjectRecord();
  pporec->addr = _wmc.address;
  pporec->pobj = new VisionData::ProtoObject();
  saveProtoObjectData(pobj, pporec->pobj);

  {
    IceUtil::RWRecMutex::WLock lock(m_protoObjectMapMutex);
    m_protoObjects[pporec->addr] = pporec;

    if (m_bShowProtoObjects) {
      sendProtoObject(_wmc.address, pobj);
    }
  }

  if (m_snapper.m_bAutoSnapshot) {
    // XXX: Added to saveSnapshot with SurfacePatches
    // XXX: ASSUME it's the same protoobject
    m_snapper.m_LastProtoObject = pobj;
    m_snapper.saveSnapshot();
  }
}

void SOIFilter::onDelete_ProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  {
    IceUtil::RWRecMutex::WLock lock(m_protoObjectMapMutex);
    m_protoObjects.erase(_wmc.address);
  }
  if (m_bShowProtoObjects) {
    sendRemoveProtoObject(_wmc.address);
  }
}

void SOIFilter::onAdd_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  try {
    VisualObjectPtr pobj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
    VisualObjectRecordPtr pvorec = new VisualObjectRecord();
    pvorec->addr = _wmc.address;
    pvorec->pobj = new VisionData::VisualObject();
    saveVisualObjectData(pobj, pvorec->pobj);
    m_visualObjects[pvorec->addr] = pvorec;

    sendSyncAllProtoObjects();
  }
  catch (exception& e) {
    error(" **** onAdd_VisualObject: \n%s\n ****", e.what());
  }
}

void SOIFilter::onUpdate_VisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  try {
    VisualObjectPtr pobj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
    VisualObjectRecordPtr pvorec = new VisualObjectRecord();
    pvorec->addr = _wmc.address;
    pvorec->pobj = new VisionData::VisualObject();
    saveVisualObjectData(pobj, pvorec->pobj);
    m_visualObjects[pvorec->addr] = pvorec;

    sendSyncAllProtoObjects();
  }
  catch (exception& e) {
    error(" **** onUpdate_VisualObject: \n%s\n ****", e.what());
  }
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
    ptup = ptzServer->getPose(); // XXX: this is slow!

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

void SOIFilter::onChange_CameraParameters(const cdl::WorkingMemoryChange & _wmc)
{
  Video::CameraParametersWrapperPtr pcampar =
    getMemoryEntry<Video::CameraParametersWrapper>(_wmc.address);
  if (pcampar->cam.id != camId)
    return;

  // XXX: static, not thread safe
  static castutils::CRunningRate rate;
  static Vector3 prevrot = vector3(0, 0, 0);
  rate.tick();

  Vector3 rot;
  toRotVector(pcampar->cam.pose.rot, rot);
  Vector3 dr = rot - prevrot;
  prevrot = rot;
  double err = fabs(dr.x) + fabs(dr.y) + fabs(dr.z);
    
  if (err >= 0.01) {
    log("Camera is MOVING. Sampling at %.2gHz.", rate.getRate());
    //log("rot x: %.4g, y: %.4g, z: %.4g", rot.x, rot.y, rot.z);
    m_bCameraMoving = true;
    m_endMoveTimeout.restart();
    m_cameraParams.pose = pcampar->cam.pose; // only pose is valid
  }
  else {
    if (m_bCameraMoving && m_endMoveTimeout.elapsed() > 500) {
      log("Camera STOPPED.");
      m_endMoveTimeout.restart();
      m_bCameraMoving = false;
      m_cameraParams.pose = pcampar->cam.pose; // only pose is valid
      m_display.sendPtuStateToDialog();
    }
  }
}

#ifdef FEAT_TRACK_ARM
void SOIFilter::onChange_ArmPosition(const cdl::WorkingMemoryChange & _wmc)
{
  m_pArmStatus = getMemoryEntry<ArmIce::ArmStatus>(_wmc.address);
}
#endif

bool SOIFilter::isCameraStable(unsigned long milliSeconds)
{
  return ! m_bCameraMoving && m_endMoveTimeout.elapsed() >= milliSeconds; // TODO && ! m_RobotMoving
}

bool SOIFilter::isPointVisible(const cogx::Math::Vector3 &pos)
{
  return m_coarsePointCloud.isPointVisible(pos);

  // OLD
  //Video::CameraParameters params;
  //if (! videoServer->getCameraParameters(camId, params))
  //  return false;

#if 0 // DEBUGGING
  //Vector3 rota, rotb;
  //toRotVector(m_cameraParams.pose.rot, rota);
  //toRotVector(params.pose.rot, rotb);
  //Vector3 dr = rotb - rota;
  //double err = fabs(dr.x) + fabs(dr.y) + fabs(dr.z);
  //if (err >= 0.01) {
  //  log("isPointVisible: Camera has MOVED, %.4g", err);
  //  log("rot from mount: %.4g, %.4g, %.4g", rota.x, rota.y, rota.z);
  //  log("rot from video: %.4g, %.4g, %.4g", rotb.x, rotb.y, rotb.z);
  //}
  //ostringstream ss;
  //ss << m_cameraParams;
  //m_display.setHtml("soif.CamPars", "0", ss.str());
#endif // DEBUGGING

  //return Video::isPointVisible(params, pos);
}

// Check the objects that are curently marked invisible.
// If an invisible object should be visible (it is in the view volume), assume
// it was removed and delete the ProtoObject.
void SOIFilter::checkInvisibleObjects()
{
  if ( ! isCameraStable(2000)) sleepComponent(50);
  if ( ! isCameraStable(2000)) return;

  vector<ProtoObjectRecordPtr> toDelete;

  // Collect objects to delete. We delete them later when m_protoObjectMapMutex is
  // unlocked to avoid waiting for unlock in onDelete_ProtoObject.
  {
    IceUtil::RWRecMutex::RLock lock(m_protoObjectMapMutex);
    for(auto itpo = m_protoObjects.begin(); itpo != m_protoObjects.end(); ++itpo) {
      ProtoObjectRecordPtr& pporec = itpo->second;
      //ProtoObjectPtr& pobj = pporec->pobj;

      if (pporec->tmDisappeared.elapsed() < 1000)
        continue;

      bool bVisible = false;
      for (auto it = m_sois.begin(); it != m_sois.end(); ++it) {
        if (it->second->protoObjectAddr == pporec->addr) {
          bVisible = true;
          break;
        }
      }
      if (bVisible) continue;

      toDelete.push_back(pporec);
    }
  }

  for(auto itpo = toDelete.begin(); itpo != toDelete.end(); ++itpo) {
    ProtoObjectRecordPtr& pporec = *itpo;
#if 0
    Vector3& pos = pporec->pobj->position;
    log("TODO: Check Visibility of PO '%s' at (%.3g %.3g %3g) -> (%s)",
        pporec->addr.id.c_str(),
        pos.x, pos.y, pos.z,
        isPointVisible(pporec->pobj->position) ? "IN" : "OUT");
#endif
    if (isPointVisible(pporec->pobj->position)) {
      log("PO '%s' was removed.", pporec->addr.id.c_str());
      WmUnlocker locker(this);

      // Mark the appropriate VO as REMOVED.  XXX: the object could be deleted, if this is preferred.
      VisualObjectRecordPtr pvorec = findVisualObjectFor(pporec->addr);
      if (! pvorec)
        pvorec = findHiddenVisualObjectFor(pporec->addr);

      if (! pvorec) {
        error("A visible PO-rec has no associated VO-rec.");
      }
      else {
        log("VO '%s': presence = VopREMOVED", pvorec->addr.id.c_str());
        int lockRetries = 5;
        while (lockRetries > 0) {
          --lockRetries;
          try {
            locker.lock(pvorec->addr, cdl::LOCKEDODR);

            VisualObjectPtr pvobj = getMemoryEntry<VisionData::VisualObject>(pvorec->addr);
            pvobj->presence = VisionData::VopREMOVED;
            overwriteWorkingMemory(pvorec->addr, pvobj);
            break;
          }
          catch(WmUnlocker::LockError& e) {
            locker.unlockAll();
            sleepComponent(100);
            if (lockRetries > 0)
              continue;
            println("RemoveInvisible: Could not lock VO");
          }
          catch(cast::DoesNotExistOnWMException){
            println("RemoveInvisible: An internal VO record exists for a non-existing wm-VO.");
          }
          break;
        }
      }

      // the object should be visible, but there is no SOI -> assume removed from scene -> delete PO
      try {
        deleteFromWorkingMemory(pporec->addr);
      }
      catch(cast::DoesNotExistOnWMException){
        println("check_invisible: ProtoObject already deleted from WM.");
      }
    }
  }
}

void SOIFilter::queueCheckVisibilityOf_PO(const cdl::WorkingMemoryAddress& protoObjectAddr)
{
  ProtoObjectRecordPtr pporec;
  try {
    IceUtil::RWRecMutex::RLock lock(m_protoObjectMapMutex);
    pporec = m_protoObjects.get(protoObjectAddr);
    // for now just mark the time the object was queued
    pporec->tmDisappeared.restart();
  }
  catch(range_error& e) {}
}

// Queue an event to be processed again later.
bool SOIFilter::retryEvent(WmEvent* pEvent, long milliSeconds, long nRetries)
{
  IceUtil::RWRecMutex::WLock lock(m_retryQueueMutex);

  if (pEvent->pRetry)
    pEvent->pRetry->tmToWaitMs = milliSeconds;
  else
    pEvent->pRetry = new EventRetryInfo(milliSeconds, nRetries);

  pEvent->pRetry->tmWaiting.restart();
  bool valid = pEvent->pRetry->retriesLeft > 0; 

  for (unsigned int i = 0; i < m_EventRetryQueue.size(); i++) {
    if (m_EventRetryQueue[i] == pEvent)
      return valid;
  }
  m_EventRetryQueue.push_back(pEvent);

  return valid;
}

// Find events that can be retried.
// Move events from retry queue to evnet queue.
// XXX: this will introduce event order inconsistencies (eg. update after delete) !!!
void SOIFilter::checkRetryEvents()
{
  if (m_EventRetryQueue.size() < 1)
    return;

  IceUtil::RWRecMutex::RLock lock(m_retryQueueMutex);
  vector<unsigned int> ready_i;
  vector<WmEvent*> ready;
  {
    for (unsigned int i = 0; i < m_EventRetryQueue.size(); i++) {
      WmEvent* pEvent = m_EventRetryQueue[i];
      if (pEvent->pRetry->isReady()) {
        ready.push_back(pEvent);
        ready_i.push_back(i);
      }
    }
  }

  if (ready.size() < 1)
    return;

  // Try to obtain a write lock (WLock m_retryQueueMutex)
  if (!lock.timedUpgrade(IceUtil::Time::milliSeconds(200)))
    return;

  for (int i = ready_i.size() - 1; i >= 0; i--)
    m_EventRetryQueue.erase(m_EventRetryQueue.begin() + ready_i[i]);

  lock.downgrade();

  for (unsigned int i = 0; i < ready.size(); i++) {
    WmEvent* pEvent = ready[i];
    if (pEvent->pRetry->retriesLeft <= 0)
      delete pEvent;
    else {
      pEvent->pRetry->retriesLeft--;
      pEvent->pRetry->retryCount++;
      m_EventQueue.addItem(pEvent);
    }
  }
}

long SOIFilter::getMillisToRetryEvent(long defaultMs)
{
  if (m_EventRetryQueue.size() < 1)
    return defaultMs;

  long minms = defaultMs;
  IceUtil::RWRecMutex::RLock lock(m_retryQueueMutex);

  for (unsigned int i = 0; i < m_EventRetryQueue.size(); i++) {
    WmEvent* pEvent = m_EventRetryQueue[i];
    long mtr = pEvent->pRetry->millisToReady();
    if (mtr < minms)
      minms = mtr;
    if (minms <= 1)
      return 1;
  }
  return minms;
}

bool SOIFilter::isRetryEvent(WmEvent* pEvent)
{
  IceUtil::RWRecMutex::RLock lock(m_retryQueueMutex);

  for (unsigned int i = 0; i < m_EventRetryQueue.size(); i++) {
    if (pEvent == m_EventRetryQueue[i])
      return true;
  }
  return false;
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
  castutils::CMilliTimer tm(true);
  double eps = 1e-2;
  while (tm.elapsed() < 3000) {
    updateRobotPosePtz();
    if (fabs(pan-m_RobotPose.pan) < eps && fabs(tilt-m_RobotPose.tilt) < eps)
      break;
    sleepComponent(100);
    log("Waiting for PTU to reach destination. dpan=%g, dtilt=%g, tm=%ld",
        fabs(pan-m_RobotPose.pan), fabs(tilt-m_RobotPose.tilt), tm.elapsed());
  }
  return true;
}

void SOIFilter::runComponent()
{
  WmTaskExecutor_Soi soiProcessor(this);
  WmTaskExecutor_MoveToViewCone moveProcessor(this);
  WmTaskExecutor_Analyze analysisProcessor(this);

  castutils::CMilliTimer tmCheckVisibility(true);
  tmCheckVisibility.setTimeout(1000);

  castutils::CMilliTimer tmSendStatus(true);
  tmSendStatus.setTimeout(3100);

  castutils::CRunningRate realRate;

  while(isRunning())
  {
    std::deque<WmEvent*> tasks;
    tasks.clear();
    realRate.tick();

    checkRetryEvents();

    bool has_events = m_EventQueue.waitForItem(getMillisToRetryEvent(600));
    if (has_events) {
      tasks = std::move(m_EventQueue.getItems(1));
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
      if (! isRetryEvent(pevent))
        delete pevent;
    }

    if (tmCheckVisibility.isTimeoutReached())
    {
      // We check for invisible objects only when there are no add-soi events in queue.
      // This is to prevent POs being deleted to quickly after a camera move.
      if (isCameraStable(4000)) {
        int addSoiCmdCount = 0;
        m_EventQueue.for_each([&addSoiCmdCount](WmEvent* const& pev) {
          if (pev->objectType == TYPE_SOI && pev->change == cdl::ADD)
            addSoiCmdCount++;
        }); 
        if (addSoiCmdCount < 1) {
          checkInvisibleObjects();
        }
      }
      tmCheckVisibility.restart();
    }

    if (tmSendStatus.isTimeoutReached()) {
      ostringstream ss;
      ss.precision(4); // set the _maximum_ precision
      ss << "<h3>SOI Filter (" << getComponentID() << ") processing rate</h3>";
      ss << "current: " << realRate.getRate() << " checks/s<br>";
      ss << "from start: " << realRate.getTotalRate() << " checks/s<br>";
      m_display.setHtml("INFO", "soif.rate/" + getComponentID(), ss.str());
      tmSendStatus.restart();
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
  double dmin = 1e99;
  double d;
  ProtoObjectRecordPtr pBest;
  const Vector3& p0 = pos;
  log("Find PO near (%.4g, %.4g, %.4g)", p0.x, p0.y, p0.z);

  IceUtil::RWRecMutex::RLock lock(m_protoObjectMapMutex);
  for(auto itpo = m_protoObjects.begin(); itpo != m_protoObjects.end(); ++itpo) {
    ProtoObjectRecordPtr& pporec = itpo->second;
    ProtoObjectPtr& pobj = pporec->pobj;
    if (!pobj.get()) {
      error("PO record without a PO (%s).", itpo->first.id.c_str());
      continue;
    }

    // XXX: We assume that pobj->cameraLocation is at pSoiFilter->m_RobotPose
    //   => we only compare robo-centric positions
    Vector3& p1 = pobj->position;

    d = dist(p0, p1);
    log("PO %s, d=%.4g (%.4g, %.4g, %.4g)", itpo->first.id.c_str(), d, p1.x, p1.y, p1.z);
    if (d < dmin) {
      dmin = d;
      pBest = pporec;
    }
  }
  if (dmin < 0.12)
    return pBest;

  return nullptr;
}

ProtoObjectRecordPtr SOIFilter::findProtoObjectAt(const SOIPtr &psoi)
{
  if (!psoi.get()) 
    return nullptr;

  return findProtoObjectAt(psoi->boundingSphere.pos);
}

// Find the VO object that is currently linked with PO
VisualObjectRecordPtr SOIFilter::findVisualObjectFor(const cdl::WorkingMemoryAddress& protoAddr)
{
  if (protoAddr.id == "") 
    return nullptr;

  for(auto itvo = m_visualObjects.begin(); itvo != m_visualObjects.end(); ++itvo) {
    VisualObjectRecordPtr& pvorec = itvo->second;
    VisualObjectPtr& pobj = pvorec->pobj;
    if (!pobj.get())
      continue;

    if (pobj->protoObject->address == protoAddr)
      return itvo->second;
  }

  return nullptr;
}

// Find the VO that was last linked with the PO
VisualObjectRecordPtr SOIFilter::findHiddenVisualObjectFor(const cdl::WorkingMemoryAddress& protoAddr)
{
  if (protoAddr.id == "") 
    return nullptr;

  // The check the hidden objects
  for(auto itvo = m_visualObjects.begin(); itvo != m_visualObjects.end(); ++itvo) {
    VisualObjectRecordPtr& pvorec = itvo->second;
    VisualObjectPtr& pobj = pvorec->pobj;
    if (!pobj.get())
      continue;

    if (pobj->lastProtoObject->address == protoAddr)
      return itvo->second;
  }

  return nullptr;
}

} // namespace
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */

