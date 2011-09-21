/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#include "TaskReceiveSoi.h"
#include "SOIFilter.h"

#include "../../VisionUtils.h"
#include <Video.hpp>
#include <Math.hpp>
#include <castutils/Timers.hpp>

#include <string>

using namespace std;
using namespace VisionData;
using namespace cogx;

namespace cast {

// Removes the successfully locked locks on destruction (eg. exiting scope)
struct WmUnlocker
{
  class LockError: public std::exception
  {
    std::string reason;
  public:
    LockError(const std::string& error = "")
    {
      reason = error;
    }
    ~LockError() throw()
    {
    }
    const char* what() const throw()
    {
      return reason.c_str();
    }
  };

  std::vector<cdl::WorkingMemoryAddress> locks;
  cast::WorkingMemoryAttachedComponent *pc;
  WmUnlocker(cast::WorkingMemoryAttachedComponent* pComponent) {
    pc = pComponent;
  }
  void lock(cdl::WorkingMemoryAddress& addr, cdl::WorkingMemoryPermissions perm) {
    if (!pc->tryLockEntry(addr, perm))
      throw LockError("Trying to lock object: " + addr.id);
    locks.push_back(addr);
  }
  void unlockAll()
  {
    for (int i = 0; i < (int) locks.size(); ++i) {
      try {
        pc->unlockEntry(locks[i]);
      }
      catch(cast::DoesNotExistOnWMException){ }
    }
  }
  ~WmUnlocker() {
    unlockAll();
  }
};

/*
 * TODO: 2 step VisualObject generation.
 *   1. If the SOI comes from coarseSource, create a proto-object with the
 *      desired ViewCones
 *   2. If the SOI comes from fineSource, we need to know which PO it belongs
 *      to (SOI matching by position), then we update the PO and create the VO.
 * 
 * When there is a single source, all SOIs come from this source: first through
 * WM filters, then through the GetStableSoisCommand.
 */
void WmTaskExecutor_Soi::handle_add_soi(WmEvent* pEvent)
{
  debug("SOIFilter::handle_add_soi");
  SOIPtr psoi;
  try {
    psoi = pSoiFilter->getMemoryEntry<VisionData::SOI>(pEvent->wmc.address);
  }
  catch(cast::DoesNotExistOnWMException){
    log("SOIFilter.add_soi: SOI deleted while working...");
    return;
  }

  println("Handle SOI %s", pEvent->wmc.address.id.c_str());

  pSoiFilter->updateRobotPosePtz();

  SoiRecordPtr psoirec = new SoiRecord();
  psoirec->addr = pEvent->wmc.address;
  psoirec->psoi = new SOI();
  pSoiFilter->saveSoiData(psoi, psoirec->psoi);
  pSoiFilter->m_sois[psoirec->addr] = psoirec;

  if (psoi->sourceId == pSoiFilter->m_coarseSource || psoi->sourceId == SOURCE_FAKE_SOI)
  {
    // Verify all known proto objects if they are at the same location as the SOI;
    // In this case, we are looking at a known PO and don't have to do
    // anything, except if we want to analyze the object in the center.

    ProtoObjectPtr pobj;
    log("FIND SOI '%s'", psoirec->addr.id.c_str());
    ProtoObjectRecordPtr pporec = pSoiFilter->findProtoObjectAt(psoi);
    if (pporec.get())
      pobj = pporec->pobj;
    if (pobj.get()) {
      log("SOI '%s' belongs to a known ProtoObject", psoirec->addr.id.c_str());
      // XXX: Do we update the PO with the new SOI?
      // XXX: The object recognizer should verify if this is the same object
      psoirec->protoObjectAddr = pporec->addr;
      MakeVisible(pporec->addr);
      return;
    }

    // XXX: CONFIG min/max distance could be an option
    double dsoi = length(psoi->boundingSphere.pos);
    if (dsoi > 3.0) {
      log("SOI '%s' is too far (%.3gm)", psoirec->addr.id.c_str(), dsoi);
      return;
    }

    if (psoi->boundingSphere.pos.x < 0.6 && psoi->boundingSphere.pos.z < 0.5) {
      log("SOI '%s' could be the Katana arm.", psoirec->addr.id.c_str());
      // XXX: may be looking at the katana arm, ignore
      return;
    }

    pobj = createProtoObject();
    pSoiFilter->m_snapper.m_LastProtoObject = pobj;
    string objId = pSoiFilter->newDataID();

    // link soi to po
    psoirec->protoObjectAddr.id = objId;
    psoirec->protoObjectAddr.subarchitecture = pSoiFilter->getSubarchitectureID();

    pobj->position = psoi->boundingSphere.pos;

    // Calculate the current and the desired View Cone
    //    We don't (shouldn't?) know an absolute position of the robot so we
    //    can only use relative coordinates. We should keep track of the robot
    //    movements from the time we calculate the VCs to the time the desired
    //    VC is reached. We could use anchors for that, but are they currently
    //    implemented?
    //    Current VC is at anchor. Desired VC(s) are relative to the anchor.
    ViewConePtr pCurVc = createViewCone();
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
    Video::CameraParameters camPars;
    if (! pSoiFilter->m_coarsePointCloud.getCameraParameters(Math::LEFT, camPars)) {
      println("FAILED to get the camera parameters from '%s'",
          pSoiFilter->m_coarsePcServer.c_str());
    }
    else {
      pobj->image.camPars = camPars; // XXX: Where do we need camPars again? In segmentation?

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
      log("Angle to SOI: pan %g, tilt %g", dirDelta, tiltDelta);

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
    ViewConePtr pBetterVc = createViewCone();
    pBetterVc->anchor = pCurVc->anchor;
    pBetterVc->x = pCurVc->x;
    pBetterVc->y = pCurVc->y;
    pBetterVc->viewDirection = pCurVc->viewDirection + dirDelta;
    pBetterVc->tilt = pCurVc->tilt + tiltDelta;
    pBetterVc->target = createWmPointer<ProtoObject>(cast::makeWorkingMemoryAddress(objId,
        pSoiFilter->getSubarchitectureID()));

    // Address at which new view cone will be stored
    cdl::WorkingMemoryAddress vcAddr = cast::makeWorkingMemoryAddress(pSoiFilter->newDataID(),
        pSoiFilter->getSubarchitectureID());
    // Write viewcone to memory
    pSoiFilter->addToWorkingMemory(vcAddr, pBetterVc);

    // Create pointer to viewcone on WM
    cdl::WorkingMemoryPointerPtr vcPtr = new cdl::WorkingMemoryPointer();
    vcPtr->address = vcAddr;
    vcPtr->type = cast::typeName<ViewCone>();
    pobj->desiredLocations.push_back(vcPtr);

    // Add PO to WM
    debug("Adding new ProtoObject '%s'", objId.c_str());
    pSoiFilter->addToWorkingMemory(objId, pobj);

    // Now it is up to the planner to create a plan to move the robot
    // The task contains: PO, target VC

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
    error("*** Received a SOI from a source of fine SOIs. ***");
    // This should never happen! Fine SOIs are retrieved with a WM command
    // in TaskAnalyzePo:
    //
    // If the SOI is in center of scene, find the PO we want to analyze and 
    // 'promote' the PO. Then somebody will have to generate a VO.
    //   - check if we have a pending recognition task
    //   - check if the robot is in the desired position
    //   - assume the PO is in the center of the scene <-- the desired position is reached
  }
}

//void WmTaskExecutor_Soi::handle_update_soi(WmEvent* pEvent)
//{
//  // TODO: update SOI position
//}

void WmTaskExecutor_Soi::handle_delete_soi(WmEvent* pEvent)
{
  const cdl::WorkingMemoryChange &wmc = pEvent->wmc;
  SoiRecordPtr psoirec;

  println("Handle SOI %s DELETE", wmc.address.id.c_str());

  try {
    psoirec = pSoiFilter->m_sois.get(wmc.address);
  }
  catch(range_error &e) {
    // Unknown SOI, nothing to do
    error("Removing a SOI that was not registered.");
    return;
  }

  pSoiFilter->m_sois.erase(wmc.address);

  pSoiFilter->updateRobotPosePtz();

  if (! psoirec->psoi.get()) {
    error("We have a SOI record without a SOI.");
  }
  else {
    if (psoirec->protoObjectAddr.id.length() < 1) {
      log("SOI is not associated with a PO");
    }
    else {
      log("SOI belongs to PO: %s", psoirec->protoObjectAddr.id.c_str());

      // check if other SOIs are associated with this PO
      bool hasMoreSois = false;
      typeof(pSoiFilter->m_sois.begin()) it;
      for (it = pSoiFilter->m_sois.begin(); it != pSoiFilter->m_sois.end(); ++it) {
        if (it->second->protoObjectAddr == psoirec->protoObjectAddr) {
          hasMoreSois = true;
          break;
        }
      }

      if (! hasMoreSois) {
        log("PO has no more SOIs -> invisible.");

        // The PO is not visible any more. The robot may have turned the head
        // or the object was removed. First of all we make the object
        // invisible. Then we have to check if the object was removed.
        //
        // The condition for removed objects: we know that we could see the
        // object from the current view cone, but it isn't there.
        castutils::CMilliTimer tmwait;
        while( ! pSoiFilter->isCameraStable()) {
          pSoiFilter->sleepComponent(100);
          if (tmwait.elapsed() > 5000) {
            println("delete_soi: waiting for camera to stop moving");
            tmwait.restart();
          }
        }

        MakeInvisible(psoirec->protoObjectAddr);

        ProtoObjectRecordPtr pporec;
        try {
          // Remember the last position of the PO
          pporec = pSoiFilter->m_protoObjects.get(psoirec->protoObjectAddr);
          pporec->pobj->position = psoirec->psoi->boundingSphere.pos;

          pSoiFilter->queueCheckVisibilityOf_PO(psoirec->protoObjectAddr);
        }
        catch(range_error& e) {}
      }
      else {
        log("PO %s has more SOIs", psoirec->protoObjectAddr.id.c_str());
      }
    }
  }
}

void WmTaskExecutor_Soi::MakeInvisible(cdl::WorkingMemoryAddress &protoObjectAddr)
{
  println("Make Invisible PO: %s", protoObjectAddr.id.c_str());

  ProtoObjectRecordPtr pporec;
  try {
    pporec = pSoiFilter->m_protoObjects.get(protoObjectAddr);
  }
  catch(range_error& e) {
    error("A deleted SOI belonged to an already deleted PO");
    return;
  }

  VisualObjectRecordPtr pvorec = pSoiFilter->findVisualObjectFor(pporec->addr);
  if (! pvorec.get()) {
    // Looks like no VO is associated with this PO => Nothing to do
    log("Looks like the PO is already marked invisible.");
    return;
  }

  // We have a link between VO and PO => load both objects and remove the link
  WmUnlocker locker(pSoiFilter);

  long lockRetries = 5;
  while (lockRetries > 0) {
    --lockRetries;
    try {
      string ot;
      try {
        ot = "PO";
        locker.lock(pporec->addr, cdl::LOCKEDODR);
        ot = "VO";
        locker.lock(pvorec->addr, cdl::LOCKEDODR);
      }
      catch(WmUnlocker::LockError& e) {
        // We may have a racing condition. Unlock all and retry.
        locker.unlockAll();
        usleep(100 * 1000);
        if (lockRetries <= 0)
          throw WmUnlocker::LockError("Could not lock PO+VO");
        continue;
      }
      catch(cast::DoesNotExistOnWMException){
        error("An internal " + ot + "record exists for a non-existing wm-" + ot + ".");
      }

      ProtoObjectPtr ppo;
      // NOTE: ppo->visualObject has info about the VO to link to when the PO re-appears
      // => so DON'T DELETE
      // ppo = pSoiFilter->getMemoryEntry<VisionData::ProtoObject>(pporec->addr);
      // ppo->visualObject.clear(); // why-oh-why is this a vector ...
      // pSoiFilter->overwriteWorkingMemory<VisionData::ProtoObject>(pporec->addr, ppo);
      // pSoiFilter->saveProtoObjectData(ppo, pporec->pobj);

      VisualObjectPtr pvo;
      pvo = pSoiFilter->getMemoryEntry<VisionData::VisualObject>(pvorec->addr);
      pvo->lastProtoObject = pvo->protoObject;
      pvo->protoObject = nullWmPointer();
      pSoiFilter->overwriteWorkingMemory<VisionData::VisualObject>(pvorec->addr, pvo);
      pSoiFilter->saveVisualObjectData(pvo, pvorec->pobj);
    }
    catch(std::exception& e) {
      error(
          "Something went wrong when updating the link between VO and PO."
          " The error is: %s", e.what());
    }
    catch(...) {
      error("Something went wrong when updating the link between VO and PO");
    }
    lockRetries = 0;
    break;
  }
}

void WmTaskExecutor_Soi::MakeVisible(cdl::WorkingMemoryAddress &protoObjectAddr)
{
  println("Make Visible PO: %s", protoObjectAddr.id.c_str());

  VisualObjectRecordPtr pvorec = pSoiFilter->findVisualObjectFor(protoObjectAddr);
  if (pvorec.get()) {
    log("PO is already visible.");
    return;
  }

  pvorec = pSoiFilter->findHiddenVisualObjectFor(protoObjectAddr);
  if ( ! pvorec.get()) {
    log("PO was not linked to a VO lately. **** Do we have to analyze it again? ****");
    // TODO: this should probably trigger a new object analysis
    return;
  }

  WmUnlocker locker(pSoiFilter);

  println(" **** re-creating the link to PO in the VO");
  long lockRetries = 5;
  while (lockRetries > 0) {
    --lockRetries;
    try {
      string ot;
      try {
        ot = "VO";
        locker.lock(pvorec->addr, cdl::LOCKEDODR);
      }
      catch(WmUnlocker::LockError& e) {
        // We may have a racing condition. Unlock all and retry.
        locker.unlockAll();
        usleep(100 * 1000);
        if (lockRetries <= 0)
          throw WmUnlocker::LockError("Could not lock VO");
        continue;
      }
      catch(cast::DoesNotExistOnWMException){
        error("An internal " + ot + "record exists for a non-existing wm-" + ot + ".");
      }

      // TODO: Update also the PO.VisualObjectPtr if it is different than in the past
      // (can this actually happen?)

      VisualObjectPtr pvo;
      pvo = pSoiFilter->getMemoryEntry<VisionData::VisualObject>(pvorec->addr);
      pvo->lastProtoObject = pvo->protoObject;
      pvo->protoObject = createWmPointer<VisionData::ProtoObject>(protoObjectAddr);
      pSoiFilter->overwriteWorkingMemory<VisionData::VisualObject>(pvorec->addr, pvo);
      pSoiFilter->saveVisualObjectData(pvo, pvorec->pobj);
    }
    catch(std::exception& e) {
      error(
          "Something went wrong when updating the link between VO and PO."
          " The error is: %s", e.what());
    }
    catch(...) {
      error("Something went wrong when updating the link between VO and PO");
    }
    lockRetries = 0;
    break;
  }
}

} // namespace
// vim: set sw=2 ts=8 sts=4 et :vim
