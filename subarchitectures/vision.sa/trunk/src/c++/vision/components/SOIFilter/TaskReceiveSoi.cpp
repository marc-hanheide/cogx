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

#include <string>

using namespace std;
using namespace VisionData;
using namespace cogx;

namespace cast {
/*
 * TODO: 2 step VisualObject generation.
 *   1. If the SOI comes from coarseSource, create a proto-object with the
 *      desired ViewCones
 *   2. If the SOI comes from fineSource, we need to know which PO it belongs
 *      to (SOI matching by position), then we update the PO and create the VO.
 * TODO: Decide how to process stuff when there is only a single source ...
 */
void WmTaskExecutor_Soi::handle_add_soi(WmEvent* pEvent)
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

  pSoiFilter->updateRobotPosePtz();

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
    Video::CameraParameters camPars;
    if (! pSoiFilter->m_coarsePointCloud.getCameraParameters(Math::LEFT, camPars)) {
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
    cdl::WorkingMemoryAddress vcAddr = cast::makeWorkingMemoryAddress(pSoiFilter->newDataID(), pSoiFilter->getSubarchitectureID());
    // Write viewcone to memory
    pSoiFilter->addToWorkingMemory(vcAddr, pBetterVc);

    // Create pointer to viewcone on WM
    cdl::WorkingMemoryPointerPtr vcPtr = new cdl::WorkingMemoryPointer();
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

void WmTaskExecutor_Soi::handle_delete_soi(WmEvent* pEvent)
{
  const cdl::WorkingMemoryChange &wmc = pEvent->wmc;
  SOIData &soi = pSoiFilter->SOIMap[wmc.address.id];
  soi.status = DELETED;

  if (soi.objId.size() > 0)
  {
    // TODO: We have to load the PO and save it again. A lot of transfer !!!
  }
}

} // namespace
// vim: set sw=2 ts=8 sts=4 et :vim
