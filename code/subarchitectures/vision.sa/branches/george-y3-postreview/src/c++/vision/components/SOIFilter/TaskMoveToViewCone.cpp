/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#include "TaskMoveToViewCone.h"
#include "SOIFilter.h"
#include <cmath>

#include "../../VisionUtils.h"

using namespace std;
using namespace VisionData;
using namespace cogx;

namespace cast {

void WmTaskExecutor_MoveToViewCone::handle_add_task(WmEvent *pEvent)
{
  switch (pEvent->objectType) {
    case TYPE_CMD_LOOK:
      moveToViewCone(pEvent);
      break;
    case TYPE_CMD_LOOK_AROUND:
      lookAround(pEvent);
      break;
  }
}

void WmTaskExecutor_MoveToViewCone::moveToViewCone(WmEvent *pEvent)
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

  println("MoveToViewConeCommand %s", pEvent->wmc.address.id.c_str());

  if (!cmd.read(pEvent->wmc.address)) {
    debug("move_to: MoveToViewConeCommand deleted while working.");
    return;
  }
  debug("move_to: GOT A MoveToViewConeCommand");

  if (! pSoiFilter->hasPtz())
    cmd.fail();
  else {
    // XXX: This command only moves the PTU, but should also move the robot
    try {
      ViewConePtr pBetterVc = pSoiFilter->getMemoryEntry<VisionData::ViewCone>(cmd.pcmd->target->address);
      double pan = pBetterVc->viewDirection - pBetterVc->anchor.z;
      double tilt = pBetterVc->tilt;
      pSoiFilter->movePtz(pan, tilt);
      cmd.succeed();
    }
    catch (cast::DoesNotExistOnWMException) {
      debug("move_to: ViewCone deleted while working.");
      cmd.fail();
    }
  }
}

void WmTaskExecutor_MoveToViewCone::lookAround(WmEvent *pEvent)
{
  class CCmd:
    public cogx::VisionCommandNotifier<LookAroundCommand, LookAroundCommandPtr>
  {
  public:
    CCmd(cast::WorkingMemoryReaderComponent* pReader)
      : cogx::VisionCommandNotifier<LookAroundCommand, LookAroundCommandPtr>(pReader) {}
  protected:
    virtual void doFail() { pcmd->status = VisionData::VCFAILED; }
    virtual void doSucceed() { pcmd->status = VisionData::VCSUCCEEDED; }
  } cmd(pSoiFilter);

  println("LookAroundCommand %s", pEvent->wmc.address.id.c_str());

  if (!cmd.read(pEvent->wmc.address)) {
    debug("move_to: LookAroundCommand deleted while working.");
    return;
  }
  debug("move_to: GOT A LookAroundCommand");

  long count = 3;
  while (count > 0) {
    double pan  = (rand() % 200 - 100) / 100.0 * (M_PI / 2);
    //double tilt = (rand() % 20 - 10) / 100.0 * (M_PI);
    if (fabs(pSoiFilter->m_RobotPose.pan - pan) * 180 / M_PI < 20)
      continue;

    bool retry = false;
    for (unsigned int i = 0; i < cmd.pcmd->viewCones.size(); i++) {
      ViewConePtr &pvc = cmd.pcmd->viewCones[i];
      double vcpan = pvc->viewDirection - pvc->anchor.z;
      if (fabs(vcpan - pan) * 180 / M_PI < 20) {
        retry = true;
        break;
      }
    }
    if (retry) continue;

    ViewConePtr pCurVc = createViewCone();
    pCurVc->anchor.x = pSoiFilter->m_RobotPose.x;
    pCurVc->anchor.y = pSoiFilter->m_RobotPose.y;
    pCurVc->anchor.z = pSoiFilter->m_RobotPose.theta;
    pCurVc->x = 0;
    pCurVc->y = 0;
    pCurVc->viewDirection = pSoiFilter->m_RobotPose.theta + pan;
    pCurVc->tilt = pSoiFilter->m_RobotPose.tilt;

    cmd.pcmd->viewCones.push_back(pCurVc);
    --count;
  }
  cmd.succeed();
}


} // namespace
// vim: set sw=2 ts=8 sts=4 et :vim
