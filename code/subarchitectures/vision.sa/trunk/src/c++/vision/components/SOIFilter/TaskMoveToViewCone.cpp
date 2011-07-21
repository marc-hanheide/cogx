/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#include "TaskMoveToViewCone.h"
#include "SOIFilter.h"

#include "../../VisionUtils.h"

using namespace std;
using namespace VisionData;
using namespace cogx;

namespace cast {

void WmTaskExecutor_MoveToViewCone::handle_add_task(WmEvent *pEvent)
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

  pSoiFilter->println("MoveToViewConeCommand %s", pEvent->wmc.address.id.c_str());

  if (!cmd.read(pEvent->wmc.address)) {
    pSoiFilter->debug("move_to: MoveToViewConeCommand deleted while working.");
    return;
  }
  pSoiFilter->debug("move_to: GOT A MoveToViewConeCommand");

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
      pSoiFilter->debug("move_to: ViewCone deleted while working.");
      cmd.fail();
    }
  }
}


} // namespace
// vim: set sw=2 ts=8 sts=4 et :vim
