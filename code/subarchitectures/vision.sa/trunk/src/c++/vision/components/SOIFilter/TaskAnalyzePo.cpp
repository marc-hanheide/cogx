/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#include "TaskAnalyzePo.h"
#include "SOIFilter.h"
#include "CmdGetSois.h"
#include "CmdLearnerRecognize.h"

#include "../../VisionUtils.h"

using namespace std;
using namespace VisionData;
using namespace cogx;

namespace cast {

void WmTaskExecutor_Analyze::handle_add_task(WmEvent* pEvent)
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

  pSoiFilter->println("AnalyzeProtoObjectCommand %s", pEvent->wmc.address.id.c_str());

  if (! cmd.read(pEvent->wmc.address)) {
    pSoiFilter->debug("analyze_task: AnalyzeProtoObjectCommand deleted while working.");
    return;
  }

  try { // DEBUGGING: NullHandleException
  // XXX: a NullHandleException is raised if wrong object type is selected in execution GUI
  // TODO: fix execution GUI to allow only sensible object/operation combiantions

  pSoiFilter->println("analyze_task: read po");
  // pobj - The proto object to segment
  ProtoObjectPtr pobj;
  try {
    pobj = pSoiFilter->getMemoryEntry<VisionData::ProtoObject>(cmd.pcmd->protoObjectAddr);
  }
  catch(cast::DoesNotExistOnWMException){
    pSoiFilter->debug("analyze_task: ProtoObject deleted while working. Aborting task.");
    cmd.fail();
    return;
  }

  /* Asynchronous call through a working memory entry.
   *
   * A WorkingMemoryChangeReceiver is created on the heap. We wait for it to
   * complete then we remove it from the change filter list, but we DON'T
   * DELETE it. It will be moved to a deletion queue by the framework and
   * deleted later. */
  pSoiFilter->debug("analyze_task: calling GetStableSoisCommand");
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
    pSoiFilter->debug("analyze_task: No Fine SOIs. Aborting task.");
    cmd.fail();
    return;
  }

  pSoiFilter->debug("analyze_task: Got some Fine SOIs.");

  // Find the SOI that is closesst to the PO
  // psoi - The fine SOI that represents the proto object
  SOIPtr psoi = sois[0];
  double d, dmin;
  dmin = dist(pobj->position, psoi->boundingSphere.pos);
  for (unsigned int i = 1; i < sois.size(); ++i) {
    d = dist(pobj->position, sois[i]->boundingSphere.pos);
    if (d < dmin) {
      dmin = d;
      psoi = sois[i];
    }
  }
  if (dmin >= 0.2) {
    pSoiFilter->println("analyze_task: SOI to be analyzed is far from PO (%.2lfm)", dmin);
  }

  bool bSegmented = pSoiFilter->m_segmenter.segmentObject(psoi, pobj->image, pobj->mask, pobj->points, pobj);
  if (! bSegmented) {
    pSoiFilter->debug("analyze_task: Segmentation failed.");
  }

  // find VO for this PO
  cdl::WorkingMemoryAddress voAddr = pSoiFilter->findVisualObjectFor(cmd.pcmd->protoObjectAddr);
  VisualObjectPtr pvo;
  bool bNewVo = false;
  if (voAddr.id != "") {
    try {
      pvo = pSoiFilter->getMemoryEntry<VisionData::VisualObject>(voAddr);
    }
    catch(cast::DoesNotExistOnWMException){
      pSoiFilter->debug("analyze_task: VisualObject deleted while working.");
    }
  }
  if (! pvo.get()) {
    pSoiFilter->debug("analyze_task: creating new VisualObject.");
    pvo = createVisualObject();
    pvo->protoObject = createWmPointer<ProtoObject>(cmd.pcmd->protoObjectAddr);
    pvo->lastProtoObject = createWmPointer<ProtoObject>(cmd.pcmd->protoObjectAddr);

    voAddr = cast::makeWorkingMemoryAddress(pSoiFilter->newDataID(), pSoiFilter->getSubarchitectureID());
    bNewVo = true;
  }

  // XXX: Problems may arise because VO points to PO and PO points to VO !!!
  // We write first the PO because usually PO is used throug the reference in VO.

  pobj->visualObject.push_back(createWmPointer<VisualObject>(voAddr));
  try {
    pSoiFilter->overwriteWorkingMemory(cmd.pcmd->protoObjectAddr, pobj);	
  }
  catch(cast::DoesNotExistOnWMException){
    pSoiFilter->debug("analyze_task (WRITE): ProtoObject deleted while working.");
  }

  // VO has to be written after PO (components usually access PO through VO)
  if (pvo.get() && voAddr.id != "") {
    try {
      if (bNewVo) pSoiFilter->addToWorkingMemory(voAddr, pvo);
      else pSoiFilter->overwriteWorkingMemory(voAddr, pvo);	
    }
    catch(cast::DoesNotExistOnWMException){
      pSoiFilter->debug("analyze_task (WRITE): VisualObject deleted while working.");
    }
  }

  // Start other recognition tasks
  LearnerRecognitionTaskRcv* pTask = new LearnerRecognitionTaskRcv(pSoiFilter, cmd.pcmd->protoObjectAddr, voAddr);
  pTask->deleteOnCompletion();

  cmd.succeed();
  } catch ( IceUtil::NullHandleException& e ) { // DEBUGGING
    pSoiFilter->println("CAUGHT");
  }
}

} // namespace
// vim: set sw=2 ts=8 sts=4 et :vim
