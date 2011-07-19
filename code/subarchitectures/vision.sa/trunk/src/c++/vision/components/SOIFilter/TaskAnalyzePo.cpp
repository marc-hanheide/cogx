/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#include "TaskAnalyzePo.h"
#include "SOIFilter.h"
#include "CmdGetSois.h"

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
  cdl::WorkingMemoryAddress voAddr = pSoiFilter->findVisualObjectFor(cmd.pcmd->protoObjectAddr);
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

} // namespace
// vim: set sw=2 ts=8 sts=4 et :vim
