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

#define USE_RECOGNIZER_V2 0
#if USE_RECOGNIZER_V2
#include <ObjectRecognizerSrv.hpp>
#endif
#include <castutils/Timers.hpp>

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

  println("AnalyzeProtoObjectCommand %s", pEvent->wmc.address.id.c_str());

  if (! cmd.read(pEvent->wmc.address)) {
    debug("analyze_task: AnalyzeProtoObjectCommand deleted while working.");
    return;
  }

#define CATCH_NULL 0
#if CATCH_NULL
  try { // DEBUGGING: NullHandleException
  // XXX: a NullHandleException is raised if wrong object type is selected in execution GUI
#endif

  println("analyze_task: read po");
  // pobj - The proto object to segment
  ProtoObjectPtr pobj;
  try {
    pobj = pSoiFilter->getMemoryEntry<VisionData::ProtoObject>(cmd.pcmd->protoObjectAddr);
  }
  catch(cast::DoesNotExistOnWMException){
    debug("analyze_task: ProtoObject deleted while working. Aborting task.");
    cmd.fail();
    return;
  }

  castutils::CMilliTimer tmwait;
  while( ! pSoiFilter->isCameraStable(1000)) {
    pSoiFilter->sleepComponent(100);
    if (tmwait.elapsed() > 5000) {
      println("analyze_task: waiting for camera to stop moving");
      tmwait.restart();
    }
  }

  /* Asynchronous call through a working memory entry.
   *
   * A WorkingMemoryChangeReceiver is created on the heap. We wait for it to
   * complete then we remove it from the change filter list, but we DON'T
   * DELETE it. It will be moved to a deletion queue by the framework and
   * deleted later. */
  debug("analyze_task: calling GetStableSoisCommand");
#if 0
  GetSoisCommandRcv* pGetSois;
  vector<SOIPtr> sois;
  pGetSois = new GetSoisCommandRcv(pSoiFilter, pSoiFilter->m_fineSource);
  bool bCompleted = pGetSois->waitForCompletion(20e3/*ms*/);
  bCompleted = bCompleted && pGetSois->m_pcmd->status == VisionData::VCSUCCEEDED;
  if (bCompleted)
    pGetSois->getSois(sois);
  pSoiFilter->removeChangeFilter(pGetSois, cdl::DELETERECEIVER);
#else
  // review 2011
  // because of problems with PPO/SOIFilter interaction we don't call GetStableSoisCommand but
  // use the sois we already have (assumpiton: 1 PC server is running)
  bool bCompleted = true;
  vector<SOIPtr> sois;
  for (auto it = pSoiFilter->m_sois.begin(); it != pSoiFilter->m_sois.end(); it++) {
    sois.push_back(it->second->psoi);
  }
#endif

  if (! bCompleted || sois.size() < 1)
  {
    debug("analyze_task: No Fine SOIs. Aborting task.");
    cmd.fail();
    return;
  }

  debug("analyze_task: Got some Fine SOIs.");

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
    println("analyze_task: SOI to be analyzed is far from PO (%.2lfm)", dmin);
  }

  bool bSegmented = pSoiFilter->m_segmenter.segmentObject(psoi, pobj->image, pobj->mask, pobj->points, pobj);
  if (! bSegmented) {
    debug("analyze_task: Segmentation failed.");
  }

  // find VO for this PO
  VisualObjectRecordPtr pvorec;
  VisualObjectPtr pvo;
  cast::cdl::WorkingMemoryAddress voAddr;
  bool bNewVo = false;

  pvorec = pSoiFilter->findVisualObjectFor(cmd.pcmd->protoObjectAddr);
  if (! pvorec.get())
    pvorec = pSoiFilter->findHiddenVisualObjectFor(cmd.pcmd->protoObjectAddr);

  if (pvorec.get())
    voAddr = pvorec->addr;

  if (voAddr.id != "") {
    try {
      pvo = pSoiFilter->getMemoryEntry<VisionData::VisualObject>(voAddr);
    }
    catch(cast::DoesNotExistOnWMException){
      debug("analyze_task: VisualObject deleted while working.");
    }
  }
  if (! pvo.get()) {
    debug("analyze_task: creating new VisualObject.");
    pvo = createVisualObject();
    cast::cdl::CASTTime tm = pSoiFilter->getCASTTime();
    pvo->salience = tm.s + 1e-6 * tm.us;

    pvo->protoObject = createWmPointer<ProtoObject>(cmd.pcmd->protoObjectAddr);
    pvo->lastProtoObject = pvo->protoObject;
    pvo->pose.pos = pobj->position;

    voAddr = cast::makeWorkingMemoryAddress(pSoiFilter->newDataID(), pSoiFilter->getSubarchitectureID());
    bNewVo = true;
  }

  // XXX: Problems may arise because VO points to PO and PO points to VO !!!
  // We write first the PO because usually PO is used through the reference in VO.

  pobj->visualObject.clear(); // at most one object is associated with PO
  pobj->visualObject.push_back(createWmPointer<VisualObject>(voAddr));
  try {
    pSoiFilter->overwriteWorkingMemory(cmd.pcmd->protoObjectAddr, pobj);	
  }
  catch(cast::DoesNotExistOnWMException){
    debug("analyze_task (WRITE): ProtoObject deleted while working.");
  }

  // VO has to be written after PO (components usually access PO through VO)
  if (pvo.get() && voAddr.id != "") {
    try {
      pvo->presence = VisionData::VopVISIBLE;
      if (bNewVo) pSoiFilter->addToWorkingMemory(voAddr, pvo);
      else pSoiFilter->overwriteWorkingMemory(voAddr, pvo);	
    }
    catch(cast::DoesNotExistOnWMException){
      debug("analyze_task (WRITE): VisualObject deleted while working.");
    }
  }

  // Start other recognition tasks
  // Visual Learner - overwrites on completion
  try {
    LearnerRecognitionTaskRcv* pTask = new LearnerRecognitionTaskRcv(pSoiFilter, cmd.pcmd->protoObjectAddr, voAddr);
    pTask->deleteOnCompletion();
  }
  catch(...) {
      log("analyze_task: caught an unknown exception creating LearnerRecognitionTaskRcv.");
  }

  // Identity recognition commands don't return any status; they just change the VO
  if (pSoiFilter->m_identityRecognizerVersion == 1) {
    try {
      // TODO: add recognition command to the recognizer
      VisionData::DetectionCommandPtr pcmd = new VisionData::DetectionCommand();
      //pcmd->visualObject = createWmPointer<VisionData::VisualObject>(voAddr);
      if (bNewVo || pvo->identLabels.size() == 0 || pvo->identLabels[0] == VisionData::IDENTITYxUNKNOWN) {
        // try with all known objects
        // NOTE: the assumption is that the recognizer will try to recognize all
        // known models if there is no label in identLabels
      }
      else {
        for (int i = 0; i < pvo->identLabels.size(); i++) {
          if (pvo->identLabels[i] == VisionData::IDENTITYxUNKNOWN) continue;
          pcmd->labels.push_back(pvo->identLabels[i]);
        }
      }
      pSoiFilter->addToWorkingMemory(pSoiFilter->newDataID(), pcmd);
    }
    catch(...) {
      log("analyze_task: caught an unknown exception writing (identity-) DetectionCommand.");
    }
  }
#if USE_RECOGNIZER_V2
  else if (pSoiFilter->m_identityRecognizerVersion == 2) {
    try {
      ObjectRecognizerIce::ObjectRecognitionTaskPtr pcmd = new ObjectRecognizerIce::ObjectRecognitionTask();
      pcmd->visualObjectAddr = voAddr;
      pcmd->overwriteVisualObject = true;
      pSoiFilter->addToWorkingMemory(pSoiFilter->newDataID(), pcmd);
    }
    catch(...) {
      log("analyze_task: caught an unknown exception writing (identity-) RecognitionTask.");
    }
  }
#endif
  else if (pSoiFilter->m_identityRecognizerVersion == 3) {
    try {
      VisionData::RecognitionCommandPtr pcmd = new VisionData::RecognitionCommand();
      pcmd->visualObject = createWmPointer<VisionData::VisualObject>(voAddr);
      if (bNewVo || pvo->identLabels.size() == 0 || pvo->identLabels[0] == VisionData::IDENTITYxUNKNOWN) {
        // try with all known objects
        // NOTE: the assumption is that the recognizer will try to recognize all
        // known models if there is no label in identLabels
      }
      else {
        for (int i = 0; i < pvo->identLabels.size(); i++) {
          if (pvo->identLabels[i] == VisionData::IDENTITYxUNKNOWN) continue;
          pcmd->labels.push_back(pvo->identLabels[i]);
        }
      }
      pSoiFilter->addToWorkingMemory(pSoiFilter->newDataID(), pcmd);
    }
    catch(...) {
      log("analyze_task: caught an unknown exception writing (identity-) RecognitionCommand.");
    }
  }
  else {
    error("analyze_task: Unknown version of 3D recognizer: %d", pSoiFilter->m_identityRecognizerVersion);
  }

  cmd.succeed();

#if CATCH_NULL
  } catch ( IceUtil::NullHandleException& e ) { // DEBUGGING
    println("CAUGHT NullHandleException");
  }
#endif
#undef CATCH_NULL
}

} // namespace
// vim: set sw=2 ts=8 sts=4 et :vim
