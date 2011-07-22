/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#include "CmdLearnerRecognize.h"
#include "SOIFilter.h"

#include <cast/architecture/ChangeFilterFactory.hpp>

using namespace std;
using namespace VisionData;
using namespace cogx;

namespace cast
{

LearnerRecognitionTaskRcv::LearnerRecognitionTaskRcv(SOIFilter* psoif,
  const cast::cdl::WorkingMemoryAddress& protoObjectAddr,
  const cast::cdl::WorkingMemoryAddress& visualObjectAddr)
{
  m_complete = false;
  m_bDelete = false;
  pSoiFilter = psoif;
  m_pcmd = new VisualLearnerRecognitionTask();
  m_pcmd->status = VisionData::VCREQUESTED;
  m_pcmd->protoObjectAddr = createWmPointer<ProtoObject>(protoObjectAddr);
  this->visualObjectAddr = visualObjectAddr;
  string id = pSoiFilter->newDataID();
  pSoiFilter->addChangeFilter(createIDFilter(id, cdl::OVERWRITE), this);  
  pSoiFilter->addToWorkingMemory(id, m_pcmd);  
}

void LearnerRecognitionTaskRcv::workingMemoryChanged(const cast::cdl::WorkingMemoryChange &_wmc)
{
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_CompletionMonitor);
  try {
    VisualLearnerRecognitionTaskPtr pcmd = pSoiFilter->getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
    m_pcmd = pcmd;
    processResults();
  }
  catch (...) {
    pSoiFilter->debug("LearnerRecognitionTaskRcv: Failed to get the results.");
    m_pcmd->status = VisionData::VCFAILED; /* complete, but failed */
  }

  m_complete = true;
  m_CompletionMonitor.notify(); // works only if m_CompletionMonitor is locked here
  if (m_bDelete) {
    // This could delete the instance before waitForCompletion wakes up.
    // For this reason there is an assertion in waitForCompletion.
    pSoiFilter->removeChangeFilter(this, cdl::DELETERECEIVER);
  }
}

bool LearnerRecognitionTaskRcv::waitForCompletion(double milliSeconds)
{
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_CompletionMonitor);
  assert(! m_bDelete);

  if (m_complete) return true;
  m_CompletionMonitor.timedWait(IceUtil::Time::milliSeconds(milliSeconds));
  return m_complete;
}

void LearnerRecognitionTaskRcv::deleteOnCompletion()
{
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_CompletionMonitor);
  if (m_complete) {
    pSoiFilter->removeChangeFilter(this, cdl::DELETERECEIVER);
  }
  m_bDelete = true;
}

// This code was moved here form ObjectAnalyzer
void LearnerRecognitionTaskRcv::processResults()
{
  const string& id = visualObjectAddr.id;
  pSoiFilter->log("LearnerRecognitionTaskRcv: process results for VisualObject '%s'", id.c_str() );

  VisualObjectPtr pvobj;
  if(pSoiFilter->existsOnWorkingMemory(visualObjectAddr)) {
    try {
      pvobj = pSoiFilter->getMemoryEntry<VisualObject>(visualObjectAddr);
    }
    catch(DoesNotExistOnWMException e) {
      pSoiFilter->log("Cannot get VisualObject '%s' from WM. It will not be updated.", id.c_str() );
      return;
    }
  }
  else {
    pSoiFilter->log("(READ) Visual Object '%s' deleted. It will not be updated.", id.c_str());
    return;
  }

  VisualLearnerRecognitionTaskPtr ptask = m_pcmd;

  // NOTE: we are assuming that all the lists have the same length
  // so we stop copying when the shortest list ends.
  vector<string>::const_iterator plabel = ptask->labels.begin();
  vector<int>::const_iterator pconcpt = ptask->labelConcepts.begin();
  vector<double>::const_iterator pdistr = ptask->distribution.begin();
  vector<double>::const_iterator pgain = ptask->gains.begin();
  pvobj->colorGain = 0.0;
  pvobj->shapeGain = 0.0;
  for(; plabel != ptask->labels.end() && pconcpt != ptask->labelConcepts.end() &&
      pdistr != ptask->distribution.end() && pgain != ptask->gains.end();
      plabel++, pconcpt++, pdistr++, pgain++)
  {
    // Concept mapping is done in Matlab
    if (*pconcpt == 1){ // color concept
      pvobj->colorLabels.push_back(*plabel);
      pvobj->colorDistrib.push_back(*pdistr);
      pvobj->colorGains.push_back(*pgain);
      if (*pgain > pvobj->colorGain) pvobj->colorGain = *pgain;
    }
    else if (*pconcpt == 2) { // shape concept
      pvobj->shapeLabels.push_back(*plabel);
      pvobj->shapeDistrib.push_back(*pdistr);
      pvobj->shapeGains.push_back(*pgain);
      if (*pgain > pvobj->shapeGain) pvobj->shapeGain = *pgain;
    }
    else {
      pSoiFilter->println(" *** VL_Recognizer Invalid concept ID: %d", *pconcpt);
    }
  }

  // ambiguity in the distribution: we use the distribution's entropy
  pvobj->identAmbiguity = 0.;
  for(size_t i = 0; i < pvobj->identDistrib.size(); i++)
    if(fpclassify(pvobj->identDistrib[i]) != FP_ZERO)
      pvobj->identAmbiguity -= pvobj->identDistrib[i]*::log(pvobj->identDistrib[i]);

  // TODO: what about identGain?

  pvobj->time = pSoiFilter->getCASTTime();
  try {
    pSoiFilter->overwriteWorkingMemory(visualObjectAddr, pvobj);
  }
  catch(DoesNotExistOnWMException e) {
    pSoiFilter->log("(WRITE) Visual Object '%s' deleted. It will not be updated.", id.c_str());
  }
}

} // namespace
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */
