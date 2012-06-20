/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#ifndef _SOIFILTER_CMDLEARNERRECOGNIZE_H_
#define _SOIFILTER_CMDLEARNERRECOGNIZE_H_

#include <VisionData.hpp>
#include <cast/architecture/WorkingMemoryChangeReceiver.hpp>
#include <IceUtil/IceUtil.h>

namespace cast {

class SOIFilter;
// An implementation of a RPC call through WM that completes on WM-overwrite.
/*
 * A WorkingMemoryChangeReceiver should be created on the heap. We wait for
 * it to complete then we remove the it from the change filter list with
 * removeChangeFilter(*, cdl::DELETERECEIVER), but we DON'T DELETE it. It
 * will be moved to a deletion queue by the framework and deleted later. */
class LearnerRecognitionTaskRcv:
  public cast::WorkingMemoryChangeReceiver
{
  bool m_bDelete;
protected:
  SOIFilter* pSoiFilter;
  bool m_complete;
  IceUtil::Monitor<IceUtil::Mutex> m_CompletionMonitor;
  cast::cdl::WorkingMemoryAddress visualObjectAddr; // to store the results
public:
  VisionData::VisualLearnerRecognitionTaskPtr m_pcmd;
public:
  LearnerRecognitionTaskRcv(SOIFilter* psoif,
      const cast::cdl::WorkingMemoryAddress& protoObjectAddr,
      const cast::cdl::WorkingMemoryAddress& visualObjectAddr);
  void workingMemoryChanged(const cast::cdl::WorkingMemoryChange &_wmc);
  void processResults();
  bool waitForCompletion(double milliSeconds);
  void deleteOnCompletion();
};
 
} // namespace
#endif
// vim: set sw=2 ts=8 sts=4 et :vim
