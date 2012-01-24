/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#ifndef _SOIFILTER_CMDGETSOIS_H_
#define _SOIFILTER_CMDGETSOIS_H_

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
class GetSoisCommandRcv:
  public cast::WorkingMemoryChangeReceiver
{
protected:
  SOIFilter* pSoiFilter;
  bool m_complete;
  IceUtil::Monitor<IceUtil::Mutex> m_CompletionMonitor;
public:
  VisionData::GetStableSoisCommandPtr m_pcmd;
public:
  GetSoisCommandRcv(SOIFilter* psoif, std::string component_id);
  void workingMemoryChanged(const cast::cdl::WorkingMemoryChange &_wmc);
  bool waitForCompletion(double milliSeconds);
  void getSois(std::vector<VisionData::SOIPtr>& sois);
};
 
} // namespace
#endif
// vim: set sw=2 ts=8 sts=4 et :vim
