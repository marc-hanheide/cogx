/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#include "CmdGetSois.h"
#include "SOIFilter.h"

#include <cast/architecture/ChangeFilterFactory.hpp>

using namespace std;
using namespace VisionData;
using namespace cogx;

namespace cast
{

GetSoisCommandRcv::GetSoisCommandRcv(SOIFilter* psoif, std::string component_id)
{
  m_complete = false;
  pSoiFilter = psoif;
  m_pcmd = new GetStableSoisCommand();
  m_pcmd->componentId = component_id;
  m_pcmd->status = VisionData::VCREQUESTED;
  string id = pSoiFilter->newDataID();
  pSoiFilter->addChangeFilter(createIDFilter(id, cdl::OVERWRITE), this);  
  pSoiFilter->addToWorkingMemory(id, m_pcmd);  
}

void GetSoisCommandRcv::workingMemoryChanged(const cast::cdl::WorkingMemoryChange &_wmc)
{
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_CompletionMonitor);
  try {
    GetStableSoisCommandPtr pcmd = pSoiFilter->getMemoryEntry<GetStableSoisCommand>(_wmc.address);
    m_pcmd = pcmd;
  }
  catch (...) {
    pSoiFilter->debug("GetSoisCommand: Failed to get the results.");
    m_pcmd->status = VisionData::VCFAILED; /* complete, but failed */
  }

  m_complete = true;
  // pSoiFilter->debug("GetSoisCommand: NOTIFYING waitForCompletion.");
  m_CompletionMonitor.notify(); // works only if m_CompletionMonitor is locked here
}

void GetSoisCommandRcv::getSois(std::vector<VisionData::SOIPtr>& sois)
{
  sois = m_pcmd->sois;
}

bool GetSoisCommandRcv::waitForCompletion(double milliSeconds)
{
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_CompletionMonitor);
  if (m_complete) return true;
  m_CompletionMonitor.timedWait(IceUtil::Time::milliSeconds(milliSeconds));
  // pSoiFilter->debug("GetSoisCommand: waitForCompletion WOKE UP.");
  return m_complete;
}

} // namespace
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */
