/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#ifndef _SOIFILTER_TASKRECEIVESOI_H_
#define _SOIFILTER_TASKRECEIVESOI_H_

#include "TaskBase.h"
#include <atomic>

namespace cast {

class WmTaskExecutor_Soi: public WmTaskExecutor
{
  std::atomic<long> mAddCount;

protected:
   virtual void handle_add_soi(WmEvent *pEvent);
   virtual void handle_delete_soi(WmEvent *pEvent);
   virtual void handle_update_soi(WmEvent *pEvent);
   void MakeVisible(cdl::WorkingMemoryAddress &protoObjectAddr);
   void MakeInvisible(cdl::WorkingMemoryAddress &protoObjectAddr);

public:
   WmTaskExecutor_Soi(SOIFilter* soif) : WmTaskExecutor(soif)
  {
    mAddCount = 0;
  }

   virtual void handle(WmEvent *pEvent)
   {
      if (pEvent->change == cdl::ADD) handle_add_soi(pEvent);
      else if (pEvent->change == cdl::DELETE) handle_delete_soi(pEvent);
      else if (pEvent->change == cdl::OVERWRITE) handle_update_soi(pEvent);
   }

   long addEventCount()
   {
     return mAddCount;
   }
};


} // namespace
#endif
// vim: set sw=2 ts=8 sts=4 et :vim
