#ifndef _SOIFILTER_TASKMOVETOVIEWCONE_H_
#define _SOIFILTER_TASKMOVETOVIEWCONE_H_

#include "TaskBase.h"

namespace cast {

class WmTaskExecutor_MoveToViewCone: public WmTaskExecutor
{
protected:
  virtual void handle_add_task(WmEvent *pEvent);
public:
  WmTaskExecutor_MoveToViewCone(SOIFilter* soif) : WmTaskExecutor(soif) {}

  virtual void handle(WmEvent *pEvent)
  {
    if (pEvent->change == cdl::ADD) handle_add_task(pEvent);
  }
};

} // namespace
#endif
// vim: set sw=2 ts=8 sts=4 et :vim
