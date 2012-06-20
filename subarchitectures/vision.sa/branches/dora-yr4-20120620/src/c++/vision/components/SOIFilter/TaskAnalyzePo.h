/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#ifndef _SOIFILTER_TASKANALYZEPO_H_
#define _SOIFILTER_TASKANALYZEPO_H_

#include "TaskBase.h"

namespace cast {

// The planner/executor is responsible for the correct ordering of Analyze
// and MoveToViewCone tasks. While an Analyze task is active, a
// MoveToViewCone should not be executed.

class WmTaskExecutor_Analyze: public WmTaskExecutor
{
protected:
  virtual void handle_add_task(WmEvent *pEvent);
public:
  WmTaskExecutor_Analyze(SOIFilter* soif) : WmTaskExecutor(soif) {}

  virtual void handle(WmEvent *pEvent)
  {
    if (pEvent->change == cdl::ADD) handle_add_task(pEvent);
  }
};


} // namespace
#endif
// vim: set sw=2 ts=8 sts=4 et :vim
