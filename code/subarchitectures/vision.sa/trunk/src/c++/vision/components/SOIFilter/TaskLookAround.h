/**
 * @author Marko Mahnič
 * @date September 2011
 *
 */

#ifndef _SOIFILTER_TASKLOOKAROUND_H_
#define _SOIFILTER_TASKLOOKAROUND_H_

#include "TaskBase.h"

namespace cast {

// When the system is idle, the planner may look around for new stuff.
// The execution.sa generates a task for the robot to look around.
// Options for this task executor:
//   - it generates new view-cones for the robot to process (agreed)
//   - maybe it could turn the camera, instead
//
// The view cones can be generated relative to the current position (eg. +-30deg)
// or at predefined positions. (TODO: decide which mode to use; maybe a config param)

class WmTaskExecutor_LookAround: public WmTaskExecutor
{
protected:
  virtual void handle_add_task(WmEvent *pEvent);

public:
  WmTaskExecutor_LookAround(SOIFilter* soif) : WmTaskExecutor(soif) {}

  virtual void handle(WmEvent *pEvent)
  {
    if (pEvent->change == cdl::ADD) handle_add_task(pEvent);
  }
};


} // namespace
#endif
// vim: set sw=2 ts=8 sts=4 et :vim
