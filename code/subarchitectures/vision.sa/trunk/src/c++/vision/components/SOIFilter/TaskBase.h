/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#ifndef _SOIFILTER_TASKBASE_H_
#define _SOIFILTER_TASKBASE_H_

#include <cast/architecture/ManagedComponent.hpp>

namespace cast {

enum {
  TYPE_SOI = 1, TYPE_CMD_LOOK = 2, TYPE_CMD_ANALYZE = 3
};

struct WmEvent
{
  int  objectType;
  int  change; // One of cdl ADD, OVERWRITE, DELETE
  long long order;
  cdl::WorkingMemoryChange wmc;
  WmEvent(int wmType, int changeType, const cdl::WorkingMemoryChange& _wmc)
  {
    objectType = wmType;
    change = changeType;
    wmc = _wmc;
    order = getEventOrder();
  }
private:
  static long long getEventOrder();
};

class SOIFilter;
class WmTaskExecutor
{
   protected:
      SOIFilter* pSoiFilter;
   public:
      WmTaskExecutor(SOIFilter* soif)
      {
         pSoiFilter = soif;
      }
      virtual void handle(WmEvent *pEvent) = 0;
};

} // namespace
#endif
// vim: set sw=2 ts=8 sts=4 et :vim
