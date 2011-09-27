/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#ifndef _SOIFILTER_TASKBASE_H_
#define _SOIFILTER_TASKBASE_H_

#include <cast/architecture/ManagedComponent.hpp>
#include <castutils/CastLoggerMixin.hpp>
#include <castutils/Timers.hpp>

namespace cast {

enum {
  TYPE_SOI = 1,
  TYPE_CMD_LOOK = 2,
  TYPE_CMD_LOOK_AROUND = 3,
  TYPE_CMD_ANALYZE = 4,
};

struct EventRetryInfo
{
  castutils::CMilliTimer tmWaiting;
  long tmToWaitMs;
  long retriesLeft;
  long retryCount;
  EventRetryInfo(long milliSeconds, long nRetries)
  {
    tmToWaitMs = milliSeconds;
    retriesLeft = nRetries;
    retryCount = 0;
  }
  bool isReady() {
    return tmWaiting.elapsed() >= tmToWaitMs;
  }
  long millisToReady() {
    if (isReady()) return 0;
    return tmToWaitMs - tmWaiting.elapsed();
  }
};

struct WmEvent
{
  int  objectType;
  int  change; // One of cdl ADD, OVERWRITE, DELETE
  long long order;
  cdl::WorkingMemoryChange wmc;
  EventRetryInfo* pRetry;
  WmEvent(int wmType, int changeType, const cdl::WorkingMemoryChange& _wmc)
  {
    pRetry = 0;
    objectType = wmType;
    change = changeType;
    wmc = _wmc;
    order = getEventOrder();
  }
  ~WmEvent() {
    if (pRetry)
      delete pRetry;
  }
private:
  static long long getEventOrder();
};

class SOIFilter;
class WmTaskExecutor:
  public castutils::CCastLoggerMixin
{
protected:
  SOIFilter* pSoiFilter;
public:
  WmTaskExecutor(SOIFilter* soif);
  virtual void handle(WmEvent *pEvent) = 0;
};

} // namespace
#endif
// vim: set sw=2 ts=8 sts=4 et :vim
