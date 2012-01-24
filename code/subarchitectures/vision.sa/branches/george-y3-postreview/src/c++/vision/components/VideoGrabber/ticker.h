
#ifndef _TICKER_H_4CEB82CF_
#define _TICKER_H_4CEB82CF_

#include <IceUtil/Timer.h>
#include <IceUtil/Mutex.h>
#include <IceUtil/Monitor.h>

namespace cogx {

// Usage:
// class MyTask: public CTickSyncedTask; // if required, make it some kind of a thread
// MyTask::run () {
//    while (isRunning) {
//       int ticks = waitForTick();
//       if (! ticks) continue;
//       if (ticks > 1) printf ("Missed ticks: %d", ticks - 1);
//       ... do something ...
//    }
// }
class CTickSyncedTask
{
protected:
   IceUtil::Monitor<IceUtil::Mutex> m_tickerMutex;
   int m_ticked;

public:
   CTickSyncedTask()
   {
      m_ticked = 0;
   }

   void tick()
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_tickerMutex);
      m_ticked++;
      m_tickerMutex.notify();
   }

   int waitForTick(int ms = 500)
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_tickerMutex);
      if (! m_ticked)
         m_tickerMutex.timedWait(IceUtil::Time::milliSeconds(ms));
      if (! m_ticked) return 0;
      int rv = m_ticked;
      m_ticked = 0;
      return rv;
   }
};

class CTickerTask: public IceUtil::TimerTask
{
   CTickSyncedTask* m_pWorker;
public:
   CTickerTask(CTickSyncedTask* pWorker)
   {
      assert(pWorker);
      m_pWorker = pWorker;
   }
   void runTimerTask()
   {
      // This should be as short as possible so that we get
      // approximately equal time periods.
      m_pWorker->tick();
   }
};

} // namespace
#endif /* end of include guard: _TICKER_H_4CEB82CF_ */
