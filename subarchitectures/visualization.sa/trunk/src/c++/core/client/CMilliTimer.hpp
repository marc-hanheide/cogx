#ifndef CMILLITIMER_PEOJ4UOI
#define CMILLITIMER_PEOJ4UOI

#include <vector>

/**
 * @brief A timer with millisecond precision.
 * @author Marko Mahnič
 * @date January 2011
 */
class CMilliTimer
{
   void* pStartTime;
public:
   CMilliTimer(bool bStart=false);
   ~CMilliTimer();
   void restart();
   long long elapsed();
   long long elapsed_micros();
};

/**
 * @brief A rate statistics (ticks per second) with a running average
 * @author Marko Mahnič
 * @date September 2011
 */
class CRunningRate
{
private:
   struct _tick_info_ {
      long tickCount;
      long long useconds;
      _tick_info_(long count=0, long long us=0)
      {
         tickCount = count;
         useconds = us;
      }
   };
private:
   CMilliTimer timer;
   long tickCount;
   long long intervalDurationUs;
   int tickInfoPos;
   std::vector<_tick_info_> ticks;

public:
   // default: 10 intervals of 3s = 30s running average
   CRunningRate(int numIntervals=10, long intervalDurationMs=3000)
   {
      tickCount = -1;
      tickInfoPos = 0;
      if (numIntervals < 5) numIntervals = 5;
      if (numIntervals > 100) numIntervals = 100;
      ticks.resize(numIntervals);
      intervalDurationUs = 1000 * intervalDurationMs;
   }
   void tick()
   {
      ++tickCount;
      if (tickCount == 0) {
         timer.restart();
         ticks.assign(ticks.size(), _tick_info_(0, 0));
         tickInfoPos = 0;
         return;
      }
      long long elapsed = timer.elapsed_micros();
      if (elapsed / intervalDurationUs != ticks[tickInfoPos].useconds / intervalDurationUs)
         tickInfoPos = (tickInfoPos + 1) % ticks.size();
      ticks[tickInfoPos] = _tick_info_(tickCount, elapsed);
   }

   // Get the total rate from the first tick till now
   double getTotalRate()
   {
      long long elapsed = timer.elapsed_micros();
      if (elapsed < 1)
         return 0;
      return tickCount * 1e6 / elapsed;
   }

   // Get the rate calculated from numIntervals intervals (ticks per second).
   // @param numIntervals number of intervals to look back
   //   negative - include all intervals ( default )
   //   0 - getRate returns 0
   double getRate(int numIntervals = -1)
   {
      int pos;
      if (numIntervals == 0) return 0;
      if (numIntervals < 0 || numIntervals >= ticks.size())
         pos = (tickInfoPos + 1) % ticks.size();
      else
         pos = (tickInfoPos - numIntervals) % ticks.size();

      _tick_info_ now = ticks[tickInfoPos];
      _tick_info_ then = ticks[pos];
      return 1e6 * (now.tickCount - then.tickCount) / (now.useconds - then.useconds);
   }
};

#endif /* end of include guard: CMILLITIMER_PEOJ4UOI */
