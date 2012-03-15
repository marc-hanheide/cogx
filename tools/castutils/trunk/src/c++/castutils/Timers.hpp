#ifndef CASTUTILS_CMILLITIMER_PEOJ4UOI
#define CASTUTILS_CMILLITIMER_PEOJ4UOI

#include <vector>
#include <sys/time.h>

namespace castutils {

/**
 * @brief A timer with millisecond precision.
 * @author Marko Mahnič
 * @date January 2011
 */
class CMilliTimer
{
   timeval mStartTime;
   long long timeout;
public:
   CMilliTimer(bool bStart=true /*unused*/)
   {
      gettimeofday(&mStartTime, 0);
      timeout = 0;
   }

   void setTimeout(double milliSeconds)
   {
      timeout = milliSeconds * 1000;
   }

   void restart()
   {
      gettimeofday(&mStartTime, 0);
   }

   long long elapsed()
   {
      return (long long) (0.5 + elapsed_micros() / 1e3);
   }

   long long elapsed_micros()
   {
      const struct timeval& start = mStartTime;
      struct timeval now;

      gettimeofday(&now, 0);

      return (now.tv_sec - start.tv_sec) * 1000 * 1000 + (now.tv_usec - start.tv_usec);
   }

   bool isTimeoutReached()
   {
      return elapsed_micros() > timeout;
   }
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
      : timer(false)
   {
      tickCount = -1; // will restart the timer on the first tick
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
      int prevPos;
      if (numIntervals == 0) return 0;
      if (numIntervals < 0 || numIntervals >= (int) ticks.size())
         prevPos = (tickInfoPos + 1) % ticks.size();
      else
         prevPos = (tickInfoPos - numIntervals) % ticks.size();

      _tick_info_& prev = ticks[prevPos];
      _tick_info_& now = ticks[tickInfoPos];
      long long elapsed = now.useconds - prev.useconds;
      long dticks = now.tickCount - prev.tickCount;
      if (elapsed < 1)
         return 0;
      return 1e6 * dticks / elapsed;
   }
};

/**
 * @author Marko Mahnič
 * @date September 2011
 */
class CPaceMaker
{
private:
   CMilliTimer timer;
   long long nextTick;
   long intervalDurationMs;
   long minSleep;
   long tickCount;
   long tickLost;

public:
   CPaceMaker(long intervalMs, long minSleepMs=0)
      : timer(false)
   {
      intervalDurationMs = intervalMs;
      minSleep = minSleepMs;
      tickCount = -1; // will restart the timer on the first tick
      tickLost = 0;
   }
   void sync()
   {
      ++tickCount;
      if (tickCount == 0) {
         timer.restart();
         nextTick = intervalDurationMs;
         return;
      }
      long long now = timer.elapsed();
      long toSleep = nextTick - now;
      if (toSleep >= 0) {
         if (toSleep > intervalDurationMs) {
            // XXX: sth's wrong; fix nextTick (normally this shouldn't happen)
            toSleep = intervalDurationMs;
         }
         else nextTick += intervalDurationMs;
      }
      else {
         nextTick = now + intervalDurationMs;
         tickLost++;
      }
      if (toSleep < minSleep)
         toSleep = minSleep;
      if (toSleep > 0 && toSleep <= intervalDurationMs)
         doSleep(toSleep);
   }
   double getTotalRate()
   {
      long long elapsed = timer.elapsed_micros();
      if (elapsed < 1)
         return 0;
      return tickCount * 1e6 / elapsed;
   }
   virtual void doSleep(long milliSeconds) = 0;
};

/**
 * @author Marko Mahnič
 * @date September 2011
 *
 * Use:
 *    void MyComponent::runComponent() {
 *       castutils::CCastPaceMaker<MyComponent> paceMaker(*this, 1000 / 15, 1); // ~15 Hz
 *       while(isRunning()) {
 *          paceMaker.sync();
 *          // ... 
 *       }
 *    }
 *
 * The CAST component must make sleepComponent public with
 *    using CASTComponent::sleepComponent;
 * in the class declaration.
 */
template<class TComponent>
class CCastPaceMaker : public CPaceMaker
{
   TComponent* pComponent;
public:
   CCastPaceMaker(TComponent& component, long intervalMs, long minSleepMs=0)
      : CPaceMaker(intervalMs, minSleepMs)
   {
      pComponent = &component;
   }
   void doSleep(long milliSeconds)
   {
      pComponent->sleepComponent(milliSeconds);
   }
};

} // namespace

#endif /* end of include guard: CASTUTILS_CMILLITIMER_PEOJ4UOI */
