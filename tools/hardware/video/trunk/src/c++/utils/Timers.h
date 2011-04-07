#ifndef TIMERS_7Z7YYAHY
#define TIMERS_7Z7YYAHY

#include <sys/time.h>

namespace Video {

/**
 * @brief A timer with millisecond precision.
 * @author Marko Mahnič
 * @date January 2011
 */
class CMilliTimer
{
   void* pStartTime;
public:
   CMilliTimer();
   ~CMilliTimer();
   void restart();
   long long elapsed();
   static long long seconds(double s)
   {
     return (long long) (s * 1000);
   }
};

/**
* @brief Class for measuring how many things happen per second. \n
* author: Nick Hawes
*/
class CTimeStats
{
public:
  CTimeStats();
  void increment();
  double getRate() const
  {
    return rate;
  }
  bool rateChange() const
  {
    return sigChange;
  }

private:
  int count;
  double rate;
  double lastRate;
  double changeThresh;
  timeval startTime;
  bool sigChange;
};

} // namespace
#endif /* end of include guard: TIMERS_7Z7YYAHY */
/* vim:set sw=2 sts=4 ts=8 et:vim */
