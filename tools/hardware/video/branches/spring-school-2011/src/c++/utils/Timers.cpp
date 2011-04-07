/**
 * @author Marko Mahnič
 * @date January 2011
 */

#include "Timers.h"
#include <cmath>

using namespace std;

namespace Video {

CMilliTimer::CMilliTimer()
{
   pStartTime = 0;
}

CMilliTimer::~CMilliTimer()
{
   if (pStartTime) delete (timeval*)pStartTime;
}

void CMilliTimer::restart()
{
   if (pStartTime) delete (timeval*)pStartTime;
   pStartTime = new timeval;
   gettimeofday((timeval*)pStartTime, 0);
}

long long CMilliTimer::elapsed()
{
   if (!pStartTime) restart();

   struct timeval& start = *(timeval*)pStartTime;
   struct timeval now;
   long long useconds;

   gettimeofday(&now, 0);

   useconds = (now.tv_sec - start.tv_sec) * 1000 + (now.tv_usec - start.tv_usec) / 1000.0 + 0.5;
   return useconds;
}

CTimeStats::CTimeStats()
: count(0),
  rate(0.),
  lastRate(0.),
  changeThresh(0.),
  sigChange(false)
{
}

void CTimeStats::increment()
{
  // increment
  count++;

  // if this is the first time
  if(count == 1)
  {
    // start timer
    gettimeofday(&startTime, 0);
  }
  else
  {
    // current time
    timeval now;
    gettimeofday(&now, 0);
    timeval start(startTime);

    // from http://www.delorie.com/gnu/docs/glibc/libc_428.html
    // Perform the carry for the later subtraction by updating y.
    if(now.tv_usec < start.tv_usec)
    {
      int nsec = (start.tv_usec - now.tv_usec) / 1000000 + 1;
      start.tv_usec -= 1000000 * nsec;
      start.tv_sec += nsec;
    }
    if(now.tv_usec - start.tv_usec > 1000000)
    {
      int nsec = (now.tv_usec - start.tv_usec) / 1000000;
      start.tv_usec += 1000000 * nsec;
      start.tv_sec -= nsec;
    }

    // tv_usec is certainly positive.
    double diffSeconds = (double)(now.tv_sec - start.tv_sec);
    double diffMicros = (double)(now.tv_usec - start.tv_usec);

    double totalDiffSeconds = diffSeconds  + (diffMicros / 1000000.);

    // get new rate
    rate = (double)count/totalDiffSeconds;

    // diff the old and new rates
    double diffRate = abs(lastRate - rate);

    // compare diff to a percentage of the old rate
    if(diffRate > changeThresh)
    {
      sigChange = true;
      lastRate = rate;
      // get 3% of the old rate... HACK WARNING CONSTANT
      changeThresh = lastRate * 0.03;
    }
    else
    {
      sigChange = false;
    }
  }
}

} // namespace
/* vim:set sw=2 sts=4 ts=8 et:vim */
