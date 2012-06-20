#include "CMilliTimer.hpp"
#include <sys/time.h>

namespace cogx { namespace display {

CMilliTimer::CMilliTimer(bool bStart)
{
   pStartTime = nullptr;
   if (bStart)
      restart();
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
   if (!pStartTime) return 0;

   struct timeval& start = *(timeval*)pStartTime;
   struct timeval now;
   long long mseconds;

   gettimeofday(&now, 0);

   mseconds  = (now.tv_sec - start.tv_sec) * 1000 + (now.tv_usec - start.tv_usec) / 1000.0 + 0.5;
   return mseconds;
}

long long CMilliTimer::elapsed_micros()
{
   if (!pStartTime) return 0;

   struct timeval& start = *(timeval*)pStartTime;
   struct timeval now;
   long long useconds;

   gettimeofday(&now, 0);

   useconds  = (now.tv_sec - start.tv_sec) * 1000 * 1000 + (now.tv_usec - start.tv_usec);
   return useconds;
}

}} // namespace
