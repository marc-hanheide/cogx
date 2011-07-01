#ifndef CMILLITIMER_PEOJ4UOI
#define CMILLITIMER_PEOJ4UOI

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

#endif /* end of include guard: CMILLITIMER_PEOJ4UOI */
