#ifndef CMILLITIMER_PEOJ4UOI
#define CMILLITIMER_PEOJ4UOI

class CMilliTimer
{
   void* pStartTime;
public:
   CMilliTimer();
   ~CMilliTimer();
   void restart();
   long long elapsed();
};

#endif /* end of include guard: CMILLITIMER_PEOJ4UOI */
