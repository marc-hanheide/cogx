#include "time.h"
#include <sys/time.h>

#define MAX_TIMER_SECONDS 3600*24*10; // Max 10 days
static struct itimerval _timerReal;
static struct itimerval _timerVirt;
static struct itimerval _timerProf;

void timerReset()
{
  itimerval timerReset;

  timerReset.it_interval.tv_sec = 0;
  timerReset.it_interval.tv_usec = 0;
  timerReset.it_value.tv_sec = MAX_TIMER_SECONDS;
  timerReset.it_value.tv_usec = 0;

  // Reset the timers
  setitimer( ITIMER_REAL, &timerReset, 0 );
  setitimer( ITIMER_VIRTUAL, &timerReset, 0 );
  setitimer( ITIMER_PROF, &timerReset, 0 );

  // Read current timer values
  getitimer(ITIMER_REAL, &_timerReal);
  getitimer(ITIMER_VIRTUAL, &_timerVirt);
  getitimer(ITIMER_PROF, &_timerProf);
}


TimeInterval timerQuery()
{
  itimerval timerRealNew;
  itimerval timerVirtNew;
  itimerval timerProfNew;

  getitimer(ITIMER_REAL, &timerRealNew);
  getitimer(ITIMER_VIRTUAL, &timerVirtNew);
  getitimer(ITIMER_PROF, &timerProfNew);

  TimeInterval ti;
  ti.real=static_cast<double>(_timerReal.it_value.tv_sec-timerRealNew.it_value.tv_sec)+
          static_cast<double>(_timerReal.it_value.tv_usec-timerRealNew.it_value.tv_usec)/1000000.0;
  ti.virt=static_cast<double>(_timerVirt.it_value.tv_sec-timerVirtNew.it_value.tv_sec)+
          static_cast<double>(_timerVirt.it_value.tv_usec-timerVirtNew.it_value.tv_usec)/1000000.0;
  ti.prof=static_cast<double>(_timerProf.it_value.tv_sec-timerProfNew.it_value.tv_sec)+
          static_cast<double>(_timerProf.it_value.tv_usec-timerProfNew.it_value.tv_usec)/1000000.0;

  return ti;
}



