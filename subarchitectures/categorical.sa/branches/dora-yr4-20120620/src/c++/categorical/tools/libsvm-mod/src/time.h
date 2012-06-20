#ifndef _TIME_H
#define _TIME_H


struct TimeInterval
{
  double prof;
  double real;
  double virt;
};


/** Resets the timers. */
void timerReset();

/** Returns the time that passed since the last timer reset. */
TimeInterval timerQuery();



#endif

