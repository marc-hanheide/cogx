#include "CASTTimer.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace std;
using namespace cast;

CASTRateMeter::CASTRateMeter()
 :m_count(0),
  m_rate(0),
  m_lastRate(0),
  m_sigChange(false)
{}

void CASTRateMeter::increment() {

  //increment
  m_count++;

  //if this is the first time
  if(1 == m_count) {
    //start timer
    gettimeofday(&m_startTime, 0);
  }
  else {
    //current time
    timeval tv;
    gettimeofday(&tv, 0);
    timeval startTime(m_startTime);

    //nah: stolen from BALTCASTRateMeter which in turn stole from
    //http://www.delorie.com/gnu/docs/glibc/libc_428.hpptml
    if (tv.tv_usec < startTime.tv_usec) {
      int nsec = (startTime.tv_usec - tv.tv_usec) / 1000000 + 1;
      startTime.tv_usec -= 1000000 * nsec;
      startTime.tv_sec += nsec;
    }
    if (tv.tv_usec - startTime.tv_usec > 1000000) {
      int nsec = (tv.tv_usec - startTime.tv_usec) / 1000000;
      startTime.tv_usec += 1000000 * nsec;
      startTime.tv_sec -= nsec;
    }

    /* Compute the time remaining to wait.
       tv_usec is certainly positive. */
    double diffSeconds = tv.tv_sec - startTime.tv_sec;
    double diffMicros = tv.tv_usec - startTime.tv_usec;

    double totalDiffSeconds = diffSeconds  + (diffMicros / 1000000.0);

    //get new rate
    m_rate = m_count/totalDiffSeconds;

    //diff the old and new rates
    double diffRate = std::abs(m_lastRate - m_rate);

    //compare diff to a percentage of the old rate
    if(diffRate > m_changeThres) {
      m_sigChange = true;
      m_lastRate = m_rate;
      //get 3% of the old rate... HACK WARNING CONSTANT
      m_changeThres = m_lastRate * 0.03;
    }
    else {
      m_sigChange = false;
    }
  }

}

CASTTimer::CASTTimer(bool autoStart)
  :m_running(false)
{
  m_startTime.tv_sec = 0;
  if (autoStart) start();
}

int
CASTTimer::start()
{
  if (m_running) return 1;

  gettimeofday(&m_startTime, NULL);
  m_running = true;

  return 0;
}

void
CASTTimer::restart()
{
  gettimeofday(&m_startTime, NULL);
  m_running = true;
}

double
CASTTimer::stop()
{
  if (m_startTime.tv_sec == 0) {
    std::cerr << "CASTTimer::stop WARNING: called stop without starting timer"
              << std::endl;
    return -1;
  }

  if (m_running) {
    timeval tv;
    gettimeofday(&tv, NULL);

    m_stopTime = ((tv.tv_sec - m_startTime.tv_sec) +
                  1e-6*(tv.tv_usec - m_startTime.tv_usec));
    m_running = false;
  }

  return m_stopTime;
}

double
CASTTimer::split()
{
  if (m_startTime.tv_sec == 0) {
    std::cerr << "CASTTimer::stop WARNING: called split without starting timer"
              << std::endl;
    return -1;
  }

  if (!m_running) return m_stopTime;
  else {
    timeval tv;
    gettimeofday(&tv, NULL);

    return ((tv.tv_sec - m_startTime.tv_sec) +
            1e-6*(tv.tv_usec - m_startTime.tv_usec));
  }
}
