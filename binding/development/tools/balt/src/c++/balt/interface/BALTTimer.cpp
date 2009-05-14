/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include "BALTTimer.hpp"

using namespace std;

timeval BALTTimer::startTime;



BALTTimer::BALTTimer() {
  reset();
}

void BALTTimer::reset() {
  //  cout<<"\t\t***************resetting BALT timer"<<endl;
  gettimeofday(&startTime, NULL);
}

/* 

http://www.delorie.com/gnu/docs/glibc/libc_428.hpptml

Subtract the `struct timeval' values X and Y,
   storing the result in RESULT.
   Return 1 if the difference is negative, otherwise 0.  */


FrameworkBasics::BALTTime BALTTimer::getBALTTime() {

  FrameworkBasics::BALTTime t;
  timeval tv;

  gettimeofday(&tv, NULL);

  /* Perform the carry for the later subtraction by updating y. */
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
  t.m_s = tv.tv_sec - startTime.tv_sec;
  t.m_us = tv.tv_usec - startTime.tv_usec;

  return t;
}


FrameworkBasics::BALTTime BALTTimer::timeDiff(const FrameworkBasics::BALTTime &_start, const FrameworkBasics::BALTTime &_end) {

  FrameworkBasics::BALTTime t, startTime = _start;

  /* start the carry for the later subtraction by updating y. */
  if (_end.m_us < startTime.m_us) {
    int nsec = (startTime.m_us - _end.m_us) / 1000000 + 1;
    startTime.m_us -= 1000000 * nsec;
    startTime.m_s += nsec;
  }
  if (_end.m_us - startTime.m_us > 1000000) {
    int nsec = (_end.m_us - startTime.m_us) / 1000000;
    startTime.m_us += 1000000 * nsec;
    startTime.m_s -= nsec;
  }

  /* Compute the time remaining to wait.
     m_us is certainly positive. */
  t.m_s = _end.m_s - startTime.m_s;
  t.m_us = _end.m_us - startTime.m_us;

  return t;


}

FrameworkBasics::BALTTime BALTTimer::getSystemTime() {

  FrameworkBasics::BALTTime t;
  timeval tv;
  gettimeofday(&tv, NULL);
  t.m_s = tv.tv_sec;
  t.m_us = tv.tv_usec;
  return t;

}


ostream & operator<<(ostream &_stream, const FrameworkBasics::BALTTime &_time) {
  _stream<<BALTTimer::toSeconds(_time);
  return _stream;
}

