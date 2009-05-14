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
 
#ifndef BALT_BALT_TIMER_H_
#define BALT_BALT_TIMER_H_

#include <sys/time.h>
#include "includes.hpp"

std::ostream & operator<<(std::ostream &_stream, const FrameworkBasics::BALTTime &_time);

class BALTTimer {

 public:

  BALTTimer();

  ~BALTTimer(){};

  static void reset();
  static FrameworkBasics::BALTTime getBALTTime();
  static FrameworkBasics::BALTTime getSystemTime();
  static FrameworkBasics::BALTTime timeDiff(const FrameworkBasics::BALTTime &_start, const FrameworkBasics::BALTTime &_end);

  static double toSeconds(const FrameworkBasics::BALTTime &_time) {
    return  _time.m_s  + (_time.m_us / 1000000.0);
  }


 private:
  static timeval startTime;
};



#endif
