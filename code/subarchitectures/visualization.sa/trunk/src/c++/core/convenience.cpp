/*
 * Author: Marko Mahnič
 * Created: 2010-04-16
 *
 * © Copyright 2010 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "convenience.hpp"

#include <sstream>
#include <iomanip>
#include <ctime>

#ifdef DEBUG_TRACE
int __Tracer::level = 0;
#endif

std::string sfloat(double f, int precision)
{
   std::ostringstream out;
   out << std::fixed << std::setprecision(precision) << f;
   return out.str();
}

double fclocks()
{
   // clock() is not working correctly
   // return ( (double) clock()) / CLOCKS_PER_SEC;
   return 1e-9 * gethrtime();
}

long long gethrtime(void)
{
  struct timespec sp;
  int ret;
  long long v;
#ifdef CLOCK_MONOTONIC_HR
  ret=clock_gettime(CLOCK_MONOTONIC_HR, &sp);
#else
  ret=clock_gettime(CLOCK_MONOTONIC, &sp);
#endif
  if(ret!=0) return 0;
  v=1000000000LL; /* seconds->nanonseconds */
  v*=sp.tv_sec;
  v+=sp.tv_nsec;
  return v;
}

