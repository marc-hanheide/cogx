// ==================================================================
// libCRFH
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of libCRFH.
//
// libCRFH is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// libCRFH is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with libCRFH. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
 * \file CPerformance.cpp
 * \author Andrzej Pronobis
 *
 * Contains implementation of the CPerformance class.
 */

#include <sys/time.h>
#include "global.h"

#include "CPerformance.h"


// - - - - - - - - - - - - - - - - - -
double CPerformance::getSecsSinceEpoch()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec + 1.0e-6*tv.tv_usec);
}


// - - - - - - - - - - - - - - - - - -
void CPerformance::tic()
{
  ticStack.push(getSecsSinceEpoch());
}


// - - - - - - - - - - - - - - - - - -
double CPerformance::toc(bool verbose)
{
  if (ticStack.isEmpty())
  {
    aout<<"ERROR: tic() needs to be called before calling toc()"<<endl;
    return 0;
  }

  double time=getSecsSinceEpoch() - ticStack.pop();
  if (verbose)
      aout<<"Time elapsed since tic(): "<<time<<endl;

  return time;
}


// - - - - - - - - - - - - - - - - - -
QStack<double> CPerformance::ticStack;

