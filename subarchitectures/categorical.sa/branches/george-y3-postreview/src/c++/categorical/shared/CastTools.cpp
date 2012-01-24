// ==================================================================
// Place.SA - Place Classification Subarchitecture
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of Place.SA.
//
// Place.SA is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Place.SA is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Place.SA. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
 * CastTools class.
 * \file CastTools.cpp
 * \author Andrzej Pronobis
 * \date 2009-10-18
 */

#include "CastTools.h"

namespace categorical
{


double castTimeToSeconds(const cast::cdl::CASTTime &time)
{
  return static_cast<double>(time.s) + 1e-6 * static_cast<double>(time.us);
}

cast::cdl::CASTTime castTimeDiff(const cast::cdl::CASTTime &time1,
    const cast::cdl::CASTTime &time2)
{
  if (time1<time2)
    return castTimeDiff(time2, time1);

  cast::cdl::CASTTime ret;
  ret.s = time1.s-time2.s;
  if (time1.us<time2.us)
  {
    ret.s-=1;
    ret.us=1e6-(time2.us-time1.us);
  }
  else
  {
    ret.us=time1.us-time2.us;
  }
  return ret;
}




}

