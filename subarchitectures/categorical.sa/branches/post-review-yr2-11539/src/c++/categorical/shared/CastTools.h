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
 * \file CastTools.h
 * \author Andrzej Pronobis
 * \date 2009-10-18
 */


#ifndef __PLACE_CAST_TOOLS__
#define __PLACE_CAST_TOOLS__

#include <cast/core/CASTUtils.hpp>

namespace categorical
{
  double castTimeToSeconds(const cast::cdl::CASTTime &time);

  cast::cdl::CASTTime castTimeDiff(const cast::cdl::CASTTime &time1,
      const cast::cdl::CASTTime &time2);

  inline double castTimeDiffToSeconds(const cast::cdl::CASTTime &time1,
      const cast::cdl::CASTTime &time2)
  {
    return castTimeToSeconds(castTimeDiff(time1,time2));
  }

}


#endif
