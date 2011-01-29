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
 * ColorMap class.
 * \file ColorMap.cpp
 * \author Andrzej Pronobis
 * \date 2008-09-02
 */

#include "ColorMap.h"

using namespace categorical;


void ColorMap::getColorForPlaceClass(int classNo, float &r, float &g, float &b)
{
  switch(classNo)
  {
    case 0:
      r=0.5;
      g=0.5;
      b=0.5;
      break;
    case 1:
      r=1.0;
      g=0.0;
      b=0.0;
      break;
    case 2:
      r=0.0;
      g=1.0;
      b=0.0;
      break;
    case 3:
      r=0.0;
      g=0.0;
      b=1.0;
      break;
    case 4:
      r=1.0;
      g=0.0;
      b=1.0;
      break;
    case 5:
      r=1.0;
      g=1.0;
      b=0.0;
      break;
    case 6:
      r=0.0;
      g=1.0;
      b=1.0;
      break;
    default:
      r=0.0;
      g=0.0;
      b=0.0;
  }
}
