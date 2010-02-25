/** @file math_helpers.cpp
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 * Copyright 2009      Sergio Roa

 * @author      Jan Hanzelka - DFKI

 This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.

 
 */

#include <tools/math_helpers.h>

namespace smlearning {

///
///generate random double nr. between min and max
///
double fRand(double min, double max) {
	return min + (max - min)*::rand()/RAND_MAX;
}

}; /* namespace smlearning */
