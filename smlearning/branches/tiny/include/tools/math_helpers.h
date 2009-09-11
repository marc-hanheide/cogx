/** @file math_helpers.h
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

#ifndef SMLEARNING_MATHHELPERS_H_
#define SMLEARNING_MATHHELPERS_H_

#include <system/Vec3.h>

using namespace golem;

namespace smlearning {

///
///function for normalizing values according to given bounds
///
Real normalize(const Real& value, const Real& min, const Real& max);	


///
///computer an orthogonal vector to some vector (which could be a surface normal vector)
///
Vec3 computeOrthogonalVec(const Vec3& normalVec);

///
///compute the normal vector for vector1 with respect to vector2
///
Vec3 computeNormalVector(const Vec3& vector1, const Vec3& vector2);

}; /* smlearning namespace */

#endif /* SMLEARNING_MATHHELPERS_H_*/
