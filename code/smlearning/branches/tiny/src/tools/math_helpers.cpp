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
///function for normalizing values according to given bounds (before storing)
///
Real normalize(const Real& value, const Real& min, const Real& max) {
	Real val;
	if (min == -MATH_PI && max == MATH_PI && (value > max || value < min)) {
		val = fmod(value, MATH_PI);
	}
	else {
		val = value;
	}
	Real interval = max - min;
	Real relativeVal = val - min;
	Real res = relativeVal/interval;
	return -1.0 + (res*2.0);
}

///
///computer an orthogonal vector to some vector (which could be a surface normal vector)
///
Vec3 computeOrthogonalVec(const Vec3& normalVec) {
	Vec3 orthogonalVec(Real(normalVec.v2), Real(-1.0*normalVec.v1), Real(0.0));
	return orthogonalVec; 
}

///
///compute the normal vector for vector1 with respect to vector2
///
Vec3 computeNormalVector(const Vec3& vector1, const Vec3& vector2) {
	Vec3 res(Real((vector2.v1 - vector1.v1)
			/sqrt(pow(vector2.v1 - vector1.v1,2) + pow(vector2.v2 - vector1.v2,2) + pow(vector2.v3 - vector1.v3,2))),
		Real((vector2.v2 - vector1.v2)
			/sqrt(pow(vector2.v1 - vector1.v1,2) + pow(vector2.v2 - vector1.v2,2) + pow(vector2.v3 - vector1.v3,2))),
		Real((vector2.v3 - vector1.v3)
			/sqrt(pow(vector2.v1 - vector1.v1,2) + pow(vector2.v2 - vector1.v2,2) + pow(vector2.v3 - vector1.v3,2))));
	return res;

}

}; /* namespace smlearning */
