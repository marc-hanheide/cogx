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

#include <math.h>
#include <Math/Math.h>
#include <stdlib.h>
#include <Math/Vec3.h>
#include <iostream>

using namespace golem;
using namespace std;

namespace smlearning {

//const double PI = 3.14159265358979323846;

template <typename R>
R donotnormalize (R const& value, R const& min, R const& max) {
	return value;
}

///
///function for normalizing values according to given bounds
///
template <typename R>
R normalize(R const& value, R const& min, R const& max) {
	R val;
	if (min == -REAL_PI && max == REAL_PI && (value > max || value < min)) {
		val = fmod(value, (R)REAL_PI);
	}
	else {
		val = value;
	}
	R interval = max - min;
	R relativeVal = val - min;
	R res = relativeVal/interval;
	res = -1.0 + (res*2.0);
	//assert (res >= -1.0 && res <= 1.0);
	if (!(res >= -1.0 && res <= 1.0))
		throw true;
	return res;
}

///
///denormalizing values according to given bounds
///
template <typename R>
R denormalize (R const& value, R const& min, R const& max) {
	return (value + 1.0)/2.0*(max-min)+min;
}

template <typename R>
R donotdenormalize (R const& value, R const& min, R const& max) {
	return value;
}


///
///computer an orthogonal vector to some vector (which could be a surface normal vector)
///
template <typename V>
V computeOrthogonalVec(V const& normalVec) {
	V orthogonalVec;
	orthogonalVec.v1 = normalVec.v2;
	orthogonalVec.v2 = -1.0*normalVec.v1;
	orthogonalVec.v3 = 0.0;
	return orthogonalVec; 
}

///
///compute a difference vector
///
template <typename V>
V computeNormalVector(V const& vector1, V const& vector2) {
	V res;
	res.v1 = (vector2.v1 - vector1.v1)
		/sqrt(pow(vector2.v1 - vector1.v1,2) + pow(vector2.v2 - vector1.v2,2) + pow(vector2.v3 - vector1.v3,2));
	res.v2 = (vector2.v2 - vector1.v2)
		/sqrt(pow(vector2.v1 - vector1.v1,2) + pow(vector2.v2 - vector1.v2,2) + pow(vector2.v3 - vector1.v3,2));
	res.v3 = (vector2.v3 - vector1.v3)
		/sqrt(pow(vector2.v1 - vector1.v1,2) + pow(vector2.v2 - vector1.v2,2) + pow(vector2.v3 - vector1.v3,2));
	return res;

}

///
///rotate around x axis
///
template <typename M, typename R>
void rotX(M& m, R angle) {
	R s = ::sin(angle), c = ::cos(angle);
	
	m.m11 = 1.;		m.m12 = 0.;		m.m13 = 0.;
	m.m21 = 0.;		m.m22 = c;		m.m23 = -s;
	m.m31 = 0.;		m.m32 = s;		m.m33 = c;
}

///
///rotate around z axis
///
template <typename M, typename R>
void rotZ(M& m, R angle) {
	R s = ::sin(angle), c = ::cos(angle);
	
	m.m11 = c;		m.m12 = -s;		m.m13 = 0.;
	m.m21 = s;		m.m22 = c;		m.m23 = 0.;
	m.m31 = 0.;		m.m32 = 0.;		m.m33 = 1.;
}

///
///generate random double nr. between min and max
///
double fRand(double min = 0., double max = 1.);

///
///function that serves as a comparator for 2 vectors
///
class compare_Vec3 { // simple comparison function
public:
	bool operator()(const Vec3& v1, const Vec3& v2) {
		Real epsilon = 1e-8;
		if (((v1.v1 - epsilon) < v2.v1) && (v2.v1 < (v1.v1 + epsilon)) &&
		    ((v1.v2 - epsilon) < v2.v2) && (v2.v2 < (v1.v2 + epsilon)) &&
		    ((v1.v3 - epsilon) < v2.v3) && (v2.v3 < (v1.v3 + epsilon)))
			return false;
		
		return v1.magnitude() < v2.magnitude();
	}
};

///
///calls set_point_coordinates for a discrete canonical number of different actions
///
void set_coordinates_into_target(const int startPosition, Vec3& positionT,const Vec3& polyflapNormalVec, const Vec3& polyflapOrthogonalVec,const Real& dist, const Real& side, const Real& center, const Real& top, const Real& over);



}; /* smlearning namespace */

#endif /* SMLEARNING_MATHHELPERS_H_*/
