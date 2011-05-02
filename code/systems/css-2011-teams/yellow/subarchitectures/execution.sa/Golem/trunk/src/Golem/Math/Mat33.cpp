/** @file Mat33.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Math/Mat33.h>
#include <Golem/Math/Quat.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::QuatToMat33(Mat33& m, const Quat& q) {
	m.m11 = REAL_ONE - q.q2*q.q2*REAL_TWO - q.q3*q.q3*REAL_TWO;
	m.m12 = q.q1*q.q2*REAL_TWO - q.q0*q.q3*REAL_TWO;	
	m.m13 = q.q1*q.q3*REAL_TWO + q.q0*q.q2*REAL_TWO;	

	m.m21 = q.q1*q.q2*REAL_TWO + q.q0*q.q3*REAL_TWO;	
	m.m22 = REAL_ONE - q.q1*q.q1*REAL_TWO - q.q3*q.q3*REAL_TWO;	
	m.m23 = q.q2*q.q3*REAL_TWO - q.q0*q.q1*REAL_TWO;	
	
	m.m31 = q.q1*q.q3*REAL_TWO - q.q0*q.q2*REAL_TWO;	
	m.m32 = q.q2*q.q3*REAL_TWO + q.q0*q.q1*REAL_TWO;	
	m.m33 = REAL_ONE - q.q1*q.q1*REAL_TWO - q.q2*q.q2*REAL_TWO;	
}

void golem::Mat33ToQuat(Quat& q, const Mat33& m) {
	Real trace = m.trace(), s;

	if (trace >= REAL_ZERO) {
		s = Math::sqrt(trace + REAL_ONE);
		q.q0 = s * REAL_HALF;
		s = REAL_HALF / s;

		q.q1 = (m.m32 - m.m23) * s;
		q.q2 = (m.m13 - m.m31) * s;
		q.q3 = (m.m21 - m.m12) * s;
	}
	else {
		int idx = 0;
		if (m.m22 > m.m11) idx = 1;
		if (m.m33 > m.m[idx][idx]) idx = 2;

		switch (idx) {
		#define MACRO(i, j, k, I, J, K)\
		case I:\
			s = Math::sqrt(m.m[I][I] - m.m[J][J] - m.m[K][K] + REAL_ONE);\
			q.i = s * REAL_HALF;\
			s = REAL_HALF / s;\
			q.j = (m.m[I][J] + m.m[J][I]) * s;\
			q.k = (m.m[K][I] + m.m[I][K]) * s;\
			q.q0 = (m.m[K][J] - m.m[J][K]) * s;\
			break
		MACRO(q1, q2, q3, 0, 1, 2);
		MACRO(q2, q3, q1, 1, 2, 0);
		MACRO(q3, q1, q2, 2, 0, 1);
		#undef MACRO
		}
	}
}

//------------------------------------------------------------------------------
