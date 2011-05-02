/** @file Mat34.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Math/Mat34.h>
#include <Golem/Math/Twist.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::TwistToMat34(Mat34& m, const Twist& t, Real theta) {
	if (t.w.isZero()) {
		// [ Id, v * theta ]
		m.R.setId();
		m.p.multiply(theta, t.v);
	}
	else {
		// [ exp(w^ theta), [Id - exp(w^ theta)] w^ v + w wT v * theta ]
		Real c1 = t.w.v2*t.v.v3 - t.w.v3*t.v.v2;
		Real c2 = t.w.v3*t.v.v1 - t.w.v1*t.v.v3;
		Real c3 = t.w.v1*t.v.v2 - t.w.v2*t.v.v1;

		Real s = t.w.dot(t.v) * theta;

		m.R.fromAngleAxis(theta, t.w);

		m.p.v1 = (REAL_ONE - m.R.m11) * c1 - m.R.m12 * c2 - m.R.m13 * c3 + t.w.v1 * s;
		m.p.v2 = -m.R.m21 * c1 + (REAL_ONE - m.R.m22) * c2 - m.R.m23 * c3 + t.w.v2 * s;
		m.p.v3 = -m.R.m31 * c1 - m.R.m32 * c2 + (REAL_ONE - m.R.m33) * c3 + t.w.v3 * s;
	}
}

void golem::Mat34ToTwist(Twist& t, Real& theta, const Mat34& m) {
	m.R.toAngleAxis(theta, t.w);

	if (Math::abs(theta) < REAL_EPS) { // see Mat33::toAngleAxis()
		// no rotation
		theta = m.p.magnitude();
		
		if (Math::abs(theta) > REAL_EPS)
			// no translation
			t.v.multiply(REAL_ONE / theta, m.p);
		else
			t.v.setZero();

		t.w.setZero();
	}
	else {
		// A * v = p => v = inv(A) * p, where A := [Id - exp(w^ theta)] w^ + w wT * theta
		// A is nonsingular for all theta E (0, 2*PI), because matrices which comprise A
		// have mutually orthogonal null spaces for theta != 0 and w != 0 (R != Id)
		Mat33 temp;

		temp.m11 = m.R.m13*t.w.v2 - m.R.m12*t.w.v3 + t.w.v1*t.w.v1*theta;
		temp.m12 = -m.R.m13*t.w.v1 - (REAL_ONE - m.R.m11)*t.w.v3 + t.w.v1*t.w.v2*theta;
		temp.m13 = (REAL_ONE - m.R.m11)*t.w.v2 + m.R.m12*t.w.v1 + t.w.v1*t.w.v3*theta;

		temp.m21 = (REAL_ONE - m.R.m22)*t.w.v3 + m.R.m23*t.w.v2 + t.w.v2*t.w.v1*theta;
		temp.m22 = m.R.m21*t.w.v3 - m.R.m23*t.w.v1 + t.w.v2*t.w.v2*theta;
		temp.m23 = -m.R.m21*t.w.v2 - (REAL_ONE - m.R.m22)*t.w.v1 + t.w.v2*t.w.v3*theta;

		temp.m31 = -m.R.m32*t.w.v3 - (REAL_ONE - m.R.m33)*t.w.v2 + t.w.v3*t.w.v1*theta;
		temp.m32 = (REAL_ONE - m.R.m33)*t.w.v1 + m.R.m31*t.w.v3 + t.w.v3*t.w.v2*theta;
		temp.m33 = m.R.m32*t.w.v1 - m.R.m31*t.w.v2 + t.w.v3*t.w.v3*theta;

		temp.multiplyByTranspose(t.v, m.p);
	}
}

// | R  p^R || v |
// | 0  R   || w |
void golem::AdjointTransform(Twist& dst, const Twist& t, const Mat34& m) {
	Real v1 = m.R.m11*t.v.v1 + m.R.m12*t.v.v2 + m.R.m13*t.v.v3 +
		(m.p.v2*m.R.m31 - m.p.v3*m.R.m21)*t.w.v1 +
		(m.p.v2*m.R.m32 - m.p.v3*m.R.m22)*t.w.v2 +
		(m.p.v2*m.R.m33 - m.p.v3*m.R.m23)*t.w.v3;
	Real v2 = m.R.m21*t.v.v1 + m.R.m22*t.v.v2 + m.R.m23*t.v.v3 +
		(m.p.v3*m.R.m11 - m.p.v1*m.R.m31)*t.w.v1 +
		(m.p.v3*m.R.m12 - m.p.v1*m.R.m32)*t.w.v2 +
		(m.p.v3*m.R.m13 - m.p.v1*m.R.m33)*t.w.v3;
	Real v3 = m.R.m31*t.v.v1 + m.R.m32*t.v.v2 + m.R.m33*t.v.v3 +
		(m.p.v1*m.R.m21 - m.p.v2*m.R.m11)*t.w.v1 +
		(m.p.v1*m.R.m22 - m.p.v2*m.R.m12)*t.w.v2 +
		(m.p.v1*m.R.m23 - m.p.v2*m.R.m13)*t.w.v3;
	
	dst.v.v1 = v1;
	dst.v.v2 = v2;
	dst.v.v3 = v3;

	dst.w.v1 = m.R.m11*t.w.v1 + m.R.m12*t.w.v2 + m.R.m13*t.w.v3;
	dst.w.v2 = m.R.m21*t.w.v1 + m.R.m22*t.w.v2 + m.R.m23*t.w.v3;
	dst.w.v3 = m.R.m31*t.w.v1 + m.R.m32*t.w.v2 + m.R.m33*t.w.v3;
}

// | RT  -RTp^ || v |   | RT*v - RTp^*w |
// | 0    RT   || w | = | RT*w          |
void golem::AdjointInverseTransform(Twist& dst, const Twist& t, const Mat34& m) {
	Real v1 = m.R.m11*t.v.v1 + m.R.m21*t.v.v2 + m.R.m31*t.v.v3 -
		(m.p.v3*m.R.m21 - m.p.v2*m.R.m31)*t.w.v1 -
		(m.p.v1*m.R.m31 - m.p.v3*m.R.m11)*t.w.v2 -
		(m.p.v2*m.R.m11 - m.p.v1*m.R.m21)*t.w.v3;
	Real v2 = m.R.m12*t.v.v1 + m.R.m22*t.v.v2 + m.R.m32*t.v.v3 -
		(m.p.v3*m.R.m22 - m.p.v2*m.R.m32)*t.w.v1 -
		(m.p.v1*m.R.m32 - m.p.v3*m.R.m12)*t.w.v2 -
		(m.p.v2*m.R.m12 - m.p.v1*m.R.m22)*t.w.v3;
	Real v3 = m.R.m13*t.v.v1 + m.R.m23*t.v.v2 + m.R.m33*t.v.v3 -
		(m.p.v3*m.R.m23 - m.p.v2*m.R.m33)*t.w.v1 -
		(m.p.v1*m.R.m33 - m.p.v3*m.R.m13)*t.w.v2 -
		(m.p.v2*m.R.m13 - m.p.v1*m.R.m23)*t.w.v3;
	
	dst.v.v1 = v1;
	dst.v.v2 = v2;
	dst.v.v3 = v3;

	dst.w.v1 = m.R.m11*t.w.v1 + m.R.m21*t.w.v2 + m.R.m31*t.w.v3;
	dst.w.v2 = m.R.m12*t.w.v1 + m.R.m22*t.w.v2 + m.R.m32*t.w.v3;
	dst.w.v3 = m.R.m13*t.w.v1 + m.R.m23*t.w.v2 + m.R.m33*t.w.v3;
}

// | RT     0  || v |   | RT*v          |
// | -RTp^  RT || w | = | RT*w - RTp^*v |
void golem::AdjointTransposedTransform(Twist& dst, const Twist& t, const Mat34& m) {
	Real v1 = m.R.m11*t.w.v1 + m.R.m21*t.w.v2 + m.R.m31*t.w.v3 -
		(m.p.v3*m.R.m21 - m.p.v2*m.R.m31)*t.v.v1 -
		(m.p.v1*m.R.m31 - m.p.v3*m.R.m11)*t.v.v2 -
		(m.p.v2*m.R.m11 - m.p.v1*m.R.m21)*t.v.v3;
	Real v2 = m.R.m12*t.w.v1 + m.R.m22*t.w.v2 + m.R.m32*t.w.v3 -
		(m.p.v3*m.R.m22 - m.p.v2*m.R.m32)*t.v.v1 -
		(m.p.v1*m.R.m32 - m.p.v3*m.R.m12)*t.v.v2 -
		(m.p.v2*m.R.m12 - m.p.v1*m.R.m22)*t.v.v3;
	Real v3 = m.R.m13*t.w.v1 + m.R.m23*t.w.v2 + m.R.m33*t.w.v3 -
		(m.p.v3*m.R.m23 - m.p.v2*m.R.m33)*t.v.v1 -
		(m.p.v1*m.R.m33 - m.p.v3*m.R.m13)*t.v.v2 -
		(m.p.v2*m.R.m13 - m.p.v1*m.R.m23)*t.v.v3;
	
	dst.w.v1 = v1;
	dst.w.v2 = v2;
	dst.w.v3 = v3;

	dst.v.v1 = m.R.m11*t.v.v1 + m.R.m21*t.v.v2 + m.R.m31*t.v.v3;
	dst.v.v2 = m.R.m12*t.v.v1 + m.R.m22*t.v.v2 + m.R.m32*t.v.v3;
	dst.v.v3 = m.R.m13*t.v.v1 + m.R.m23*t.v.v2 + m.R.m33*t.v.v3;
}

//------------------------------------------------------------------------------
