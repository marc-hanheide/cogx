/** @file Quat.h
 * 
 * Mathematical routines.
 * 
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk">The University Of Birmingham</A>
 *
 * @version 1.0
 *
 */

#ifndef SYSTEM_QUAT_H_
#define SYSTEM_QUAT_H_

//------------------------------------------------------------------------------

#include "Vec3.h"

//------------------------------------------------------------------------------
namespace msk {

class Quat;
class Mat33;

extern void QuatToMat33(Mat33& m, const Quat& q);
extern void Mat33ToQuat(Quat& q, const Mat33& m);

//------------------------------------------------------------------------------

/** Quaternion class Q = q0 + q1*i + q2*j + q3*k
*/
class Quat {
	friend class Mat33;

	friend void QuatToMat33(class Mat33& m, const class Quat& q);
	friend void Mat33ToQuat(class Quat& q, const class Mat33& m);

protected:
	/** q0 - scalar component; q1, q2, q3 - vector components */
	Real q0, q1, q2, q3;

public:
	/** Default constructor does not do any initialisation.
	*/
	inline Quat() {}

	/** Initialise with q0, q1, q2, q3.
	*/
	inline Quat(Real q0, Real q1, Real q2, Real q3) : q0(q0), q1(q1), q2(q2), q3(q3) {}

	/** Copies elements from v, and scalar from q0 (defaults to 0).
	*/
	inline Quat(const Vec3& v, Real q0 = REAL_ZERO) : q0(q0), q1(v.v1), q2(v.v2), q3(v.v3) {}

	/** Creates quaternion from rotation around axis for normalised axis.
	*/
	inline Quat(Real angle, const Vec3& axis) {
		fromAngleAxis(angle, axis);
	}

	/** Creates from rotation matrix.
	*	@param	m	rotation matrix to extract quaterion from.
	*/
	inline Quat(const class Mat33 &m) {
		Mat33ToQuat(*this, m);
	}

	/** Copy constructor.
	*/
	inline Quat(const Quat& q) : q0(q.q0), q1(q.q1), q2(q.q2), q3(q.q3) {}

	/** Set the members of the quaterion
	*/
	inline void set(const Quat& q) {
		this->q0 = q.q0;
		this->q1 = q.q1;
		this->q2 = q.q2;
		this->q3 = q.q3;
	}

	/** Set the members of the quaterion
	*/
	inline void set(Real q0, Real q1, Real q2, Real q3) {
		this->q0 = q0;
		this->q1 = q1;
		this->q2 = q2;
		this->q3 = q3;
	}

	/** Set the members of the quaterion from array
	*/
	inline void set(const F32 q[]) {
		q0 = (Real)q[0];
		q1 = (Real)q[1];
		q2 = (Real)q[2];
		q3 = (Real)q[3];
	}

	/** Set the members of the quaterion from array
	*/
	inline void set(const F64 q[]) {
		q0 = (Real)q[0];
		q1 = (Real)q[1];
		q2 = (Real)q[2];
		q3 = (Real)q[3];
	}

	/** Set the quaternion to the identity rotation [1,0,0,0]
	*/
	inline void setId() {
		q0 = REAL_ONE;
		q1 = REAL_ZERO;
		q2 = REAL_ZERO;
		q3 = REAL_ZERO;
	}

	/** sets the quat to Id
	*/
	inline void setZero() {
		setId();
	}

	inline void get(F32 q[]) const {
		q[0] = (F32)q0;
		q[1] = (F32)q1;
		q[2] = (F32)q2;
		q[3] = (F32)q3;
	}

	inline void get(F64 q[]) const {
		q[0] = (F64)q0;
		q[1] = (F64)q1;
		q[2] = (F64)q2;
		q[3] = (F64)q3;
	}

	/** Creates quaternion from rotation matrix.
	*/
	inline void fromMat33(const class Mat33& m) {
		msk::Mat33ToQuat(*this, m);
	}

	/** Creates rotation matrix from quaternion.
	*/
	inline void toMat33(class Mat33& m) {
		msk::QuatToMat33(m, *this);
	}

	/** Creates quaternion from rotation around axis for normalised axis.
	*/
	inline void fromAngleAxis(Real angle, const Vec3& axis) {
		Real s;
		Math::sinCos(angle * (Real)MATH_HALF, s, q0);
		//s /= axis.magnitude();
		q1 = axis.v1 * s;
		q2 = axis.v2 * s;
		q3 = axis.v3 * s;
	}

	/** Returns rotation angle and axis of quaternion
	*/
	inline void toAngleAxis(Real& angle, Vec3& axis) const {
		angle = Math::acos(q0) * (Real)MATH_TWO;
		Real s = Math::sqrt(REAL_ONE - q0*q0);// sin(angle/2), q0=cos(angle/2)
		if (Math::abs(s) > 0)
			axis.set(q1/s, q2/s, q3/s);
		else
			axis.setZero();
	}

	/** Gets the angle between this quat and the identity quaternion.
	*/
	inline Real getAngle() const {
		return Math::acos(q0) * (Real)MATH_TWO;
	}

	/** Gets the angle between this quaternion and the argument
	*/
	inline Real getAngle(const Quat& q) const {
		return Math::acos(dot(q)) * (Real)MATH_TWO;
	}

	/** 4D vector length.
	*/
	inline Real magnitude() const {
		return Math::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	}

	/** squared 4D vector length.
	*/
	inline Real magnitudeSquared() const {
		return q0*q0 + q1*q1 + q2*q2 + q3*q3;
	}

	/** returns the scalar product of this and other.
	*/
	inline Real dot(const Quat& q) const {
		return q0*q.q0 + q1*q.q1 + q2*q.q2 + q3*q.q3;
	}

	/** maps to the closest unit quaternion.
	*/
	inline Real normalise() {
		Real m = magnitude();
		
		if (m > REAL_ZERO) {
			const Real length = REAL_ONE / m;
			q0 *= length;
			q1 *= length;
			q2 *= length;
			q3 *= length;
		}

		return m;
	}

	/* assigns its own conjugate to itself.
	*/
	inline void conjugate() {
		q1 = -q1;
		q2 = -q2;
		q3 = -q3;
	}

	/** Sets this to the opposite rotation of this.
	*/
	inline void invert() {
		conjugate();
	}

	/** Sets this to the opposite rotation of this.
	*/
	inline void invertNormalise() {
		normalise();
		invert();
	}

	/** this = q * p = (q0p0 - (q * p), q0p + p0q + (q x p))
	*/
	inline void multiply(const Quat& q, const Quat& p) {
		Real a = q.q0*p.q0 - (q.q1*p.q1 + q.q2*p.q2 + q.q3*p.q3);
		Real b = q.q0*p.q1 + p.q0*q.q1 + (q.q2*p.q3 - p.q2*q.q3);
		Real c = q.q0*p.q2 + p.q0*q.q2 + (q.q3*p.q1 - p.q3*q.q1);
		Real d = q.q0*p.q3 + p.q0*q.q3 + (q.q1*p.q2 - p.q1*q.q2);

		q0 = a;
		q1 = b;
		q2 = c;
		q3 = d;
	}

	/** this = q * v, v is interpreted as quaternion [0, q1, q2, q3]
	*/
	inline void multiply(const Quat& q, const Vec3& v) {
		Real a = - (q.q1*v.v1 + q.q2*v.v2 + q.q3*v.v3);
		Real b = q.q0*v.v1 + (q.q2*v.v3 - v.v2*q.q3);
		Real c = q.q0*v.v2 + (q.q3*v.v1 - v.v3*q.q1);
		Real d = q.q0*v.v3 + (q.q1*v.v2 - v.v1*q.q2);

		q0 = a;
		q1 = b;
		q2 = c;
		q3 = d;
	}

	/** rotates passed vector by rotation expressed by the quaternion
	*/
	inline void rotate(Vec3& v) const {
		//Real m = REAL_ONE/magnitudeSquared();
		//Quat inverse(q0/m, -q1/m, -q2/m, -q3/m);
		Quat inverse(q0, -q1, -q2, -q3);

		//v = (this * v) x inverse;
		Quat q;
		q.multiply(*this, v);
		v.v1 = q.q0*inverse.q1 + inverse.q0*q.q1 + q.q2*inverse.q3 - inverse.q2*q.q3;
		v.v2 = q.q0*inverse.q2 + inverse.q0*q.q2 + q.q3*inverse.q1 - inverse.q3*q.q1;
		v.v3 = q.q0*inverse.q3 + inverse.q0*q.q3 + q.q1*inverse.q2 - inverse.q1*q.q2;
	}

	/** rotates passed vector opposite to the rotation expressed by the quaternion.
	*/
	inline void inverseRotate(Vec3& v) const {
		//Real m = REAL_ONE/magnitudeSquared();
		//Quat inverse(q0/m, -q1/m, -q2/m, -q3/m);
		Quat inverse(q0, -q1, -q2, -q3);

		//v = (inverse * v) x this;
		Quat q;
		q.multiply(inverse, v);
		v.v1 = q.q0*q1 + q0*q.q1 + q.q2*q3 - q2*q.q3;
		v.v2 = q.q0*q2 + q0*q.q2 + q.q3*q1 - q3*q.q1;
		v.v3 = q.q0*q3 + q0*q.q3 + q.q1*q2 - q1*q.q2;
	}

	/**	Slerp - minimum torque rotation interpolation.
	*/
	inline void slerp(const Quat& p, const Quat& q, Real t) {
		const Real dot = p.dot(q);
		Math::clamp(dot, -REAL_ONE, REAL_ONE);
		
		const Real acos = Math::acos(dot);
		if (acos > REAL_ONE - REAL_EPS) {
			// optionally interpolate linearly
			*this = p;
			return;
		}
	
		q0 = q.q0 - dot*p.q0;
		q1 = q.q1 - dot*p.q1;
		q2 = q.q2 - dot*p.q2;
		q3 = q.q3 - dot*p.q3;
		normalise();

		Real theta = acos*t, sin, cos;
		Math::sinCos(theta, sin, cos);
		q0 = cos*p.q0 + sin*q0;
		q1 = cos*p.q1 + sin*q1;
		q2 = cos*p.q2 + sin*q2;
		q3 = cos*p.q3 + sin*q3;
	}

	/**	negates all the elements of the quat: q and -q represent the same rotation.
	*/
	inline void negate() {
		q0 = -q0;
		q1 = -q1;
		q2 = -q2;
		q3 = -q3;
	}

	/** Test if the quaterion is the identity rotation.
	*/
	inline bool isIdentityTransformation() const {
		return q0 == Math::abs(REAL_ONE) && !REAL_AUR(q1) && !REAL_AUR(q2) && !REAL_AUR(q3);
	}

	/**	Assignment operator.
	*/
	inline Quat& operator = (const Quat& q) {
		q0 = q.q0;	q1 = q.q1;	q2 = q.q2;	q3 = q.q3;
		return *this;
	}

	/** Implicitly extends vector by a 0 q0 element.
	*/
	inline Quat& operator = (const Vec3& v) {
		q0 = REAL_ZERO;
		q1 = v.v1;
		q2 = v.v2;
		q3 = v.v3;
		return *this;
	}

	inline Quat operator - () const {
		return Quat(-q0, -q1, -q2, -q3);
	}

	inline Quat& operator *= (const Quat& q) {
		multiply(*this, q);
		return *this;
	}

	inline Quat& operator += (const Quat& q) {
		q0 += q.q0;
		q1 += q.q1;
		q2 += q.q2;
		q3 += q.q3;
		return *this;
	}

	inline Quat& operator -= (const Quat& q) {
		q0 -= q.q0;
		q1 -= q.q1;
		q2 -= q.q2;
		q3 -= q.q3;
		return *this;
	}

	inline Quat& operator *= (const Real s) {
		q0 *= s;
		q1 *= s;
		q2 *= s;
		q3 *= s;
		return *this;
	}

	/** quaternion multiplication this * p = (q0p0 - (this * p), q0p + p0q + (this x p)
	*/
	inline Quat operator * (const Quat& p) const {
		return
			Quat(
				q0*p.q0 - (q1*p.q1 + q2*p.q2 + q3*p.q3),
				q0*p.q1 + p.q0*q1 + (q2*p.q3 - p.q2*q3),
				q0*p.q2 + p.q0*q2 + (q3*p.q1 - p.q3*q1),
				q0*p.q3 + p.q0*q3 + (q1*p.q2 - p.q1*q2)
			);
	}
 
	/** quaternion addition
	*/
	inline Quat operator + (const Quat& q) const {
		return Quat(q.q0 + q0, q.q1 + q1, q.q2 + q2, q.q3 + q3);
	}
 
	/** quaternion subtraction
	*/
	inline Quat operator - (const Quat& q) const {
		return Quat(q.q0 - q0, q.q1 - q1, q.q2 - q2, q.q3 - q3);
	}
 
	/** quaternion conjugate
	*/
	inline Quat operator ! () const {
		return Quat(q0, -q1, -q2, -q3);
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*SYSTEM_QUAT_H_*/
