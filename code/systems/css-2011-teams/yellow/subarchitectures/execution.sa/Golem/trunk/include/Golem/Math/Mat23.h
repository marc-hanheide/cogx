/** @file Mat23.h
 * 
 * Mathematical routines.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_MATH_MAT23_H_
#define _GOLEM_MATH_MAT23_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Mat22.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Homogeneous representation of SE(2) rigid body transformations.
*/
class Mat23 {
public:
	/** rotation matrix	*/
	Mat22 R;
	/** translation	*/
	Vec2 p;

	/** Default constructor does not do any initialisation.
	*/
	inline Mat23() {
	}

	/** Creates matrix from rotation matrix and translation vector
	*/
	inline Mat23(const Mat22& R, const Vec2& p) : R(R), p(p) {
	}

	/** Creates matrix from rotation angle and translation vector
	*/
	inline Mat23(Real angle, const Vec2& trn) : R(angle), p(trn) {
	}

	/** Copy constructor.
	*/
	inline Mat23(const Mat23& m) {
		R = m.R;	p = m.p;
	}

	/** set the matrix given a column matrix
	*/
	inline void setColumn33(const F32 m[]) {
		R.setColumn33(m);
		p.v1 = (Real)m[6];
		p.v2 = (Real)m[7];
	}

	/** set the matrix given a column matrix
	*/
	inline void setColumn33(const F64 m[]) {
		R.setColumn33(m);
		p.v1 = (Real)m[6];
		p.v2 = (Real)m[7];
	}

	/** retrieve the matrix in a column format
	*/
	inline void getColumn33(F32 m[]) const {
		R.getColumn33(m);
		m[6] = (F32)p.v1;
		m[7] = (F32)p.v2;
		m[2] = m[5] = F32(REAL_ZERO);
		m[8] = F32(REAL_ONE);
	}

	/** retrieve the matrix in a column format
	*/
	inline void getColumn33(F64 m[]) const {
		R.getColumn33(m);
		m[6] = (F64)p.v1;
		m[7] = (F64)p.v2;
		m[2] = m[5] = F64(REAL_ZERO);
		m[8] = F64(REAL_ONE);
	}

	/** set the matrix given a row matrix.
	*/
	inline void setRow33(const F32 m[]) {
		R.setRow33(m);
		p.v1 = (Real)m[2];
		p.v2 = (Real)m[5];
	}

	/** set the matrix given a row matrix.
	*/
	inline void setRow33(const F64 m[]) {
		R.setRow33(m);
		p.v1 = (Real)m[2];
		p.v2 = (Real)m[5];
	}

	/** retrieve the matrix in a row format.
	*/
	inline void getRow33(F32 m[]) const {
		R.getRow33(m);
		m[2] = (F32)p.v1;
		m[5] = (F32)p.v2;
		m[6] = m[7] = F32(REAL_ZERO);
		m[8] = F32(REAL_ONE);
	}

	/** retrieve the matrix in a row format.
	*/
	inline void getRow33(F64 m[]) const {
		R.getRow33(m);
		m[2] = (F64)p.v1;
		m[5] = (F64)p.v2;
		m[6] = m[7] = F64(REAL_ZERO);
		m[8] = F64(REAL_ONE);
	}

	inline void setZero() {
		R.setZero();
		p.setZero();
	}

	inline void setId() {
		R.setId();
		p.setZero();
	}

	/** Creates matrix from rotation angle and translation
	*/
	inline void fromAngleTranslation(Real angle, const Vec2& p) {
		this->R.fromAngle(angle);
		this->p = p;
	}

	/** Creates rotation angle and translation from matrix
	*/
	inline void toAngleTranslation(Real& angle, Vec2& p) const {
		this->R.toAngle(angle);
		p = this->p;
	}

	/** Returns true for identity matrix
	*/
	inline bool isIdentity() const {
		return R.isIdentity() && p.isZero();
	}

	/** Returns true if all elems are finite.
	*/
	inline bool isFinite() const {
		return R.isFinite() && p.isFinite();
	}

	/** returns true if this and arg's elems are within epsilon of each other.
	*/
	inline bool equals(const Mat23& m, Real epsilon) const {
		return p.equals(m.p, epsilon) && R.equals(m.R, epsilon);
	}

	/** this = inverse(m): [ inv(R) , inv(R) * -p ], for orthonormal m: [ RT , RT * -p ].
	*/
	inline void setInverseRT(const Mat23& m) {
		R.setTransposed(m.R);
		p.multiply(-REAL_ONE, m.p);
		R.multiply(p, p); 
	}

	/** a = this * b
	*/
	inline void multiply(Vec2& a, const Vec2& b) const {
		// a = R * b + p;
		R.multiply(a, b);
		a.add(a, p);
	}

	/** a = inverse(this) * b, (assumes R is rotation matrix)
	*/
	inline void multiplyByInverseRT(Vec2& a, const Vec2& b) const {
		// b = R * a + p => a = RT * b - RT * p = RT * (b - p) = R^-1 * (b - p)
		a.subtract(b, p);
		R.multiplyByTranspose(a, a);
	}

	/** this = a * b: [aR, ap] * [bR, bp] = [aR * bR, aR * bp + ap].
	*/
	inline void multiply(const Mat23& a, const Mat23& b) {
		Vec2 tmp;

		a.R.multiply(tmp, b.p);
		p.add(tmp, a.p);
		R.multiply(a.R, b.R);
	}

	/**	Assignment operator.
	*/
	inline const Mat23& operator = (const Mat23 &m) {
		R = m.R;	p = m.p;
		return *this;
	}

	/** operator wrapper for multiply
	*/
	inline Vec2 operator * (const Vec2& a) const {
		Vec2 tmp;
		multiply(tmp, a);
		return tmp;
	}

	/** operator wrapper for multiply
	*/
	inline Mat23 operator * (const Mat23& b) const {
		Mat23 a;
		a.multiply(*this, b);
		return a;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_MAT23_H_*/
