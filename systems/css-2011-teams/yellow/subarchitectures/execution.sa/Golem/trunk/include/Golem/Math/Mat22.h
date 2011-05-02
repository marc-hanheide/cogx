/** @file Mat22.h
 * 
 * Mathematical routines.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_MATH_MAT22_H_
#define _GOLEM_MATH_MAT22_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Vec2.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Matrix representation of SO(2) group of rotations.
*/
class Mat22 {
public:
	/** matrix elements */
	union {
		struct {
			Real m11, m12;
			Real m21, m22;
		};
		Real m[2][2];
	};

	/** Default constructor does not do any initialisation.
	*/
	inline Mat22() {}

	/** Creates matrix from row vectors.
	*/
	inline Mat22(const Vec2 &row1, const Vec2 &row2) {
		m11 = row1.v1;  m12 = row1.v2;
		m21 = row2.v1;  m22 = row2.v2;
	}

	/** Creates SO(2) matrix from rotation angle.
	*/
	inline Mat22(Real angle) {
		fromAngle(angle);
	}

	/** this = m
	*/
	inline void  set(const Mat22 &m) {
		*this = m;
	}

	inline void setRow22(const F32 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[1];
		m21 = (Real)m[2];	m22 = (Real)m[3];
	}

	inline void setRow22(const F64 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[1];
		m21 = (Real)m[2];	m22 = (Real)m[3];
	}

	inline void setColumn22(const F32 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[2];
		m21 = (Real)m[1];	m22 = (Real)m[3];
	}

	inline void setColumn22(const F64 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[2];
		m21 = (Real)m[1];	m22 = (Real)m[3];
	}

	inline void getRow22(F32 m[]) const {
		m[0] = (F32)m11;		m[1] = (F32)m12;
		m[2] = (F32)m21;		m[3] = (F32)m22;
	}

	inline void getRow22(F64 m[]) const {
		m[0] = (F64)m11;		m[1] = (F64)m12;
		m[2] = (F64)m21;		m[3] = (F64)m22;
	}

	inline void getColumn22(F32 m[]) const {
		m[0] = (F32)m11;		m[2] = (F32)m12;
		m[1] = (F32)m21;		m[3] = (F32)m22;
	}

	inline void getColumn22(F64 m[]) const {
		m[0] = (F64)m11;		m[2] = (F64)m12;
		m[1] = (F64)m21;		m[3] = (F64)m22;
	}

	//for loose 3-padded data.
	inline void setRow33(const F32 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[1];
		m21 = (Real)m[3];	m22 = (Real)m[4];
	}

	inline void setRow33(const F64 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[1];
		m21 = (Real)m[3];	m22 = (Real)m[4];
	}

	inline void setColumn33(const F32 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[3];
		m21 = (Real)m[1];	m22 = (Real)m[4];
	}

	inline void setColumn33(const F64 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[3];
		m21 = (Real)m[1];	m22 = (Real)m[4];
	}

	inline void getRow33(F32 m[]) const {
		m[0] = (F32)m11;		m[1] = (F32)m12;
		m[2] = (F32)m21;		m[3] = (F32)m22;
	}

	inline void getRow33(F64 m[]) const {
		m[0] = (F64)m11;		m[1] = (F64)m12;
		m[2] = (F64)m21;		m[3] = (F64)m22;
	}

	inline void getColumn33(F32 m[]) const {
		m[0] = (F32)m11;		m[3] = (F32)m12;
		m[1] = (F32)m21;		m[4] = (F32)m22;
	}

	inline void getColumn33(F64 m[]) const {
		m[0] = (F64)m11;		m[3] = (F64)m12;
		m[1] = (F64)m21;		m[4] = (F64)m22;
	}

	inline void setRow(size_t row, const Vec2& v) {
		m[row][0] = v.v1;		m[row][1] = v.v2;
	}

	inline void setColumn(size_t col, const Vec2& v) {
		m[0][col] = v.v1;		m[1][col] = v.v2;
	}

	inline void getRow(size_t row, Vec2& v) const {
		v.v1 = m[row][0];		v.v2 = m[row][1];
	}

	inline void getColumn(size_t col, Vec2& v) const {
		v.v1 = m[0][col];		v.v2 = m[1][col];
	}

	inline Vec2 getRow(size_t row) const {
		return Vec2(m[row][0], m[row][1]);
	}

	inline Vec2 getColumn(size_t col) const {
		return Vec2(m[0][col], m[1][col]);
	}

	inline Real& operator () (size_t row, size_t col) {
		return m[row][col];
	}

	/** returns true for exact identity matrix
	*/
	inline bool isIdentity() const {
		return
			m11 == REAL_ONE && Math::isZero(m12) &&
			Math::isZero(m21) && m22 == REAL_ONE;
	}

	/** returns true for exact zero matrix
	*/
	inline bool isZero() const {
		return
			Math::isZero(m11) && Math::isZero(m12) &&
			Math::isZero(m21) && Math::isZero(m22);
	}

	/** returns true if all elems are finite
	*/
	inline bool isFinite() const {
		return
			Math::isFinite(m11) && Math::isFinite(m12) &&
			Math::isFinite(m21) && Math::isFinite(m22);
	}

	/** returns true if this and arg's elems are within epsilon of each other.
	*/
	inline bool equals(const Mat22& m, Real epsilon) const {
		return
			Math::equals(m11, m.m11, epsilon) && Math::equals(m12, m.m12, epsilon) &&
			Math::equals(m21, m.m21, epsilon) && Math::equals(m22, m.m22, epsilon);
	}

	/** sets this matrix to the zero matrix.
	*/
	inline void setZero() {
		m11 = REAL_ZERO;			m12 = REAL_ZERO;
		m21 = REAL_ZERO;			m22 = REAL_ZERO;
	}

	/** sets this matrix to the identity matrix.
	*/
	inline void setId() {
		m11 = REAL_ONE;			m12 = REAL_ZERO;
		m21 = REAL_ZERO;			m22 = REAL_ONE;
	}

	/** this = -this
	*/
	inline void setNegative() {
		m11 = -m11;			m12 = -m12;
		m21 = -m21;			m22 = -m22;
	}

	/** sets this matrix to the diagonal matrix.
	*/
	inline void setDiagonal(const Vec2 &v) {
		m11 = v.v1;				m12 = REAL_ZERO;
		m21 = REAL_ZERO;			m22 = v.v2;
	}

	/** Creates SO(2) matrix from rotation angle.
	*/
	inline void fromAngle(Real angle) {
		Real s, c;
		Math::sinCos(angle,	s, c);

		m11 = c;			m12 = -s;
		m21 = s;			m22 = c;
	}

	/** Returns rotation angle of SO(2) matrix
	*/
	inline void toAngle(Real& angle) const {
		angle = Math::atan2(m21, m11);
	}

	/** Creates SO(2) matrix from the specified coordinate frame axes.
	*/
	inline void fromAxes(const Vec2& xb, const Vec2& yb) {
		Vec2 xa(REAL_ONE, REAL_ZERO);
		Vec2 ya(REAL_ZERO, REAL_ONE);
		
		m11 = xa.dot(xb);
		m21 = ya.dot(xb);
		
		m12 = xa.dot(yb);
		m22 = ya.dot(yb);
	}

	/** returns trace
	*/
	inline Real trace() const {
		return m11 + m22;
	}

	/** returns determinant
	*/
	inline Real determinant() const {
		return
			m11*m22 - m12*m21;
	}

	/** this = transpose(m), (inverse if  R is rotation matrix)
	*/
	inline void setTransposed(const Mat22& m) {
		if (this != &m) {
			m11 = m.m11;		m12 = m.m21;
			m21 = m.m12;		m22 = m.m22;
		}
		else
			setTransposed();
	}

	/** this = transposed(this), (inverse if  R is rotation matrix)
	*/
	inline void setTransposed() {
		std::swap(m12, m21);
	}

	/** a = this * b
	*/
	inline void multiply(Vec2& a, const Vec2& b) const {
		Real v1 = m11 * b.v1 + m12 * b.v2;
		Real v2 = m21 * b.v1 + m22 * b.v2;

		a.v1 = v1;
		a.v2 = v2;
	}

	/** a = transpose(this) * b, (inverse if R is rotation matrix)
	*/
	inline void multiplyByTranspose(Vec2& a, const Vec2& b) const {
		Real v1 = m11 * b.v1 + m21 * b.v2;
		Real v2 = m12 * b.v1 + m22 * b.v2;

		a.v1 = v1;
		a.v2 = v2;
	}

	/** this = a + b
	*/
	inline void  add(const Mat22& a, const Mat22& b) {
		m11 = a.m11 + b.m11;		m12 = a.m12 + b.m12;
		m21 = a.m21 + b.m21;		m22 = a.m22 + b.m22;
	}

	/** this = a - b
	*/
	inline void  subtract(const Mat22& a, const Mat22& b) {
		m11 = a.m11 - b.m11;		m12 = a.m12 - b.m12;
		m21 = a.m21 - b.m21;		m22 = a.m22 - b.m22;
	}

	/** this = s * m;
	*/
	inline void multiply(Real s, const Mat22& m) {
		m11 = m.m11 * s;		m12 = m.m12 * s;
		m21 = m.m21 * s;		m22 = m.m22 * s;
	}

	/** this = a * b
	*/
	inline void multiply(const Mat22& a, const Mat22& b) {
		Real a11 = a.m11 * b.m11 + a.m12 * b.m21;
		Real a12 = a.m11 * b.m12 + a.m12 * b.m22;

		Real a21 = a.m21 * b.m11 + a.m22 * b.m21;
		Real a22 = a.m21 * b.m12 + a.m22 * b.m22;

		m11 = a11;		m12 = a12;
		m21 = a21;		m22 = a22;
	}

	inline Mat22& operator += (const Mat22 &m) {
		m11 += m.m11;		m12 += m.m12;
		m21 += m.m21;		m22 += m.m22;
		return *this;
	}

	inline Mat22& operator -= (const Mat22 &m) {
		m11 -= m.m11;		m12 -= m.m12;
		m21 -= m.m21;		m22 -= m.m22;
		return *this;
	}

	inline Mat22& operator *= (const Mat22& m) {
		multiply(*this, m);
		return *this;
	}

	inline Mat22& operator *= (Real s) {
		m11 *= s;			m12 *= s;
		m21 *= s;			m22 *= s;
		return *this;
	}

	inline Mat22& operator /= (Real s) {
		s = REAL_ONE / s;
		m11 *= s;			m12 *= s;
		m21 *= s;			m22 *= s;
		return *this;
	}

	/** matrix vector product
	*/
	inline Vec2 operator * (const Vec2& v) const {
		Vec2 tmp;
		multiply(tmp, v);
		return tmp;
	}

	/** matrix difference
	*/
	inline Mat22 operator - (const Mat22& m) const {
		Mat22 tmp;
		tmp.subtract(*this, m);
		return tmp;
	}

	/** matrix addition
	*/
	inline Mat22 operator + (const Mat22& m) const {
		Mat22 tmp;
		tmp.add(*this, m);
		return tmp;
	}

	/** matrix product
	*/
	inline Mat22 operator * (const Mat22& m) const {
		Mat22 tmp;
		tmp.multiply(*this, m);
		return tmp;
	}

	/** matrix scalar product
	*/
	inline Mat22 operator * (Real s) const {
		Mat22 tmp;
		tmp.multiply(s, *this);
		return tmp;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_MAT22_H_*/
