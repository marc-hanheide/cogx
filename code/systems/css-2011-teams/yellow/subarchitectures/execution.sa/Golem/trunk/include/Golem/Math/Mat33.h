/** @file Mat33.h
 * 
 * Mathematical routines.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_MATH_MAT33_H_
#define _GOLEM_MATH_MAT33_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Vec3.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class Quat;
class Mat33;

extern void QuatToMat33(Mat33& m, const Quat& q);
extern void Mat33ToQuat(Quat& q, const Mat33& m);

/** Matrix representation of SO(3) group of rotations.
*/
class Mat33 {
public:
	/** matrix elements */
	union {
		struct {
			Real m11, m12, m13;
			Real m21, m22, m23;
			Real m31, m32, m33;		
		};
		Real m[3][3];
	};

	/** Default constructor does not do any initialisation.
	*/
	inline Mat33() {}

	/** Creates matrix from row vectors.
	*/
	inline Mat33(const Vec3 &row1, const Vec3 &row2, const Vec3 &row3) {
		m11 = row1.v1;  m12 = row1.v2;  m13 = row1.v3;
		m21 = row2.v1;  m22 = row2.v2;  m23 = row2.v3;
		m31 = row3.v1;  m32 = row3.v2;  m33 = row3.v3;
	}

	/** Creates SO(3) matrix from rotation around axis for normalised axis.
	*/
	inline Mat33(Real angle, const Vec3& axis) {
		fromAngleAxis(angle, axis);
	}

	/** Creates from quaternion.
	*	@param	q	quaterion to extract rotation matrix from.
	*/
	inline Mat33(const Quat &q) {
		fromQuat(q);
	}

	/** this = m
	*/
	inline void  set(const Mat33 &m) {
		*this = m;
	}

	inline void setRow33(const F32 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[1];	m13 = (Real)m[2];
		m21 = (Real)m[3];	m22 = (Real)m[4];	m23 = (Real)m[5];
		m31 = (Real)m[6];	m32 = (Real)m[7];	m33 = (Real)m[8];
	}

	inline void setRow33(const F64 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[1];	m13 = (Real)m[2];
		m21 = (Real)m[3];	m22 = (Real)m[4];	m23 = (Real)m[5];
		m31 = (Real)m[6];	m32 = (Real)m[7];	m33 = (Real)m[8];
	}

	inline void setColumn33(const F32 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[3];	m13 = (Real)m[6];
		m21 = (Real)m[1];	m22 = (Real)m[4];	m23 = (Real)m[7];
		m31 = (Real)m[2];	m32 = (Real)m[5];	m33 = (Real)m[8];
	}

	inline void setColumn33(const F64 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[3];	m13 = (Real)m[6];
		m21 = (Real)m[1];	m22 = (Real)m[4];	m23 = (Real)m[7];
		m31 = (Real)m[2];	m32 = (Real)m[5];	m33 = (Real)m[8];
	}

	inline void getRow33(F32 m[]) const {
		m[0] = (F32)m11;		m[1] = (F32)m12;		m[2] = (F32)m13;
		m[3] = (F32)m21;		m[4] = (F32)m22;		m[5] = (F32)m23;
		m[6] = (F32)m31;		m[7] = (F32)m32;		m[8] = (F32)m33;
	}

	inline void getRow33(F64 m[]) const {
		m[0] = (F64)m11;		m[1] = (F64)m12;		m[2] = (F64)m13;
		m[3] = (F64)m21;		m[4] = (F64)m22;		m[5] = (F64)m23;
		m[6] = (F64)m31;		m[7] = (F64)m32;		m[8] = (F64)m33;
	}

	inline void getColumn33(F32 m[]) const {
		m[0] = (F32)m11;		m[3] = (F32)m12;		m[6] = (F32)m13;
		m[1] = (F32)m21;		m[4] = (F32)m22;		m[7] = (F32)m23;
		m[2] = (F32)m31;		m[5] = (F32)m32;		m[8] = (F32)m33;
	}

	inline void getColumn33(F64 m[]) const {
		m[0] = (F64)m11;		m[3] = (F64)m12;		m[6] = (F64)m13;
		m[1] = (F64)m21;		m[4] = (F64)m22;		m[7] = (F64)m23;
		m[2] = (F64)m31;		m[5] = (F64)m32;		m[8] = (F64)m33;
	}

	//for loose 4-padded data.
	inline void setRow44(const F32 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[1];	m13 = (Real)m[2];
		m21 = (Real)m[4];	m22 = (Real)m[5];	m23 = (Real)m[6];
		m31 = (Real)m[8];	m32 = (Real)m[9];	m33 = (Real)m[10];
	}

	inline void setRow44(const F64 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[1];	m13 = (Real)m[2];
		m21 = (Real)m[4];	m22 = (Real)m[5];	m23 = (Real)m[6];
		m31 = (Real)m[8];	m32 = (Real)m[9];	m33 = (Real)m[10];
	}

	inline void setColumn44(const F32 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[4];	m13 = (Real)m[8];
		m21 = (Real)m[1];	m22 = (Real)m[5];	m23 = (Real)m[9];
		m31 = (Real)m[2];	m32 = (Real)m[6];	m33 = (Real)m[10];
	}

	inline void setColumn44(const F64 m[]) {
		m11 = (Real)m[0];	m12 = (Real)m[4];	m13 = (Real)m[8];
		m21 = (Real)m[1];	m22 = (Real)m[5];	m23 = (Real)m[9];
		m31 = (Real)m[2];	m32 = (Real)m[6];	m33 = (Real)m[10];
	}

	inline void getRow44(F32 m[]) const {
		m[0] = (F32)m11;		m[1] = (F32)m12;		m[2] = (F32)m13;
		m[4] = (F32)m21;		m[5] = (F32)m22;		m[6] = (F32)m23;
		m[8] = (F32)m31;		m[9] = (F32)m32;		m[10]= (F32)m33;
	}

	inline void getRow44(F64 m[]) const {
		m[0] = (F64)m11;		m[1] = (F64)m12;		m[2] = (F64)m13;
		m[4] = (F64)m21;		m[5] = (F64)m22;		m[6] = (F64)m23;
		m[8] = (F64)m31;		m[9] = (F64)m32;		m[10]= (F64)m33;
	}

	inline void getColumn44(F32 m[]) const {
		m[0] = (F32)m11;		m[4] = (F32)m12;		m[8] = (F32)m13;
		m[1] = (F32)m21;		m[5] = (F32)m22;		m[9] = (F32)m23;
		m[2] = (F32)m31;		m[6] = (F32)m32;		m[10]= (F32)m33;
	}

	inline void getColumn44(F64 m[]) const {
		m[0] = (F64)m11;		m[4] = (F64)m12;		m[8] = (F64)m13;
		m[1] = (F64)m21;		m[5] = (F64)m22;		m[9] = (F64)m23;
		m[2] = (F64)m31;		m[6] = (F64)m32;		m[10]= (F64)m33;
	}

	inline void setRow(size_t row, const Vec3& v) {
		m[row][0] = v.v1;		m[row][1] = v.v2;		m[row][2] = v.v3;
	}

	inline void setColumn(size_t col, const Vec3& v) {
		m[0][col] = v.v1;		m[1][col] = v.v2;		m[2][col] = v.v3;
	}

	inline void getRow(size_t row, Vec3& v) const {
		v.v1 = m[row][0];		v.v2 = m[row][1];		v.v3 = m[row][2];
	}

	inline void getColumn(size_t col, Vec3& v) const {
		v.v1 = m[0][col];		v.v2 = m[1][col];		v.v3 = m[2][col];
	}

	inline Vec3 getRow(size_t row) const {
		return Vec3(m[row][0], m[row][1], m[row][2]);
	}

	inline Vec3 getColumn(size_t col) const {
		return Vec3(m[0][col], m[1][col], m[2][col]);
	}

	inline Real& operator () (size_t row, size_t col) {
		return m[row][col];
	}

	/** returns true for exact identity matrix
	*/
	inline bool isIdentity() const {
		return
			m11 == REAL_ONE && Math::isZero(m12) && Math::isZero(m13) &&
			Math::isZero(m21) && m22 == REAL_ONE && Math::isZero(m23) &&
			Math::isZero(m31) && Math::isZero(m32) && m33 == REAL_ONE;
	}

	/** returns true for exact zero matrix
	*/
	inline bool isZero() const {
		return
			Math::isZero(m11) && Math::isZero(m12) && Math::isZero(m13) &&
			Math::isZero(m21) && Math::isZero(m22) && Math::isZero(m23) &&
			Math::isZero(m31) && Math::isZero(m32) && Math::isZero(m33);
	}

	/** returns true if all elems are finite
	*/
	inline bool isFinite() const {
		return
			Math::isFinite(m11) && Math::isFinite(m12) && Math::isFinite(m13) &&
			Math::isFinite(m21) && Math::isFinite(m22) && Math::isFinite(m23) &&
			Math::isFinite(m31) && Math::isFinite(m32) && Math::isFinite(m33);
	}

	/** returns true if this and arg's elems are within epsilon of each other.
	*/
	inline bool equals(const Mat33& m, Real epsilon) const {
		return
			Math::equals(m11, m.m11, epsilon) && Math::equals(m12, m.m12, epsilon) && Math::equals(m13, m.m13, epsilon) &&
			Math::equals(m21, m.m21, epsilon) && Math::equals(m22, m.m22, epsilon) && Math::equals(m23, m.m23, epsilon) &&
			Math::equals(m31, m.m31, epsilon) && Math::equals(m32, m.m32, epsilon) && Math::equals(m33, m.m33, epsilon);
	}

	/** sets this matrix to the zero matrix.
	*/
	inline void setZero() {
		m11 = REAL_ZERO;			m12 = REAL_ZERO;			m13 = REAL_ZERO;
		m21 = REAL_ZERO;			m22 = REAL_ZERO;			m23 = REAL_ZERO;
		m31 = REAL_ZERO;			m32 = REAL_ZERO;			m33 = REAL_ZERO;
	}

	/** sets this matrix to the identity matrix.
	*/
	inline void setId() {
		m11 = REAL_ONE;			m12 = REAL_ZERO;			m13 = REAL_ZERO;
		m21 = REAL_ZERO;			m22 = REAL_ONE;			m23 = REAL_ZERO;
		m31 = REAL_ZERO;			m32 = REAL_ZERO;			m33 = REAL_ONE;
	}

	/** this = -this
	*/
	inline void setNegative() {
		m11 = -m11;			m12 = -m12;			m13 = -m13;
		m21 = -m21;			m22 = -m22;			m23 = -m23;
		m31 = -m31;			m32 = -m32;			m33 = -m33;
	}

	/** sets this matrix to the diagonal matrix.
	*/
	inline void setDiagonal(const Vec3 &v) {
		m11 = v.v1;				m12 = REAL_ZERO;			m13 = REAL_ZERO;
		m21 = REAL_ZERO;			m22 = v.v2;				m23 = REAL_ZERO;
		m31 = REAL_ZERO;			m32 = REAL_ZERO;			m33 = v.v3;
	}

	/** Finds axis of SO(3) matrix: R * axis = axis
	Returns false for degenerate case, i.e. if R == Id
	*/
	inline bool getAxis(Vec3 &axis) const {
		if (isIdentity()) {
			axis.setZero();
			return false;
		}

		// TODO: solve system of linear equations R * axis = axis

		return true;
	}

	/** Creates skew-symmetric so(3) matrix from axis (wedge operator ^).
	*/
	inline void axisToSkew(const Vec3 &axis) {
		m11 = REAL_ZERO;			m12 = -axis.v3;			m13 = axis.v2;
		m21 = axis.v3;			m22 = REAL_ZERO;			m23 = -axis.v1;
		m31 = -axis.v2;			m32 = axis.v1;			m33 = REAL_ZERO;
	}

	/** Creates axis from skew-symmetric so(3) matrix (vee operator v).
	*/
	inline void skewToAxis(Vec3 &axis) const {
		axis.v1 = -m23;
		axis.v2 =  m13;
		axis.v3 = -m12;
	}

	/** Creates SO(3) matrix from rotation around axis for normalised axis.
		Rodrigues' formula for exponential maps:
			exp(axis^ angle) = Id + axis^ sin(angle) + axis^ axis^ (1 - cos(angle)),
		where axis^ is so(3) matrix generated from axis.
	*/
	inline void fromAngleAxis(Real angle, const Vec3& axis) {
		Real s, c;
		Math::sinCos(angle, s, c);
		Real v = REAL_ONE - c, v1 = axis.v1*v, v2 = axis.v2*v, v3 = axis.v3*v;

		m11 = axis.v1*v1 + c;			m12 = axis.v1*v2 - axis.v3*s;	m13 = axis.v1*v3 + axis.v2*s;	
		m21 = axis.v2*v1 + axis.v3*s;	m22 = axis.v2*v2 + c;			m23 = axis.v2*v3 - axis.v1*s;	
		m31 = axis.v3*v1 - axis.v2*s;	m32 = axis.v3*v2 + axis.v1*s;	m33 = axis.v3*v3 + c;	
	}

	/** Returns rotation angle and axis of rotation of SO(3) matrix
	*/
	inline void toAngleAxis(Real& angle, Vec3& axis) const {
		angle = Math::acos(REAL_HALF * (trace() - REAL_ONE));
		Real s = REAL_TWO * Math::sin(angle);
		if (Math::abs(s) > REAL_EPS)
			axis.set((m32 - m23)/s, (m13 - m31)/s, (m21 - m12)/s);
		else
			axis.setZero();
	}

	inline void fromQuat(const Quat& q) {
		golem::QuatToMat33(*this, q);
	}

	inline void toQuat(Quat& q) const {
		golem::Mat33ToQuat(q, *this);
	}

	/** Creates SO(3) matrix from the specified coordinate frame axes.
	*/
	inline void fromAxes(const Vec3& xb, const Vec3& yb, const Vec3& zb) {
		Vec3 xa(REAL_ONE, REAL_ZERO, REAL_ZERO);
		Vec3 ya(REAL_ZERO, REAL_ONE, REAL_ZERO);
		Vec3 za(REAL_ZERO, REAL_ZERO, REAL_ONE);
		
		m11 = xa.dot(xb);
		m21 = ya.dot(xb);
		m31 = za.dot(xb);
		
		m12 = xa.dot(yb);
		m22 = ya.dot(yb);
		m32 = za.dot(yb);
		
		m13 = xa.dot(zb);
		m23 = ya.dot(zb);
		m33 = za.dot(zb);
	}

	/** this = rotation matrix around X axis
	*/
	inline void rotX(Real angle) {
		Real s, c;
		Math::sinCos(angle,	s, c);

		m11 = REAL_ONE;	m12 = REAL_ZERO;	m13 = REAL_ZERO;
		m21 = REAL_ZERO;	m22 = c;			m23 = -s;
		m31 = REAL_ZERO;	m32 = s;			m33 = c;
	}

	/** this = rotation matrix around Y axis
	*/
	inline void rotY(Real angle) {
		Real s, c;
		Math::sinCos(angle,	s, c);

		m11 = c;			m12 = REAL_ZERO;	m13 = s;
		m21 = REAL_ZERO;	m22 = REAL_ONE;	m23 = REAL_ZERO;
		m31 = -s;		m32 = REAL_ZERO;	m33 = c;
	}

	/** this = rotation matrix around Z axis
	*/
	inline void rotZ(Real angle) {
		Real s, c;
		Math::sinCos(angle,	s, c);

		m11 = c;			m12 = -s;		m13 = REAL_ZERO;
		m21 = s;			m22 = c;			m23 = REAL_ZERO;
		m31 = REAL_ZERO;	m32 = REAL_ZERO;	m33 = REAL_ONE;
	}

	/** Rotation matrix from Euler angles - rotation about: X (roll) -> Y(pitch) -> Z(yaw).
	*   roll, pitch, yaw  in <-PI/2, PI/2>
	*/
	inline void fromEuler(Real roll, Real pitch, Real yaw) {
		Real sg, cg;
		Math::sinCos(roll, sg, cg);
		Real sb, cb;
		Math::sinCos(pitch, sb, cb);
		Real sa, ca;
		Math::sinCos(yaw, sa, ca);

		m11 = ca*cb;		m12 = ca*sb*sg - sa*cg;	m13 = ca*sb*cg + sa*sg;
		m21 = sa*cb;		m22 = sa*sb*sg + ca*cg;	m23 = sa*sb*cg - ca*sg;
		m31 = -sb;		m32 = cb*sg;				m33 = cb*cg;
	}

	/** Rotation matrix to Euler angles - rotation about: X (roll) -> Y(pitch) -> Z(yaw).
	*/
	inline void toEuler(Real &roll, Real &pitch, Real &yaw) const {
		roll = Math::atan2(m32, m33);
		pitch = Math::atan2(-m31, Math::sqrt(m32*m32 + m33*m33));
		yaw = Math::atan2(m21, m11);
	}

	/** returns trace
	*/
	inline Real trace() const {
		return m11 + m22 + m33;
	}

	/** returns determinant
	*/
	inline Real determinant() const {
		return
			m11*m22*m33 + m12*m23*m31 + m13*m21*m32 -
			m13*m22*m31 - m12*m21*m33 - m11*m23*m32;
	}

	/** this = inverse(m).
	@return false if singular (i.e. if no inverse exists)
	*/
	inline bool setInverse(const Mat33& m) {
		Real temp11 = m.m22*m.m33 - m.m23*m.m32;
		Real temp12 = m.m13*m.m32 - m.m12*m.m33;
		Real temp13 = m.m12*m.m23 - m.m13*m.m22;
		
		Real temp21 = m.m23*m.m31 - m.m21*m.m33;
		Real temp22 = m.m11*m.m33 - m.m13*m.m31;
		Real temp23 = m.m13*m.m21 - m.m11*m.m23;

		Real temp31 = m.m21*m.m32 - m.m22*m.m31;
		Real temp32 = m.m12*m.m31 - m.m11*m.m32;
		Real temp33 = m.m11*m.m22 - m.m12*m.m21;
		
		Real det = temp11*m.m11 + temp12*m.m21 + temp13*m.m31;
		
		if (Math::abs(det) < REAL_EPS)
			return false;
		
		det = REAL_ONE / det;
		
		m11 = temp11*det;		m12 = temp12*det;		m13 = temp13*det;
		m21 = temp21*det;		m22 = temp22*det;		m23 = temp23*det;
		m31 = temp31*det;		m32 = temp32*det;		m33 = temp33*det;
		
		return true;
	}

	/** this = transpose(m), (inverse if  R is rotation matrix)
	*/
	inline void setTransposed(const Mat33& m) {
		if (this != &m) {
			m11 = m.m11;		m12 = m.m21;		m13 = m.m31;
			m21 = m.m12;		m22 = m.m22;		m23 = m.m32;
			m31 = m.m13;		m32 = m.m23;		m33 = m.m33;
		}
		else
			setTransposed();
	}

	/** this = transpose(this), (inverse if  R is rotation matrix)
	*/
	inline void setTransposed() {
		std::swap(m12, m21);
		std::swap(m23, m32);
		std::swap(m13, m31);
	}

	/** this = this * [ v.v1 0 0; 0 v.v2 0; 0 0 v.v3].
	*/
	inline void multiplyDiagonal(const Vec3& v) {
		m11 *= v.v1;		m12 *= v.v2;		m13 *= v.v3;
		m21 *= v.v1;		m22 *= v.v2;		m23 *= v.v3;
		m31 *= v.v1;		m32 *= v.v2;		m33 *= v.v3;
	}

	/** this = transpose(this) * [ v.v1 0 0; 0 v.v2 0; 0 0 v.v3].
	*/
	inline void multiplyDiagonalTranspose(const Vec3& v) {
		m11 *= v.v1;		m12 *= v.v1;		m13 *= v.v1;
		m21 *= v.v2;		m22 *= v.v2;		m23 *= v.v2;
		m31 *= v.v3;		m32 *= v.v3;		m33 *= v.v3;
	}

	/** m = this * [ v.v1 0 0; 0 v.v2 0; 0 0 v.v3];
	*/
	inline void multiplyDiagonal(Mat33& m, const Vec3& v) const {
		m.m11 = m11 * v.v1;		m.m12 = m12 * v.v2;		m.m13 = m13 * v.v3;
		m.m21 = m21 * v.v1;		m.m22 = m22 * v.v2;		m.m23 = m23 * v.v3;
		m.m31 = m31 * v.v1;		m.m32 = m32 * v.v2;		m.m33 = m33 * v.v3;
	}

	/** m = transpose(this) * [ v.v1 0 0; 0 v.v2 0; 0 0 v.v3];
	*/
	inline void multiplyDiagonalTranspose(Mat33& m, const Vec3& v) const {
		m.m11 = m11 * v.v1;		m.m12 = m21 * v.v2;		m.m13 = m31 * v.v3;
		m.m21 = m12 * v.v1;		m.m22 = m22 * v.v2;		m.m23 = m32 * v.v3;
		m.m31 = m13 * v.v1;		m.m32 = m23 * v.v2;		m.m33 = m33 * v.v3;
	}

	/** a = this * b
	*/
	inline void multiply(Vec3& a, const Vec3& b) const {
		Real v1 = m11 * b.v1 + m12 * b.v2 + m13 * b.v3;
		Real v2 = m21 * b.v1 + m22 * b.v2 + m23 * b.v3;
		Real v3 = m31 * b.v1 + m32 * b.v2 + m33 * b.v3;

		a.v1 = v1;
		a.v2 = v2;
		a.v3 = v3;	
	}

	/** a = transpose(this) * b
	*/
	inline void multiplyByTranspose(Vec3& a, const Vec3& b) const {
		Real v1 = m11 * b.v1 + m21 * b.v2 + m31 * b.v3;
		Real v2 = m12 * b.v1 + m22 * b.v2 + m32 * b.v3;
		Real v3 = m13 * b.v1 + m23 * b.v2 + m33 * b.v3;

		a.v1 = v1;
		a.v2 = v2;
		a.v3 = v3;
	}

	/** this = a + b
	*/
	inline void  add(const Mat33& a, const Mat33& b) {
		m11 = a.m11 + b.m11;		m12 = a.m12 + b.m12;		m13 = a.m13 + b.m13;
		m21 = a.m21 + b.m21;		m22 = a.m22 + b.m22;		m23 = a.m23 + b.m23;
		m31 = a.m31 + b.m31;		m32 = a.m32 + b.m32;		m33 = a.m33 + b.m33;
	}

	/** this = a - b
	*/
	inline void  subtract(const Mat33& a, const Mat33& b) {
		m11 = a.m11 - b.m11;		m12 = a.m12 - b.m12;		m13 = a.m13 - b.m13;
		m21 = a.m21 - b.m21;		m22 = a.m22 - b.m22;		m23 = a.m23 - b.m23;
		m31 = a.m31 - b.m31;		m32 = a.m32 - b.m32;		m33 = a.m33 - b.m33;
	}

	/** this = s * m;
	*/
	inline void multiply(Real s, const Mat33& m) {
		m11 = m.m11 * s;		m12 = m.m12 * s;		m13 = m.m13 * s;
		m21 = m.m21 * s;		m22 = m.m22 * s;		m23 = m.m23 * s;
		m31 = m.m31 * s;		m32 = m.m32 * s;		m33 = m.m33 * s;
	}

	/** this = a * b
	*/
	inline void multiply(const Mat33& a, const Mat33& b) {
		Real a11 = a.m11 * b.m11 + a.m12 * b.m21 + a.m13 * b.m31;
		Real a12 = a.m11 * b.m12 + a.m12 * b.m22 + a.m13 * b.m32;
		Real a13 = a.m11 * b.m13 + a.m12 * b.m23 + a.m13 * b.m33;

		Real a21 = a.m21 * b.m11 + a.m22 * b.m21 + a.m23 * b.m31;
		Real a22 = a.m21 * b.m12 + a.m22 * b.m22 + a.m23 * b.m32;
		Real a23 = a.m21 * b.m13 + a.m22 * b.m23 + a.m23 * b.m33;

		Real a31 = a.m31 * b.m11 + a.m32 * b.m21 + a.m33 * b.m31;
		Real a32 = a.m31 * b.m12 + a.m32 * b.m22 + a.m33 * b.m32;
		Real a33 = a.m31 * b.m13 + a.m32 * b.m23 + a.m33 * b.m33;

		m11 = a11;		m12 = a12;		m13 = a13;
		m21 = a21;		m22 = a22;		m23 = a23;
		m31 = a31;		m32 = a32;		m33 = a33;
	}

	inline Mat33& operator += (const Mat33 &m) {
		m11 += m.m11;		m12 += m.m12;		m13 += m.m13;
		m21 += m.m21;		m22 += m.m22;		m23 += m.m23;
		m31 += m.m31;		m32 += m.m32;		m33 += m.m33;
		return *this;
	}

	inline Mat33& operator -= (const Mat33 &m) {
		m11 -= m.m11;		m12 -= m.m12;		m13 -= m.m13;
		m21 -= m.m21;		m22 -= m.m22;		m23 -= m.m23;
		m31 -= m.m31;		m32 -= m.m32;		m33 -= m.m33;
		return *this;
	}

	inline Mat33& operator *= (const Mat33& m) {
		multiply(*this, m);
		return *this;
	}

	inline Mat33& operator *= (Real s) {
		m11 *= s;			m12 *= s;			m13 *= s;
		m21 *= s;			m22 *= s;			m23 *= s;
		m31 *= s;			m32 *= s;			m33 *= s;
		return *this;
	}

	inline Mat33& operator /= (Real s) {
		s = REAL_ONE / s;
		m11 *= s;			m12 *= s;			m13 *= s;
		m21 *= s;			m22 *= s;			m23 *= s;
		m31 *= s;			m32 *= s;			m33 *= s;
		return *this;
	}

	/** returns transpose(this)*a
	*/
	inline Vec3 operator % (const Vec3& a) const {
		Vec3 tmp;
		multiplyByTranspose(tmp, a);
		return tmp;
	}

	/** matrix vector product
	*/
	inline Vec3 operator * (const Vec3& v) const {
		Vec3 tmp;
		multiply(tmp, v);
		return tmp;
	}

	/** matrix difference
	*/
	inline Mat33 operator - (const Mat33& m) const {
		Mat33 tmp;
		tmp.subtract(*this, m);
		return tmp;
	}

	/** matrix addition
	*/
	inline Mat33 operator + (const Mat33& m) const {
		Mat33 tmp;
		tmp.add(*this, m);
		return tmp;
	}

	/** matrix product
	*/
	inline Mat33 operator * (const Mat33& m) const {
		Mat33 tmp;
		tmp.multiply(*this, m);
		return tmp;
	}

	/** matrix scalar product
	*/
	inline Mat33 operator * (Real s) const {
		Mat33 tmp;
		tmp.multiply(s, *this);
		return tmp;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_MAT33_H_*/
