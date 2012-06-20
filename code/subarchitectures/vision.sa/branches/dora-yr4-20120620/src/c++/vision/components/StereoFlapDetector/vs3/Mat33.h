/** @file Mat33.h
 * 
 * Mathematical routines.
 * 
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk">The University Of Birmingham</A>
 *
 * @version 1.0
 *
 */

#ifndef SYSTEM_MAT33_H_
#define SYSTEM_MAT33_H_

//------------------------------------------------------------------------------

#include "Vec3.h"

//------------------------------------------------------------------------------

namespace msk {

class Quat;
class Mat33;

extern void QuatToMat33(Mat33& m, const Quat& q);
extern void Mat33ToQuat(Quat& q, const Mat33& m);

//------------------------------------------------------------------------------

/** Matrix representation of SO(3) group of rotations.
*/
class Mat33 {
public:
	struct Elements {
		/** matrix elements */
		Real _11, _12, _13;
		Real _21, _22, _23;
		Real _31, _32, _33;		
	};
	union {
		struct Elements _m;
		Real m[3][3];
	};

	/** Default constructor does not do any initialisation.
	*/
	inline Mat33() {}

	/** Creates matrix from row vectors.
	*/
	inline Mat33(const Vec3 &row1, const Vec3 &row2, const Vec3 &row3) {
		_m._11 = row1.v1;  _m._12 = row1.v2;  _m._13 = row1.v3;
		_m._21 = row2.v1;  _m._22 = row2.v2;  _m._23 = row2.v3;
		_m._31 = row3.v1;  _m._32 = row3.v2;  _m._33 = row3.v3;
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

	/** Copy constructor.
	*/
	inline Mat33(const Mat33& m) {
		_m = m._m;
	}

	/** this = m
	*/
	inline void  set(const Mat33 &m) {
		_m = m._m;
	}

	inline void setRow33(const F32 m[]) {
		_m._11 = (Real)m[0];	_m._12 = (Real)m[1];	_m._13 = (Real)m[2];
		_m._21 = (Real)m[3];	_m._22 = (Real)m[4];	_m._23 = (Real)m[5];
		_m._31 = (Real)m[6];	_m._32 = (Real)m[7];	_m._33 = (Real)m[8];
	}

	inline void setRow33(const F64 m[]) {
		_m._11 = (Real)m[0];	_m._12 = (Real)m[1];	_m._13 = (Real)m[2];
		_m._21 = (Real)m[3];	_m._22 = (Real)m[4];	_m._23 = (Real)m[5];
		_m._31 = (Real)m[6];	_m._32 = (Real)m[7];	_m._33 = (Real)m[8];
	}

	inline void setColumn33(const F32 m[]) {
		_m._11 = (Real)m[0];	_m._12 = (Real)m[3];	_m._13 = (Real)m[6];
		_m._21 = (Real)m[1];	_m._22 = (Real)m[4];	_m._23 = (Real)m[7];
		_m._31 = (Real)m[2];	_m._32 = (Real)m[5];	_m._33 = (Real)m[8];
	}

	inline void setColumn33(const F64 m[]) {
		_m._11 = (Real)m[0];	_m._12 = (Real)m[3];	_m._13 = (Real)m[6];
		_m._21 = (Real)m[1];	_m._22 = (Real)m[4];	_m._23 = (Real)m[7];
		_m._31 = (Real)m[2];	_m._32 = (Real)m[5];	_m._33 = (Real)m[8];
	}

	inline void getRow33(F32 m[]) const {
		m[0] = (F32)_m._11;		m[1] = (F32)_m._12;		m[2] = (F32)_m._13;
		m[3] = (F32)_m._21;		m[4] = (F32)_m._22;		m[5] = (F32)_m._23;
		m[6] = (F32)_m._31;		m[7] = (F32)_m._32;		m[8] = (F32)_m._33;
	}

	inline void getRow33(F64 m[]) const {
		m[0] = (F64)_m._11;		m[1] = (F64)_m._12;		m[2] = (F64)_m._13;
		m[3] = (F64)_m._21;		m[4] = (F64)_m._22;		m[5] = (F64)_m._23;
		m[6] = (F64)_m._31;		m[7] = (F64)_m._32;		m[8] = (F64)_m._33;
	}

	inline void getColumn33(F32 m[]) const {
		m[0] = (F32)_m._11;		m[3] = (F32)_m._12;		m[6] = (F32)_m._13;
		m[1] = (F32)_m._21;		m[4] = (F32)_m._22;		m[7] = (F32)_m._23;
		m[2] = (F32)_m._31;		m[5] = (F32)_m._32;		m[8] = (F32)_m._33;
	}

	inline void getColumn33(F64 m[]) const {
		m[0] = (F64)_m._11;		m[3] = (F64)_m._12;		m[6] = (F64)_m._13;
		m[1] = (F64)_m._21;		m[4] = (F64)_m._22;		m[7] = (F64)_m._23;
		m[2] = (F64)_m._31;		m[5] = (F64)_m._32;		m[8] = (F64)_m._33;
	}

	//for loose 4-padded data.
	inline void setRow44(const F32 m[]) {
		_m._11 = (Real)m[0];	_m._12 = (Real)m[1];	_m._13 = (Real)m[2];
		_m._21 = (Real)m[4];	_m._22 = (Real)m[5];	_m._23 = (Real)m[6];
		_m._31 = (Real)m[8];	_m._32 = (Real)m[9];	_m._33 = (Real)m[10];
	}

	inline void setRow44(const F64 m[]) {
		_m._11 = (Real)m[0];	_m._12 = (Real)m[1];	_m._13 = (Real)m[2];
		_m._21 = (Real)m[4];	_m._22 = (Real)m[5];	_m._23 = (Real)m[6];
		_m._31 = (Real)m[8];	_m._32 = (Real)m[9];	_m._33 = (Real)m[10];
	}

	inline void setColumn44(const F32 m[]) {
		_m._11 = (Real)m[0];	_m._12 = (Real)m[4];	_m._13 = (Real)m[8];
		_m._21 = (Real)m[1];	_m._22 = (Real)m[5];	_m._23 = (Real)m[9];
		_m._31 = (Real)m[2];	_m._32 = (Real)m[6];	_m._33 = (Real)m[10];
	}

	inline void setColumn44(const F64 m[]) {
		_m._11 = (Real)m[0];	_m._12 = (Real)m[4];	_m._13 = (Real)m[8];
		_m._21 = (Real)m[1];	_m._22 = (Real)m[5];	_m._23 = (Real)m[9];
		_m._31 = (Real)m[2];	_m._32 = (Real)m[6];	_m._33 = (Real)m[10];
	}

	inline void getRow44(F32 m[]) const {
		m[0] = (F32)_m._11;		m[1] = (F32)_m._12;		m[2] = (F32)_m._13;
		m[4] = (F32)_m._21;		m[5] = (F32)_m._22;		m[6] = (F32)_m._23;
		m[8] = (F32)_m._31;		m[9] = (F32)_m._32;		m[10]= (F32)_m._33;
	}

	inline void getRow44(F64 m[]) const {
		m[0] = (F64)_m._11;		m[1] = (F64)_m._12;		m[2] = (F64)_m._13;
		m[4] = (F64)_m._21;		m[5] = (F64)_m._22;		m[6] = (F64)_m._23;
		m[8] = (F64)_m._31;		m[9] = (F64)_m._32;		m[10]= (F64)_m._33;
	}

	inline void getColumn44(F32 m[]) const {
		m[0] = (F32)_m._11;		m[4] = (F32)_m._12;		m[8] = (F32)_m._13;
		m[1] = (F32)_m._21;		m[5] = (F32)_m._22;		m[9] = (F32)_m._23;
		m[2] = (F32)_m._31;		m[6] = (F32)_m._32;		m[10]= (F32)_m._33;
	}

	inline void getColumn44(F64 m[]) const {
		m[0] = (F64)_m._11;		m[4] = (F64)_m._12;		m[8] = (F64)_m._13;
		m[1] = (F64)_m._21;		m[5] = (F64)_m._22;		m[9] = (F64)_m._23;
		m[2] = (F64)_m._31;		m[6] = (F64)_m._32;		m[10]= (F64)_m._33;
	}

	inline void setRow(int row, const Vec3& v) {
		m[row][0] = v.v1;		m[row][1] = v.v2;		m[row][2] = v.v3;
	}

	inline void setColumn(int col, const Vec3& v) {
		m[0][col] = v.v1;		m[1][col] = v.v2;		m[2][col] = v.v3;
	}

	inline void getRow(int row, Vec3& v) const {
		v.v1 = m[row][0];		v.v2 = m[row][1];		v.v3 = m[row][2];
	}

	inline void getColumn(int col, Vec3& v) const {
		v.v1 = m[0][col];		v.v2 = m[1][col];		v.v3 = m[2][col];
	}

	inline Vec3 getRow(int row) const {
		return Vec3(m[row][0], m[row][1], m[row][2]);
	}

	inline Vec3 getColumn(int col) const {
		return Vec3(m[0][col], m[1][col], m[2][col]);
	}

	inline Real& operator () (int row, int col) {
		return m[row][col];
	}

	/** returns true for identity matrix
	*/
	inline bool isIdentity() const {
		return
			_m._11 == REAL_ONE && Math::isZero(_m._12) && Math::isZero(_m._13) &&
			Math::isZero(_m._21) && _m._22 == REAL_ONE && Math::isZero(_m._23) &&
			Math::isZero(_m._31) && Math::isZero(_m._32) && _m._33 == REAL_ONE;
	}

	/** returns true for zero matrix
	*/
	inline bool isZero() const {
		return
			Math::isZero(_m._11) && Math::isZero(_m._12) && Math::isZero(_m._13) &&
			Math::isZero(_m._21) && Math::isZero(_m._22) && Math::isZero(_m._23) &&
			Math::isZero(_m._31) && Math::isZero(_m._32) && Math::isZero(_m._33);
	}

	/** returns true if all elems are finite
	*/
	inline bool isFinite() const {
		return
			Math::isFinite(_m._11) && Math::isFinite(_m._12) && Math::isFinite(_m._13) &&
			Math::isFinite(_m._21) && Math::isFinite(_m._22) && Math::isFinite(_m._23) &&
			Math::isFinite(_m._31) && Math::isFinite(_m._32) && Math::isFinite(_m._33);
	}

	/** sets this matrix to the zero matrix.
	*/
	inline void setZero() {
		_m._11 = REAL_ZERO;			_m._12 = REAL_ZERO;			_m._13 = REAL_ZERO;
		_m._21 = REAL_ZERO;			_m._22 = REAL_ZERO;			_m._23 = REAL_ZERO;
		_m._31 = REAL_ZERO;			_m._32 = REAL_ZERO;			_m._33 = REAL_ZERO;
	}

	/** sets this matrix to the identity matrix.
	*/
	inline void setId() {
		_m._11 = REAL_ONE;			_m._12 = REAL_ZERO;			_m._13 = REAL_ZERO;
		_m._21 = REAL_ZERO;			_m._22 = REAL_ONE;			_m._23 = REAL_ZERO;
		_m._31 = REAL_ZERO;			_m._32 = REAL_ZERO;			_m._33 = REAL_ONE;
	}

	/** this = -this
	*/
	inline void setNegative() {
		_m._11 = -_m._11;			_m._12 = -_m._12;			_m._13 = -_m._13;
		_m._21 = -_m._21;			_m._22 = -_m._22;			_m._23 = -_m._23;
		_m._31 = -_m._31;			_m._32 = -_m._32;			_m._33 = -_m._33;
	}

	/** sets this matrix to the diagonal matrix.
	*/
	inline void setDiagonal(const Vec3 &v) {
		_m._11 = v.v1;				_m._12 = REAL_ZERO;			_m._13 = REAL_ZERO;
		_m._21 = REAL_ZERO;			_m._22 = v.v2;				_m._23 = REAL_ZERO;
		_m._31 = REAL_ZERO;			_m._32 = REAL_ZERO;			_m._33 = v.v3;
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
		_m._11 = REAL_ZERO;			_m._12 = -axis.v3;			_m._13 = axis.v2;
		_m._21 = axis.v3;			_m._22 = REAL_ZERO;			_m._23 = -axis.v1;
		_m._31 = -axis.v2;			_m._32 = axis.v1;			_m._33 = REAL_ZERO;
	}

	/** Creates axis from skew-symmetric so(3) matrix (vee operator v).
	*/
	inline void skewToAxis(Vec3 &axis) const {
		axis.v1 = -_m._23;
		axis.v2 =  _m._13;
		axis.v3 = -_m._12;
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

		_m._11 = axis.v1*v1 + c;			_m._12 = axis.v1*v2 - axis.v3*s;	_m._13 = axis.v1*v3 + axis.v2*s;	
		_m._21 = axis.v2*v1 + axis.v3*s;	_m._22 = axis.v2*v2 + c;			_m._23 = axis.v2*v3 - axis.v1*s;	
		_m._31 = axis.v3*v1 - axis.v2*s;	_m._32 = axis.v3*v2 + axis.v1*s;	_m._33 = axis.v3*v3 + c;	
	}

	/** Returns rotation angle and axis of rotation of SO(3) matrix
	*/
	inline void toAngleAxis(Real& angle, Vec3& axis) const {
		angle = Math::acos((trace() - REAL_ONE) / (Real)MATH_TWO);
		Real s = (Real)MATH_TWO * Math::sin(angle);
		if (Math::abs(s) > REAL_EPS_DIV)
			axis.set((_m._32 - _m._23)/s, (_m._13 - _m._31)/s, (_m._21 - _m._12)/s);
		else
			axis.setZero();
	}

	inline void fromQuat(const Quat& q) {
		msk::QuatToMat33(*this, q);
	}

	inline void toQuat(Quat& q) const {
		msk::Mat33ToQuat(q, *this);
	}

	/** returns trace
	*/
	inline Real trace() const {
		return _m._11 + _m._22 + _m._33;
	}

	/** returns determinant
	*/
	inline Real determinant() const {
		return
			_m._11*_m._22*_m._33 + _m._12*_m._23*_m._31 + _m._13*_m._21*_m._32 -
			_m._13*_m._22*_m._31 - _m._12*_m._21*_m._33 - _m._11*_m._23*_m._32;
	}

	/** assigns inverse to m.
	@return false if singular (i.e. if no inverse exists)
	*/
	inline bool getInverse(Mat33& m) const {
		Real temp11 = _m._22*_m._33 - _m._23*_m._32;
		Real temp12 = _m._13*_m._32 - _m._12*_m._33;
		Real temp13 = _m._12*_m._23 - _m._13*_m._22;
		
		Real temp21 = _m._23*_m._31 - _m._21*_m._33;
		Real temp22 = _m._11*_m._33 - _m._13*_m._31;
		Real temp23 = _m._13*_m._21 - _m._11*_m._23;

		Real temp31 = _m._21*_m._32 - _m._22*_m._31;
		Real temp32 = _m._12*_m._31 - _m._11*_m._32;
		Real temp33 = _m._11*_m._22 - _m._12*_m._21;
		
		Real det = temp11*_m._11 + temp12*_m._21 + temp13*_m._31;
		
		if (Math::abs(det) < REAL_EPS_DIV)
			return false;
		
		det = REAL_ONE / det;
		
		m._m._11 = temp11*det;		m._m._12 = temp12*det;		m._m._13 = temp13*det;
		m._m._21 = temp21*det;		m._m._22 = temp22*det;		m._m._23 = temp23*det;
		m._m._31 = temp31*det;		m._m._32 = temp32*det;		m._m._33 = temp33*det;
		
		return true;
	}

	/** this = transpose(other)
	*/
	inline void setTransposed(const Mat33& m) {
		if (this != &m) {
			_m._11 = m._m._11;		_m._12 = m._m._21;		_m._13 = m._m._31;
			_m._21 = m._m._12;		_m._22 = m._m._22;		_m._23 = m._m._32;
			_m._31 = m._m._13;		_m._32 = m._m._23;		_m._33 = m._m._33;
		}
		else
			setTransposed();
	}

	/** this = transpose(this)
	*/
	inline void setTransposed() {
		Math::swap(_m._12, _m._21);
		Math::swap(_m._23, _m._32);
		Math::swap(_m._13, _m._31);
	}

	/** this = this * [ v.v1 0 0; 0 v.v2 0; 0 0 v.v3].
	*/
	inline void multiplyDiagonal(const Vec3& v) {
		_m._11 *= v.v1;		_m._12 *= v.v2;		_m._13 *= v.v3;
		_m._21 *= v.v1;		_m._22 *= v.v2;		_m._23 *= v.v3;
		_m._31 *= v.v1;		_m._32 *= v.v2;		_m._33 *= v.v3;
	}

	/** this = transpose(this) * [ v.v1 0 0; 0 v.v2 0; 0 0 v.v3].
	*/
	inline void multiplyDiagonalTranspose(const Vec3& v) {
		_m._11 *= v.v1;		_m._12 *= v.v1;		_m._13 *= v.v1;
		_m._21 *= v.v2;		_m._22 *= v.v2;		_m._23 *= v.v2;
		_m._31 *= v.v3;		_m._32 *= v.v3;		_m._33 *= v.v3;
	}

	/** m = this * [ v.v1 0 0; 0 v.v2 0; 0 0 v.v3];
	*/
	inline void multiplyDiagonal(Mat33& m, const Vec3& v) const {
		m._m._11 = _m._11 * v.v1;		m._m._12 = _m._12 * v.v2;		m._m._13 = _m._13 * v.v3;
		m._m._21 = _m._21 * v.v1;		m._m._22 = _m._22 * v.v2;		m._m._23 = _m._23 * v.v3;
		m._m._31 = _m._31 * v.v1;		m._m._32 = _m._32 * v.v2;		m._m._33 = _m._33 * v.v3;
	}

	/** m = transpose(this) * [ v.v1 0 0; 0 v.v2 0; 0 0 v.v3];
	*/
	inline void multiplyDiagonalTranspose(Mat33& m, const Vec3& v) const {
		m._m._11 = _m._11 * v.v1;		m._m._12 = _m._21 * v.v2;		m._m._13 = _m._31 * v.v3;
		m._m._21 = _m._12 * v.v1;		m._m._22 = _m._22 * v.v2;		m._m._23 = _m._32 * v.v3;
		m._m._31 = _m._13 * v.v1;		m._m._32 = _m._23 * v.v2;		m._m._33 = _m._33 * v.v3;
	}

	/** a = this * b
	*/
	inline void multiply(Vec3& a, const Vec3& b) const {
		Real v1 = _m._11 * b.v1 + _m._12 * b.v2 + _m._13 * b.v3;
		Real v2 = _m._21 * b.v1 + _m._22 * b.v2 + _m._23 * b.v3;
		Real v3 = _m._31 * b.v1 + _m._32 * b.v2 + _m._33 * b.v3;

		a.v1 = v1;
		a.v2 = v2;
		a.v3 = v3;	
	}

	/** a = transpose(this) * b
	*/
	inline void multiplyByTranspose(Vec3& a, const Vec3& b) const {
		Real v1 = _m._11 * b.v1 + _m._21 * b.v2 + _m._31 * b.v3;
		Real v2 = _m._12 * b.v1 + _m._22 * b.v2 + _m._32 * b.v3;
		Real v3 = _m._13 * b.v1 + _m._23 * b.v2 + _m._33 * b.v3;

		a.v1 = v1;
		a.v2 = v2;
		a.v3 = v3;
	}

	/** this = a + b
	*/
	inline void  add(const Mat33& a, const Mat33& b) {
		_m._11 = a._m._11 + b._m._11;		_m._12 = a._m._12 + b._m._12;		_m._13 = a._m._13 + b._m._13;
		_m._21 = a._m._21 + b._m._21;		_m._22 = a._m._22 + b._m._22;		_m._23 = a._m._23 + b._m._23;
		_m._31 = a._m._31 + b._m._31;		_m._32 = a._m._32 + b._m._32;		_m._33 = a._m._33 + b._m._33;
	}

	/** this = a - b
	*/
	inline void  subtract(const Mat33& a, const Mat33& b) {
		_m._11 = a._m._11 - b._m._11;		_m._12 = a._m._12 - b._m._12;		_m._13 = a._m._13 - b._m._13;
		_m._21 = a._m._21 - b._m._21;		_m._22 = a._m._22 - b._m._22;		_m._23 = a._m._23 - b._m._23;
		_m._31 = a._m._31 - b._m._31;		_m._32 = a._m._32 - b._m._32;		_m._33 = a._m._33 - b._m._33;
	}

	/** this = s * m;
	*/
	inline void multiply(Real s, const Mat33& m) {
		_m._11 = m._m._11 * s;		_m._12 = m._m._12 * s;		_m._13 = m._m._13 * s;
		_m._21 = m._m._21 * s;		_m._22 = m._m._22 * s;		_m._23 = m._m._23 * s;
		_m._31 = m._m._31 * s;		_m._32 = m._m._32 * s;		_m._33 = m._m._33 * s;
	}

	/** this = a * b
	*/
	inline void multiply(const Mat33& a, const Mat33& b) {
		Real a11 = a._m._11 * b._m._11 + a._m._12 * b._m._21 + a._m._13 * b._m._31;
		Real a12 = a._m._11 * b._m._12 + a._m._12 * b._m._22 + a._m._13 * b._m._32;
		Real a13 = a._m._11 * b._m._13 + a._m._12 * b._m._23 + a._m._13 * b._m._33;

		Real a21 = a._m._21 * b._m._11 + a._m._22 * b._m._21 + a._m._23 * b._m._31;
		Real a22 = a._m._21 * b._m._12 + a._m._22 * b._m._22 + a._m._23 * b._m._32;
		Real a23 = a._m._21 * b._m._13 + a._m._22 * b._m._23 + a._m._23 * b._m._33;

		Real a31 = a._m._31 * b._m._11 + a._m._32 * b._m._21 + a._m._33 * b._m._31;
		Real a32 = a._m._31 * b._m._12 + a._m._32 * b._m._22 + a._m._33 * b._m._32;
		Real a33 = a._m._31 * b._m._13 + a._m._32 * b._m._23 + a._m._33 * b._m._33;

		_m._11 = a11;		_m._12 = a12;		_m._13 = a13;
		_m._21 = a21;		_m._22 = a22;		_m._23 = a23;
		_m._31 = a31;		_m._32 = a32;		_m._33 = a33;
	}

	/** this = rotation matrix around X axis
	*/
	inline void rotX(Real angle) {
		Real s, c;
		Math::sinCos(angle,	s, c);

		_m._11 = REAL_ONE;	_m._12 = REAL_ZERO;	_m._13 = REAL_ZERO;
		_m._21 = REAL_ZERO;	_m._22 = c;			_m._23 = -s;
		_m._31 = REAL_ZERO;	_m._32 = s;			_m._33 = c;
	}

	/** this = rotation matrix around Y axis
	*/
	inline void rotY(Real angle) {
		Real s, c;
		Math::sinCos(angle,	s, c);

		_m._11 = c;			_m._12 = REAL_ZERO;	_m._13 = s;
		_m._21 = REAL_ZERO;	_m._22 = REAL_ONE;	_m._23 = REAL_ZERO;
		_m._31 = -s;		_m._32 = REAL_ZERO;	_m._33 = c;
	}

	/** this = rotation matrix around Z axis
	*/
	inline void rotZ(Real angle) {
		Real s, c;
		Math::sinCos(angle,	s, c);

		_m._11 = c;			_m._12 = -s;		_m._13 = REAL_ZERO;
		_m._21 = s;			_m._22 = c;			_m._23 = REAL_ZERO;
		_m._31 = REAL_ZERO;	_m._32 = REAL_ZERO;	_m._33 = REAL_ONE;
	}

	/**	Assignment operator.
	*/
	inline const Mat33& operator = (const Mat33 &m) {
		_m = m._m;
		return *this;
	}

	inline Mat33& operator += (const Mat33 &m) {
		_m._11 += m._m._11;		_m._12 += m._m._12;		_m._13 += m._m._13;
		_m._21 += m._m._21;		_m._22 += m._m._22;		_m._23 += m._m._23;
		_m._31 += m._m._31;		_m._32 += m._m._32;		_m._33 += m._m._33;
		return *this;
	}

	inline Mat33& operator -= (const Mat33 &m) {
		_m._11 -= m._m._11;		_m._12 -= m._m._12;		_m._13 -= m._m._13;
		_m._21 -= m._m._21;		_m._22 -= m._m._22;		_m._23 -= m._m._23;
		_m._31 -= m._m._31;		_m._32 -= m._m._32;		_m._33 -= m._m._33;
		return *this;
	}

	inline Mat33& operator *= (const Mat33& m) {
		multiply(*this, m);
		return *this;
	}

	inline Mat33& operator *= (Real s) {
		_m._11 *= s;			_m._12 *= s;			_m._13 *= s;
		_m._21 *= s;			_m._22 *= s;			_m._23 *= s;
		_m._31 *= s;			_m._32 *= s;			_m._33 *= s;
		return *this;
	}

	inline Mat33& operator /= (Real s) {
		s = REAL_ONE / s;
		_m._11 *= s;			_m._12 *= s;			_m._13 *= s;
		_m._21 *= s;			_m._22 *= s;			_m._23 *= s;
		_m._31 *= s;			_m._32 *= s;			_m._33 *= s;
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

#endif /*SYSTEM_MAT33_H_*/
