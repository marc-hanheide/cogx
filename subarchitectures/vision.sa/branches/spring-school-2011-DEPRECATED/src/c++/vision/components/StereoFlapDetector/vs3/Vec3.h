/** @file Vec3.h
 * 
 * Mathematical routines.
 * 
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk">The University Of Birmingham</A>
 *
 * @version 1.0
 *
 */

#ifndef SYSTEM_VEC3_H_
#define SYSTEM_VEC3_H_

//------------------------------------------------------------------------------

#include "system/Math.h"

//------------------------------------------------------------------------------

namespace msk {

//------------------------------------------------------------------------------

/** 3 Element vector class.
*/
class Vec3 {
public:
	/** vector components */
	Real v1, v2, v3;

	/** Default constructor does not do any initialisation.
	*/
	inline Vec3() {}

	/**	Assigns scalar parameters to all elements.
	*	@param	a		Value to assign to elements.
	*/
	inline Vec3(Real a) : v1(a), v2(a), v3(a) {}

	/** Initialises from 3 scalar parameters.
	*	@param	v1		Value to initialise v1 component.
	*	@param	v2		Value to initialise v1 component.
	*	@param	v3		Value to initialise v1 component.
	*/
	inline Vec3(Real v1, Real v2, Real v3) : v1(v1), v2(v2), v3(v3) {}
		
	/**	Copy constructor.
	*/
	inline Vec3(const Vec3 &v) : v1(v.v1), v2(v.v2), v3(v.v3) {}

	/**	Access the data as an array.
	*	@return		Array of 3 floats.
	*/
	inline const Real *get() const {
		return &v1;
	}
	
	/**	Access the data as an array.
	*	@return		Array of 3 floats.
	*/
	inline Real* get() {
		return &v1;
	}

	/** Writes out the 3 values to dest.
	*	@param	v	Array to write elements to.
	*/
	inline void get(F32 v[]) const {
		v[0] = (F32)this->v1;
		v[1] = (F32)this->v2;
		v[2] = (F32)this->v3;
	}

	/** Writes out the 3 values to dest.
	*	@param	v	Array to write elements to.
	*/
	inline void get(F64 v[]) const {
		v[0] = (F64)this->v1;
		v[1] = (F64)this->v2;
		v[2] = (F64)this->v3;
	}

	inline void set(Real a) {
		v1 = a;
		v2 = a;
		v3 = a;
	}

	inline void set(Real v1, Real v2, Real v3) {
		this->v1 = v1;
		this->v2 = v2;
		this->v3 = v3;
	}
	
	/** reads 3 consecutive values from the ptr passed
	*/
	inline void  set(const F32* v) {
		v1 = (Real)v[0];
		v2 = (Real)v[1];
		v3 = (Real)v[2];
	}

	/** reads 3 consecutive values from the ptr passed
	*/
	inline void set(const F64* v) {
		v1 = (Real)v[0];
		v2 = (Real)v[1];
		v3 = (Real)v[2];
	}
	
	/** this = v
	*/
	inline void  set(const Vec3& v) {
		v1 = v.v1;
		v2 = v.v2;
		v3 = v.v3;
	}

	/** this = -a
	*/
	inline void  setNegative(const Vec3& v) {
		v1 = -v.v1;
		v2 = -v.v2;
		v3 = -v.v3;
	}

	/** this = -this
	*/
	inline void  setNegative() {
		v1 = -v1;
		v2 = -v2;
		v3 = -v3;
	}

	inline void setZero() {
		v1 = REAL_ZERO;
		v2 = REAL_ZERO;
		v3 = REAL_ZERO;
	}
	
	/** sets the vector's magnitude
	*/
	inline void setMagnitude(Real length) {
		Real m = magnitude();

		if (Math::abs(m) > REAL_EPS_DIV) {
			Real newLength = length / m;
			v1 *= newLength;
			v2 *= newLength;
			v3 *= newLength;
		}
	}

	/** normalises the vector
	*/
	inline Real normalise() {
		Real m = magnitude();
		
		if (Math::abs(m) > REAL_EPS_DIV) {
			const Real length = REAL_ONE / m;
			v1 *= length;
			v2 *= length;
			v3 *= length;
		}
		
		return m;
	}

	/** this = element wise min(this,other)
	*/
	inline void min(const Vec3& v) {
		if (v1 < v.v1) v1 = v.v1;
		if (v2 < v.v2) v2 = v.v2;
		if (v3 < v.v3) v3 = v.v3;
	}
	/** this = element wise max(this,other)
	*/
	inline void max(const Vec3& v) {
		if (v1 > v.v1) v1 = v.v1;
		if (v2 > v.v2) v2 = v.v2;
		if (v3 > v.v3) v3 = v.v3;
	}

	/** this = a + b
	*/
	inline void add(const Vec3& a, const Vec3& b) {
		v1 = a.v1 + b.v1;
		v2 = a.v2 + b.v2;
		v3 = a.v3 + b.v3;
	}

	/** this = a - b
	*/
	inline void subtract(const Vec3& a, const Vec3& b) {
		v1 = a.v1 - b.v1;
		v2 = a.v2 - b.v2;
		v3 = a.v3 - b.v3;
	}

	/** this = s * a;
	*/
	inline void multiply(Real s, const Vec3& a) {
		v1 = a.v1 * s;
		v2 = a.v2 * s;
		v3 = a.v3 * s;
	}

	/** this[i] = a[i] * b[i], for all i.
	*/
	inline void arrayMultiply(const Vec3& a, const Vec3& b) {
		v1 = a.v1 * b.v1;
		v2 = a.v2 * b.v2;
		v3 = a.v3 * b.v3;
	}

	/** this = s * a + b;
	*/
	inline void multiplyAdd(Real s, const Vec3& a, const Vec3& b) {
		v1 = s * a.v1 + b.v1;
		v2 = s * a.v2 + b.v2;
		v3 = s * a.v3 + b.v3;
	}

	/** this = s * a + t * b;
	*/
	inline void linear(Real s, const Vec3& a, Real t, const Vec3& b) {
		v1 = s * a.v1 + t * b.v1;
		v2 = s * a.v2 + t * b.v2;
		v3 = s * a.v3 + t * b.v3;
	}

	/** returns the magnitude
	*/
	inline Real magnitude() const {
		return Math::sqrt(v1 * v1 + v2 * v2 + v3 * v3);
	}

	/** returns the squared magnitude
	*/
	inline Real magnitudeSquared() const {
		return v1 * v1 + v2 * v2 + v3 * v3;
	}

	/** returns (this - other).magnitude();
	*/
	inline Real distance(const Vec3& v) const {
		Real dv1 = v1 - v.v1;
		Real dv2 = v2 - v.v2;
		Real dv3 = v3 - v.v3;
		return Math::sqrt(dv1 * dv1 + dv2 * dv2 + dv3 * dv3);
	}

	/** returns (this - other).magnitudeSquared();
	*/
	inline Real distanceSquared(const Vec3& v) const {
		Real dv1 = v1 - v.v1;
		Real dv2 = v2 - v.v2;
		Real dv3 = v3 - v.v3;
		return dv1 * dv1 + dv2 * dv2 + dv3 * dv3;
	}

	/** returns the dot/scalar product of this and other.
	*/
	inline Real dot(const Vec3& v) const {
		return v1 * v.v1 + v2 * v.v2 + v3 * v.v3;
	}

	/** cross product, this = left v1 right
	*/
	inline void cross(const Vec3& left, const Vec3& right) {
		// temps needed in case left or right is this.
		Real a = (left.v2 * right.v3) - (left.v3 * right.v2);
		Real b = (left.v3 * right.v1) - (left.v1 * right.v3);
		Real c = (left.v1 * right.v2) - (left.v2 * right.v1);

		v1 = a;
		v2 = b;
		v3 = c;
	}
	/** cross product
	*/
	Vec3 cross(const Vec3& v) const {
		Vec3 temp;
		temp.cross(*this, v);
		return temp;
	}

	/** tests for exact zero vector
	*/
	inline bool isZero() const {
		return Math::isZero(v1) && Math::isZero(v2) && Math::isZero(v3);
	}

	/** tests for positive vector
	*/
	inline bool isPositive() const {
		return v1 > REAL_ZERO && v2 > REAL_ZERO && v3 > REAL_ZERO ;
	}

	/** tests for finite vector
	*/
	inline bool isFinite() const {
		return Math::isFinite(v1) && Math::isFinite(v2) && Math::isFinite(v3);
	}

	/** returns true if this and arg's elems are within epsilon of each other.
	*/
	inline bool equals(const Vec3& v, Real epsilon) const {
		return
			Math::equals(v1, v.v1, epsilon) &&
			Math::equals(v2, v.v2, epsilon) &&
			Math::equals(v3, v.v3, epsilon);
	}

	/**	Assignment operator.
	*/
	inline const Vec3& operator = (const Vec3& v) {
		v1 = v.v1;	v2 = v.v2;	v3 = v.v3;
		return *this;
	}

	/** Access the data as an array.
	*	@param	idx	Array index.
	*	@return		Array element pointed by idx.
	*/
	inline Real& operator [] (int idx) {
		ASSERT(idx >= 0 && idx <= 2)
		return (&v1)[idx];
	}
	inline const Real& operator [] (int idx) const {
		ASSERT(idx >= 0 && idx <= 2)
		return (&v1)[idx];
	}
	
	/** true if all the members are smaller.
	*/
	inline bool operator < (const Vec3& v) const {
		return (v1 < v.v1) && (v2 < v.v2) && (v3 < v.v3);
	}

	/** returns true if the two vectors are exactly equal.
	*/
	inline bool operator == (const Vec3& v) const {
		return (v1 == v.v1) && (v2 == v.v2) && (v3 == v.v3);
	}

	/** returns true if the two vectors are exactly unequal.
	*/
	inline bool operator != (const Vec3& v) const {
		return (v1 != v.v1) || (v2 != v.v2) || (v3 != v.v3);
	}

	/** negation
	*/
	Vec3 operator - () const {
		return Vec3(-v1, -v2, -v3);
	}
	/** vector addition
	*/
	Vec3 operator + (const Vec3 & v) const {
		return Vec3(v1 + v.v1, v2 + v.v2, v3 + v.v3);
	}
	/** vector difference
	*/
	Vec3 operator - (const Vec3 & v) const {
		return Vec3(v1 - v.v1, v2 - v.v2, v3 - v.v3);
	}
	/** scalar post-multiplication
	*/
	Vec3 operator * (Real f) const {
		return Vec3(v1 * f, v2 * f, v3 * f);
	}
	/** scalar division
	*/
	Vec3 operator / (Real f) const {
		f = Real(1.0) / f;
		return Vec3(v1 * f, v2 * f, v3 * f);
	}
	/** vector addition
	*/
	Vec3& operator += (const Vec3& v) {
		v1 += v.v1;
		v2 += v.v2;
		v3 += v.v3;
		return *this;
	}
	/** vector difference
	*/
	Vec3& operator -= (const Vec3& v) {
		v1 -= v.v1;
		v2 -= v.v2;
		v3 -= v.v3;
		return *this;
	}
	/** scalar multiplication
	*/
	Vec3& operator *= (Real f) {
		v1 *= f;
		v2 *= f;
		v3 *= f;
		return *this;
	}
	/** scalar division
	*/
	Vec3& operator /= (Real f) {
		f = Real(1.0) / f;
		v1 *= f;
		v2 *= f;
		v3 *= f;
		return *this;
	}
	/** cross product
	*/
	Vec3 operator ^ (const Vec3& v) const {
		Vec3 temp;
		temp.cross(*this, v);
		return temp;
	}
	/** dot product
	*/
	Real operator | (const Vec3& v) const {
		return v1 * v.v1 + v2 * v.v2 + v3 * v.v3;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*SYSTEM_VEC3_H_*/
