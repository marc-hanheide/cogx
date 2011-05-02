/** @file Vec2.h
 * 
 * Mathematical routines.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_MATH_VEC2_H_
#define _GOLEM_MATH_VEC2_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Math.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** 2 Element vector class.
*/
class Vec2 {
public:
	/** vector components */
	union {
		struct {
			Real x, y;
		};
		struct {
			Real v1, v2;
		};
		Real v[2];
	};

	/** Default constructor does not do any initialisation.
	*/
	inline Vec2() {}

	/**	Assigns scalar parameters to all elements.
	*	@param	a		Value to assign to elements.
	*/
	inline Vec2(Real a) : v1(a), v2(a) {}

	/** Initialises from 3 scalar parameters.
	*	@param	v1		Value to initialise v1 component.
	*	@param	v2		Value to initialise v1 component.
	*/
	inline Vec2(Real v1, Real v2) : v1(v1), v2(v2) {}
		
	/**	Copy constructor.
	*/
	inline Vec2(const Vec2 &v) : v1(v.v1), v2(v.v2) {}

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
	}

	/** Writes out the 3 values to dest.
	*	@param	v	Array to write elements to.
	*/
	inline void get(F64 v[]) const {
		v[0] = (F64)this->v1;
		v[1] = (F64)this->v2;
	}

	inline void set(Real a) {
		v1 = a;
		v2 = a;
	}

	inline void set(Real v1, Real v2) {
		this->v1 = v1;
		this->v2 = v2;
	}
	
	/** reads 3 consecutive values from the ptr passed
	*/
	inline void  set(const F32* v) {
		v1 = (Real)v[0];
		v2 = (Real)v[1];
	}

	/** reads 3 consecutive values from the ptr passed
	*/
	inline void set(const F64* v) {
		v1 = (Real)v[0];
		v2 = (Real)v[1];
	}
	
	/** this = v
	*/
	inline void  set(const Vec2& v) {
		v1 = v.v1;
		v2 = v.v2;
	}

	/** this = -a
	*/
	inline void  setNegative(const Vec2& v) {
		v1 = -v.v1;
		v2 = -v.v2;
	}

	/** this = -this
	*/
	inline void  setNegative() {
		v1 = -v1;
		v2 = -v2;
	}

	inline void setZero() {
		v1 = REAL_ZERO;
		v2 = REAL_ZERO;
	}
	
	/** sets the vector's magnitude
	*/
	inline void setMagnitude(Real length) {
		Real m = magnitude();

		if (Math::abs(m) > REAL_EPS) {
			Real newLength = length / m;
			v1 *= newLength;
			v2 *= newLength;
		}
	}

	/** normalises the vector
	*/
	inline Real normalise() {
		Real m = magnitude();
		
		if (Math::abs(m) > REAL_EPS) {
			const Real length = REAL_ONE / m;
			v1 *= length;
			v2 *= length;
		}
		
		return m;
	}

	/** this = element wise min(this,other)
	*/
	inline void min(const Vec2& v) {
		if (v1 < v.v1) v1 = v.v1;
		if (v2 < v.v2) v2 = v.v2;
	}
	/** this = element wise max(this,other)
	*/
	inline void max(const Vec2& v) {
		if (v1 > v.v1) v1 = v.v1;
		if (v2 > v.v2) v2 = v.v2;
	}

	/** this = a + b
	*/
	inline void add(const Vec2& a, const Vec2& b) {
		v1 = a.v1 + b.v1;
		v2 = a.v2 + b.v2;
	}

	/** this = a - b
	*/
	inline void subtract(const Vec2& a, const Vec2& b) {
		v1 = a.v1 - b.v1;
		v2 = a.v2 - b.v2;
	}

	/** this = s * a;
	*/
	inline void multiply(Real s, const Vec2& a) {
		v1 = a.v1 * s;
		v2 = a.v2 * s;
	}

	/** this[i] = a[i] * b[i], for all i.
	*/
	inline void arrayMultiply(const Vec2& a, const Vec2& b) {
		v1 = a.v1 * b.v1;
		v2 = a.v2 * b.v2;
	}

	/** this = s * a + b;
	*/
	inline void multiplyAdd(Real s, const Vec2& a, const Vec2& b) {
		v1 = s * a.v1 + b.v1;
		v2 = s * a.v2 + b.v2;
	}

	/** this = s * a + t * b;
	*/
	inline void linear(Real s, const Vec2& a, Real t, const Vec2& b) {
		v1 = s * a.v1 + t * b.v1;
		v2 = s * a.v2 + t * b.v2;
	}

	/** returns the magnitude
	*/
	inline Real magnitude() const {
		return Math::sqrt(v1 * v1 + v2 * v2);
	}

	/** returns the squared magnitude
	*/
	inline Real magnitudeSquared() const {
		return v1 * v1 + v2 * v2;
	}

	/** returns (this - other).magnitude();
	*/
	inline Real distance(const Vec2& v) const {
		Real dv1 = v1 - v.v1;
		Real dv2 = v2 - v.v2;
		return Math::sqrt(dv1 * dv1 + dv2 * dv2);
	}

	/** returns (this - other).magnitudeSquared();
	*/
	inline Real distanceSquared(const Vec2& v) const {
		Real dv1 = v1 - v.v1;
		Real dv2 = v2 - v.v2;
		return dv1 * dv1 + dv2 * dv2;
	}

	/** returns the dot/scalar product of this and other.
	*/
	inline Real dot(const Vec2& v) const {
		return v1 * v.v1 + v2 * v.v2;
	}

	/** tests for exact zero vector
	*/
	inline bool isZero() const {
		return Math::isZero(v1) && Math::isZero(v2);
	}

	/** tests for positive vector
	*/
	inline bool isPositive() const {
		return v1 > REAL_ZERO && v2 > REAL_ZERO;
	}

	/** tests for finite vector
	*/
	inline bool isFinite() const {
		return Math::isFinite(v1) && Math::isFinite(v2);
	}

	/** returns true if this and arg's elems are within epsilon of each other.
	*/
	inline bool equals(const Vec2& v, Real epsilon) const {
		return
			Math::equals(v1, v.v1, epsilon) &&
			Math::equals(v2, v.v2, epsilon);
	}

	/**	Assignment operator.
	*/
	inline const Vec2& operator = (const Vec2& v) {
		v1 = v.v1;	v2 = v.v2;
		return *this;
	}

	/** Access the data as an array.
	*	@param	idx	Array index.
	*	@return		Array element pointed by idx.
	*/
	inline Real& operator [] (size_t idx) {
		ASSERT(idx <= 2)
		return (&v1)[idx];
	}
	inline const Real& operator [] (size_t idx) const {
		ASSERT(idx <= 2)
		return (&v1)[idx];
	}
	
	/** true if all the members are smaller.
	*/
	inline bool operator < (const Vec2& v) const {
		return (v1 < v.v1) && (v2 < v.v2);
	}

	/** true if all the members are larger.
	*/
	inline bool operator > (const Vec2& v) const {
		return (v1 > v.v1) && (v2 > v.v2);
	}

	/** returns true if the two vectors are exactly equal.
	*/
	inline bool operator == (const Vec2& v) const {
		return (v1 == v.v1) && (v2 == v.v2);
	}

	/** returns true if the two vectors are exactly unequal.
	*/
	inline bool operator != (const Vec2& v) const {
		return (v1 != v.v1) || (v2 != v.v2);
	}

	/** negation
	*/
	Vec2 operator - () const {
		return Vec2(-v1, -v2);
	}
	/** vector addition
	*/
	Vec2 operator + (const Vec2 & v) const {
		return Vec2(v1 + v.v1, v2 + v.v2);
	}
	/** vector difference
	*/
	Vec2 operator - (const Vec2 & v) const {
		return Vec2(v1 - v.v1, v2 - v.v2);
	}
	/** scalar post-multiplication
	*/
	Vec2 operator * (Real f) const {
		return Vec2(v1 * f, v2 * f);
	}
	/** scalar division
	*/
	Vec2 operator / (Real f) const {
		f = Real(1.0) / f;
		return Vec2(v1 * f, v2 * f);
	}
	/** vector addition
	*/
	Vec2& operator += (const Vec2& v) {
		v1 += v.v1;
		v2 += v.v2;
		return *this;
	}
	/** vector difference
	*/
	Vec2& operator -= (const Vec2& v) {
		v1 -= v.v1;
		v2 -= v.v2;
		return *this;
	}
	/** scalar multiplication
	*/
	Vec2& operator *= (Real f) {
		v1 *= f;
		v2 *= f;
		return *this;
	}
	/** scalar division
	*/
	Vec2& operator /= (Real f) {
		f = Real(1.0) / f;
		v1 *= f;
		v2 *= f;
		return *this;
	}
	/** dot product
	*/
	Real operator | (const Vec2& v) const {
		return v1 * v.v1 + v2 * v.v2;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_VEC2_H_*/
