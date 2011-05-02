/** @file VecN.h
 * 
 * Multi-dimensional real-valued vector with variable size and mask operations.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_MATH_VECN_H_
#define _GOLEM_MATH_VECN_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Math.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Default zero flag for mask operations */
struct ZeroFlag {
	template <typename Type> static inline bool is(Type val) {
		return val == numeric_const<Type>::ZERO; // by default test for zero
	}
};

/** Default non-Zero flag for mask operations */
struct NZeroFlag {
	template <typename Type> static inline bool is(Type val) {
		return val != numeric_const<Type>::ZERO; // by default test for zero
	}
};

/** Multi-dimensional real-valued vector with variable size (up to _N) and mask operations. */
template <typename Real, size_t _N> class VecN {
public:
	/** Maximum vector size */
	static const size_t N = _N;

	/** vector components */
	Real v[N];

	/** Default constructor sets size to 0 */
	inline VecN() {
		n = 0;
	}
	/** Sets vector size without initialisation */
	inline VecN(size_t n) {
		resize(n);
	}
	/** Resizes vector and initialises with given value */
	inline VecN(size_t n, Real value) {
		assign(n, value);
	}
	/** Copying constructor */
	inline VecN(const VecN& vec) {
		set(vec);
	}
	/** Copying constructor */
	template <typename Flag, typename Ptr> inline VecN(Ptr mask, const VecN& vec) {
		set<Flag, Ptr>(mask, vec);
	}
	/** Resizes vector and initialises with given value */
	template <typename Ptr> inline VecN(Ptr begin, Ptr end) {
		assign(end, begin);
	}

	/** Resizes vector */
	inline void resize(size_t n) {
		this->n = std::min(n, N);
	}
	
	/** Vector size */
	inline size_t size() const {
		return n;
	}
	/** Vector effective size */
	template <typename Flag> inline size_t effectiveSize() const {
		return effectiveSize<Flag>(v, v + n);
	}
	/** Effective size from data sequence */
	template <typename Flag, typename Ptr> inline static size_t effectiveSize(Ptr begin, Ptr end) {
		size_t n = 0;
		while (begin != end)
			if (Flag::template is(*begin++))
				++n;
		return n;
	}

	/** Initialises data with given value */
	inline void fill(Real value) {
		for (size_t i = 0; i < n; ++i)
			v[i] = value;
	}

	/** Resizes vector and initialises data with given value */
	inline void assign(size_t n, Real value) {
		resize(n);
		fill(value);
	}

	/** Resizes vector and initialises data with given value */
	template <typename Ptr> inline void assign(Ptr begin, Ptr end) {
		resize(end - begin);
		for (size_t i = 0; i < n; ++i)
			v[i] = Real(*begin++);
	}

	/** Sets vector */
	inline void set(const VecN& vec) {
		n = vec.n;
		for (size_t i = 0; i < vec.n; ++i)
			v[i] = vec[i];
	}
	/** Sets vector using mask, resize target vector */
	template <typename Flag, typename Ptr> inline void set(Ptr mask, const VecN& vec) {
		n = 0;
		for (size_t i = 0; i < vec.n; ++i)
			if (Flag::template is(*mask++))
				v[n++] = vec[i];
	}

	/** Sets value without resizing */
	template <typename Flag, typename Ptr> inline void set(Ptr mask, Real value) {
		for (size_t i = 0; i < n; ++i)
			if (Flag::template is(*mask++))
				v[i] = value;
	}
	
	/** Sets zero without resizing */
	inline void setZero() {
		fill(numeric_const<Real>::ZERO);
	}
	/** Sets zero without resizing */
	template <typename Flag, typename Ptr> inline void setZero(Ptr mask) {
		set<Flag, Ptr>(mask, numeric_const<Real>::ZERO);
	}
	
	/** Sets negative of the specified vector */
	inline void setNegative(const VecN& vec) {
		n = vec.n;
		for (size_t i = 0; i < n; ++i)
			v[i] = -vec[i];
	}

	/** Sets inverse of the specified vector */
	inline void setInverse(const VecN& vec) {
		n = vec.n;
		for (size_t i = 0; i < n; ++i) {
			const Real c = vec[i];
			v[i] = c != numeric_const<Real>::ZERO ? numeric_const<Real>::ONE/c : numeric_const<Real>::ZERO;
		}
	}
	/** Sets inverse of the specified vector, resize target vector */
	template <typename Flag, typename Ptr> inline void setInverse(Ptr mask, const VecN& vec) {
		n = 0;
		for (size_t i = 0; i < n; ++i)
			if (Flag::template is(*mask++)) {
				const Real c = vec[i];
				v[n++] = c != numeric_const<Real>::ZERO ? numeric_const<Real>::ONE/c : numeric_const<Real>::ZERO;
			}
	}

	/** Sets absolute of the specified vector */
	inline void setAbs(const VecN& vec) {
		n = vec.n;
		for (size_t i = 0; i < n; ++i)
			v[i] = Math::abs(vec[i]);
	}

	/** sets the vector's magnitude
	*/
	inline void setMagnitude(Real length) {
		const Real m = magnitude();

		if (Math::abs(m) > numeric_const<Real>::ZERO) {
			const Real newLength = length / m;
			for (size_t i = 0; i < n; ++i)
				v[i] *= newLength;
		}
	}

	/** normalises the vector
	*/
	inline Real normalise() {
		const Real m = magnitude();
		
		if (Math::abs(m) > numeric_const<Real>::ZERO) {
			const Real length = numeric_const<Real>::ONE / m;
			for (size_t i = 0; i < n; ++i)
				v[i] *= length;
		}
		
		return m;
	}

	/** this = element wise min(this,other) */
	inline void min(const VecN& vec) {
		for (size_t i = 0; i < n; ++i)
			if (v[i] > vec[i])
				v[i] = vec[i];
	}
	/** this = element wise max(this,other)
	*/
	inline void max(const VecN& vec) {
		for (size_t i = 0; i < n; ++i)
			if (v[i] < vec[i])
				v[i] = vec[i];
	}

	/** this = a + b
	*/
	inline void add(const VecN& a, const VecN& b) {
		for (size_t i = 0; i < n; ++i)
			v[i] = a[i] + b[i];
	}

	/** this = a - b
	*/
	inline void subtract(const VecN& a, const VecN& b) {
		for (size_t i = 0; i < n; ++i)
			v[i] = a[i] - b[i];
	}

	/** this = s * a;
	*/
	inline void multiply(Real s, const VecN& a) {
		for (size_t i = 0; i < n; ++i)
			v[i] = s*a[i];
	}

	/** this[i] = a[i] * b[i], for all i.
	*/
	inline void arrayMultiply(const VecN& a, const VecN& b) {
		for (size_t i = 0; i < n; ++i)
			v[i] = a[i]*b[i];
	}

	/** this = s * a + b;
	*/
	inline void multiplyAdd(Real s, const VecN& a, const VecN& b) {
		for (size_t i = 0; i < n; ++i)
			v[i] = s*a[i] + b[i];
	}

	/** this = s * a + t * b;
	*/
	inline void linear(Real s, const VecN& a, Real t, const VecN& b) {
		for (size_t i = 0; i < n; ++i)
			v[i] = s*a[i] + t*b[i];
	}

	/** this = a + s * (b - a);
	*/
	inline void interpolate(const VecN& a, const VecN& b, Real s) {
		for (size_t i = 0; i < n; ++i)
			v[i] = a[i] + s*(b[i] - a[i]);
	}

	/** returns the magnitude
	*/
	inline Real magnitude() const {
		return Math::sqrt(magnitudeSquared());
	}

	/** returns the squared magnitude
	*/
	inline Real magnitudeSquared() const {
		Real s = numeric_const<Real>::ZERO;
		for (size_t i = 0; i < n; ++i)
			s += Math::sqr(v[i]);
		return s;
	}

	/** returns (this - other).magnitude();
	*/
	inline Real distance(const VecN& vec) const {
		return Math::sqrt(distanceSquared(vec));
	}

	/** returns (this - other).magnitudeSquared();
	*/
	inline Real distanceSquared(const VecN& vec) const {
		Real s = numeric_const<Real>::ZERO;
		for (size_t i = 0; i < n; ++i)
			s += Math::sqr(v[i] - vec[i]);
		return s;
	}

	/** returns the dot/scalar product of this and other.
	*/
	inline Real dot(const VecN& vec) const {
		Real s = numeric_const<Real>::ZERO;
		for (size_t i = 0; i < n; ++i)
			s += v[i]*vec[i];
		return s;
	}

	/** tests for exact zero vector
	*/
	inline bool isZero() const {
		for (size_t i = 0; i < n; ++i)
			if (!Math::isZero(v[i]))
				return false;
		return true;
	}

	/** tests for positive vector
	*/
	inline bool isPositive() const {
		for (size_t i = 0; i < n; ++i)
			if (v[i] < numeric_const<Real>::ZERO)
				return false;
		return true;
	}

	/** tests for finite vector
	*/
	inline bool isFinite() const {
		for (size_t i = 0; i < n; ++i)
			if (!Math::isFinite(v[i]))
				return false;
		return true;
	}

	/** returns true if this and arg's elems are within epsilon of each other.
	*/
	inline bool equals(const VecN& vec, Real epsilon) const {
		for (size_t i = 0; i < n; ++i)
			if (!Math::equals(v[i], vec[i], epsilon))
				return false;
		return true;
	}
	
	/**	Assignment operator. */
	inline VecN& operator = (const VecN &vec) {
		set(vec);
		return *this;
	}
	/** Access vector as an array. */
	inline Real& operator [] (size_t idx) {
		return v[idx];
	}
	/** Access vector as an array. */
	inline const Real& operator [] (size_t idx) const {
		return v[idx];
	}

protected:
	/** Vector size */
	size_t n;
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_VECN_H_*/
