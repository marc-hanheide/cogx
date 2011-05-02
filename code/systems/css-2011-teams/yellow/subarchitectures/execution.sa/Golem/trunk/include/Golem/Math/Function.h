/** @file Function.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_MATH_FUNCTION_H_
#define _GOLEM_MATH_FUNCTION_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Pointers.h>
#include <Golem/Math/Math.h>

namespace golem {

//------------------------------------------------------------------------------

/** Closed finite real interval */
template <typename REAL>
class Interval {
public:
	typedef shared_ptr< Interval<REAL> > Ptr;

	REAL left, right;

	Interval(REAL left, REAL right) : left(left), right(right) {
	}
	Interval(const Interval<REAL> &interval) : left(interval.left), right(interval.right) {
	}

	void setLeft(REAL left) {
		this->left = left;
	}
	REAL getLeft() {
		return left;
	}
	const REAL getLeft() const {
		return left;
	}

	void setRight(REAL right) {
		this->right = right;
	}
	REAL getRight() {
		return right;
	}
	const REAL getRight() const {
		return right;
	}

	void set(REAL left, REAL right) {
		this->left = left;
		this->right = right;
	}
	REAL get() const {
		return right - left;
	}
	REAL min() const {
		return left < right ? left : right;
	}
	REAL max() const {
		return left < right ? right : left;
	}

	bool contains(REAL x) const {
		return x >= left && x <= right;
	}
	bool increasing() const {
		return left < right;
	}
	bool nondecreasing() const {
		return left <= right;
	}
	bool decreasing() const {
		return left > right;
	}
	bool nonincreasing() const {
		return left >= right;
	}

	void swap() {
		std::swap(left, right);
	}

	const Interval<REAL>& operator = (const Interval<REAL> &interval) {
		left = interval.left;
		right = interval.right;
		return *this;
	}
};

//------------------------------------------------------------------------------

/** Measurement */
template <typename REAL>
class Measurement {
private:
	U32 count;
	REAL val, sum, avr, min, max;
	
public:
	Measurement() {
		reset();
	}

	void reset() {
		count = 0;

		val = numeric_const<REAL>::ZERO;
		sum = numeric_const<REAL>::ZERO;
		avr = numeric_const<REAL>::ZERO;
		
		min = numeric_const<REAL>::MAX;
		max = numeric_const<REAL>::MIN;
	}

	void update(REAL val) {
		count++;

		this->val = val;
		sum += val;
		avr = sum/count;

		if (val < min)
			min = val;
		if (val > max)
			max = val;
	}

	REAL getCurr() const {
		return val;
	}
	REAL getAvr() const {
		return avr;
	}
	REAL getMin() const {
		return min;
	}
	REAL getMax() const {
		return max;
	}
	U32 getCount() const {
		return count;
	}
};

//------------------------------------------------------------------------------

/** Single variable function on real interval */
template <typename REAL>
class Function {
public:
	typedef shared_ptr< Function<REAL> > Ptr;

	virtual ~Function() {}
	
	virtual REAL get(REAL x) const = 0;
};

//------------------------------------------------------------------------------

template <typename REAL>
class StdFunc : public Function<REAL> {
public:
	typedef REAL (*FuncPtr)(REAL);

protected:
	FuncPtr func;

public:
	StdFunc(FuncPtr func) : func(func) {
	}
	
	virtual REAL get(REAL x) const {
		return func(x);
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_FUNCTION_H_*/
