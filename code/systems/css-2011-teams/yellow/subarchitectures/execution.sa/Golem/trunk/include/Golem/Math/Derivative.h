/** @file Derivative.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_MATH_DERIVATIVE_H_
#define _GOLEM_MATH_DERIVATIVE_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Function.h>
#include <vector>

namespace golem {

//------------------------------------------------------------------------------

/** Equation solver for single variable real functions */
template <typename REAL>
class Derivative {
public:
	typedef shared_ptr< Derivative<REAL> > Ptr;
	
	static const REAL EPS_DFLT;
	
	virtual ~Derivative() {}

	/** Single solution equation solver */
	virtual REAL findFirst(REAL x, const Function<REAL> &function) const = 0;
	
	/** Multiple solutions equation solver */
	virtual REAL findSecond(REAL x, const Function<REAL> &function) const = 0;
};

template <typename REAL> const REAL Derivative<REAL>::EPS_DFLT = REAL(1.0e-6);

//------------------------------------------------------------------------------

template <typename REAL>
class DerivativeSecant : public Derivative<REAL> {
protected:
	const REAL epsilon;

public:
	DerivativeSecant(REAL epsilon = Derivative<REAL>::EPS_DFLT) :
		epsilon(epsilon)
	{}

	virtual REAL findFirst(REAL x, const Function<REAL> &function) const {
		return (function.get(x + epsilon) - function.get(x - epsilon))/(REAL(2.0)*epsilon);
	}

	virtual REAL findSecond(REAL x, const Function<REAL> &function) const {
		return (function.get(x + epsilon) - REAL(2.0)*function.get(x) + function.get(x - epsilon))/(epsilon*epsilon);
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_DERIVATIVE_H_*/
