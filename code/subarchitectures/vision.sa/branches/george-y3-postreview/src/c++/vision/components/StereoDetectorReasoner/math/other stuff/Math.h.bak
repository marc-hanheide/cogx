/** @file Math.h
 * 
 * Mathematical routines.
 * 
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk">The University Of Birmingham</A>
 *
 * @version 1.0
 *
 */

#ifndef SYSTEM_MATH_H_
#define SYSTEM_MATH_H_

//------------------------------------------------------------------------------

#include "system/System.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>

//------------------------------------------------------------------------------

namespace msk {

//------------------------------------------------------------------------------

/** Some constants */

#define MATH_ZERO					F_ZEROP

#define MATH_TWO					2.0
#define REAL_TWO					((Real)MATH_TWO)

#define MATH_HALF					0.5
#define REAL_HALF					((Real)MATH_HALF)

#define MATH_PI						3.14159265358979323846
#define REAL_PI						((Real)MATH_PI)

#define MATH_PI_2					1.57079632679489661923
#define REAL_PI_2					((Real)MATH_PI_2)

#define MATH_2_PI					6.28318530717958647692
#define REAL_2_PI					((Real)MATH_2_PI)

#define MATH_INV_PI					0.31830988618379067154
#define REAL_INV_PI					((Real)MATH_INV_PI)

#define MATH_LN2					0.69314718055994530942
#define REAL_LN2					((Real)MATH_LN2)

#define MATH_SQRT_2					1.41421356237309504880
#define REAL_SQRT_2					((Real)MATH_SQRT_2)

#define MATH_GOLD1					1.61803398874989484820 // Golden Ratio
#define REAL_GOLD1					((Real)MATH_GOLD)

#define MATH_GOLD2					0.61803398874989484820 // Golden Ratio
#define REAL_GOLD2					((Real)MATH_GOLD)

/** "Small" value */
#define MATH_EPS					1.0e-6
#define REAL_EPS					((Real)MATH_EPS)

/** The smallest divisor */
#define MATH_EPS_DIV				1.0e-15
#define REAL_EPS_DIV				((Real)MATH_EPS_DIV)

///** "Small" time value [sec] */
//#define MATH_EPS_TIME				1.0e-6
//#define REAL_EPS_TIME				((Real)MATH_EPS_TIME)
//
///** "Small" distance value [m] */
//#define MATH_EPS_DIST				1.0e-6
//#define REAL_EPS_DIST				((Real)MATH_EPS_DIST)
//
///** "Small" angle value [rad] */
//#define MATH_EPS_ANGLE				1.0e-6
//#define REAL_EPS_ANGLE				((Real)MATH_EPS_ANGLE)

//------------------------------------------------------------------------------

class Math {
public:
	template <typename TYPE> inline static void swap(TYPE &a, TYPE &b) {
		const TYPE x = a; a = b; b = x;
	}

	template <typename TYPE> inline static void rotate(TYPE &a, TYPE &b, TYPE &c) {
		const TYPE x = a; a = b; b = c; c = x;
	}
	
	template <typename TYPE> inline static void rotate(TYPE &a, TYPE &b, TYPE &c, TYPE &d) {
		const TYPE x = a; a = b; b = c; c = d; d = x;
	}
	
	template <typename TYPE> inline static void shift(TYPE &a, TYPE &b, TYPE c) {
		a = b; b = c;
	}
	
	template <typename TYPE> inline static void shift(TYPE &a, TYPE &b, TYPE &c, TYPE d) {
		a = b; b = c; c = d;
	}
	
	template <typename TYPE> inline static TYPE min(TYPE a, TYPE b) {
		return a < b ? a : b;
	}

	template <typename TYPE> inline static TYPE max(TYPE a, TYPE b) {
		return a > b ? a : b;
	}

	template <typename TYPE> inline static TYPE sign(TYPE a, TYPE sign)  {
		return sign < TYPE(0.0) ? -a : a;
	}

	template <typename TYPE> inline static TYPE clamp(TYPE a, TYPE min, TYPE max)  {
		return a < min ? min : a > max ? max : a;
	}

	template <typename TYPE> inline static TYPE guard(TYPE a, TYPE value, TYPE cutoff)  {
		return Math::sign(Math::max(Math::abs(a - value), cutoff), a - value);
	}

	//template <typename TYPE> inline static TYPE abs(TYPE a) {
	//	return std::abs(a);
	//}
	inline static F32 abs(F32 a) {
		return ::fabsf(a);
	}
	inline static F64 abs(F64 a) {
		return ::fabs(a);
	}
	inline static I32 abs(I32 a) {
		return ::abs(a);
	}

	template <typename TYPE> inline static bool equals(TYPE a, TYPE b, TYPE eps) {
		return (Math::abs(a - b) < eps);
	}

	template <typename TYPE> inline static TYPE floor(TYPE a) {
		return ::floor(a);
	}

	template <typename TYPE> inline static TYPE floor(TYPE a, TYPE quant) {
		a /= quant;
		return quant * (a < REAL_ZERO ? Math::ceil(a) : Math::floor(a));
	}

	template <typename TYPE> inline static TYPE ceil(TYPE a) {
		return ::ceil(a);
	}

	template <typename TYPE> inline static TYPE ceil(TYPE a, TYPE quant) {
		a /= quant;
		return quant * (a < REAL_ZERO ? Math::floor(a) : Math::ceil(a));
	}

	template <typename TYPE> inline static TYPE round(TYPE a) {
		return Math::floor(a + (TYPE)0.5);
	}

	template <typename TYPE> inline static TYPE round(TYPE a, TYPE quant) {
		return quant * Math::floor(a/quant + (TYPE)0.5);
	}

	template <typename TYPE> inline static TYPE sqr(TYPE a) {
		return a*a;
	}

	template <typename TYPE> inline static TYPE sqrt(TYPE a) {
		return ::sqrt(a);
	}

	template <typename TYPE> inline static TYPE sin(TYPE a) {
		return ::sin(a);
	}

	template <typename TYPE> inline static TYPE asin(TYPE a) {
		return
			a >=  REAL_ONE ?  REAL_PI_2 :
			a <= -REAL_ONE ? -REAL_PI_2 : ::asin(a);
	}

	template <typename TYPE> inline static TYPE cos(TYPE a) {
		return ::cos(a);
	}

	template <typename TYPE> inline static TYPE acos(TYPE a) {
		return
			a >=  REAL_ONE ?  REAL_ZERO :
			a <= -REAL_ONE ?  REAL_PI   : ::acos(a);
	}

	template <typename TYPE> inline static void sinCos(TYPE a, TYPE& s, TYPE& c) {
		s = ::sin(a);
		c = ::cos(a);
	}

	template <typename TYPE> inline static TYPE tan(TYPE a) {
		return ::tan(a);
	}

	template <typename TYPE> inline static TYPE atan(TYPE a) {
		return ::atan(a);
	}

	template <typename TYPE> inline static TYPE atan2(TYPE x, TYPE y) {
		return ::atan2(x,y);
	}

	template <typename TYPE> inline static TYPE pow(TYPE x, TYPE y) {
		return ::pow(x,y);
	}

	template <typename TYPE> inline static TYPE exp(TYPE a) {
		return ::exp(a);
	}

	template <typename TYPE> inline static TYPE ln(TYPE a) {
		return ::log(a);
	}

	template <typename TYPE> inline static TYPE log2(TYPE a) {
		return ::log(a) / (TYPE)REAL_LN2;
	}

	template <typename TYPE> inline static TYPE log10(TYPE a) {
		return ::log10(a);
	}

	template <typename TYPE> inline static bool isFinite(TYPE a) {
#ifdef WIN32
		return !((_FPCLASS_SNAN | _FPCLASS_QNAN | _FPCLASS_NINF | _FPCLASS_PINF) & ::_fpclass(a));
#else
		return isfinite(a);
#endif
	}
	template <typename TYPE> inline static bool isZero(TYPE a) {
		return fpclassify(a) == FP_ZERO;
	}

	template <typename TYPE> inline static TYPE degToRad(TYPE a) {
		return (TYPE)(0.01745329251994329547 * a);
	}

	template <typename TYPE> inline static TYPE radToDeg(TYPE a) {
		return (TYPE)(57.29577951308232286465 * a);
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*SYSTEM_MATH_H_*/
