/**
 * Various math stuff.
 *
 * @author Michael Zillich
 * @date Februrary 2009
 */

#ifndef COGX_MATH_BASE_H
#define COGX_MATH_BASE_H

#include <cmath>
#include <cfloat>
#include <algorithm>

namespace cogx
{

namespace Math
{

// note that
//   +0.0 = 0x0... HEX
//   -0.0 = 0x8... HEX
// we take positive zero to be zero
static const double REAL_ZERO = ((double)+0.0);
static const double REAL_ONE  = ((double)1.0);

/** "Small" epsilon value */
static const double REAL_EPS  = 1.0e-12;

#ifdef REAL_IS_FLOAT
static const double REAL_MAX = FLT_MAX;
#else
static const double REAL_MAX = DBL_MAX;
#endif

// Quite often (e.g. stereo stufF) we need left and right sides
static const int LEFT = 0;
static const int RIGHT = 1;

template <typename TYPE>
inline TYPE sqr(TYPE x)
{
  return x*x;
}

inline bool equals(float a, float b, float eps)
{
  return (fabsf(a - b) < eps);
}

inline bool equals(double a, double b, double eps)
{
  return (fabs(a - b) < eps);
}

/**
 * similar to isnan() or isinf(), check a float for being zero
 * Note: This is safer than testing for == REAL_ZERO as +0.0 and -0.0 are
 * distinct values.
 * Note 2: It seems however that still +0.0 == -0.0 is true.
 * Note 3: Anyway.
 */
inline bool iszero(float a)
{
  return std::fpclassify(a) == FP_ZERO;
}

/**
 * similar to isnan() or isinf(), check a double for being zero
 * Note: This is safer than testing for == REAL_ZERO as +0.0 and -0.0 are
 * distinct values.
 * Note 2: It seems however that still +0.0 == -0.0 is true.
 * Note 3: Anyway.
 */
inline bool iszero(double a)
{
  return std::fpclassify(a) == FP_ZERO;
}

/**
 * clamp value x to l <= x <= u
 */
template <class T>
inline T clamp(T x, T l, T u)
{
  if(x < l)
    x = l;
  if(x > u)
    x = u;
  return x;
}

template <class T>
inline bool inrange_open(T x, T l, T u)
{
  return x > l && x < u;
}

template <class T>
inline bool inrange_closed(T x, T l, T u)
{
  return x >= l && x <= u;
}

}

}
#endif


