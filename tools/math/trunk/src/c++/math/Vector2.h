/**
 * Yet another set of 2D vector utilities.
 *
 * Based on code by Marek Kopicky.
 *
 * A lot of set/get functions are provided to allow You to interface the Vector2
 * class to representations or linear algebra packages You are already using.
 * 
 * @author Michael Zillich
 * @date February 2009
 */

#ifndef VECTOR2_H
#define VECTOR2_H

#include <iostream>
#include <vector>
#include <stdexcept>
#include <cast/core/CASTUtils.hpp>
#include <cogxmath_base.h>
#include <Math.hpp>

namespace cogx 
{
namespace Math
{

using namespace std;
using namespace cast;

/**
 * Initialises from 2 scalars.
 */
inline Vector2 vector2(double x, double y)
{
  Vector2 a;
  a.x = x;
  a.y = y;
  return a;
}

/**
 * Access the data as an array.
 * @return  const array of 3 doubles
 */
inline const double *get(const Vector2 &a)
{
  return &a.x;
}

/**
 * Access the data as an array.
 * @return  array of 3 doubles
 */
inline double* get(Vector2 &a)
{
  return &a.x;
}

/**
 * Get an element specified via index.
 */
inline double get(const Vector2 &a, int idx) throw(runtime_error)
{
  if(idx < 0 || idx > 1)
    throw runtime_error(exceptionMessage(__HERE__, 
          "invlaid index %d must be [0,1]", idx));
  return (&a.x)[idx];
}

/**
 * Set an element specified via index.
 */
inline void set(Vector2 &a, int idx, double d) throw(runtime_error)
{
  if(idx < 0 || idx > 1)
    throw runtime_error(exceptionMessage(__HERE__, 
          "invlaid index %d must be [0,1]", idx));
  (&a.x)[idx] = d;
}


/**
 * Get the 2 values to a float array.
 * @param v  array of size at least offset + 2
 * @param offset  position where to start in the array
 */
inline void get(const Vector2 &a, float v[], size_t offset = 0)
{
  v[offset] = (float)a.x;
  v[offset + 1] = (float)a.y;
}

/**
 * Get the 2 values to a double array.
 * @param v  array of size at least offset + 2
 * @param offset  position where to start in the array
 */
inline void get(const Vector2 &a, double v[], size_t offset = 0)
{
  v[offset] = a.x;
  v[offset + 1] = a.y;
}

/**
 * Get the 2 values to a float STL vector.
 * @param v  STL vector of size at least offset + 2
 * @param offset  position where to start in the STL vector
 */
inline void get(const Vector2 &a, vector<float> &v, size_t offset = 0)
  throw(runtime_error)
{
  if(v.size() < offset + 2)
    throw runtime_error(exceptionMessage(__HERE__,
          "vector not big enough: %d <  %d", (int)v.size(), (int)offset + 2));
  v[offset] = (float)a.x;
  v[offset + 1] = (float)a.y;
}

/**
 * Get the 2 values to a double STL vector.
 * @param v  STL vector of size at least offset + 2
 * @param offset  position where to start in the STL vector
 */
inline void get(const Vector2 &a, vector<double> &v, size_t offset = 0)
  throw(runtime_error)
{
  if(v.size() < offset + 2)
    throw runtime_error(exceptionMessage(__HERE__,
          "vector not big enough: %d <  %d", (int)v.size(), (int)offset + 2));
  v[offset] = a.x;
  v[offset + 1] = a.y;
}

/**
 * Set from 2 scalar values.
 */
inline void set(Vector2 &a, double x, double y)
{
  a.x = x;
  a.y = y;
}

/**
 * Set from 2 consecutive values in a float array.
 * @param v  array of size at least offset + 2
 * @param offset  position where to start in the array
 */
inline void set(Vector2 &a, const float v[], size_t offset = 0)
{
  a.x = (double)v[offset];
  a.y = (double)v[offset + 1];
}

/**
 * Set from 2 consecutive values in a double array.
 * @param v  array of size at least offset + 2
 * @param offset  position where to start in the array
 */
inline void set(Vector2 &a, const double v[], size_t offset = 0)
{
  a.x = v[offset];
  a.y = v[offset + 1];
}

/**
 * Set from 2 consecutive values in a float STL vector.
 * @param v  STL vector of size at least offset + 2
 * @param offset  position where to start in the STL vector
 */
inline void set(Vector2 &a, const vector<float> &v, size_t offset = 0)
  throw(runtime_error)
{
  if(v.size() < offset + 2)
    throw runtime_error(exceptionMessage(__HERE__,
          "vector not big enough: %d <  %d", (int)v.size(), (int)offset + 2));
  a.x = (double)v[offset];
  a.y = (double)v[offset + 1];
}

/**
 * Set from 2 consecutive values in a double STL vector.
 * @param v  STL vector of size at least offset + 2
 * @param offset  position where to start in the STL vector
 */
inline void set(Vector2 &a, const vector<double> &v, size_t offset = 0)
  throw(runtime_error)
{
  if(v.size() < offset + 2)
    throw runtime_error(exceptionMessage(__HERE__,
          "vector not big enough: %d <  %d", (int)v.size(), (int)offset + 2));
  a.x = (double)v[offset];
  a.y = (double)v[offset + 1];
}

/**
 * a = -a
 */
inline void setNegative(Vector2 &a)
{
  a.x = -a.x;
  a.y = -a.y;
}

inline void setZero(Vector2 &a)
{
  a.x = REAL_ZERO;
  a.y = REAL_ZERO;
}

/**
 * Tests for exact zero vector.
 */
inline bool isZero(const Vector2 &a)
{
  return iszero(a.x) && iszero(a.y);
}

/**
  * Tests for elementwise positive vector.
  */
inline bool isPositive(const Vector2 &a)
{
  return a.x > REAL_ZERO && a.y > REAL_ZERO;
}

/**
 * Tests for elementwise negative vector.
 */
inline bool isNegative(const Vector2 &a)
{
  return a.x < REAL_ZERO && a.y < REAL_ZERO;
}

/**
 * Tests for finite vector.
 */
inline bool isFinite(const Vector2 &a)
{
  return isfinite(a.x) && isfinite(a.y);
}

inline Vector2& operator += (Vector2 &a, const Vector2 &b)
{
  a.x += b.x;
  a.y += b.y;
  return a;
}

inline Vector2& operator -= (Vector2 &a, const Vector2 &b)
{
  a.x -= b.x;
  a.y -= b.y;
  return a;
}

inline Vector2& operator *= (Vector2 &a, double s)
{
  a.x *= s;
  a.y *= s;
  return a;
}

inline Vector2& operator /= (Vector2 &a, double s) throw(runtime_error)
{
  if(iszero(s))
    throw runtime_error(exceptionMessage(__HERE__, "division by zero"));
  a.x /= s;
  a.y /= s;
  return a;
}

/**
 * Print a vector in text form to a stream: '[x y]'
 */
inline void writeText(ostream &os, const Vector2 &a)
{
  os << '[' << a.x << ' ' << a.y <<  ']';
}

/**
 * Read a vector in text form from a stream.
 * The expected format is: '[x y]', white spaces are ignored.
 */
inline void readText(istream &is, Vector2 &a) throw(runtime_error)
{
  char c;
  is >> c;
  if(c == '[')
  {
    is >> a.x >> a.y >> c;
    if(c != ']')
      throw runtime_error(exceptionMessage(__HERE__,
            "error reading Vector2: ']' expected"));
  }
  else
    throw runtime_error(exceptionMessage(__HERE__,
          "error reading Vector2: '[' expected"));
}

/**
 * Writing to a stream is taken to be a textual output, rather than a
 * serialisation of the actual binary data.
 */
inline ostream& operator << (ostream &os, const Vector2 &a)
{
  writeText(os, a);
  return os;
}

/**
 * Reading from a stream is taken to read a textual input, rather than
 * de-serialising the actual binary data.
 */
inline istream& operator >> (istream &is, Vector2 &a)
{
  readText(is, a);
  return is;
}

/**
 * Returns true if the two vectors are exactly equal.
 */
inline bool operator == (const Vector2 &a, const Vector2 &b)
{
  return a.x == b.x && a.y == b.y;
}

/**
 * Returns true if a and b's elems are within epsilon of each other.
 */
inline bool vequals(const Vector2& a, const Vector2& b, double eps)
{
  return equals(a.x, b.x, eps) &&
         equals(a.y, b.y, eps);
}

// Is now provided in Ice
// /**
//  * Returns true if the two vectors are not exactly equal.
//  */
// inline bool operator != (const Vector2 &a, const Vector2 &b)
// {
//   return a.x != b.x || a.y != b.y;
// }


inline Vector2 operator - (const Vector2 &a)
{
  return vector2(-a.x, -a.y);
}

inline Vector2 operator + (const Vector2 &a, const Vector2 &b)
{
  return vector2(a.x + b.x, a.y + b.y);
}
inline Vector2 operator - (const Vector2 &a, const Vector2 &b)
{
  return vector2(a.x - b.x, a.y - b.y);
}

inline Vector2 operator * (const double s, const Vector2 &a)
{
  return vector2(a.x*s, a.y*s);
}

inline Vector2 operator * (const Vector2 &a, const double s)
{
  return s*a;
}

inline Vector2 operator / (const Vector2 &a, const double s) throw(runtime_error)
{
  if(iszero(s))
    throw runtime_error(exceptionMessage(__HERE__, "division by zero"));
  return vector2(a.x/s, a.y/s);
}

/**
 *  c = a + b
 */
inline void add(const Vector2& a, const Vector2& b, Vector2& c)
{
  c.x = a.x + b.x;
  c.y = a.y + b.y;
}

/**
 * c = a - b
 */
inline void sub(const Vector2& a, const Vector2& b, Vector2& c)
{
  c.x = a.x - b.x;
  c.y = a.y - b.y;
}

/**
 * c = s * a
 */
inline void mult(double s, const Vector2& a, Vector2& c)
{
  c.x = a.x * s;
  c.y = a.y * s;
}

/**
 * c = s * a + b
 */
inline void multAdd(double s, const Vector2 &a, const Vector2 &b, Vector2 &c)
{
  c.x = s*a.x + b.x;
  c.y = s*a.y + b.y;
}

/**
 * c = s * a + t * b
 */
inline void linear(double s, const Vector2 &a, double t, const Vector2 &b,
    Vector2 &c)
{
  c.x = s*a.x + t*b.x;
  c.y = s*a.y + t*b.y;
}

inline double norm(const Vector2 &a)
{
  return sqrt(sqr(a.x) + sqr(a.y));
}

inline double normSqr(const Vector2 &a)
{
  return sqr(a.x) + sqr(a.y);
}

inline double length(const Vector2 &a)
{
  return norm(a);
}

inline double lengthSqr(const Vector2 &a)
{
  return normSqr(a);
}

/**
 * Euclidian distance.
 */
inline double dist(const Vector2 &a, const Vector2 &b)
{
  return sqrt(sqr(a.x - b.x) + sqr(a.y - b.y));
}

/**
 * Squared euclidian distance.
 */
inline double distSqr(const Vector2 &a, const Vector2 &b)
{
  return sqr(a.x - b.x) + sqr(a.y - b.y);
}

/**
 * Normalises the vector, returns the length before normalisation.
 */
inline double normalise(Vector2 &a)
{
  double n = norm(a);
  a /= n;
  return n;
}

/**
 * Get any (clockwise or anti-clockwise) normal.
 */
inline Vector2 normal(const Vector2 &a)
{
  return vector2(-a.y, a.x);
}

/**
 * Get clockwise (mathematically negative) normal, i.e. rotation to the right.
 * ATTENTION: For typical image co-ordinate systems with x-axis pointiing to
 * the right and y-axis pointing downwards things are reversed: clockwise
 * rotation in this case means rotation to the left.
 */
inline Vector2 normalClockwise(const Vector2 &a)
{
  return vector2(a.y, -a.x);
}

/**
 * Get anti-clockwise (mathematically positive) normal, i.e. rotation to the
 * left.
 * ATTENTION: For typical image co-ordinate systems with x-axis pointiing to
 * the right and y-axis pointing downwards things are reversed: anti-clockwise
 * rotation in this case means rotation to the right.
 */
inline Vector2 NormalAntiClockwise(const Vector2 &a)
{
  return vector2(-a.y, a.x);
}

inline double polarAngle(const Vector2 &a)
{
  return atan2(a.y, a.x);
}

/**
 * Vector dot product.
 */
inline double dot(const Vector2 &a, const Vector2 &b)
{
  return a.x*b.x + a.y*b.y;
}

/**
 * Vector cross product.
 * note: positive cross product a x b means b counterclockwise (i.e. left) to a
 */
inline double cross(const Vector2 &a, const Vector2 &b)
{
  return a.x*b.y - a.y*b.x;
}

/**
 * Returns true if b is counterclockwise to a.
 * Same as left of.
 */
inline bool counterClockwiseTo(const Vector2 &a, const Vector2 &b)
{
  return cross(a, b) > 0.;
}

/**
 * Returns true if b is left of a.
 * Same as counterclockwise.
 */
inline bool leftOf(const Vector2 &a, const Vector2 &b)
{
  return counterClockwiseTo(a, b);
}

/**
 * Returns true if b is clockwise to a.
 * Same as right of.
 */
inline bool clockwiseTo(const Vector2 &a, const Vector2 &b)
{
  return cross(a, b) < 0.;
}

/**
 * Returns true if b is right of a.
 * Same as clockwise.
 */
inline bool rightOf(const Vector2 &a, const Vector2 &b)
{
  return clockwiseTo(a, b);
}

/**
 * Midpoint between two points.
 */
inline Vector2 midPoint(const Vector2 &a, const Vector2 &b)
{
  return vector2((a.x + b.x)/2., (a.y + b.y)/2.);
}

/**
 * Rotate around angle given in [rad].
 */
inline Vector2 rotate(const Vector2 &a, double phi)
{
  double si = sin(phi), co = cos(phi);
  return vector2(co*a.x - si*a.y, si*a.x + co*a.y);
}

/**
 * Returns signed distance of point q from line defined by point p and unit
 * direction vector d.
 */
inline double distPointToLine(const Vector2 &q, const Vector2 &p,
    const Vector2 &d)
{
  Vector2 p_to_q = q - p;
  return cross(p_to_q, d);
}

inline double absDistPointToLine(const Vector2 &q, const Vector2 &p,
    const Vector2 &d)
{
  return abs(distPointToLine(q, p, d));
}

/**
 * Returns intersection of lines defined by point p and direction d (needs not
 * be a unit vector).
 */
inline void lineIntersection(const Vector2 &p1, const Vector2 &d1,
    const Vector2 &p2, const Vector2 &d2, Vector2 &i) throw(runtime_error)
{
  double d = cross(d2, d1);
  if(d == 0.)
    throw runtime_error(exceptionMessage(__HERE__, "lines do not intersect"));
  Vector2 p12 = p2 - p1;
  double l = cross(d2, p12)/d;
  set(i, p1.x + l*d1.x, p1.y + l*d1.y);
}

/**
 * Returns intersection of lines defined by point p and direction d (needs not
 * be a unit vector).
 * l1 and l2 contain the lengths along the lines where the intersection lies,
 * measured in lengths of d1 and d2.
 * So if You want l1 and l2 in pixels, d1 and d2 must be unit vectors.
 */
inline void lineIntersection(const Vector2 &p1, const Vector2 &d1,
    const Vector2 &p2, const Vector2 &d2, Vector2 &i, double &l1, double &l2)
    throw(runtime_error)
{
  double d = cross(d2, d1);
  if(d == 0.)
    throw runtime_error(exceptionMessage(__HERE__, "lines do not intersect"));
  Vector2 p12 = p2 - p1;
  l1 = cross(d2, p12)/d;
  l2 = cross(d1, p12)/d;
  set(i, p1.x + l1*d1.x, p1.y + l1*d1.y);
}


/**
 * Checks whether lines a and b defined by their end points intersect.
 */
inline bool linesIntersecting(const Vector2 &a1, const Vector2 &a2,
    const Vector2 &b1, const Vector2 &b2)
{
  double l1 = 0., l2 = 0.;
  Vector2 i;
  Vector2 a12 = a2 - a1;
  Vector2 b12 = b2 - b1;
  lineIntersection(a1, a12, b1, b12, i, l1, l2);
  return (l1 >= 0. && l1 <= 1.) && (l2 >= 0. && l2 <= 1.);
}

/*
 * Calculate center of circle from 3 points.
 * note: throws an exception if center cannot be calculated.
 */
inline void circleCenter(const Vector2 &pi, const Vector2 &pj,
   const Vector2 &pk, Vector2 &c)
{
  // throws an exception if intersection cannot be calculated
  lineIntersection(
      vector2((pi.x + pj.x)/2., (pi.y + pj.y)/2.),
      vector2(pj.y - pi.y, pi.x - pj.x),
      vector2((pj.x + pk.x)/2., (pj.y + pk.y)/2.),
      vector2(pk.y - pj.y, pj.x - pk.x),
      c);
}

}

}
#endif

