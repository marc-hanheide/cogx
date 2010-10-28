/**
 * Yet another set of 3D vector utilities.
 *
 * Based on code by Marek Kopicky.
 *
 * A lot of set/get functions are provided to allow You to interface the Vector3
 * class to representations or linear algebra packages You are already using.
 * 
 * @author Michael Zillich
 * @date February 2009
 */

#ifndef VECTOR3_H
#define VECTOR3_H

#include <vector>
#include <iostream>
#include <stdexcept>
//#include <cast/core/CASTUtils.hpp>
//#include <cogxmath_base.h>
//#include <Math.hpp>
#include <Math.hh>


namespace Z
{

using namespace std;

/**
 * Initialises from 3 scalars.
 */
inline Vector3 vector3(double x, double y, double z)
{
  Vector3 a;
  a.x = x;
  a.y = y;
  a.z = z;
  return a;
}

/**
 * Access the data as an array.
 * @return  const array of 3 doubles
 */
inline const double *get(const Vector3 &a)
{
  return &a.x;
}

/**
 * Access the data as an array.
 * @return  array of 3 doubles
 */
inline double* Get(Vector3 &a)
{
  return &a.x;
}

/**
 * Get an element specified via index.
 */
inline double get(const Vector3 &a, int idx) throw(runtime_error)
{
  if(idx < 0 || idx > 2)
    throw runtime_error(exceptionMessage(__HERE__, 
          "invlaid index %d must be [0,2]", idx));
  return (&a.x)[idx];
}

/**
 * Set an element specified via index.
 */
inline void set(Vector3 &a, int idx, double d) throw(runtime_error)
{
  if(idx < 0 || idx > 2)
    throw runtime_error(exceptionMessage(__HERE__, 
          "invlaid index %d must be [0,2]", idx));
  (&a.x)[idx] = d;
}

/**
 * Get the 3 values to a float array.
 * @param v  array of size at least offset + 3
 * @param offset  position where to start in the array
 */
inline void get(const Vector3 &a, float v[], size_t offset = 0)
{
  v[offset] = (float)a.x;
  v[offset + 1] = (float)a.y;
  v[offset + 2] = (float)a.z;
}

/**
 * Get the 3 values to a double array.
 * @param v  array of size at least offset + 3
 * @param offset  position where to start in the array
 */
inline void get(const Vector3 &a, double v[], size_t offset = 0)
{
  v[offset] = a.x;
  v[offset + 1] = a.y;
  v[offset + 2] = a.z;
}

/**
 * Get the 3 values to a float STL vector.
 * @param v  STL vector of size at least offset + 3
 * @param offset  position where to start in the STL vector
 */
inline void get(const Vector3 &a, vector<float> &v, size_t offset = 0)
  throw(runtime_error)
{
  if(v.size() < offset + 3)
    throw runtime_error(exceptionMessage(__HERE__,
          "vector not big enough: %d <  %d", (int)v.size(), (int)offset + 3));
  v[offset] = (float)a.x;
  v[offset + 1] = (float)a.y;
  v[offset + 2] = (float)a.z;
}

/**
 * Get the 3 values to a double STL vector.
 * @param v  STL vector of size at least offset + 3
 * @param offset  position where to start in the STL vector
 */
inline void get(const Vector3 &a, vector<double> &v, size_t offset = 0)
  throw(runtime_error)
{
  if(v.size() < offset + 3)
    throw runtime_error(exceptionMessage(__HERE__,
          "vector not big enough: %d <  %d", (int)v.size(), (int)offset + 3));
  v[offset] = a.x;
  v[offset + 1] = a.y;
  v[offset + 2] = a.z;
}

/**
 * Set from 3 scalar values.
 */
inline void set(Vector3 &a, double x, double y, double z)
{
  a.x = x;
  a.y = y;
  a.z = z;
}

/**
 * Reads 3 consecutive values from a float array.
 */
inline void set(Vector3 &a, const float v[])
{
  a.x = v[0];
  a.y = v[1];
  a.z = v[2];
}

/**
 * Set from 3 consecutive values in a float array.
 * @param v  array of size at least offset + 3
 * @param offset  position where to start in the array
 */
inline void set(Vector3 &a, const float v[], size_t offset = 0)
{
  a.x = (double)v[offset];
  a.y = (double)v[offset + 1];
  a.z = (double)v[offset + 2];
}

/**
 * Set from 3 consecutive values in a double array.
 * @param v  array of size at least offset + 3
 * @param offset  position where to start in the array
 */
inline void set(Vector3 &a, const double v[], size_t offset = 0)
{
  a.x = (double)v[offset];
  a.y = (double)v[offset + 1];
  a.z = (double)v[offset + 2];
}

/**
 * Set from 3 consecutive values in a float STL vector.
 * @param v  STL vector of size at least offset + 3
 * @param offset  position where to start in the STL vector
 */
inline void set(Vector3 &a, const vector<float> &v, size_t offset = 0)
  throw(runtime_error)
{
  if(v.size() < offset + 3)
    throw runtime_error(exceptionMessage(__HERE__,
          "vector not big enough: %d <  %d", (int)v.size(), (int)offset + 3));
  a.x = (double)v[offset];
  a.y = (double)v[offset + 1];
  a.z = (double)v[offset + 2];
}

/**
 * Set from 3 consecutive values in a double STL vector.
 * @param v  STL vector of size at least offset + 3
 * @param offset  position where to start in the STL vector
 */
inline void set(Vector3 &a, const vector<double> &v, size_t offset = 0)
  throw(runtime_error)
{
  if(v.size() < offset + 3)
    throw runtime_error(exceptionMessage(__HERE__,
          "vector not big enough: %d <  %d", (int)v.size(), (int)offset + 3));
  a.x = (double)v[offset];
  a.y = (double)v[offset + 1];
  a.z = (double)v[offset + 2];
}

/**
 * a = -a
 */
inline void setNegative(Vector3 &a)
{
  a.x = -a.x;
  a.y = -a.y;
  a.z = -a.z;
}

inline void setZero(Vector3 &a)
{
  a.x = REAL_ZERO;
  a.y = REAL_ZERO;
  a.z = REAL_ZERO;
}

/**
 * Tests for exact zero vector.
 */
inline bool isZero(const Vector3 &a)
{
  return iszero(a.x) && iszero(a.y) && iszero(a.z);
}

/**
 * Tests for elementwise positive vector.
 */
inline bool isPositive(const Vector3 &a)
{
  return a.x > REAL_ZERO && a.y > REAL_ZERO && a.z > REAL_ZERO;
}

/**
 * Tests for elementwise negative vector.
 */
inline bool isNegative(const Vector3 &a)
{
  return a.x < REAL_ZERO && a.y < REAL_ZERO && a.z < REAL_ZERO;
}

/**
 * Tests for finite vector.
 */
inline bool isFinite(const Vector3 &a)
{
  return isfinite(a.x) && isfinite(a.y) && isfinite(a.z);
}

inline Vector3& operator += (Vector3 &a, const Vector3 &b)
{
  a.x += b.x;
  a.y += b.y;
  a.z += b.z;
  return a;
}

inline Vector3& operator -= (Vector3 &a, const Vector3 &b)
{
  a.x -= b.x;
  a.y -= b.y;
  a.z -= b.z;
  return a;
}

inline Vector3& operator *= (Vector3 &a, double f)
{
  a.x *= f;
  a.y *= f;
  a.z *= f;
  return a;
}

inline Vector3& operator /= (Vector3 &a, double f) throw(runtime_error)
{
  if(iszero(f))
    throw runtime_error(exceptionMessage(__HERE__, "division by zero"));
  a.x /= f;
  a.y /= f;
  a.z /= f;
  return a;
}

/**
 * Print a vector in text form to a stream: '[x y z]`
 */
inline void writeText(ostream &os, const Vector3 &a)
{
  os << '[' << a.x << ' ' << a.y << ' ' << a.z << ']';
}

/**
 * Read a vector in text from a stream.
 * The expected format is: '[x y z]', white spaces are ignored.
 */
inline void readText(istream &is, Vector3 &a) throw(runtime_error)
{
  char c;
  is >> c;
  if(c == '[')
  {
    is >> a.x >> a.y >> a.z >> c;
    if(c != ']')
      throw runtime_error(exceptionMessage(__HERE__,
            "error reading Vector3: ']' expected"));
  }
  else
    throw runtime_error(exceptionMessage(__HERE__,
          "error reading Vector3: '[' expected"));
}


/**
 * Writing to a stream is taken to be a textual output, rather than a
 * serialisation of the actual binary data.
 */
inline ostream& operator<<(ostream &os, const Vector3 &a)
{
  writeText(os, a);
  return os;
}

/**
 * Reading from a stream is taken to read a textual input, rather than
 * de-serialising the actual binary data.
 */
inline istream& operator>>(istream &is, Vector3 &a)
{
  readText(is, a);
  return is;
}

/**
 * Returns true if the two vectors are exactly equal.
 */
inline bool operator == (const Vector3& a, const Vector3& b)
{
  return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
}

/**
 * Returns true if a and b's elems are within epsilon of each other.
 */
inline bool vequals(const Vector3& a, const Vector3& b, double eps)
{
  return equals(a.x, b.x, eps) &&
         equals(a.y, b.y, eps) &&
         equals(a.z, b.z, eps);
}

/**
 * Returns true if the two vectors are not exactly equal.
 */
inline bool operator != (const Vector3& a, const Vector3& b)
{
  return (a.x != b.x) || (a.y != b.y) || (a.z != b.z);
}

inline Vector3 operator - (const Vector3 &v)
{
  return vector3(-v.x, -v.y, -v.z);
}

inline Vector3 operator + (const Vector3& a, const Vector3& b)
{
  return vector3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline Vector3 operator - (const Vector3& a, const Vector3& b)
{
  return vector3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline Vector3 operator * (double f, const Vector3& v)
{
  return vector3(v.x * f, v.y * f, v.z * f);
}

inline Vector3 operator * (const Vector3& v, double f)
{
  return vector3(v.x * f, v.y * f, v.z * f);
}

inline Vector3 operator / (const Vector3& v, double f) throw(runtime_error)
{
  if(iszero(f))
    throw runtime_error(exceptionMessage(__HERE__, "division by zero"));
  return vector3(v.x/f, v.y/f, v.z/f);
}

/**
 * c = a + b
 */
inline void add(const Vector3& a, const Vector3& b, Vector3& c)
{
  c.x = a.x + b.x;
  c.y = a.y + b.y;
  c.z = a.z + b.z;
}

/**
 * c = a - b
 */
inline void sub(const Vector3& a, const Vector3& b, Vector3& c)
{
  c.x = a.x - b.x;
  c.y = a.y - b.y;
  c.z = a.z - b.z;
}

/**
 * c = s * a
 */
inline void mult(double s, const Vector3& a, Vector3& c)
{
  c.x = a.x * s;
  c.y = a.y * s;
  c.z = a.z * s;
}

/**
 * c = s * a + b
 */
inline void multAdd(double s, const Vector3& a, const Vector3& b, Vector3 &c)
{
  c.x = s*a.x + b.x;
  c.y = s*a.y + b.y;
  c.z = s*a.z + b.z;
}

/**
 * c = s * a + t * b
 */
inline void linear(double s, const Vector3& a, double t, const Vector3& b,
    Vector3 &c)
{
  c.x = s*a.x + t*b.x;
  c.y = s*a.y + t*b.y;
  c.z = s*a.z + t*b.z;
}

inline double norm(const Vector3 &a)
{
  return sqrt(sqr(a.x) + sqr(a.y) + sqr(a.z));
}

inline double normSqr(const Vector3 &a)
{
  return sqr(a.x) + sqr(a.y) + sqr(a.z);
}

inline double length(const Vector3 &a)
{
  return norm(a);
}

inline double lengthSqr(const Vector3 &a)
{
  return normSqr(a);
}

/**
 * Euclidian distance.
 */
inline double dist(const Vector3& a, const Vector3& b)
{
  return sqrt(sqr(a.x - b.x) + sqr(a.y - b.y) + sqr(a.z - b.z));
}

/**
 * Squared euclidian distance.
 */
inline double distSqr(const Vector3& a, const Vector3& b)
{
  return sqr(a.x - b.x) + sqr(a.y - b.y) + sqr(a.z - b.z);
}

/**
 * Normalises the vector, returns the length before normalisation.
 */
inline double normalise(Vector3 &a)
{
  double n = norm(a);
  a /= n;
  return n;
}

/**
 * Vector dot product.
 */
inline double dot(const Vector3& a, const Vector3& b)
{
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

/**
 * Vector cross product, c = a x b
 */
inline Vector3 cross(const Vector3& a, const Vector3& b)
{
  // works in case a or b is c.
  return vector3((a.y*b.z) - (a.z*b.y),
                 (a.z*b.x) - (a.x*b.z),
                 (a.x*b.y) - (a.y*b.x));
}


/**
 * Returns distance of point q from line defined by point p and unit
 * direction vector d.
 */
inline double distPointToLine(const Vector3 &q, const Vector3 &p,
    const Vector3 &d)
{
  Vector3 pq = q - p;
  return dist(d*dot(pq, d), pq);
}

}

#endif
