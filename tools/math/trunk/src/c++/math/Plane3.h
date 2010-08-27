/**
 * @author Michael Zillich
 * @date February 2010
 */

#ifndef PLANE3_H
#define PLANE3_H

#include <vector>
#include <iostream>
#include <stdexcept>
#include <cast/core/CASTUtils.hpp>
#include <cogxmath_base.h>
#include <Vector3.h>
#include <Math.hpp>

namespace cogx
{

namespace Math
{

/**
 * note: strictly speaking this is not necessary (the plane equation can be
 * scaled by an arbitrary factor) but it is generally a good idea to normalise
 * the equation
 */
inline void normalisePlane(Plane3 &pl) throw(runtime_error)
{
  double s = sqrt(sqr(pl.a) + sqr(pl.b) + sqr(pl.c));
  if(iszero(s))
    throw runtime_error(exceptionMessage(__HERE__, "invalid plane - zero normal vector"));
  pl.a /= s;
  pl.b /= s;
  pl.c /= s;
  pl.d /= s;
  if(pl.d < 0)
  {
    pl.a = -pl.a;
    pl.b = -pl.b;
    pl.c = -pl.c;
    pl.d = -pl.d;
  }
}

inline Plane3 plane3(double a, double b, double c, double d)
{
  Plane3 pl;
  pl.a = a;
  pl.b = b;
  pl.c = c;
  pl.d = d;
  normalisePlane(pl);
  return pl;
}

/**
 * create plane from 3 points
 * returns false if points were in degenerate configuration and no plane could
 * be calcuated, true otherwise
 */
inline bool planeFromPoints(Plane3 &pl, const Vector3 &p, const Vector3 &q,
    const Vector3 &r)
{
  Vector3 pq = q - p;
  Vector3 pr = r - p;
  Vector3 n = cross(pq, pr);
  if(!isZero(n))
  {
    pl = plane3(n.x, n.y, n.z, -dot(p, n));
    return true;
  }
  else
  {
    return false;
  }
}

inline double distPointToPlane(const Vector3 &p, const Plane3 &pl)
{
  return p.x*pl.a + p.y*pl.b + p.z*pl.c + pl.d;
}

/**
 * Print a plane in text form to a stream: '[a b c d]`
 */
inline void writeText(ostream &os, const Plane3 &p)
{
  os << '[' << p.a << ' ' << p.b << ' ' << p.c <<  ' ' << p.d << ']';
}

/**
 * Read a plane in text from a stream.
 * The expected format is: '[a b c d]', white spaces are ignored.
 */
inline void readText(istream &is, Plane3 &p) throw(runtime_error)
{
  char c;
  is >> c;
  if(c == '[')
  {
    is >> p.a >> p.b >> p.c >> p.d >> c;
    if(c != ']')
      throw runtime_error(exceptionMessage(__HERE__,
            "error reading Plane3: ']' expected"));
  }
  else
    throw runtime_error(exceptionMessage(__HERE__,
          "error reading Plane3: '[' expected"));
}

/**
 * Writing to a stream is taken to be a textual output, rather than a
 * serialisation of the actual binary data.
 */
inline ostream& operator<<(ostream &os, const Plane3 &a)
{
  writeText(os, a);
  return os;
}

/**
 * Reading from a stream is taken to read a textual input, rather than
 * de-serialising the actual binary data.
 */
inline istream& operator>>(istream &is, Plane3 &a)
{
  readText(is, a);
  return is;
}

}

}

#endif

