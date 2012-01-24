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
 * Project a point to a plane.
 */
inline Vector3 projectPointToPlane(const Plane3 &pl, const Vector3 &p)
{
  // let plane origin o = [-a*d -b*d -c*d] and normal vector n = [a b c]
  // (plane equation is normalised)
  // then p' = p - ((p - o)*n)*n
  return vector3(p.x*(1. - sqr(pl.a)) - pl.a*(pl.b*p.y + pl.c*p.z + pl.d),
                 p.y*(1. - sqr(pl.b)) - pl.b*(pl.a*p.x + pl.c*p.z + pl.d),
                 p.z*(1. - sqr(pl.c)) - pl.c*(pl.a*p.x + pl.b*p.y + pl.d));
}

inline void definePlaneAxes(const Plane3 &pl, Matrix33 &rot)
{
  // first find the major direction of the plane normal vector (its largest
  // component)
  int majorDir = 0;
  if(fabs(pl.b) > fabs(pl.a))
  {
    if(fabs(pl.c) > fabs(pl.b))
      majorDir = 2;
    else
      majorDir = 1;
  }
  else
  {
    if(fabs(pl.c) > fabs(pl.a))
      majorDir = 2;
    else
      majorDir = 0;
  }

  // z axis of the plane coordsys is always along plane normal vector
  Vector3 z = vector3(pl.a, pl.b, pl.c);
  // x axis of the plane coordsys is defined to be parallel to either xy, yz or
  // xz plane, depending on the major direction of the normal vector
  Vector3 x;
  // y axis follows as cross product
  Vector3 y;
  // if plane is mostly parallel to yz plane, x is major dir and thus guaranteed
  // nonzero
  if(majorDir == 0)
  {
    // if plane is more parallel to y axis than to z axis
    if(fabs(pl.b) < fabs(pl.c))
      // then x is parallel to global xy plane
      x = vector3(-pl.b/pl.a, 1., 0.);
    else
      // x is parallel to global xz plane
      x = vector3(-pl.c/pl.a, 0., 1.);
  }
  else if(majorDir == 1)
  {
    if(fabs(pl.a) < fabs(pl.c))
      x = vector3(1., -pl.a/pl.b, 0.);
    else
      x = vector3(0., -pl.c/pl.b, 1.);
  }
  else // majorDir == 2
  {
    if(fabs(pl.a) < fabs(pl.b))
      x = vector3(1., 0., -pl.a/pl.c);
    else
      x = vector3(0., 1., -pl.b/pl.c);
  }
  normalise(x);
  y = cross(z, x);

  // now set pose elements
  setColumn(rot, 0, x);
  setColumn(rot, 1, y);
  setColumn(rot, 2, z);
}

/**
 * Define a 'nice' coordinate system for a plane.
 * origin in [-a*d -b*d -c*d]
 * z = n = [a b c]
 * x and y defined parallel to global x or y
 */
inline void definePlaneCoordSys(const Plane3 &pl, Pose3 &pose)
{
  // orgin of the plane coordsys is along a ray normal to the plane and passing
  // through world origin (note: plane equation is normalised)
  pose.pos = vector3(-pl.a*pl.d, -pl.b*pl.d, -pl.c*pl.d);
  definePlaneAxes(pl, pose.rot);
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

