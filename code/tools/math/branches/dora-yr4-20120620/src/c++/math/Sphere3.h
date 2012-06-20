/**
 * @author Michael Zillich
 * @date August 2009
 */

#ifndef SPHERE3_H
#define SPHERE3_H

#include <Math.hpp>

namespace cogx
{

namespace Math
{

inline
bool pointInsideSphere(const Sphere3 &s, const Vector3 &p)
{
  return dist(p, s.pos) <= s.rad;
}

}

}

#endif

