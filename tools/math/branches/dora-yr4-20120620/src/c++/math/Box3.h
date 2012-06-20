/**
 * @author Michael Zillich
 * @date August 2009
 */

#ifndef BOX3_H
#define BOX3_H

#include <Math.hpp>

namespace cogx
{

namespace Math
{

inline
bool pointInsideBox(const Box3 &b, const Vector3 &p)
{
  return b.pos.x - b.size.x/2. <= p.x && p.x <= b.pos.x + b.size.x/2. &&
         b.pos.y - b.size.y/2. <= p.y && p.y <= b.pos.y + b.size.y/2. &&
         b.pos.z - b.size.z/2. <= p.z && p.z <= b.pos.z + b.size.z/2.;
}

}

}

#endif

