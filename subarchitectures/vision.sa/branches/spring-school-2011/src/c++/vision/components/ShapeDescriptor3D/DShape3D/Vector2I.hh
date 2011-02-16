/**
 * $Id: Vector2I.hh,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 */

#ifndef VECTOR2I_HH
#define VECTOR2I_HH

#include "PNamespace.hh"

namespace P
{

/**
 *  Vector2 
 */
class Vector2I 
{

public:
  int x, y;
  Vector2I();
  Vector2I(const Vector2I &);
  Vector2I(int xx, int yy) : x(xx), y(yy) {}
  ~Vector2I(){}
  Vector2I &operator=(const Vector2I &rhs);
  int operator==(const Vector2I &rhs) const;
  int operator<(const Vector2I &rhs) const;
};

}

#endif
