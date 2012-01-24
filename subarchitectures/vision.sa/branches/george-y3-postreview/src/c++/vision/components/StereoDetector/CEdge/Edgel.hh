/**
 * $Id$
 */

#ifndef P_EDGEL_HH
#define P_EDGEL_HH

#include "PNamespace.hh"
#include "Vector2.hh"

namespace P
{

class Edgel
{
public:
  Vector2 p;
  double dir;

  Edgel() {}
  Edgel(const Vector2 &p_in, double d_in) {p = p_in; dir = d_in;}
  Edgel(const Vector2 &p_in) : dir(0.) {p = p_in;}
};

}

#endif

