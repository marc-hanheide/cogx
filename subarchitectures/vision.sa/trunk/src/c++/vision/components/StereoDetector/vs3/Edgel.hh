/**
 * @file Edgel.hh
 * @author Michael Zillich
 * @date 2006
 * @version 0.1
 * @brief Prototype class for Edgels.
 **/

#ifndef Z_EDGEL_HH
#define Z_EDGEL_HH

// #include "Namespace.hh"
#include "Vector.hh"

namespace Z
{

class Edgel
{
public:
  Vector2 p;								///< Point (x,y)
  double dir;								///< Direction

  Edgel() {}
  Edgel(const Vector2 &p_in) {p = p_in;}
  Edgel(const Vector2 &p_in, double d_in) {p = p_in; dir = d_in;}
};

}

#endif

