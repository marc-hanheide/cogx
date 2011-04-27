/**
 * $Id: Edgel.hh,v 1.5 2006/11/24 13:47:03 mxz Exp mxz $
 */

#ifndef Z_EDGEL_HH
#define Z_EDGEL_HH

#include "Namespace.hh"
#include "Vector2.hh"

namespace Z
{

class Edgel
{
public:
  Vector2 p;
  double dir;

  Edgel() {}
  Edgel(const Vector2 &p_in, double d_in) {p = p_in; dir = d_in;}
};

}

#endif

