/**
 * $Id: Flap.cc,v 1.2 2006/11/24 13:47:03 mxz Exp mxz $
 */

#include <assert.h>
#include "Closure.hh"
#include "Flap.hh"

namespace Z
{

Flap::Flap(VisionCore *c, Closure *c0, Closure *c1)
  : Gestalt(c, FLAP)
{
  assert(c0 != 0 && c1 != 0);
  clos[0] = c0;
  clos[1] = c1;
}

void Flap::Draw(int detail)
{
  clos[0]->Draw(detail);
  clos[1]->Draw(detail);
}

const char* Flap::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size, "%sclosure: %u %u\n",
      Gestalt::GetInfo(), clos[0]->ID(), clos[1]->ID());
  return info_text;
}

bool Flap::IsAtPosition(int x, int y)
{
  return clos[0]->IsAtPosition(x, y) || clos[1]->IsAtPosition(x, y);
}

}

