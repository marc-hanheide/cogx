/**
 * $Id: Flap.hh,v 1.2 2006/11/24 13:47:03 mxz Exp mxz $
 */

#ifndef Z_FLAP_HH
#define Z_FLAP_HH

#include "Gestalt.hh"
#include "VisionCore.hh"

namespace Z
{

class Closure;

class Flap : public Gestalt
{
public:
  Closure *clos[2];

  Flap(VisionCore *c, Closure *c0, Closure *c1);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& Flaps(VisionCore *core)
{
  return core->Gestalts(Gestalt::FLAP);
}
inline Flap* Flaps(VisionCore *core, unsigned id)
{
  return (Flap*)core->Gestalts(Gestalt::FLAP, id);
}
inline unsigned NumFlaps(VisionCore *core)
{
  return core->NumGestalts(Gestalt::FLAP);
}

}

#endif

