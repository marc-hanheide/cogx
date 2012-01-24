/**
 * $Id: Arc.hh,v 1.18 2006/11/24 13:47:03 mxz Exp mxz $
 */

#ifndef Z_A_JUNCTION_HH
#define Z_A_JUNCTION_HH

#include "Vector2.hh"
#include "VisionCore.hh"
#include "Gestalt.hh"
#include "Segment.hh"

namespace Z
{

class Arc;

class AJunction : public Gestalt
{
private:
  //void CalculateSignificance();

public:
  Arc *arc[2];   ///< START and END arc

  AJunction(VisionCore *vc, Arc *arc_i, Arc *arc_j, int end_i, int end_j);
  virtual void Draw(int detail = 0);
  virtual void DrawInfo();
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& AJunctions(VisionCore *core)
{
  return core->Gestalts(Gestalt::A_JUNCTION);
}
inline AJunction* AJunctions(VisionCore *core, unsigned id)
{
  return (AJunction*)core->Gestalts(Gestalt::A_JUNCTION, id);
}
inline unsigned NumAJunctions(VisionCore *core)
{
  return core->NumGestalts(Gestalt::A_JUNCTION);
}

}

#endif


