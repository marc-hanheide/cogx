/**
 * $Id: Segment.hh,v 1.11 2006/11/24 13:47:03 mxz Exp mxz $
 *
 */

#ifndef Z_SEGMENT_HH
#define Z_SEGMENT_HH

#include "Array.hh"
#include "VisionCore.hh"
#include "Gestalt.hh"
#include "Edgel.hh"

namespace Z
{
  
class Segment : public Gestalt
{
private:
  void CalculateSignificance();

public:
  Array<Edgel> edgels;

  Segment(VisionCore *c, const Array<Edgel> &arr);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  Vector2 Tangent(int i, int l = 0, int u = UNDEF_ID);
};

inline Array<Gestalt*>& Segments(VisionCore *core)
{
  return core->Gestalts(Gestalt::SEGMENT);
}
inline Segment* Segments(VisionCore *core, unsigned id)
{
  return (Segment*)core->Gestalts(Gestalt::SEGMENT, id);
}
inline unsigned NumSegments(VisionCore *core)
{
  return core->NumGestalts(Gestalt::SEGMENT);
}

}

#endif

