/**
 * $Id: Rectangle.hh,v 1.8 2007/02/04 23:53:03 mxz Exp mxz $
 */

#ifndef Z_RECTANGLE_HH
#define Z_RECTANGLE_HH

#include "Gestalt.hh"
#include "VisionCore.hh"

namespace Z
{

class LJunction;
class Closure;

class Rectangle : public Gestalt
{
private:
  void CalculateSignificance();

public:
  Closure *clos;      ///< the underlying closure
  LJunction *jcts[4]; ///< junctions, in counter-clockwise order

  Rectangle(VisionCore *c, Closure *cl, LJunction *js[4]);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  double SumGaps();
  double Area();
  double Circumference();
};

inline Array<Gestalt*>& Rectangles(VisionCore *core)
{
  return core->Gestalts(Gestalt::RECTANGLE);
}
inline Rectangle* Rectangles(VisionCore *core, unsigned id)
{
  return (Rectangle*)core->Gestalts(Gestalt::RECTANGLE, id);
}
inline unsigned NumRectangles(VisionCore *core)
{
  return core->NumGestalts(Gestalt::RECTANGLE);
}

}

#endif

