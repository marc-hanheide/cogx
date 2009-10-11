/**
 * $Id: Corner.hh,v 1.2 2007/11/26 13:47:03 mxz Exp mxz $
 */

#ifndef Z_CORNER_HH
#define Z_CORNER_HH

#include "Gestalt.hh"
#include "Array.hh"
#include "VisionCore.hh"
#include "Vector2.hh"

namespace Z
{

class Corner : public Gestalt
{
public:
  Array<unsigned> ljcts;	// ljcts of the corner (min. 2 / max. 3)
  Array<unsigned> lines;	// lines of the corner (line[0] is shared line)
  Array<unsigned> near_point; // which end of line is nearer to the corner (S/E)
  Vector2 isct;			// intersection point of the corner
  double gap;			// gap between the intersection points of the L-Jcts
  double meanLength; 	// mean of the length from the lines

  Corner(Array<unsigned> l, Array<unsigned> j, Array<unsigned> np);
  void AddLJunctions(Array<unsigned> &lj, Array<unsigned> &li, Array<unsigned> &np);
  Vector2 CalculateIntersection(unsigned lj0, unsigned lj1, double &gap);
  void CalculateSignificance(double gap);
  void Recalc(unsigned lj);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& Corners()
{
  return Gestalts(Gestalt::CORNER);
}
inline Corner* Corners(unsigned id)
{
  return (Corner*)Gestalts(Gestalt::CORNER, id);
}
inline unsigned NumCorners()
{
  return NumGestalts(Gestalt::CORNER);
}

}

#endif
