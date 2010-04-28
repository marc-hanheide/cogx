/**
 * $Id: ExtRectangle.hh,v 1.0 2007/10/23 16:45:03 mxz Exp mxz $
 */

#ifndef Z_EXTRECTANGLE_HH
#define Z_EXTRECTANGLE_HH

#include "Gestalt.hh"
#include "VisionCore.hh"

namespace Z
{

class ExtRectangle : public Gestalt
{
public:
  unsigned rect;		// rectangle
  Array<unsigned> extJcts;	// extended L-Junctions
  Array<unsigned> extLines;	// extended Lines
  Array<unsigned> extColls;	// extended Collinearities

  ExtRectangle(unsigned rec, Array<unsigned> jcts, 
	Array<unsigned> lines, Array<unsigned> colls);
  void CalculateSignificance();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& ExtRectangles()
{
  return Gestalts(Gestalt::EXTRECTANGLE);
}
inline ExtRectangle* ExtRectangles(unsigned id)
{
  return (ExtRectangle*)Gestalts(Gestalt::EXTRECTANGLE, id);
}
inline unsigned NumExtRectangles()
{
  return NumGestalts(Gestalt::EXTRECTANGLE);
}

}

#endif
