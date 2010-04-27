/**
 * $Id: WallLine.hh,v 1.0 2008/07/02 13:47:03 mxz Exp mxz $
 */

#ifndef Z_WALL_LINE_HH
#define Z_WALL_LINE_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"

namespace Z
{

class WallLine : public Gestalt
{
public:
	unsigned outLine;											// line which goes out of the image border TODO: ist eigentlich doppelt, weil = lines[0]
	unsigned outLineBorder;								// 0/1/2/3 = left, top, right, bottom border
	Array<unsigned> lines;
	Array<unsigned> linesNearPoints;

	Vector2 point[2];											// Start and end point of the line
	Vector2 dir;													// direction of line, normalised to 1

	bool twoOutLines;

  WallLine(unsigned oL, unsigned oLB, Array<unsigned> &l, Array<unsigned> &lNP, bool wL);
  void CalculateSignificance();
	bool TwoOutLines() {return twoOutLines;}
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& WallLines()
{
  return Gestalts(Gestalt::WALL_LINE);
}
inline WallLine* WallLines(unsigned id)
{
  return (WallLine*)Gestalts(Gestalt::WALL_LINE, id);
}
inline unsigned NumWallLines()
{
  return NumGestalts(Gestalt::WALL_LINE);
}

}

#endif
