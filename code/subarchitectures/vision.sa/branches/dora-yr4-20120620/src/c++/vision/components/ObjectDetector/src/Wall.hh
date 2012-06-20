/**
 * $Id: WALL.hh,v 1.0 2007/12/13 13:47:03 mxz Exp mxz $
 */

#ifndef Z_WALL_HH
#define Z_WALL_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"

namespace Z
{

class Wall : public Gestalt
{
public:
	bool corner;														// true, if wall with an corner
	bool longLine;													// true, if wall is created only from one long line
	unsigned cornerPos;											// position of corner in wallLine (connection point between two lines)
	Vector2 borderPoints[2];								// the 2 points at the border of the image
	unsigned wallLines[2];									// the 2 wall lines of the wall
	Vector2 points[2];											// connection line points // if corner => corner point and corner edge top point
	double lineLength;											// sum of length of both wall lines (not for "corner-walls")

	Wall(bool c, bool lL, Vector2 *bP, unsigned *wL, Vector2 *p, unsigned cP);
	void LengthMasking();
	bool IsInside(unsigned wall);
  void CalculateSignificance();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& Walls()
{
  return Gestalts(Gestalt::WALL);
}
inline Wall* Walls(unsigned id)
{
  return (Wall*)Gestalts(Gestalt::WALL, id);
}
inline unsigned NumWalls()
{
  return NumGestalts(Gestalt::WALL);
}

}

#endif
