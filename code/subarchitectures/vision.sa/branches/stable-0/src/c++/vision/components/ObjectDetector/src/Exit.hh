/**
 * $Id: Exit.hh,v 1.0 2008/09/02 13:47:03 mxz Exp mxz $
 */

#ifndef Z_EXIT_HH
#define Z_EXIT_HH

#include "Vector2.hh"
#include "Gestalt.hh"
#include "VisionCore.hh"

namespace Z
{

class Exit : public Gestalt
{
public:
	Vector2 wallBorder[2];								// points of wall at both borders
	Vector2 exitLinesStart[2];						// start points of exit-lines
	Vector2 exitLinesIntersection[2];			// intersection points of exit-lines

	Exit(Vector2 *wB, Vector2 *eLS, Vector2 *eLI);
  void CalculateSignificance();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& Exits()
{
  return Gestalts(Gestalt::EXIT_);
}
inline Exit* Exits(unsigned id)
{
  return (Exit*)Gestalts(Gestalt::EXIT_, id);
}
inline unsigned NumExits()
{
  return NumGestalts(Gestalt::EXIT_);
}

}

#endif
