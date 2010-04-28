/**
 * $Id: TJunction.hh,v 1.6 2007/03/25 21:35:57 mxz Exp mxz $
 */

#ifndef Z_T_JUNCTION_HH
#define Z_T_JUNCTION_HH

#include "Vector2.hh"
#include "Gestalt.hh"

namespace Z
{

// we have a LEFT bar line, a RIGHT bar line and a MID pole line
#define POLE MID

class TJunction : public Gestalt
{
private:
  void CalculateSignificance();

public:
  unsigned line[3];     ///< LEFT/RIGHT/POLE
  unsigned lend[3][2];  ///< line ends, LEFT/RIGHT/POLE and INNER/OUTER
  unsigned coll;        ///< the collinearity between left and right line
  unsigned ljct[2];     ///< the L-jcts between pole and left and right line
  Vector2 isct;
  Vector2 dir;  ///< pointing from pole inner end to intersection, i.e. to the
                // occluding surface
  double gap;

  TJunction(unsigned pole, unsigned left, unsigned right,
      unsigned end_p, unsigned end_l, unsigned end_r,
      double g, const Vector2 &inter);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  unsigned WhichLineIs(unsigned l) throw(Except);
  void Recalc();
};

inline Array<Gestalt*>& TJunctions()
{
  return Gestalts(Gestalt::T_JUNCTION);
}
inline TJunction* TJunctions(unsigned id)
{
  return (TJunction*)Gestalts(Gestalt::T_JUNCTION, id);
}
inline unsigned NumTJunctions()
{
  return NumGestalts(Gestalt::T_JUNCTION);
}

}

#endif

