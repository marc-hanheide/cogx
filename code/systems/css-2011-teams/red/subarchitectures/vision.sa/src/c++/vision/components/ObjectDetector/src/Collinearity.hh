/**
 * $Id: Collinearity.hh,v 1.10 2007/02/11 10:59:14 mxz Exp mxz $
 */

#ifndef Z_COLLINEARITY_HH
#define Z_COLLINEARITY_HH

#include "Gestalt.hh"
#include "Vector2.hh"

namespace Z
{

class Collinearity : public Gestalt
{
private:
  void CalculateColors();
  void CalculateSignificance();

public:
  unsigned line[2];			///< line 0 is the longer, line 1 the shorter one
  double sense[2];   		///< whether line dirs point to or from coll
  unsigned near_point[2];	///< START/END - Point is nearer to intersection
  Vector2 dir[2];    		///< directions of arms, pointing outwards
  double gap;  				///< distance between endpoints
  double gap_cor;  			///< corrected distance if coll terminated by T-jcts
  double col_dist;			///< Color-Distance?
  Vector2 vertex;			///< Midpoint (between end-points of lines)


  Collinearity(unsigned i, unsigned j, unsigned end_i, unsigned end_j);
  virtual void Draw(int detail = 0);
  virtual void DrawInfo();
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  void Recalc();
  void CorrectGap();
  double OpeningAngle();
  unsigned WhichLineIs(unsigned l) throw(Except);
  unsigned OtherLine(unsigned l);
  unsigned WhichEndIs(unsigned l); 
};

inline Array<Gestalt*>& Collinearities()
{
  return Gestalts(Gestalt::COLLINEARITY);
}
inline Collinearity* Collinearities(unsigned id)
{
  return (Collinearity*)Gestalts(Gestalt::COLLINEARITY, id);
}
inline unsigned NumCollinearities()
{
  return NumGestalts(Gestalt::COLLINEARITY);
}

}

#endif
