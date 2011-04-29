/**
 * $Id: Collinearity.hh,v 1.10 2007/02/11 10:59:14 mxz Exp mxz $
 */

#ifndef Z_COLLINEARITY_HH
#define Z_COLLINEARITY_HH

#include "Gestalt.hh"
#include "Vector2.hh"

namespace Z
{

class Line;

class Collinearity : public Gestalt
{
private:
  void CalculateColors();
  void CalculateSignificance();

public:
  Line* line[2];   ///< line 0 is the longer, line 1 the shorter one
  int near_point[2];
  double gap;      ///< distance between endpoints
  double gap_cor;  ///< corrected distance if coll terminated by T-jcts
  double col_dist;
  Vector2 vertex;  // TODO: change name to midpoint or something

  Collinearity(VisionCore *c, Line *line_i, Line *line_j, int end_i, int end_j);
  virtual void Draw(int detail = 0);
  virtual void DrawInfo();
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  void Recalc();
  void CorrectGap();
  int WhichLineIs(Line *l) throw(Except);
  Line* OtherLine(Line *l);
  int WhichEndIs(Line *l); 
};

inline Array<Gestalt*>& Collinearities(VisionCore *core)
{
  return core->Gestalts(Gestalt::COLLINEARITY);
}
inline Collinearity* Collinearities(VisionCore *core, unsigned id)
{
  return (Collinearity*)core->Gestalts(Gestalt::COLLINEARITY, id);
}
inline unsigned NumCollinearities(VisionCore *core)
{
  return core->NumGestalts(Gestalt::COLLINEARITY);
}

}

#endif

