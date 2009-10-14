/**
 * $Id: Line.hh,v 1.21 2007/04/14 20:50:59 mxz Exp mxz $
 *
 */

#ifndef Z_LINE_HH
#define Z_LINE_HH

#include <list>
#include <set>
#include "Vector2.hh"
#include "VisionCore.hh"
#include "Segment.hh"

namespace Z
{

class LJunction;
class Collinearity;
class TJunction;
class Closure;

/**
 * A directed straight line.
 */
class Line : public Gestalt
{
protected:
  RGBColor mean_col[2];  ///< mean color LEFT/RIGHT
  void CalculateParameters();
  virtual void CalculateColors() = 0;
  void DrawArrow();
  void MoveJunctions(Line *l2, int end);

public:
  Vector2 point[2];    ///< end points, START/END
  Vector2 dir;         ///< direction of the line, normalised to 1
  Vector2 tang[2];     ///< tangents STAR/END
  double phi;          ///< angular direction
  double len;
  //double s;          ///< hessian form: distance
  //double theta;      ///< hessian form: angle
  Array<Collinearity*> coll[2];    ///< collinearities
  Array<LJunction*> l_jct[2][2];   ///< L-junctions at START/END and LEFT/RIGHT
  TJunction* t_jct[2];             ///< T-junctions at ends, START/END
  Array<TJunction*> pt_jct[2][2];  ///< passive T-jcts, START/END and LEFT/RIGHT
  Array<Closure*> closures; // TODO: have two lists, for both senses
  int label; //[2];       ///< label LEFT/RIGHT
  double energy;          ///< energy for global conistency
  double stability;       ///< stability for global consistency
  set<Line*> neighbors;   ///< neighbors for global consistency
  Line* next;          ///< next line if split
  Line* defer_vote;    ///< lines created by splitting don't vote themselves
                       /// but defer votes to original line

protected:
  Line(VisionCore *c);

public:
  virtual void Draw(int detail = 0);
  virtual void DrawInfo();
  virtual const char* GetInfo();
  double Length() {return len;}
  double MinEndpointDistance(const Line*l);
  double DistanceToPoint(const Vector2 &q);
  void AddLJunction(int end, int side, LJunction* jct);
  void AddCollinearity(int end, Collinearity* co);
  void AddPassiveTJunction(int end, int side, TJunction* jct);
  RGBColor MeanCol(int side) {return mean_col[side];}
  virtual Line* Split(const Vector2 &p) = 0;
  bool IsSplit() {return next != 0;}
};

/**
 * Visible line fitted to edgels.
 */
class VisibleLine : public Line
{
private:
  void CalculateSignificance();
  virtual void CalculateColors();
  void DrawVotes();
  unsigned FindSplitIdx(const Vector2 &p);

public:
  Segment* seg;        ///< the originating segment
  unsigned idx[2];     ///< index of start and end point

  VisibleLine(VisionCore *c, Segment* s, unsigned i, unsigned j);
  void Recalc();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  unsigned NumEdgels() {return idx[END] - idx[START] + 1;}
  virtual Line* Split(const Vector2 &p);
};

/**
 * Note: just an idea for now
 * Cotermination Line
 * Whenever a segment terminates unexplained (no T, L or C) create a
 * termination. Link two terminations with cotermination line = inferred
 * contour.
 */
class CotLine : public Line
{
};

inline Array<Gestalt*>& Lines(VisionCore *core)
{
  return core->Gestalts(Gestalt::LINE);
}
inline Line* Lines(VisionCore *core, unsigned id)
{
  return (Line*)core->Gestalts(Gestalt::LINE, id);
}
inline unsigned NumLines(VisionCore *core)
{
  return core->NumGestalts(Gestalt::LINE);
}

}

#endif

