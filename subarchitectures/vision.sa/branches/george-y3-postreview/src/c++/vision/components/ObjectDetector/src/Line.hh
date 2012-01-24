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
#include "Gestalt.hh"

namespace Z
{

/**
 * A directed straight line.
 */
class LineBase : public Gestalt
{
protected:
  RGBColor mean_col[2];  ///< mean color LEFT/RIGHT
  void CalculateParameters();
  virtual void CalculateColors() = 0;
  void DrawArrow();

public:
  Vector2 point[2];    ///< end points, START/END
  Vector2 dir;         ///< direction of the line, normalised to 1
  Vector2 tang[2];     ///< tangents START/END
  double phi;          ///< angular direction
  double len;						///< Length of line
  //double s;          ///< hessian form: distance
  //double theta;      ///< hessian form: angle
  Array<unsigned> coll[2];  ///< collinearities START/END
  Array<unsigned> l_jct[2][2];  ///< L-junctions at START/END and LEFT/RIGHT
  unsigned t_jct[2];     		///< T-junctions at ends, START/END
  Array<unsigned> e_jct[2];		///< Ellipse-Junction START/END
  Array<unsigned> pt_jct[2][2]; ///< passive T-jcts, START/END and LEFT/RIGHT
  Array<unsigned> corners;
  Array<unsigned> extEllipses;
  Array<unsigned> cylinders;													// TODO ARI: sind cylinder immer eingetragen?
  Array<unsigned> closures; 	///< TODO: have two lists, for both senses
  Array<unsigned> rects;		///< Rectangles									// TODO ARI: Rectangles eintragen: siehe Rectangles.cc
  Array<unsigned> cubes;		///< Cubes										// TODO ARI: Cubes eintragen (FormJunctions::UpdateCube)
  int label; //[2];  ///< label LEFT/RIGHT
  double energy;  ///< energy for global conistency
  double stability;  ///< stability for global consistency
  set<unsigned> neighbors;  ///< neighbors for global consistency

protected:
  LineBase();

public:
  virtual void Draw(int detail = 0);
  virtual void DrawInfo();
  virtual const char* GetInfo();
  double Length() {return len;}
  double MinEndpointDistance(const LineBase *l);
  double DistanceToPoint(const Vector2 &q);
  void AddColLine(unsigned l, double sig);
  void AddLJunction(unsigned end, unsigned side, unsigned jct);
  void AddEJunction(unsigned end, unsigned eJct);
  void AddCollinearity(unsigned end, unsigned co);
  void AddExtEllipse(unsigned eEll);
  void AddCylinder(unsigned cyl);
  void AddRectangle(unsigned rect);
  void AddCube(unsigned cube);
  void AddPassiveTJunction(unsigned end, unsigned side, unsigned jct);
  RGBColor MeanCol(int side) {return mean_col[side];}
};

/**
 * @brief Visible line fitted to edgels.
 */
class Line : public LineBase
{
private:
  void CalculateSignificance();
  virtual void CalculateColors();
  void CalculateWeight();
  void DrawVotes();

public:
  unsigned seg;         ///< the originating segment
  unsigned idx[2];      ///< index of start and end point
  unsigned next;        ///< next line if split
  unsigned defer_vote;  ///< lines created by splitting don't vote themselves
                        /// but defer votes to original line

  Line(unsigned seg_id, unsigned i, unsigned j);
  void Recalc();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  void GetAllLJunctions(Array<unsigned> *lj);
  void GetLJunctions(unsigned end, Array<unsigned> &lj);
  void GetAllCollinearities(Array<unsigned> *col);
  void GetCollinearities(unsigned end, Array<unsigned> &c);
  void GetAllExtEllipses(Array<unsigned> *ee);
  unsigned NumEdgels() {return idx[END] - idx[START] + 1;}
  bool IsSplit() {return next != UNDEF_ID;}
};

inline Array<Gestalt*>& Lines()
{
  return Gestalts(Gestalt::LINE);
}
// TODO: hmm,actually this should return a LineBase pointer
inline Line* Lines(unsigned id)
{
  return (Line*)Gestalts(Gestalt::LINE, id);
}
inline unsigned NumLines()
{
  return NumGestalts(Gestalt::LINE);
}

/**
 * Note: just an idea for now
 * Cotermination Line
 * Whenever a segment terminates unexplained (no T, L or C) create a
 * termination. Link two terminations with cotermination line = inferred
 * contour.
 */
class CotLine : public LineBase
{
};

}

#endif
