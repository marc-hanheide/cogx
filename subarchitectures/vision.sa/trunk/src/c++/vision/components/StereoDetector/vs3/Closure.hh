/**
 * @file Closure.hh
 * @author Michael Zillich
 * @date 2007
 * @version 0.1
 * @brief Header file of Gestalt Closure.
 **/

#ifndef Z_CLOSURE_HH
#define Z_CLOSURE_HH

#include <set>
#include <map>
#include "Gestalt.hh"
#include "VisionCore.hh"

namespace Z
{

class Line;
class Collinearity;
class LJunction;

/**
 * @brief Lines are in counter-clockwise order.\n
 * jct i is the jct between line i-1 and line i.
 */
class Closure : public Gestalt
{
public:
  Array<LJunction*> jcts;									/// HACK ARI: Diese Liste hat Löcher (mit 0 gefüllt), daher ist ...
  Array<Collinearity*> colls;							/// Anzahl nicht = jcts-Anzahl (Abfrage über NumLJunctions();)
  Array<Line*> lines;
  Array<int> senses;
  map<Closure*, double> neighbors_above;
  map<Closure*, double> neighbors_below;
  map<Closure*, double> neighbors_inside;
  map<Closure*, double> neighbors_outside;
  map<Closure*, double> neighbors_aside;
  set<Closure*> neighbors;
  double stability;
  double energy;
  int label;

  Closure(VisionCore *c);
  void FindTNeighbors();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  unsigned NumLJunctions();
  unsigned NumCollinearities();
  unsigned NumLines() {return lines.Size();}
  double SumGaps();
  double SumLines();
  double Circumference();
  void CalculateSignificance();
  Vector2 GetVertex(unsigned i);
  double GetJunctionSig(unsigned i);
  bool Inside(Vector2 p);
  double Area();
};

inline Array<Gestalt*>& Closures(VisionCore *core)
{
  return core->Gestalts(Gestalt::CLOSURE);
}
inline Closure* Closures(VisionCore *core, unsigned id)
{
  return (Closure*)core->Gestalts(Gestalt::CLOSURE, id);
}
inline unsigned NumClosures(VisionCore *core)
{
  return core->NumGestalts(Gestalt::CLOSURE);
}

}

#endif

