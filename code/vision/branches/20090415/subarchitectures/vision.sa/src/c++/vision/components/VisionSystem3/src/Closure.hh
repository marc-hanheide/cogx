/**
 * $Id: Closure.hh,v 1.8 2007/03/25 21:35:57 mxz Exp mxz $
 */

#ifndef Z_CLOSURE_HH
#define Z_CLOSURE_HH

#include <set>
#include <map>
#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"

namespace Z
{

class Closure : public Gestalt
{
public:
  //Array<unsigned> sides; ?
  Array<unsigned> jcts;			// TODO: ARI: Wahrscheinlich gleich wie bei colls
  Array<unsigned> colls;		// TODO: ARI: Array ist so groß wie alle jcts => nur colls sind eingetragen
  Array<unsigned> lines;		// TODO: ARI: Wie sieht es hier aus?
  Array<unsigned> senses;		// TODO: ARI: Sollte gleich groß wie lines sein?
  map<unsigned, double> neighbors_above;	// TODO ARI: Werden diese irgendwo verwendet?
  map<unsigned, double> neighbors_below;
  map<unsigned, double> neighbors_inside;
  map<unsigned, double> neighbors_outside;
  map<unsigned, double> neighbors_aside;
  set<unsigned> neighbors;
  double stability;
  double energy;
  int label;

  Closure();
  Closure(Array<unsigned> l, Array<unsigned> lj, Array<unsigned> c, Array<unsigned> s);
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
};

inline Array<Gestalt*>& Closures()
{
  return Gestalts(Gestalt::CLOSURE);
}
inline Closure* Closures(unsigned id)
{
  return (Closure*)Gestalts(Gestalt::CLOSURE, id);
}
inline unsigned NumClosures()
{
  return NumGestalts(Gestalt::CLOSURE);
}

}

#endif
