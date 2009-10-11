/**
 * $Id: ExtClosure.hh,v 1.0 2007/10/31 16:45:03 mxz Exp mxz $
 */

#ifndef Z_EXTRECTANGLE_HH
#define Z_EXTRECTANGLE_HH

#include "Gestalt.hh"
#include "VisionCore.hh"

namespace Z
{

class ExtClosure : public Gestalt
{
public:
  unsigned clos[2];				// closures
  Array<unsigned> jcts;			// TODO: ARI: Wahrscheinlich gleich wie bei colls
  Array<unsigned> colls;		// TODO: ARI: Array ist so groß wie alle jcts => nur colls sind eingetragen
  Array<unsigned> lines;		// lines of ExtClosure
  Array<unsigned> senses;		// TODO: ARI: Sollte gleich groß wie lines sein?	
  Array<unsigned> sharedLines;	// sharedLines

  ExtClosure(unsigned c0, unsigned c1, Array<unsigned> l, Array<unsigned> shL);
  Vector2 GetVertex(unsigned i);
  void CalculateSignificance();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& ExtClosures()
{
  return Gestalts(Gestalt::EXTCLOSURE);
}
inline ExtClosure* ExtClosures(unsigned id)
{
  return (ExtClosure*)Gestalts(Gestalt::EXTCLOSURE, id);
}
inline unsigned NumExtClosures()
{
  return NumGestalts(Gestalt::EXTCLOSURE);
}

}

#endif
