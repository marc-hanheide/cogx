/**
 * $Id: ConvexArcGroup.hh,v 1.14 2006/11/24 13:47:03 mxz Exp mxz $
 */

#ifndef Z_CONVEX_ARC_GROUP_HH
#define Z_CONVEX_ARC_GROUP_HH

#include "Math.hh"
#include "Array.hh"
#include "Gestalt.hh"
#include "VisionCore.hh"

namespace Z
{

class ConvexArcGroup : public Gestalt
{
public:
  Array<unsigned> arcs;
  double ang_cover;
  double fit_quality;
  double closedness;
  double sig2;

private:
  void CalculateSignificance();

public:
  ConvexArcGroup();
  ConvexArcGroup(Array<unsigned> &a, unsigned l, unsigned u, double s = -HUGE);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  bool HasArc(unsigned arc);
  bool StillConvex(unsigned arc);
  unsigned FirstArc() {return (arcs.Size() > 0 ? arcs[0] : UNDEF_ID);}
  unsigned LastArc() {return (arcs.Size() > 0 ? arcs[arcs.Size()-1] : UNDEF_ID);}
};

extern double ArcGroupLargestAngularGap(Array<unsigned> &arcs);
extern double ArcGroupAngularCoverage(Array<unsigned> &arcs);
extern double ArcGroupClosedness(Array<unsigned> &arcs);
extern double ArcGroupArea(Array<unsigned> &arcs);
extern double ArcGroupGapAreaRatio(Array<unsigned> &arcs);
extern double ArcGroupSignificance(Array<unsigned> &arcs, unsigned l,
    unsigned u);

inline Array<Gestalt*>& ConvexArcGroups()
{
  return Gestalts(Gestalt::CONVEX_ARC_GROUP);
}
inline ConvexArcGroup* ConvexArcGroups(unsigned id)
{
  return (ConvexArcGroup*)Gestalts(Gestalt::CONVEX_ARC_GROUP, id);
}
inline unsigned NumConvexArcGroups()
{
  return NumGestalts(Gestalt::CONVEX_ARC_GROUP);
}

}

#endif

