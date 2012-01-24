/**
 * $Id$
 */

#ifndef P_FORM_ARCS_HH
#define P_FORM_ARCS_HH

#include "Array.hh"
#include "Edgel.hh"
#include "Arc.hh"
#include "Segment.hh"

namespace P 
{

class FormArcs
{
public:
  static const double MAX_CIRCLE_DIST;  // max dist of edgel from circle
  static const double RANSAC_DIST;  // max dist of edgel from circle
  static const unsigned MIN_LENGTH;  // min number of pixels in arc
  static const double MIN_RADIUS;    // min radius
  static const double MAX_RADIUS;  // max radius
  int FIT_METHOD;
 
private:
  bool EdgelsOnCircle(Array<Edgel> &edgels, unsigned i, unsigned j,
      unsigned k);
  int Support(Array<Edgel> &edgels, int l, int u, int &i, int &j, int &k,
      Vector2 &center, double &radius);
  bool FitArc(Segment *seg, unsigned *cur_pos, Array<Arc*> &arcs);
  void FitArcRANSAC(Segment *seg, int l, int u, Array<Arc*> &arcs);

public:
  FormArcs();
  void Operate(Array<Segment*> &segments, Array<Arc*> &arcs);
  void SetParameter(int method){FIT_METHOD=method;}
};


 
}

#endif

