/**
 * @file FormArcs.cc
 * @author Michael Zillich
 * @date Februar 2006
 * @version 0.1
 * @brief Gestalt principle FormArcs
 **/

#ifndef Z_FORM_ARCS_HH
#define Z_FORM_ARCS_HH

#include "GestaltPrinciple.hh"
#include "Array.hh"
#include "Edgel.hh"

namespace Z
{

/**
 *	@brief Class of Gestalt principle FormArcs
 */
class FormArcs : public GestaltPrinciple
{
public:
  static const double MAX_CIRCLE_DIST;  // max dist of edgel from circle
  static const double RANSAC_DIST;  // max dist of edgel from circle
  static const unsigned MIN_LENGTH;  // min number of pixels in arc
  static const double MIN_RADIUS;    // min radius
  static const double MAX_RADIUS;  // max radius
  int FIT_METHOD;
 
private:
  bool done;

  void Create();
  void Rank();
  bool EdgelsOnCircle(Array<Edgel> &edgels, unsigned i, unsigned j,
      unsigned k);
  int Support(Array<Edgel> &edgels, int l, int u, int &i, int &j, int &k,
      Vector2 &center, double &radius);
  bool FitArc(unsigned seg, unsigned *cur_pos);
  void FitArcRANSAC(unsigned seg, int l, int u);

public:
  FormArcs(Config *cfg);
  virtual void Reset(const Image *img);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate() {return !done;}
};
 
}

#endif

