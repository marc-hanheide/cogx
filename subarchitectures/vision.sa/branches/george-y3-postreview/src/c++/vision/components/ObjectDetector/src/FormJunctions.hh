/**
 * $Id: FormJunctions.hh,v 1.20 2007/03/25 21:35:57 mxz Exp mxz $
 */

#ifndef Z_FORM_JUNCTIONS_HH
#define Z_FORM_JUNCTIONS_HH

#include "VoteImage.hh"
#include "GestaltPrinciple.hh"

namespace Z
{

class FormJunctions : public GestaltPrinciple
{
private:
  bool first_op;
  bool activeEJcts;	// true, for generating E-Junctions
  bool activeTJcts; // true, for generating T-Junctions
  Array<VoteImage::Elem> iscts;
  unsigned isct_ok[12][12]; // admissibility matrix: isct_ok[baseIndex][baseIndex]
  
  void ActiveEJunctions(bool active);
  void ActiveTJunctions(bool active);
  void SetupAdmissibilityMatrix();
  void Rank();
  void InitSearchLines(unsigned line);
  void InitEllipseSearchLines(unsigned ellipse);
  void ExtendSearchLines(unsigned line);
  void ExtendOneSearchLine(unsigned line);
  void ExtendSmartLines(Gestalt::Type type, unsigned idx);
  void FollowEnd(unsigned line, int end, unsigned stop_line);
  void ExtendEnd(unsigned line, unsigned end);
  void ExtendEllipseEnd(unsigned ellipse, unsigned end);
  void CreateLineJunctions(unsigned sline, Array<VoteImage::Elem> &iscts);
  void CreateL(unsigned i, unsigned vtype_i, unsigned j, unsigned vtype_j);
  void CreateC(unsigned i, unsigned vtype_i, unsigned j, unsigned vtype_j);
  void CreateT(unsigned i, unsigned vtype_i, unsigned j, unsigned vtype_j);
  void CreateE(unsigned line, unsigned ellipse, unsigned end, unsigned vertex);
  void SplitLine(unsigned l, const Vector2 &inter, const Vector2 &dir,
      unsigned *l_left, unsigned *l_right,
      unsigned *end_left, unsigned *end_right, unsigned *c_new);
  unsigned SplitLine(unsigned l, const Vector2 &p, unsigned *c_new);
  unsigned FindLineSplitIdx(unsigned l, const Vector2 &p);
  void MoveJunctions(unsigned l1, unsigned l2, unsigned end);
  void UpdateGestalts(unsigned l, unsigned l_new, unsigned c_new);
  void UpdateClosures(unsigned l1, unsigned l2, unsigned coll);
  void UpdateCorners(unsigned l1, unsigned l2);
  void UpdateExtEllipses(unsigned l1, unsigned l2);
  void UpdateCylinders(unsigned l1, unsigned l2);
  void UpdateCubes(unsigned l1, unsigned l2);
  unsigned NewC(unsigned i, unsigned j,unsigned end_i, unsigned end_j,
  	  bool inform);
  unsigned NewL(unsigned i, unsigned j, unsigned end_i, unsigned end_j);
  unsigned NewT(unsigned pole, unsigned left, unsigned right,
      unsigned end_p, unsigned end_l, unsigned end_r,
      unsigned coll, unsigned ljct_l, unsigned ljct_r);

public:
  static VoteImage *vote_img;
  static unsigned baseIndex;	// base index for search lines
  static unsigned baseOffset;	// base offset for ellipse-lines 
								// = number of original lines

  FormJunctions(Config *cfg);
  virtual ~FormJunctions();
  virtual void Reset(const Image *img);
  virtual void Operate(bool incremental);
};

}

#endif
