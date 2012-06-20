/**
 * $Id: FormJunctions.hh,v 1.20 2007/03/25 21:35:57 mxz Exp mxz $
 */

#ifndef Z_FORM_JUNCTIONS_HH
#define Z_FORM_JUNCTIONS_HH

#include "VoteImage.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "TJunction.hh"
#include "Collinearity.hh"
#include "GestaltPrinciple.hh"

namespace Z
{

class FormJunctions : public GestaltPrinciple
{
private:
  Array<VoteImage::Elem> iscts;
  bool isct_ok[8][8];
  bool first_op;

  void SetupAdmissibilityMatrix();
  void Rank();
  void InitSearchLines(Line *l);
  void ExtendSearchLines(Line *l);
  void ExtendSmartLines(Line *l);
  void FollowEnd(Line *line, int end, Line *stop_line);
  void ExtendEnd(Line *line, int end);
  void CreateLineJunctions(unsigned sline, Array<VoteImage::Elem> &iscts);
  void CreateL(Line *line_i, int vtype_i, Line *line_j, int vtype_j);
  void CreateC(Line *line_i, int vtype_i, Line *line_j, int vtype_j);
  void CreateT(Line *line_i, int vtype_i, Line *line_j, int vtype_j);
  void SplitLine(Line *l, const Vector2 &inter, const Vector2 &dir,
      Line **l_left, Line **l_right,
      int *end_left, int *end_right, Collinearity **c_new);
  Line* SplitLine(Line *l, const Vector2 &p, Collinearity **c_new);
  unsigned FindLineSplitIdx(Line *l, const Vector2 &p);
  void UpdateClosures(Line *l1, Line *l2, Collinearity *coll);
  Collinearity* NewCollinearity(Line *line_i, Line *line_j,
      int end_i, int end_j, bool inform);
  LJunction* NewL(Line *line_i, Line *line_j, int end_i, int end_j);
  TJunction* NewT(Line *pole, Line *left, Line *right,
      int end_p, int end_l, int end_r,
      Collinearity *coll, LJunction *ljct_l, LJunction *ljct_r);

public:
  static VoteImage *vote_img;

  FormJunctions(VisionCore *vc);
  virtual ~FormJunctions();
  virtual void Reset();
  virtual void Operate(bool incremental);
};

}

#endif

