/**
 * $Id: FormJunctions.hh,v 1.20 2007/03/25 21:35:57 mxz Exp mxz $
 */

#ifndef Z_FORM_ARC_JUNCTIONS_HH
#define Z_FORM_ARC_JUNCTIONS_HH

#include "VoteImage.hh"
#include "Arc.hh"
//#include "ArcJunction.hh"
#include "GestaltPrinciple.hh"

namespace Z
{

class FormArcJunctions : public GestaltPrinciple
{
private:
  Array<VoteImage::Elem> iscts;
  bool isct_ok[8][8];
  bool first_op;

  void SetupAdmissibilityMatrix();
  bool IsctTypeAdmissible(int type_i, int type_j)
    {return isct_ok[type_i][type_j];}
  bool IsJunctionBetween(Arc *arc_i, Arc *arc_j);
  void CreateJunctions(unsigned sline, Array<VoteImage::Elem> &iscts);
  void OperateIncremental();
  void OperateNonIncremental();
  void InitSearchLines(Arc *arc);
  void ExtendSearchLines(Arc *arc);
  //void ExtendSmartLines(Arc *arc);
  /*void FollowEnd(Line *line, int end, Line *stop_line);
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
      Collinearity *coll, LJunction *ljct_l, LJunction *ljct_r);*/

public:
  static VoteImage *vote_img;

  FormArcJunctions(VisionCore *vc);
  virtual ~FormArcJunctions();
  virtual void Reset();
  virtual void Operate(bool incremental);
};

}

#endif


