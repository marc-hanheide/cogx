/**
 * $Id: FormArcJunctions.hh,v 1.2 2010/05/26 12:03:44 mz Exp mz $
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

public:
  static VoteImage *vote_img;

  FormArcJunctions(VisionCore *vc);
  virtual ~FormArcJunctions();
  virtual void Reset();
  virtual void Operate(bool incremental);
};

}

#endif


