/**
 * @file FormEJunctions.cc
 * @author Zillich, Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Header file of Gestalt principle FormArcJunctions.
 **/

#ifndef Z_FORM_ARC_JUNCTIONS_HH
#define Z_FORM_ARC_JUNCTIONS_HH

#include "VoteImage.hh"
#include "Arc.hh"
#include "GestaltPrinciple.hh"

namespace Z
{

/**
 * @brief Class FormArcJunctions, creating junctions between arcs.
 */
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


