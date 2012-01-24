/**
 * $Id: FormSegments.hh,v 1.14 2007/02/18 18:02:48 mxz Exp mxz $
 */

#ifndef Z_FORM_SEGMENTS_HH
#define Z_FORM_SEGMENTS_HH

#include "IdImage.hh"
#include "Segment.hh"
#include "GestaltPrinciple.hh"

namespace Z
{

class FormSegments : public GestaltPrinciple
{
private:
  bool done;

  void CreateSegmentsMatas();
  void DrawToEdgeImage(Segment *seg);
  void Create();
  void Rank();
  void SegmentArcsLines();

public:
  static IdImage *edge_img; ///< id image containing segment ids for each edgel
  static float *dir_img;    ///< image containing gradient orientations
  static int num_edgels;    ///< total number of edgels

  FormSegments(VisionCore *vc);
  virtual ~FormSegments();
  virtual void Reset();
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate() {return !done;}
};

}

#endif
