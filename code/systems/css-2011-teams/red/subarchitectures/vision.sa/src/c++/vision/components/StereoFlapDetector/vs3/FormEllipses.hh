/**
 * $Id: FormEllipses.hh,v 1.8 2006/11/24 13:47:03 mxz Exp mxz $
 * TODO - try computing size just from number of edgels of arcs (minus edgels
 * TODO   grabbed from lines), evtl. again weighted by amount of arc on ellipse
 */

#ifndef Z_FORM_ELLIPSES_HH
#define Z_FORM_ELLIPSES_HH

#include "GestaltPrinciple.hh"

namespace Z
{

class FormEllipses : public GestaltPrinciple
{
private:
  void Rank();
  void Mask();
  void HaveNewArcGroup(unsigned idx);

public:
  FormEllipses(VisionCore *vc);
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}

#endif

