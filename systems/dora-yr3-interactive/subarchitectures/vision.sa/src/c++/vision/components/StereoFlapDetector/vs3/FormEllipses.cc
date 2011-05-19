/**
 * $Id: FormEllipses.cc,v 1.13 2006/11/24 13:47:03 mxz Exp mxz $
 */

#include "Arc.hh"
#include "ConvexArcGroup.hh"
#include "Ellipse.hh"
#include "FormArcs.hh"
#include "FormEllipses.hh"

namespace Z
{

static int CmpEllipses(const void *a, const void *b)
{
  if( (*(Ellipse**)a)->sig > (*(Ellipse**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

FormEllipses::FormEllipses(VisionCore *vc)
: GestaltPrinciple(vc)
{
}

void FormEllipses::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  switch(type)
  {
    case Gestalt::CONVEX_ARC_GROUP:
      HaveNewArcGroup(idx);
      break;
    default:
      break;
  }
}

void FormEllipses::HaveNewArcGroup(unsigned idx)
{
  double x, y, a, b, phi;
  ConvexArcGroup *group = ConvexArcGroups(core, idx);
  // HACK: do we really want to reject if angular coverage smaller pi?
  if(group->ang_cover < M_PI)
    return;
  if(FitEllipse(group->arcs, x, y, a, b, phi))
  {
    // note: in FormArcs we defined MIN_RADIUS for an arc, thus
    // the smaller axis cannot be smaller than MIN_RADIUS
    if(b > FormArcs::MIN_RADIUS)
      core->NewGestalt(new Ellipse(core, group, x, y, a, b, phi));
  }
  Rank();
  Mask();
}

void FormEllipses::Rank()
{
  RankGestalts(Gestalt::ELLIPSE, CmpEllipses);
}

void FormEllipses::Mask()
{
  // array containing for each arc the ID of the highest ranking ellipse which
  // contains that arc
  Array<unsigned> owner(NumArcs(core));
  owner.Set(UNDEF_ID);
  for(unsigned j = 0; j < NumEllipses(core); j++)
  {
    Ellipse *ell = (Ellipse*)core->RankedGestalts(Gestalt::ELLIPSE, j);
    ell->Mask(UNDEF_ID);
    // first check if any previous (i.e. stronger) ellipse uses one of my arcs
    for(unsigned a = 0; a < ell->group->arcs.Size(); a++)
    {
      if(owner[ell->group->arcs[a]->ID()] != UNDEF_ID)
        ell->Mask(owner[ell->group->arcs[a]->ID()]);
      owner[ell->group->arcs[a]->ID()] = ell->ID();
    }
  }
}

}

