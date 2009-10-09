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
  if( Ellipses(*(unsigned*)a)->sig > Ellipses(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

FormEllipses::FormEllipses(Config *cfg)
: GestaltPrinciple(cfg)
{
  done = false;
}

void FormEllipses::Reset(const Image *img)
{
  done = false;
}

void FormEllipses::Operate(bool incremental)
{
  StartRunTime();
//printf("FormEllipses - ");
  if(!done)
  {
    Create();
    Rank();
    Mask();
    //Prune();
    done = true;
  }
//printf("END \n");
  StopRunTime();
}

void FormEllipses::Create()
{
  for(unsigned rank = 0; rank < NumConvexArcGroups(); rank++)
  {
    unsigned group = RankedGestalts(Gestalt::CONVEX_ARC_GROUP, rank);
    // TODO: I really don't like that constraint:
    //if(ConvexArcGroups(group)->ang_cover >= M_PI)
    {
      double x, y, a, b, phi;
      if(FitEllipse(ConvexArcGroups(group)->arcs, x, y, a, b, phi))
      {
        // note: in FormArcs we defined MIN_RADIUS for an arc, thus
        // the smaller axis cannot be smaller than MIN_RADIUS
        if(b > FormArcs::MIN_RADIUS)
        {
          NewGestalt(new Ellipse(group, x, y, a, b, phi));
        }
      }
    }
  }
}

void FormEllipses::Rank()
{
  RankGestalts(Gestalt::ELLIPSE, CmpEllipses);
}

/**
 * Prune all ellipse hypotheses that share an arc with a stronger ellipse.
 */
void FormEllipses::Prune()
{
  Array<Gestalt*> good_ellipses;  // ellipses which were not pruned
  Array<unsigned> used_arcs(NumArcs()); // which ellipse used which arc
  used_arcs.Set(UNDEF_ID);
  for(unsigned i = 0; i < NumEllipses(); i++)
  {
    bool i_survive = true;
    ConvexArcGroup *g = ConvexArcGroups(Ellipses(i)->group);
    // first check if any previous (i.e. stronger) ellipse uses one of my arcs
    for(unsigned a = 0; a < g->arcs.Size(); a++)
      if(used_arcs[g->arcs[a]] != UNDEF_ID)
      {
        i_survive = false;
        break;
      }
    if(i_survive)
    {
      good_ellipses.PushBack(Ellipses(i));
      // now mark my arcs in the array
      for(unsigned a = 0; a < g->arcs.Size(); a++)
        used_arcs[g->arcs[a]] = i;
    }
    else
      delete Ellipses(i);
  }
  Ellipses().DeepCopy(good_ellipses);
}

void FormEllipses::Mask()
{
  Array<unsigned> used_arcs(NumArcs()); // which ellipse used which arc
  used_arcs.Set(UNDEF_ID);
  for(unsigned j = 0; j < NumEllipses(); j++)
  {
    unsigned i = RankedGestalts(Gestalt::ELLIPSE, j);
    ConvexArcGroup *g = ConvexArcGroups(Ellipses(i)->group);
    // first check if any previous (i.e. stronger) ellipse uses one of my arcs
    for(unsigned a = 0; a < g->arcs.Size(); a++)
    {
      if(used_arcs[g->arcs[a]] != UNDEF_ID)
        Ellipses(i)->Mask(used_arcs[g->arcs[a]]);
      used_arcs[g->arcs[a]] = i;
    }
  }
}

}
