/**
 * $Id: FormJunctions.hh,v 1.20 2007/03/25 21:35:57 mxz Exp mxz $
 */

#include "Arc.hh"
#include "AJunction.hh"
#include "FormArcJunctions.hh"

namespace Z
{

enum GrowMethod {GROW_EQUAL, GROW_WEIGHTED, GROW_SMART};
static GrowMethod grow_method = GROW_WEIGHTED;

VoteImage *FormArcJunctions::vote_img = 0;

FormArcJunctions::FormArcJunctions(VisionCore *vc)
: GestaltPrinciple(vc)
{
  next_principles.PushBack(FORM_CONVEX_ARC_GROUPS);
  vote_img = 0;
  first_op = true;
  SetupAdmissibilityMatrix();
}

FormArcJunctions::~FormArcJunctions()
{
  delete vote_img;
}

void FormArcJunctions::SetupAdmissibilityMatrix()
{
  int i, j;
  for(i = 0; i < 8; i++)
    for(j = 0; j < 8; j++)
      isct_ok[i][j] = false;

  // note: We fill out the upper right side of the matrix. It is important to
  // keep in mind the numbers corresponding to the symbolic names.
  // I.e. in isct_ok[i][j] i must be <= j.
 
  // basic "daisy chain" constraint: connect only START <-> END of two arcs
  // tangents/tangents
  isct_ok[VOTE_TS][VOTE_TE] = true;
  // tangents/normals
  isct_ok[VOTE_TS][VOTE_NLE] = true;
  isct_ok[VOTE_TS][VOTE_NRE] = true;
  isct_ok[VOTE_TE][VOTE_NLS] = true;
  isct_ok[VOTE_TE][VOTE_NRS] = true;

  // now mirror along diagonal to fill lower left side of matrix
  for(i = 0; i < 8; i++)
    for(j = i + 1; j < 8; j++)
      isct_ok[j][i] = isct_ok[i][j];
}

void FormArcJunctions::Reset()
{
  if(core->HaveImage())
  {
    if(vote_img != 0 &&
       !(vote_img->width == core->GetImage()->width &&
         vote_img->height == core->GetImage()->height))
    {
      delete vote_img;
      vote_img = 0;
    }
    if(vote_img == 0)
      vote_img = new VoteImage(core->GetImage()->width,
          core->GetImage()->height);
  }
  vote_img->Clear();
  first_op = true;
}

void FormArcJunctions::Operate(bool incremental)
{
  if(incremental)
    OperateIncremental();
  else
    OperateNonIncremental();
}

void FormArcJunctions::OperateIncremental()
{
  if(first_op)
  {
    // in first call just init search lines, don't create junctions
    vote_img->SetNumLines(NumArcs(core)*8);
    for(unsigned r = 0; r < NumArcs(core); r++)
      InitSearchLines((Arc*)core->RankedGestalts(Gestalt::ARC, r));
    first_op = false;
  }
  else
  {
    // for all subsequent calls 
    // try to be smart about growing search lines
    if(grow_method == GROW_SMART)
    {
      // HACK: not implemented yet
      //int r = ExpSelect(NumArcs(core) - 1);
      //ExtendSmartLines((Arc*)core->RankedGestalts(Gestalt::ARC, r));
    }
    // lines grow probabilistically according to length
    else if(grow_method == GROW_WEIGHTED)
    {
      int r = ExpSelect(NumArcs(core) - 1);
      ExtendSearchLines((Arc*)core->RankedGestalts(Gestalt::ARC, r));
    }
    // all search lines grow equally, this is less "anytime-ish"
    else if(grow_method == GROW_EQUAL)
    {
      for(unsigned r = 0; r < NumArcs(core); r++)
        ExtendSearchLines((Arc*)core->RankedGestalts(Gestalt::ARC, r));
    }
  }
}

void FormArcJunctions::OperateNonIncremental()
{
  if(first_op)  // do only once
  {
    unsigned narcs = NumArcs(core);
    vote_img->SetNumLines(narcs*8);
    for(unsigned r = 0; r < narcs; r++)
    {
      Arc *arc = (Arc*)core->RankedGestalts(Gestalt::ARC, r);
      double len = min(arc->Radius(), arc->ArcLength());
      InitSearchLines(arc);
      // draw all search lines to a length of l
      for(int j = 0; j < len; j++)
        ExtendSearchLines(arc);
    }
    first_op = false;
  }
}

void FormArcJunctions::InitSearchLines(Arc *arc)
{
  double l = 1000; // TODO: replace this by "until image border"
  unsigned sline = arc->ID()*8;  // search line base index for this line

  // tangent
  vote_img->InitLine(arc->point[START],
      arc->point[START] + l*arc->norm[START].NormalAntiClockwise(),
      /*Gestalt::ARC,*/ sline + VOTE_TS);
  // radii
  vote_img->InitLine(arc->point[START],
      arc->point[START] + l*arc->norm[START],
      /*Gestalt::ARC,*/ sline + VOTE_NLS);
  vote_img->InitLine(arc->point[START],
      arc->point[START] - l*arc->norm[START],
      /*Gestalt::ARC,*/ sline + VOTE_NRS);
  // tangent
  vote_img->InitLine(arc->point[END],
      arc->point[END] + l*arc->norm[END].NormalClockwise(),
      /*Gestalt::ARC,*/ sline + VOTE_TE);
  // radii
  vote_img->InitLine(arc->point[END],
      arc->point[END] + l*arc->norm[END],
      /*Gestalt::ARC,*/ sline + VOTE_NLE);
  vote_img->InitLine(arc->point[END],
      arc->point[END] - l*arc->norm[END],
      /*Gestalt::ARC,*/ sline + VOTE_NRE);
}

void FormArcJunctions::ExtendSearchLines(Arc *arc)
{
  unsigned sline = arc->ID()*8 + VOTE_TS;
  unsigned smax = arc->ID()*8 + VOTE_NRE;
  for(; sline <= smax; sline++)
    if(vote_img->ExtendLine(sline, iscts) > 0)
      CreateJunctions(sline, iscts);
}

/**
 * sline is the search line which triggered the new intersections, it is
 * therefore a tangent or normal vote line.
 */
void FormArcJunctions::CreateJunctions(unsigned sline,
    Array<VoteImage::Elem> &iscts)
{
  unsigned i = sline/8, vtype_i = sline%8;
  int end_i = VOTE_END(vtype_i);
  Arc *arc_i = Arcs(core, i);
  for(unsigned k = 0; k < iscts.Size(); k++)
  {
    unsigned j = iscts[k].id/8, vtype_j = iscts[k].id%8;
    int end_j = VOTE_END(vtype_j);
    Arc *arc_j = Arcs(core, j);
    // check if the intersection between vote line tyes is admissible
    // TODO: this admissibility check could go into VoteImage
    if(IsctTypeAdmissible(vtype_i, vtype_j))
    {
      // TODO: wenn eine tangente eine normale von hinten trifft, ignoriere!
      if(!IsJunctionBetween(arc_i, arc_j))
        //if(arc_i->ConvexWith(arc_j))
        // HACK: 0.5 is an arbitrary threshold!
        if(arc_i->Convexity(arc_j) >= 0.5)
          core->NewGestalt(new AJunction(core, arc_i, arc_j, end_i, end_j));
    }
  }
}

bool FormArcJunctions::IsJunctionBetween(Arc *arc_i, Arc *arc_j)
{
  for(int end = START; end <= END; end++)
    for(unsigned k = 0; k < arc_i->jct[end].Size(); k++)
      if(arc_i->jct[end][k]->arc[end] == arc_j)
        return true;
  return false;
}

}

