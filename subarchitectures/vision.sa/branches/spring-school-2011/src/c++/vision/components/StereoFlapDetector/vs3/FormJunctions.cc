/**
 * $Id: FormJunctions.cc,v 1.34 2007/04/14 20:50:59 mxz Exp mxz $
 *
 * TODO: remember and ignore search lines which reached the image border
 * TODO: avoid collinearity C(ac)    ----- ------ ------
 *                                     a      b      c
 *      note: this requires (possibly expensive) path search
 * TODO: stop if (at some very late time) no single search line can be
 *       extended any further.
 * TODO: check whether junction between lines i and j at respective ends as
 *       early as possible, right after detection of search line intersection
 */

#include <assert.h>
#include "Segment.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "TJunction.hh"
#include "Collinearity.hh"
#include "Closure.hh"
#include "FormJunctions.hh"
#include "HCF.hh"

namespace Z
{

// Minimal line length. A line of 2 pixels is per definition always a line and
// therefore totally insignificant. Any meaningful line must be at least 3
// pixels long.
static const double MIN_LINE_LENGTH = 3.;

enum GrowMethod {GROW_EQUAL, GROW_WEIGHTED, GROW_SMART};
static GrowMethod grow_method = GROW_SMART;

static int CmpLJunctions(const void *a, const void *b)
{
  if( (*(LJunction**)a)->acc < (*(LJunction**)b)->acc )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

static int CmpCollinearities(const void *a, const void *b)
{
  if( (*(Collinearity**)a)->acc < (*(Collinearity**)b)->acc )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

static int CmpTJunctions(const void *a, const void *b)
{
  if( (*(TJunction**)a)->acc < (*(TJunction**)b)->acc )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

static int CmpLines(const void *a, const void *b)
{
  if( (*(Line**)a)->sig > (*(Line**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

static bool InsideLine(Line *l, const Vector2 &p)
{
  double d = Dot(l->dir, p - l->point[START]);
  return d >= 0. && d <= l->Length();
}

/**
 * Check whether there is an L-jct between lines i and j.
 */
static LJunction* LBetween(Line *l_i, int end_i, int side_i, Line *l_j)
{
  for(unsigned k = 0; k < l_i->l_jct[end_i][side_i].Size(); k++)
    if(l_i->l_jct[end_i][side_i][k]->line[OtherSide(side_i)] == l_j)
      return l_i->l_jct[end_i][side_i][k];
  return 0;
}

/**
 * Check whether there is no L-jct between lines i and j.
 */
static bool NoLJunctionYet(Line *l_i, int end_i, Line *l_j)
{
  // TODO: it would be nice to know which side to look
  for(int side = LEFT; side <= RIGHT; side++)
    for(unsigned k = 0; k < l_i->l_jct[end_i][side].Size(); k++)
      if(l_i->l_jct[end_i][side][k]->line[OtherSide(side)] == l_j)
        return false;
  return true;
}

static bool NoTJunctionYet(Line *l_i, int end_i)
{
  return l_i->t_jct[end_i] == 0;
}

/**
 * Check whether there is no collinearity yet between lines i and j. 
 */
static bool NoCollinearityYet(Line *l_i, int end_i, Line *l_j)
{
  for(unsigned k = 0; k < l_i->coll[end_i].Size(); k++)
    if(l_i->coll[end_i][k]->OtherLine(l_i) == l_j)
      return false;
  return true;
}

VoteImage *FormJunctions::vote_img = 0;

FormJunctions::FormJunctions(VisionCore *vc)
: GestaltPrinciple(vc)
{
  next_principles.PushBack(FORM_CLOSURES);
  vote_img = 0;
  first_op = true;
  SetupAdmissibilityMatrix();
}

FormJunctions::~FormJunctions()
{
  delete vote_img;
}

void FormJunctions::SetupAdmissibilityMatrix()
{
  int i, j;
  for(i = 0; i < 8; i++)
    for(j = 0; j < 8; j++)
      isct_ok[i][j] = false;
  // note: We fill out the upper right side of the matrix. It is important to
  // keep in mind the numbers corresponding to the symbolic names.
  // I.e. in isct_ok[i][j] i must be <= j.
  // tangents/edge
  isct_ok[VOTE_E][VOTE_TS] = true;
  isct_ok[VOTE_E][VOTE_TE] = true;
  // tangents/tangents
  isct_ok[VOTE_TS][VOTE_TE] = true;
  isct_ok[VOTE_TS][VOTE_TS] = true;
  isct_ok[VOTE_TE][VOTE_TE] = true;
  // tangents/normals
  isct_ok[VOTE_TS][VOTE_NLS] = true;
  isct_ok[VOTE_TS][VOTE_NLE] = true;
  isct_ok[VOTE_TS][VOTE_NRS] = true;
  isct_ok[VOTE_TS][VOTE_NRE] = true;
  isct_ok[VOTE_TE][VOTE_NLS] = true;
  isct_ok[VOTE_TE][VOTE_NLE] = true;
  isct_ok[VOTE_TE][VOTE_NRS] = true;
  isct_ok[VOTE_TE][VOTE_NRE] = true;
  // now mirror along diagonal to fill lower left side of matrix
  for(i = 0; i < 8; i++)
    for(j = i + 1; j < 8; j++)
      isct_ok[j][i] = isct_ok[i][j];
}

void FormJunctions::Reset()
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

/**
 * Create collinearities, L-junction and T-junctions.
 * If incremental, search lines grow incrementally with each call.
 * If not incremental, search lines of length l (= line length) are drawn in one
 * step and then all junctions created. Note that this can take _very_ long.
 */
void FormJunctions::Operate(bool incremental)
{
  if(incremental)
  {
    if(first_op)
    {
      // in first call just init search lines, don't create junctions
      vote_img->SetNumLines(NumLines(core)*8);
      for(unsigned r = 0; r < NumLines(core); r++)
        InitSearchLines((Line*)core->RankedGestalts(Gestalt::LINE, r));
      first_op = false;
    }
    else
    {
      // for all subsequent calls 
      // try to be smart about growing search lines
      if(grow_method == GROW_SMART)
      {
        int r = ExpSelect(NumLines(core) - 1);
        ExtendSmartLines((Line*)core->RankedGestalts(Gestalt::LINE, r));
        core->stats.pix_cnt += 1;
      }
      // lines grow probabilistically according to length
      else if(grow_method == GROW_WEIGHTED)
      {
        int r = ExpSelect(NumLines(core) - 1);
        ExtendSearchLines((Line*)core->RankedGestalts(Gestalt::LINE, r));
        core->stats.pix_cnt += 6;
      }
      // all search lines grow equally, this is less "anytime-ish"
      else if(grow_method == GROW_EQUAL)
      {
        for(unsigned r = 0; r < NumLines(core); r++)
        {
          ExtendSearchLines((Line*)core->RankedGestalts(Gestalt::LINE, r));
          core->stats.pix_cnt += 6;
        }
      }
    }
  }
  else
  {
    if(first_op)  // do only once
    {
      // note that the number of lines grows as T-jcts are formed and lines
      // split. remember the original number of lines.
      unsigned nlines = NumLines(core);
      vote_img->SetNumLines(nlines*8);
      for(unsigned r = 0; r < nlines; r++)
      {
        Line *l = (Line*)core->RankedGestalts(Gestalt::LINE, r);
        int len = (int)l->Length();
        InitSearchLines(l);
        // draw all search lines to a length of l
        for(int j = 0; j < len; j++)
        {
          ExtendSearchLines(l);
          core->stats.pix_cnt += 6;
        }
      }
      first_op = false;
    }
  }
}

void FormJunctions::Rank()
{
  RankGestalts(Gestalt::L_JUNCTION, CmpLJunctions);
  RankGestalts(Gestalt::COLLINEARITY, CmpCollinearities);
  RankGestalts(Gestalt::T_JUNCTION, CmpTJunctions);
  RankGestalts(Gestalt::LINE, CmpLines);
}

void FormJunctions::InitSearchLines(Line *l)
{
  double len = 1000; // TODO: replace this by "until image border"
  unsigned sline = l->ID()*8;  // search line base index for this line

  // draw edge itself
  vote_img->DrawLine(l->point[START], l->point[END],
      /*Gestalt::LINE,*/ sline + VOTE_E);
  // initialise tangents for growing
  vote_img->InitLine(l->point[START], l->point[START] - len*l->dir,
      /*Gestalt::LINE,*/ sline + VOTE_TS);
  vote_img->InitLine(l->point[END], l->point[END] + len*l->dir,
      /*Gestalt::LINE,*/ sline + VOTE_TE);
  // initialise normals for growing
  vote_img->InitLine(l->point[START],
      l->point[START] + len*l->dir.NormalAntiClockwise(),
      /*Gestalt::LINE,*/ sline + VOTE_NLS);
  vote_img->InitLine(l->point[END],
      l->point[END] + len*l->dir.NormalAntiClockwise(),
      /*Gestalt::LINE,*/ sline + VOTE_NLE);
  vote_img->InitLine(l->point[START],
      l->point[START] - len*l->dir.NormalAntiClockwise(),
      /*Gestalt::LINE,*/ sline + VOTE_NRS);
  vote_img->InitLine(l->point[END],
      l->point[END] - len*l->dir.NormalAntiClockwise(),
      /*Gestalt::LINE,*/ sline + VOTE_NRE);
}

void FormJunctions::ExtendSearchLines(Line *l)
{
  // lines created by splitting have no vote lines
  if(l->defer_vote == l)
  {
    unsigned sline = l->ID()*8 + VOTE_TS;
    unsigned smax = l->ID()*8 + VOTE_NRE;
    for(; sline <= smax; sline++)
      if(vote_img->ExtendLine(sline, iscts) > 0)
        CreateLineJunctions(sline, iscts);
  }
}

/**
 * Be somewhat smart about extending search lines.
 * Prefer tangential search lines and open line ends, boost neighbouring lines.
 */
void FormJunctions::ExtendSmartLines(Line *l)
{
  // Selecting which end to extend is given by the number of junctions already
  // present at that end: p(START)/p(END) = n(END)/n(START).
  // Regarding T-junctions: Each T-jct at one end is accompanied by two L-jcts.
  // Therefore reduce the count of jcts at this side by two.
  // Furthermore add 1 to the jct count on the other end to further favour this
  // end.
  int n_t[2] = {(l->t_jct[START] != 0 ? 1 : 0), (l->t_jct[END] != 0 ? 1 : 0)};
  int n_s = 1 + l->coll[START].Size() + l->l_jct[START][LEFT].Size() +
    l->l_jct[START][RIGHT].Size() - 2*n_t[START] + n_t[END];
  int n_e = 1 + l->coll[END].Size() + l->l_jct[END][LEFT].Size() +
    l->l_jct[END][RIGHT].Size() - 2*n_t[END] + n_t[START];
  int r = RandInt()%(n_s + n_e);
  int end;
  if(r < n_s)  // 0 .. n_s - 1
    end = END;
  else         // n_s .. n_s + n_e - 1
    end = START;
  FollowEnd(l, end, l);
}

/**
 * TODO: maybe following scheme taking into account completed Closures:
 * line itself and T-jct vote for line itself
 * completed closures (of which line is part) vote for any other line
 * L and C vote for hopping
 * P(other) : P(self) : P(hop) = n_clos : 1 + n_t : n_c + n_l + n_r
 * plus: take into accound significances: e.g. a highly significant T-jct votes
 * for tangential extension
 */
void FormJunctions::FollowEnd(Line *line, int end, Line *stop_line)
{
  int n_c = line->coll[end].Size();
  int n_l = line->l_jct[end][LEFT].Size();
  int n_r = line->l_jct[end][RIGHT].Size();
  int n_t = (line->t_jct[end] != 0 ? 1 : 0);

  // extend line itself if no junctions at all
  if(n_c + n_l + n_r == 0)
  {
    ExtendEnd(line, end);
  }
  else
  {
    // favour extension of this line if there is a T-jct
    int n_self = 1 + n_t;
    // note: L-jcts belonging to a T-jct do not count -> -2*n_t
    int n_hop = n_c + n_l + n_r - 2*n_t;
    int w = RandInt()%(n_self + n_hop);
    if(w < n_self)
    {
      ExtendEnd(line, end);
    }
    else
    {
      int i = 0;
      Line *fline = 0;
      int fend = UNDEF_ID;

      core->stats.hop_cnt++;
      w = RandInt()%(n_c + n_l + n_r);
      // if one of the collinearities
      if(w < n_c)
      {
        i = w;
        fline = line->coll[end][i]->OtherLine(line);
        fend = Other(line->coll[end][i]->WhichEndIs(fline));
      }
      // if one of the left L-jcts
      else if(w < n_c + n_l)
      {
        i = w - n_c;
        fline = line->l_jct[end][LEFT][i]->line[RIGHT];
        fend = Other(line->l_jct[end][LEFT][i]->near_point[RIGHT]);
      }
      // if on of the right L-jcts
      else
      {
        i = w - n_c - n_l;
        fline = line->l_jct[end][RIGHT][i]->line[LEFT];
        fend = Other(line->l_jct[end][RIGHT][i]->near_point[LEFT]);
      }
      // avoid infinite loops
      if(fline != stop_line)
        FollowEnd(fline, fend, stop_line);
      else
      {
        // select any line at random (ignoring rank)
        fline = Lines(core, RandInt()%NumLines(core));
        fend = RandInt()%2;
        FollowEnd(fline, fend, fline);
      } 
    }
  }
}

void FormJunctions::ExtendEnd(Line *line, int end)
{
  int r = RandInt();
  // factor for tangent extension: equal chance for open end, favour tangents
  // if T-junction (look for amodal completion)
  int t = 1 + (line->t_jct[end] != 0 ? 1 : 0);
  // factor for normal extension
  int n = 1;
  Line *voteline = line;
  unsigned sline;

  // lines created by splitting defer voting to their original lines
  while(voteline->defer_vote != voteline)
    voteline = voteline->defer_vote;
  sline = voteline->ID()*8;
  if(r%(t + n) < t) // tangent
  {
    sline += (end == START ? VOTE_TS : VOTE_TE);
  }
  else  // normal
  {
    int side = (r/(t + n))%2;  // equal chance
    if(side == LEFT)
      sline += (end == START ? VOTE_NLS : VOTE_NLE);
    else
      sline += (end == START ? VOTE_NRS : VOTE_NRE);
  }
  if(vote_img->ExtendLine(sline, iscts) > 0)
    CreateLineJunctions(sline, iscts);
}

/**
 * sline is the search line which triggered the new intersections, it is
 * therefore a tangent or normal vote line.
 */
void FormJunctions::CreateLineJunctions(unsigned sline,
    Array<VoteImage::Elem> &iscts)
{
  for(unsigned k = 0; k < iscts.Size(); k++)
  {
    unsigned i = sline/8, vtype_i = sline%8;
    unsigned j = iscts[k].id/8, vtype_j = iscts[k].id%8;
    Line *line_i = Lines(core, i);
    Line *line_j = Lines(core, j);
    // check if the intersection between vote line tyes is admissible
    // TODO: this admissibility check could go into VoteImage
    if(isct_ok[vtype_i][vtype_j])
    {
      // TODO: After a T-jct there can be no further T-jcts or type1
      // collinearities. -> really?
      if(vtype_j == VOTE_E)
        CreateT(line_i, vtype_i, line_j, vtype_j);
      else
      {
        // HACK: This is a stupid arbitrary threshold! As the distinction
        // between collinearity and L-junction will fall, so will this
        // threshold.
        if(AngleBetweenLines(line_i->phi, line_j->phi) < M_PI/6)
          CreateC(line_i, vtype_i, line_j, vtype_j);
        else
          CreateL(line_i, vtype_i, line_j, vtype_j);
      }
      /*else if(VOTE_IS_TANGENT(vtype_i) && VOTE_IS_TANGENT(vtype_j))
        CreateL(i, vtype_i, j, vtype_j);
      else if((VOTE_IS_NORMAL(vtype_i) && VOTE_IS_TANGENT(vtype_j)) ||
              (VOTE_IS_TANGENT(vtype_i) && VOTE_IS_NORMAL(vtype_j)))
        CreateC(i, vtype_i, j, vtype_j);*/
    }
  }
}

/**
 * Creates a T-junction between lines i and j.
 * The extension of line i at given end meets line j. I.e. line i is the pole of
 * the T, Line j is the bar.
 * Line j is split into left and right bar and L-junctions with the pole are
 * created accordingly.
 *              b
 *             --+---- j
 *                g
 *               |
 *               | i
 *
 * TODO: a line which is already pole of a T-jct can not become arm of a T-jct
 */
void FormJunctions::CreateT(Line *line_i, int vtype_i, Line *line_j,
    int vtype_j)
{
  try
  {
    int end_i = VOTE_END(vtype_i);
    // if a line is part of a split and we have junction at end, defer
    // creation of junctions to last line in that split
    // TODO: use convenience function for this
    if(end_i == END)
      while(line_i->next != 0)
        line_i = line_i->next;
    // if line j is part of a split find that part where the intersection
    // point actually lies
    if(line_j->IsSplit())
      while(line_j != 0 && !InsideLine(line_j, line_i->point[end_i]))
        line_j = line_j->next;
    if(line_j != 0 && InsideLine(line_j, line_i->point[end_i]))
    {
      double g;  // gap between end of line i and line j
      double b;  // smaller bar length
      Vector2 inter = LineIntersection(
          line_i->point[end_i], line_i->tang[end_i],
          line_j->point[START], line_j->dir, &g, &b);
      // if smaller bar length is long enough for a line
      if(min(b, line_j->Length() - b) >= MIN_LINE_LENGTH)
      {
        // note: for now we allow only one T-jct per end
        // TODO: maybe relax this at some later stage
        if(NoTJunctionYet(line_i, end_i))
        {
          Line *j_left = 0, *j_right = 0;
          int end_left, end_right;
          Collinearity *new_c = 0;
          SplitLine(line_j, inter, line_i->tang[end_i], &j_left, &j_right,
              &end_left, &end_right, &new_c);
          LJunction *new_ljct_left = NewL(line_i, j_left, end_i, end_left);
          LJunction *new_ljct_right = NewL(j_right, line_i, end_right, end_i);
          NewT(line_i, j_left, j_right, end_i, end_left, end_right,
              new_c, new_ljct_left, new_ljct_right);
        }
      }
      else // create just an L-junction
      {
        // find out what end of j the L-junction should be
        int end_j = (b < line_j->Length()/2. ? START : END);
        // HACK: This is a stupid arbitrary threshold! As the distinction
        // between collinearity and L-junction will fall, so will this
        // threshold.
        if(AngleBetweenLines(line_i->phi, line_j->phi) < M_PI/6)
          NewCollinearity(line_i, line_j, end_i, end_j, true);
        else
          NewL(line_i, line_j, end_i, end_j);
      }
    }
  }
  catch(Except &e)
  {
    // if tangents are parallel -> no intersections
  }
}

void FormJunctions::CreateL(Line *line_i, int vtype_i, Line *line_j,
    int vtype_j)
{
  int end_i = VOTE_END(vtype_i);
  int end_j = VOTE_END(vtype_j);

  // if a line is part of a split and we have junction at end, defer
  // creation of junctions to last line in that split
  if(end_i == END)
    while(line_i->next != 0)
      line_i = line_i->next;
  if(end_j == END)
    while(line_j->next != 0)
      line_j = line_j->next;

  LJunction *new_l = NewL(line_i, line_j, end_i, end_j);

  // create T-jcts from new L-jct and old collinearities
  // TODO: tidy this up
  if(new_l != 0)
  {
    Line *left = new_l->line[LEFT];
    Line *right = new_l->line[RIGHT];
    int end_l = new_l->near_point[LEFT];
    int end_r = new_l->near_point[RIGHT];
    for(unsigned c = 0; c < right->coll[end_r].Size(); c++)
    {
      // third would be the right arm of a new T-jct
      Line *third = right->coll[end_r][c]->OtherLine(right);
      int end_3 = right->coll[end_r][c]->WhichEndIs(third);
      // Note that sometimes there is a collinearity AND L-jct between two
      // lines.  Do not create a T-jct for those cases.
      if(third != left) // TODO: can be removed
      {
        LJunction *right_l = LBetween(third, end_3, LEFT, left);
        // only if there is also an L-jct between the other two lines
        if(right_l != 0)
          if(NoTJunctionYet(left, end_l))
          {
            NewT(left, right, third, end_l, end_r, end_3,
              right->coll[end_r][c], new_l, right_l);
          }
      }
    }
    for(unsigned c = 0; c < left->coll[end_l].Size(); c++)
    {
      // third would be the left arm of a new T-jct
      Line *third = left->coll[end_l][c]->OtherLine(left);
      int end_3 = left->coll[end_l][c]->WhichEndIs(third);
      // Note that sometimes there is a collinearity AND L-jct between two
      // lines.  Do not create a T-jct for those cases.
      if(third != right) // TODO: can be removed
      {
        LJunction *left_l = LBetween(third, end_3, RIGHT, right);
        // only if there is also an L-jct between the other two lines
        if(left_l != 0)
          if(NoTJunctionYet(right, end_r))
          {
            NewT(right, third, left, end_r, end_3, end_l,
              left->coll[end_l][c], left_l, new_l);
          }
      }
    }
  }
}

void FormJunctions::CreateC(Line *line_i, int vtype_i, Line *line_j,
    int vtype_j)
{
  int end_i = VOTE_END(vtype_i);
  int end_j = VOTE_END(vtype_j);

  // if a line is part of a split and we have junction at end, defer
  // creation of junctions to last line in that split
  if(end_i == END)
    while(line_i->next != 0)
      line_i = line_i->next;
  if(end_j == END)
    while(line_j->next != 0)
      line_j = line_j->next;

  Collinearity *new_c = NewCollinearity(line_i, line_j, end_i, end_j, true);

  // create T-jcts from new collinearity and old L-jcts
  if(new_c != 0)
  {
    for(int side = LEFT; side <= RIGHT; side++)
      for(unsigned l = 0; l < line_i->l_jct[end_i][side].Size(); l++)
      {
        LJunction *ljct = line_i->l_jct[end_i][side][l];
        Line *third = ljct->line[Other(side)];
        int end_3 = ljct->near_point[Other(side)];
        // Note that sometimes there is a collinearity AND L-jct between two
        // lines.  Do not create a T-jct for those cases.
        if(third != line_j) // TODO: can be rmoved
        {
          if(NoTJunctionYet(third, end_3))
          {
            TJunction *new_t = 0;
            if(side == RIGHT)  // i is RIGHT side of L-jct
            {
              // then i is left arm of T-jct, ljct is the left L-jct
              LJunction *ljct_right = LBetween(third, end_3, RIGHT, line_j);
              // only if there is also an L-jct between the other two lines
              if(ljct_right != 0)
                new_t = NewT(third, line_i, line_j, end_3, end_i, end_j,
                    new_c, ljct, ljct_right);
            }
            else  // i is LEFT side of L-jct
            {
              // then i is the right arm of T-jct, ljct is the right L-jct
              LJunction *ljct_left = LBetween(third, end_3, LEFT, line_j);
              if(ljct_left != 0)
                new_t = NewT(third, line_j, line_i, end_3, end_j, end_i,
                    new_c, ljct_left, ljct);
            }
          }
        }
      }
  }
}

/**
 * Do some checks and if ok create collinearity.
 * @param inform  Inform the system of new L-jct. Normally this will be true.
 *                But not if coll. is created tue to a line split.
 */
Collinearity* FormJunctions::NewCollinearity(Line *line_i, Line *line_j,
   int end_i, int end_j, bool inform)
{
  Line *line[2];
  int near_point[2];
  // line 0 is shorter
  if(line_i->Length() <= line_j->Length())
  {
    line[0] = line_i;
    line[1] = line_j;
    near_point[0] = end_i;
    near_point[1] = end_j;
  }
  else
  {
    line[0] = line_j;
    line[1] = line_i;
    near_point[0] = end_j;
    near_point[1] = end_i;
  }
  // tangents must point at each other
  if(Dot(line_i->tang[end_i], line_j->tang[end_j]) < 0.)
    if(NoCollinearityYet(line_i, end_i, line_j) &&
       NoLJunctionYet(line_i, end_i, line_j))
    {
      Vector2 v = line[0]->point[near_point[0]] - line[1]->point[near_point[1]];
      Collinearity *new_c = new Collinearity(core, line_i, line_j, end_i, end_j);
      core->NewGestalt(new_c, inform);
      return new_c;
    }
  return 0;
}

LJunction* FormJunctions::NewL(Line *line_i, Line *line_j, int end_i, int end_j)
{
  try
  {
    double si, sj;  // lengths to intersection point
    Vector2 inter = LineIntersection(
        line_i->point[end_i], line_i->tang[end_i],
        line_j->point[end_j], line_j->tang[end_j], &si, &sj);
    if(NoLJunctionYet(line_i, end_i, line_j) &&
       NoCollinearityYet(line_i, end_i, line_j))
    {
      LJunction *new_l = new LJunction(core, line_i, line_j, end_i, end_j, inter);
      core->NewGestalt(new_l);
      return new_l;
    }
  }
  catch(Except &e)
  {
    // if tangents are parallel -> no L-jct
  }
  return 0;
}

TJunction* FormJunctions::NewT(Line *pole, Line *left, Line *right,
    int end_p, int end_l, int end_r,
    Collinearity *coll, LJunction *ljct_l, LJunction *ljct_r)
{
  try
  {
    if(coll != 0 && ljct_l != 0 && ljct_r != 0)
      if(NoTJunctionYet(pole, end_p) && NoTJunctionYet(left, end_l) &&
         NoTJunctionYet(right, end_r))
      {
        double b, g;
        Vector2 inter = LineIntersection(
                pole->point[end_p], pole->tang[end_p],
                left->point[end_l], left->tang[end_l], &g, &b);
        TJunction *new_t = new TJunction(core, pole, left, right,
              end_p, end_l, end_r, coll, ljct_l, ljct_r, g, inter);
        core->NewGestalt(new_t);
        return new_t;
      }
  }
  catch(Except &e)
  {
    // if tangents are parallel -> no L-jct
  }
  return 0;
}

void FormJunctions::SplitLine(Line *l, const Vector2 &inter,
    const Vector2 &dir, Line **l_left, Line **l_right,
    int *end_left, int *end_right, Collinearity **c_new)
{
  Line *l_new = SplitLine(l, inter, c_new);
  if(LeftOf(dir, l->point[START] - inter))
  {
    *l_left = l;
    *l_right = l_new;
    *end_left = END;
    *end_right = START;
  }
  else
  {
    *l_left = l_new;
    *l_right = l;
    *end_left = START;
    *end_right = END;
  }
}

Line* FormJunctions::SplitLine(Line *l, const Vector2 &p, Collinearity **c_new)
{
  // create new line
  Line *l_new = l->Split(p);
  // create new collinearity.
  // Note that if one of the split lines is very short and was near a
  // high-curvature part of the segment (i.e. near an L-jct), this line might
  // change orientation so much that a collinearity is rejected. In that case
  // form an L-jct.
  *c_new = NewCollinearity(l, l_new, END, START, false);
  // TODO: c_new actually can not be UNDEF
  if(*c_new == 0)
    NewL(l, l_new, END, START);
  UpdateClosures(l, l_new, *c_new);
  return l_new;
}

/**
 * Line l1 has been split in to l1 and l2 with a colliinearity between them.
 * Update all closures which l1 is part of accordingly.
 */
void FormJunctions::UpdateClosures(Line *l1, Line *l2, Collinearity *coll)
{
  for(unsigned i = 0; i < l1->closures.Size(); i++)
  {
    Closure *cl = l1->closures[i];
    unsigned j = cl->lines.Find(l1);
    // note: depending on sense of l1, l2 is inserted before or after l2, but
    // the new collinearity is always inserted after the starting junction of l1
    if(cl->senses[j] == SAME)
    {
      cl->lines.InsertAfter(j, l2);
      cl->senses.InsertAfter(j, cl->senses[j]);
      cl->jcts.InsertAfter(j, 0);
      cl->colls.InsertAfter(j, coll);
    }
    else
    {
      cl->lines.InsertBefore(j, l2);
      cl->senses.InsertBefore(j, cl->senses[j]);
      cl->jcts.InsertAfter(j, 0);
      cl->colls.InsertAfter(j, coll);
    }
    l2->closures.PushBack(cl);
  }
}

}

