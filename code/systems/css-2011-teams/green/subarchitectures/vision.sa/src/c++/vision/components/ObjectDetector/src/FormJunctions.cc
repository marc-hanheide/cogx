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
#include "Ellipse.hh"
#include "ExtEllipse.hh"
#include "Cylinder.hh"
#include "LJunction.hh"
#include "TJunction.hh"
#include "EJunction.hh"
#include "Collinearity.hh"
#include "Corner.hh"
#include "Closure.hh"
#include "Cube.hh"
#include "FormJunctions.hh"
#include "HCF.hh"
#include "Vector2.hh"

namespace Z
{

// Minimal line length. A line of 2 pixels is per definition always a line and
// therefore totally insignificant. Any meaningful line must be at least 3
// pixels long.
static const double MIN_LINE_LENGTH = 3.;
	
enum GrowMethod {GROW_EQUAL, GROW_WEIGHTED, GROW_SMART};
static GrowMethod grow_method = GROW_SMART;

/**
 * Returns a pseudo random number in [0.0, 1.0]
 */
inline static float FRand()
{
  return RandInt()/((float)RAND_MAX + 1.);
}

inline static float ExpDev(float lambda)
{
  float dum;
  do
    dum = FRand();
  while (dum == 0.);
  return -log(dum)/lambda;
}

inline static int ExpSelect(int max)
{
  int i;
  /* we want 99% probability of getting with expdev() a number smaller max
   * this requires a lambda of the exponential distribution:
   * lambda = -log(0.01)/max;    (-log(0.01) = 4.6) */
  float lambda = 4.6/(float)max;
  do
    i = (int)(ExpDev(lambda));
  while(i > max);
  return i;
}

static int CmpLJunctions(const void *a, const void *b)
{
  if( LJunctions(*(unsigned*)a)->acc < LJunctions(*(unsigned*)b)->acc )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

static int CmpCollinearities(const void *a, const void *b)
{
  if( Collinearities(*(unsigned*)a)->acc < Collinearities(*(unsigned*)b)->acc )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

static int CmpTJunctions(const void *a, const void *b)
{
  if( TJunctions(*(unsigned*)a)->acc < TJunctions(*(unsigned*)b)->acc )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

static int CmpLines(const void *a, const void *b)
{
  if( Lines(*(unsigned*)a)->sig > Lines(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

static bool InsideLine(unsigned l, const Vector2 &p)
{
  double d = Dot(Lines(l)->dir, p - Lines(l)->point[START]);
  return d >= 0. && d <= Lines(l)->Length();
}

/**
 * Check whether there is an L-jct between lines i and j.
 */
static unsigned LBetween(unsigned i, unsigned end_i, unsigned side_i,
    unsigned j)
{
  Line *l = Lines(i);
  for(unsigned k = 0; k < l->l_jct[end_i][side_i].Size(); k++)
    if(LJunctions(l->l_jct[end_i][side_i][k])->line[OtherSide(side_i)] == j)
      return l->l_jct[end_i][side_i][k];
  return UNDEF_ID;
}

/**
 * Check whether there is no L-jct between lines i and j.
 */
static bool NoLJunctionYet(unsigned i, unsigned end_i, unsigned j)
{
  Line *l = Lines(i);
  // TODO: it would be nice to know which side to look
  for(unsigned side = LEFT; side <= RIGHT; side++)
    for(unsigned k = 0; k < l->l_jct[end_i][side].Size(); k++)
      if(LJunctions(l->l_jct[end_i][side][k])->line[OtherSide(side)] == j)
        return false;
  return true;
}

/**
 * Check whether there is no E-jct with line and ellipse.
 * (ignore end!)
 */
static bool NoEJunctionYet(unsigned line, unsigned ellipse, unsigned end)
{
  Line *l = Lines(line);
//  for(unsigned i = 0; i < l->e_jct[end].Size(); i++)
//    if(EJunctions(l->e_jct[end][i])->ellipse == ellipse)
//      return false;
	
  // one line at one end (not allowed at both sides)
  for(unsigned i = 0; i < l->e_jct[START].Size(); i++)
    if(EJunctions(l->e_jct[START][i])->ellipse == ellipse)
      return false;
  for(unsigned i = 0; i < l->e_jct[END].Size(); i++)
    if(EJunctions(l->e_jct[END][i])->ellipse == ellipse)
      return false;
	
  return true;
}

static bool NoTJunctionYet(unsigned i, unsigned end_i)
{
  return Lines(i)->t_jct[end_i] == UNDEF_ID;
}

/**
 * Check whether there is no collinearity yet between lines i and j. 
 */
static bool NoCollinearityYet(unsigned i, unsigned end_i, unsigned j)
{
  Line *l = Lines(i);
  for(unsigned k = 0; k < l->coll[end_i].Size(); k++)
    if(Collinearities(l->coll[end_i][k])->OtherLine(i) == j)
      return false;
  return true;
}

VoteImage *FormJunctions::vote_img = 0;
unsigned FormJunctions::baseIndex = 12;
unsigned FormJunctions::baseOffset = NumLines();

FormJunctions::FormJunctions(Config *cfg)
: GestaltPrinciple(cfg)
{
  ActiveEJunctions(true);	// false, to disable EJunctions
  ActiveTJunctions(true);	// falso, to disable TJunctions (only generation of)

  next_principles.PushBack(FORM_RECTANGLES);
  vote_img = 0;
  first_op = true;
  SetupAdmissibilityMatrix();
	
}

FormJunctions::~FormJunctions()
{
  delete vote_img;
}

void FormJunctions::ActiveEJunctions(bool active)
{
  activeEJcts = active;
}

void FormJunctions::ActiveTJunctions(bool active)
{
  activeTJcts = active;
}

void FormJunctions::SetupAdmissibilityMatrix()
{
  unsigned i, j;
  for(i = 0; i < baseIndex; i++)
    for(j = 0; j < baseIndex; j++)
      isct_ok[i][j] = false;
  // note: We fill out the upper right side of the matrix. It is important to
  // keep in mind the numbers corresponding to the symbolic names.
  // I.e. in isct_ok[i][j] i must be <= j.
  // tangents/edge						=> T-Junctions
  isct_ok[VOTE_E][VOTE_TS] = true;
  isct_ok[VOTE_E][VOTE_TE] = true;
  // tangents/tangents					=> L-Junctions or Collinearities
  isct_ok[VOTE_TS][VOTE_TE] = true;
  isct_ok[VOTE_TS][VOTE_TS] = true;
  isct_ok[VOTE_TE][VOTE_TE] = true;
  // tangents/normals					=> L-Junctions or Collinearities
  isct_ok[VOTE_TS][VOTE_NLS] = true;
  isct_ok[VOTE_TS][VOTE_NLE] = true;
  isct_ok[VOTE_TS][VOTE_NRS] = true;
  isct_ok[VOTE_TS][VOTE_NRE] = true;
  isct_ok[VOTE_TE][VOTE_NLS] = true;
  isct_ok[VOTE_TE][VOTE_NLE] = true;
  isct_ok[VOTE_TE][VOTE_NRS] = true;
  isct_ok[VOTE_TE][VOTE_NRE] = true;
	
  // ellipses							=> E-Junctions
  if (activeEJcts)
  {
    isct_ok[VOTE_E][VOTE_EOTL] = true; 	// edge itself & ellipse tangents
    isct_ok[VOTE_E][VOTE_EOTR] = true;	
    isct_ok[VOTE_E][VOTE_EITL] = true;
    isct_ok[VOTE_E][VOTE_EITR] = true;
  
    isct_ok[VOTE_TS][VOTE_EOTL] = true;	// tangents - outer ellipse tangents
    isct_ok[VOTE_TS][VOTE_EOTR] = true;
    isct_ok[VOTE_TE][VOTE_EOTL] = true;
    isct_ok[VOTE_TE][VOTE_EOTR] = true;

    isct_ok[VOTE_NLS][VOTE_EITR] = false; 	// ??
    isct_ok[VOTE_NLE][VOTE_EITR] = false; 	// ??
    isct_ok[VOTE_NRS][VOTE_EITR] = false; 	// ??
    isct_ok[VOTE_NRE][VOTE_EITR] = false; 	// ??

    isct_ok[VOTE_NLS][VOTE_EITL] = false; 	// ??
    isct_ok[VOTE_NLE][VOTE_EITL] = false; 	// ??
    isct_ok[VOTE_NRS][VOTE_EITL] = false; 	// ??
    isct_ok[VOTE_NRE][VOTE_EITL] = false; 	// ??
  
    isct_ok[VOTE_TS][VOTE_EITL] = true;	// Tangents - inner ellipse tangents
    isct_ok[VOTE_TS][VOTE_EITR] = true;
    isct_ok[VOTE_TE][VOTE_EITL] = true;
    isct_ok[VOTE_TE][VOTE_EITR] = true;
  }
  
  // now mirror along diagonal to fill lower left side of matrix
  for(i = 0; i < baseIndex; i++)
    for(j = i + 1; j < baseIndex; j++)
      isct_ok[j][i] = isct_ok[i][j];
}

void FormJunctions::Reset(const Image *img)
{
  if(vote_img != 0 &&
     !(vote_img->width == (int)img->Width() &&
       vote_img->height == (int)img->Height()))
  {
    delete vote_img;
    vote_img = 0;
  }
  if(vote_img == 0)
  {
    vote_img = new VoteImage(img->Width(), img->Height());
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
  StartRunTime();
  if(incremental)
  {
    if(first_op)
    {
			// set baseOffset for ellipse lines
			baseOffset = NumLines();

      // in first call just init search lines, don't create junctions
      vote_img->SetNumLines((baseOffset + NumEllipses())*baseIndex);	
		
			// init search lines for lines
      for(unsigned ri = 0; ri < NumLines(); ri++)
        InitSearchLines(RankedGestalts(Gestalt::LINE, ri));
	  
  	  // init search lines for ellipses
	  	if(activeEJcts)
        for(unsigned ri = 0; ri < NumEllipses(); ri++)
		  InitEllipseSearchLines(RankedGestalts(Gestalt::ELLIPSE, ri));

      first_op = false;
    }
    else
    {
      // for all subsequent calls
      // try to be smart about growing search lines
      if(grow_method == GROW_SMART)
      {
		// TODO ARI: gest wählt per Zufallszahl ob Ellipse oder Line extended 
		// wird => diese Wahl wird je nach Anzahl von Linien und Ellipsen
		// gewählt
		float gest = FRand();  // ARI: Random number 0-1
		float LiToEl = (float)NumLines()/((float)NumLines()+(float)NumEllipses());
		  
		if (gest < LiToEl)	
	    {
		  int r = ExpSelect(NumLines() - 1);
          ExtendSmartLines(Gestalt::LINE, RankedGestalts(Gestalt::LINE, r));
		}
		else if(activeEJcts)
		{
		  int r = ExpSelect(NumEllipses() - 1);
          ExtendSmartLines(Gestalt::ELLIPSE, RankedGestalts(Gestalt::ELLIPSE,r));
		}
		
		VisionCore::stats.pix_cnt += 1;
      }
      // lines grow probabilistically according to length
      else if(grow_method == GROW_WEIGHTED)										// TODO ARI: no EJunctions with GROW_WEIGHTED
      {
        int r = ExpSelect(NumLines() - 1);
        ExtendSearchLines(RankedGestalts(Gestalt::LINE, r));
        VisionCore::stats.pix_cnt += 6;
      }
      // all search lines grow equally, this is less "anytime-ish"				// TODO ARI: no EJunctions with GROW_EQUAL
      else if(grow_method == GROW_EQUAL)
      {
        for(unsigned ri = 0; ri < NumLines(); ri++)
        {
          ExtendSearchLines(RankedGestalts(Gestalt::LINE, ri));
          VisionCore::stats.pix_cnt += 6;
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
      unsigned nlines = NumLines();
      vote_img->SetNumLines(nlines*8);
      for(unsigned ri = 0; ri < nlines; ri++)
      {
        unsigned i = RankedGestalts(Gestalt::LINE, ri);
        unsigned l = (unsigned)Lines(i)->Length();
        InitSearchLines(i);
        // draw all search lines to a length of l
        for(unsigned j = 0; j < l; j++)
        {
          ExtendSearchLines(i);
          VisionCore::stats.pix_cnt += 6;
        }
      }
      first_op = false;
    }
  }
  StopRunTime();
}

void FormJunctions::Rank()
{
  RankGestalts(Gestalt::L_JUNCTION, CmpLJunctions);
  RankGestalts(Gestalt::COLLINEARITY, CmpCollinearities);
  RankGestalts(Gestalt::T_JUNCTION, CmpTJunctions);
  RankGestalts(Gestalt::LINE, CmpLines);
}

void FormJunctions::InitSearchLines(unsigned line)
{
  double len = 1000; 								// (max) length									// TODO: replace this by "until image border"
  unsigned sline = line*baseIndex;  // search line 
  Line *l = Lines(line);						// line

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

void FormJunctions::InitEllipseSearchLines(unsigned ellipse)
{
  Ellipse *e = Ellipses(ellipse);		
	
  // calculate length of the major axis
  double majorAxis = (Ellipses(ellipse)->vertex[1] - 
					  Ellipses(ellipse)->vertex[0]).Norm();

  // limit the maximal length of the search lines
  unsigned len = (unsigned) majorAxis/4; 

  // search line base index for this line
  unsigned sline = (baseOffset + ellipse)*baseIndex;  

  // initialise outer major axis search lines
  vote_img->InitLine(e->vertex[LEFT], e->vertex[LEFT] - len*e->dir,
      /*Gestalt::LINE,*/ sline + VOTE_EOTL);
  vote_img->InitLine(e->vertex[RIGHT], e->vertex[RIGHT] + len*e->dir,
      /*Gestalt::LINE,*/ sline + VOTE_EOTR);

  // initialise inner major axis search lines
  vote_img->InitLine(e->vertex[LEFT], e->vertex[LEFT] + len*e->dir,
      /*Gestalt::LINE,*/ sline + VOTE_EITL);
  vote_img->InitLine(e->vertex[RIGHT], e->vertex[RIGHT] - len*e->dir,
      /*Gestalt::LINE,*/ sline + VOTE_EITR);
}

void FormJunctions::ExtendSearchLines(unsigned line)
{
  // lines created by splitting have no vote lines
  if(Lines(line)->defer_vote == line)
  {
    unsigned sline = line*baseIndex + VOTE_TS;
    unsigned smax = line*baseIndex + VOTE_NRE;
    for(; sline <= smax; sline++)
      if(vote_img->ExtendLine(sline, iscts) > 0)
        CreateLineJunctions(sline, iscts);
  }
}

/**
 * Be somewhat smart about extending search lines.
 * Prefer tangential search lines and open line ends, boost neighbouring lines.
 */
void FormJunctions::ExtendSmartLines(Gestalt::Type type, unsigned idx)
{
  int end;
	  
  // if type is a line
  if (type == Gestalt::LINE)
  {
    unsigned line = idx;
	// Selecting which end to extend is given by the number of junctions already
	// present at that end: p(START)/p(END) = n(END)/n(START).
	// Regarding T-junctions: Each T-jct at one end is accompanied by two L-jcts.
	// Therefore reduce the count of jcts at this side by two.
	// Furthermore add 1 to the jct count on the other end to further favour this
	// end.
	int n_t[2] = {(Lines(line)->t_jct[START] != UNDEF_ID ? 1 : 0),
        (Lines(line)->t_jct[END] != UNDEF_ID ? 1 : 0)};
	int n_s = 1 + Lines(line)->coll[START].Size() +
    	Lines(line)->l_jct[START][LEFT].Size() +
    	Lines(line)->l_jct[START][RIGHT].Size() - 2*n_t[START] + n_t[END];
  	int n_e = 1 + Lines(line)->coll[END].Size() +
    	Lines(line)->l_jct[END][LEFT].Size() +
    	Lines(line)->l_jct[END][RIGHT].Size() - 2*n_t[END] + n_t[START];
  	int r = RandInt()%(n_s + n_e);
				
	if(r < n_s)  // 0 .. n_s - 1
      end = END;
	else         // n_s .. n_s + n_e - 1
  	  end = START;
    FollowEnd(line, end, line);
  }

  // if type is an ellipse
  if (type == Gestalt::ELLIPSE)
  {
		int n_l = 1 + Ellipses(idx)->e_jct[LEFT].Size();	// E-Jcts LEFT
		int n_r = 1 + Ellipses(idx)->e_jct[RIGHT].Size();	// E-Jcts RIGHT

		int r = RandInt()%(n_l + n_r);
		unsigned end;
		if(r < n_l)  // 0 .. n_l - 1
			end = LEFT;
		else         // n_l .. n_l + n_r - 1
			end = RIGHT;
		ExtendEllipseEnd(idx, end);	  
  }	

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
void FormJunctions::FollowEnd(unsigned line, int end, unsigned stop_line)
{
  int n_c = Lines(line)->coll[end].Size();
  int n_l = Lines(line)->l_jct[end][LEFT].Size();
  int n_r = Lines(line)->l_jct[end][RIGHT].Size();
  int n_t = (Lines(line)->t_jct[end] != UNDEF_ID ? 1 : 0);

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
      unsigned fline = UNDEF_ID;
      int fend = UNDEF_ID;

      VisionCore::stats.hop_cnt++;
      w = RandInt()%(n_c + n_l + n_r);
      // if one of the collinearities
      if(w < n_c)
      {
        i = w;
        fline =
          Collinearities(Lines(line)->coll[end][i])->OtherLine(line);
        fend =
          Other(Collinearities(Lines(line)->coll[end][i])->WhichEndIs(fline));
      }
      // if one of the left L-jcts
      else if(w < n_c + n_l)
      {
        i = w - n_c;
        fline =
          LJunctions(Lines(line)->l_jct[end][LEFT][i])->line[RIGHT];
        fend =
          Other(LJunctions(Lines(line)->l_jct[end][LEFT][i])->near_point[RIGHT]);
      }
      // if on of the right L-jcts
      else
      {
        i = w - n_c - n_l;
        fline =
          LJunctions(Lines(line)->l_jct[end][RIGHT][i])->line[LEFT];
        fend =
          Other(LJunctions(Lines(line)->l_jct[end][RIGHT][i])->near_point[LEFT]);
      }
      // avoid infinite loops
      if(fline != stop_line)
        FollowEnd(fline, fend, stop_line);
      else
      {
        // select any line at random (ignoring rank)
        fline = RandInt()%NumLines();
        fend = RandInt()%2;
        FollowEnd(fline, fend, fline);
      } 
    }
  }
}

void FormJunctions::ExtendEnd(unsigned line, unsigned end)
{
  int r = RandInt();
  // factor for tangent extension: equal chance for open end, favour tangents
  // if T-junction (look for amodal completion)
  int t = 1 + (Lines(line)->t_jct[end] != UNDEF_ID ? 1 : 0);
  // factor for normal extension
  int n = 1;
  unsigned voteline = line, sline;

  // lines created by splitting defer voting to their original lines
  while(Lines(voteline)->defer_vote != voteline)
    voteline = Lines(voteline)->defer_vote;
  sline = voteline*baseIndex;
  if(r%(t + n) < t) // tangent
  {
    sline += (end == START ? VOTE_TS : VOTE_TE);
  }
  else  // normal
  {
    unsigned side = (r/(t + n))%2;  // equal chance
    if(side == LEFT)
      sline += (end == START ? VOTE_NLS : VOTE_NLE);
    else
      sline += (end == START ? VOTE_NRS : VOTE_NRE);
  }
//	if (vote_img->ImageEnd(sline)) printf("XXX Image End: %u\n", line);																							// TODO ARI
  if(vote_img->ExtendLine(sline, iscts) > 0)
    CreateLineJunctions(sline, iscts);
}

/*
**	ExtendEllipseEnd
**	
*/
void FormJunctions::ExtendEllipseEnd(unsigned ellipse, unsigned end)
{
  // Inner our outer line? (equal chance)
  unsigned side = RandInt()%2; 

  // calculate search line
  unsigned sline = (baseOffset + ellipse)*baseIndex;	

  if (end == LEFT)
  {
	if (side == INNER)
	  if(vote_img->ExtendLine(sline+=VOTE_EITL, iscts) > 0)
    	CreateLineJunctions(sline, iscts);
	if (side == OUTER)
	  if(vote_img->ExtendLine(sline+=VOTE_EOTL, iscts) > 0)
    	CreateLineJunctions(sline, iscts);
  }
  else 
  {
	if (side == INNER)
	  if(vote_img->ExtendLine(sline+=VOTE_EITR, iscts) > 0)
    	CreateLineJunctions(sline, iscts);
	if (side == OUTER)
	  if(vote_img->ExtendLine(sline+=VOTE_EOTR, iscts) > 0)
    	CreateLineJunctions(sline, iscts);
  }
  
}

/*
**	Create Line Junctions:
**	sline is the search line which triggered the new intersections
*/
void FormJunctions::CreateLineJunctions(unsigned sline,
    Array<VoteImage::Elem> &iscts)
{
  for(unsigned k = 0; k < iscts.Size(); k++)
  {
    unsigned i = sline/baseIndex;
		unsigned vtype_i = sline%baseIndex;
    unsigned j = iscts[k].id/baseIndex;
		unsigned vtype_j = iscts[k].id%baseIndex;

		// check if the intersection between vote line types is admissible
		if(isct_ok[vtype_i][vtype_j])									    		// TODO: this admissibility check could go into VoteImage
    {
	  	// vtype_i is from ellipse j => Change i-j and vtype_i-vtype_j
      if(vtype_j == VOTE_EOTL || vtype_j == VOTE_EOTR || vtype_j == VOTE_EITL || vtype_j == VOTE_EITR)			
	  	{		
				unsigned z = vtype_j;
				vtype_j = vtype_i;
				vtype_i = z;
				z = i; i = j; j = z;
	  	}
	  
	  	// vtype_i is from ellipse i => Create E-Junction
      if(vtype_i == VOTE_EOTL || vtype_i == VOTE_EOTR || vtype_i == VOTE_EITL || vtype_i == VOTE_EITR)			
	  	{		
				// ellipse => i-baseOffset
				i -= baseOffset;
		  
				// get the line end	
				unsigned end_j = VOTE_END(vtype_j);

				// get vertex of ellipse (LEFT/RIGHT)
				unsigned vertex;
				if(vtype_i == VOTE_EOTL || vtype_i == VOTE_EITL) vertex = LEFT;
				else vertex = RIGHT;

				// calculate the nearer end and define line-end (end_j)
				if(vtype_j == VOTE_E)
				{
					// Position of line-ends & ellipse-vertex
					Vector2 linePos[2] = Lines(j)->point;
					Vector2 vertexPos = Ellipses(i)->vertex[vertex];
					
					if((linePos[0]-vertexPos).Norm() < (linePos[1]-vertexPos).Norm())
					end_j=START; 
					else
					end_j=END;
				}

				// get the last line in the split, if end_j = END
				if(end_j == END)
					while(Lines(j)->next != UNDEF_ID)
							j = Lines(j)->next;

				CreateE(j, i, end_j, vertex);
			}		  
	  																	     	// TODO: After a T-jct there can be no further T-jcts or type1
      																			// collinearities. -> really
		  // vtype_j is from edge itself => Create T-Junction
      else if(vtype_j == VOTE_E)
        CreateT(i, vtype_i, j, vtype_j);

			// no E- or T-Junction => Create L-Junction or Collinearity
      else 
      {
				// HACK ARI: Check if line is part of a split
				unsigned end_i = VOTE_END(vtype_i);
  			unsigned end_j = VOTE_END(vtype_j);
  			// if a line is part of a split and we have junction at end, defer
  			// creation of junctions to last line in that split
  			if(end_i == END)
    			while(Lines(i)->next != UNDEF_ID)
      			i = Lines(i)->next;
				if(end_j == END)
					while(Lines(j)->next != UNDEF_ID)
						j = Lines(j)->next;

        // HACK: This is a stupid arbitrary threshold! As the distinction
        // between collinearity and L-junction will fall, so will this
        // threshold.
        if(AngleBetweenLines(Lines(i)->phi, Lines(j)->phi) < M_PI/6)				// TODO Threshold
          CreateC(i, vtype_i, j, vtype_j);
        else
				  CreateL(i, vtype_i, j, vtype_j);
      }
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
void FormJunctions::CreateT(unsigned i, unsigned vtype_i, unsigned j,
    unsigned vtype_j)
{
  try
  {
    unsigned end_i = VOTE_END(vtype_i);
    // if a line is part of a split and we have junction at end, defer
    // creation of junctions to last line in that split
    // TODO: use convenience function for this
    if(end_i == END)
      while(Lines(i)->next != UNDEF_ID)
        i = Lines(i)->next;
    // if line j is part of a split find that part where the intersection
    // point actually lies
    if(Lines(j)->IsSplit())
      while(j != UNDEF_ID && !InsideLine(j, Lines(i)->point[end_i]))
        j = Lines(j)->next;
    if(j != UNDEF_ID && InsideLine(j, Lines(i)->point[end_i]))
    {
      double g;  // gap between end of line i and line j
      double b;  // smaller bar length
      Vector2 inter = LineIntersection(
          Lines(i)->point[end_i], Lines(i)->tang[end_i],
          Lines(j)->point[START], Lines(j)->dir, &g, &b);
      // if smaller bar length is long enough for a line
      if(min(b, Lines(j)->Length() - b) >= MIN_LINE_LENGTH)
      {
        // note: for now we allow only one T-jct per end
        // TODO: maybe relax this at some later stage
        if(NoTJunctionYet(i, end_i))
        {
          unsigned j_left, j_right;
          unsigned end_left, end_right;
          unsigned new_c = UNDEF_ID, new_t = UNDEF_ID;
 		  SplitLine(j, inter, Lines(i)->tang[end_i], &j_left, &j_right,
              &end_left, &end_right, &new_c);
          unsigned new_ljct_left = NewL(i, j_left, end_i, end_left);
          unsigned new_ljct_right = NewL(j_right, i, end_right, end_i);
          new_t = NewT(i, j_left, j_right, end_i, end_left, end_right,
              new_c, new_ljct_left, new_ljct_right);
        }
      }
      else // create just an L-junction
      {
        // find out what end of j the L-junction should be
        unsigned end_j = (b < Lines(j)->Length()/2. ? START : END);
        // HACK: This is a stupid arbitrary threshold! As the distinction
        // between collinearity and L-junction will fall, so will this
        // threshold.
        if(AngleBetweenLines(Lines(i)->phi, Lines(j)->phi) < M_PI/6)
          NewC(i, j, end_i, end_j, true);
        else
          NewL(i, j, end_i, end_j);
      }
    }
  }
  catch(Except &e)
  {
    // if tangents are parallel -> no intersections
  }
}

void FormJunctions::CreateL(unsigned i, unsigned vtype_i, unsigned j,
    unsigned vtype_j)
{
  unsigned end_i = VOTE_END(vtype_i);
  unsigned end_j = VOTE_END(vtype_j);

  unsigned new_l = NewL(i, j, end_i, end_j);

  // create T-jcts from new L-jct and old collinearities
  // TODO: tidy this up
  if(new_l != UNDEF_ID)
  {
    unsigned left = LJunctions(new_l)->line[LEFT];
    unsigned right = LJunctions(new_l)->line[RIGHT];
    unsigned end_l = LJunctions(new_l)->near_point[LEFT];
    unsigned end_r = LJunctions(new_l)->near_point[RIGHT];
    for(unsigned c = 0; c < Lines(right)->coll[end_r].Size(); c++)
    {
      // third would be the right arm of a new T-jct
      unsigned third =
        Collinearities(Lines(right)->coll[end_r][c])->OtherLine(right);
      unsigned end_3 =
        Collinearities(Lines(right)->coll[end_r][c])->WhichEndIs(third);
      // Note that sometimes there is a collinearity AND L-jct between two
      // lines.  Do not create a T-jct for those cases.
      if(third != left) // TODO: can be removed
      {
        unsigned right_l = LBetween(third, end_3, LEFT, left);
        // only if there is also an L-jct between the other two lines
        if(right_l != UNDEF_ID)
          if(NoTJunctionYet(left, end_l))
          {
            NewT(left, right, third, end_l, end_r, end_3,
              Lines(right)->coll[end_r][c], new_l, right_l);
          }
      }
    }
    for(unsigned c = 0; c < Lines(left)->coll[end_l].Size(); c++)
    {
      // third would be the left arm of a new T-jct
      unsigned third =
        Collinearities(Lines(left)->coll[end_l][c])->OtherLine(left);
      unsigned end_3 =
        Collinearities(Lines(left)->coll[end_l][c])->WhichEndIs(third);
      // Note that sometimes there is a collinearity AND L-jct between two
      // lines.  Do not create a T-jct for those cases.
      if(third != right) // TODO: can be removed
      {
        unsigned left_l = LBetween(third, end_3, RIGHT, right);
        // only if there is also an L-jct between the other two lines
        if(left_l != UNDEF_ID)
          if(NoTJunctionYet(right, end_r))
          {
            NewT(right, third, left, end_r, end_3, end_l,
              Lines(left)->coll[end_l][c], left_l, new_l);
          }
      }
    }
  }
}

void FormJunctions::CreateC(unsigned i, unsigned vtype_i, unsigned j,
    unsigned vtype_j)
{
  unsigned end_i = VOTE_END(vtype_i);
  unsigned end_j = VOTE_END(vtype_j);

  // HACK ARI: copied to CreateLineJunctions
  // if a line is part of a split and we have junction at end, defer
  // creation of junctions to last line in that split
  //if(end_i == END)
  //  while(Lines(i)->next != UNDEF_ID)
  //    i = Lines(i)->next;
  //if(end_j == END)
  //  while(Lines(j)->next != UNDEF_ID)
  //    j = Lines(j)->next;

  unsigned new_c = NewC(i, j, end_i, end_j, true);

  // create T-jcts from new collinearity and old L-jcts
  if(new_c != UNDEF_ID)
  {
    for(unsigned side = LEFT; side <= RIGHT; side++)
      for(unsigned l = 0; l < Lines(i)->l_jct[end_i][side].Size(); l++)
      {
        unsigned ljct = Lines(i)->l_jct[end_i][side][l];
        unsigned third = LJunctions(ljct)->line[Other(side)];
        unsigned end_3 = LJunctions(ljct)->near_point[Other(side)];
        // Note that sometimes there is a collinearity AND L-jct between two
        // lines.  Do not create a T-jct for those cases.
        if(third != j) // TODO: can be rmoved
        {
          if(NoTJunctionYet(third, end_3))
          {
            unsigned new_t = UNDEF_ID;
            if(side == RIGHT)  // i is RIGHT side of L-jct
            {
              // then i is left arm of T-jct, ljct is the left L-jct
              unsigned ljct_right = LBetween(third, end_3, RIGHT, j);
              // only if there is also an L-jct between the other two lines
              if(ljct_right != UNDEF_ID)
                new_t = NewT(third, i, j, end_3, end_i, end_j,
                    new_c, ljct, ljct_right);
            }
            else  // i is LEFT side of L-jct
            {
              // then i is the right arm of T-jct, ljct is the right L-jct
              unsigned ljct_left = LBetween(third, end_3, LEFT, j);
              if(ljct_left != UNDEF_ID)
                new_t = NewT(third, j, i, end_3, end_j, end_i,
                    new_c, ljct_left, ljct);
            }
          }
        }
      }
  }
}

/*
**	Create Ellipse Junction
**	TODO ARI: Calculate parameters (intersection, ect. )
*/
void FormJunctions::CreateE(unsigned line, unsigned ellipse, unsigned end, unsigned vertex)
{
  double gap[2];  // gap between line, ellipse and intersection point 

  // get ellipse-vertex and direction
  Vector2 ellVertex = Ellipses(ellipse)->vertex[vertex];
  Vector2 ellDir = Ellipses(ellipse)->dir;
	
  // calculate LineIntersection and gaps
	Vector2 inter;
	try
  {
		inter = LineIntersection(Lines(line)->point[end], Lines(line)->tang[end], ellVertex, ellDir, &gap[0], &gap[1]);
	}
	catch (Except &e)
	{
		printf("FormJunctions::CreateE(): Lines do not intersect exception.\n");
		inter.x = 0.01;
		inter.y = 0.01;
	}
  gap[0]=fabs(gap[0]);
  gap[1]=fabs(gap[1]);
	
  // Don´t create new EJunction, if it already exists
  if(NoEJunctionYet(line, ellipse, end))
  	NewGestalt(new EJunction(line, ellipse, end, vertex, inter, gap));						
}


/**
 * Do some checks and if ok create collinearity.
 * @param inform  Inform the system of new L-jct. Normally this will be true.
 *                But not if coll. is created tue to a line split.
 * 
 */
unsigned FormJunctions::NewC(unsigned i, unsigned j, unsigned end_i,
    unsigned end_j, bool inform)
{
  unsigned line[2], near_point[2];
  // line 0 is shorter
  if(Lines(i)->Length() <= Lines(j)->Length())
  {
    line[0] = i;
    line[1] = j;
    near_point[0] = end_i;
    near_point[1] = end_j;
  }
  else
  {
    line[0] = j;
    line[1] = i;
    near_point[0] = end_j;
    near_point[1] = end_i;
  }
  // tangents must point at each other
  if(Dot(Lines(i)->tang[end_i], Lines(j)->tang[end_j]) < 0.)
    if(NoCollinearityYet(i, end_i, j) && NoLJunctionYet(i, end_i, j))
    {
      Vector2 v = Lines(line[0])->point[near_point[0]] -
        Lines(line[1])->point[near_point[1]];
      return NewGestalt(new Collinearity(i, j, end_i, end_j), inform);
    }
  return UNDEF_ID;
}

unsigned FormJunctions::NewL(unsigned i, unsigned j, unsigned end_i,
    unsigned end_j)
{
  try
  {
    double si, sj;  // lengths to intersection point
    Vector2 inter = LineIntersection(
        Lines(i)->point[end_i], Lines(i)->tang[end_i],
        Lines(j)->point[end_j], Lines(j)->tang[end_j], &si, &sj);
    // For very "flat" intersections, the geometric intersection might be
    // "behind" (i.e. on the other end) of the voted intersection
    // In such a case si or sj would be negative. Reject those.
    // TODO: replace "2" by something proper
    //       is this check still needed?
    //if(si >= -2. && sj >= -2.)
      if(NoLJunctionYet(i, end_i, j) && NoCollinearityYet(i, end_i, j))
      {
        return NewGestalt(new LJunction(i, j, end_i, end_j, inter));
     }
  }
  catch(Except &e)
  {
    // if tangents are parallel -> no L-jct
  }
  return UNDEF_ID;
}

unsigned FormJunctions::NewT(unsigned pole, unsigned left, unsigned right,
    unsigned end_p, unsigned end_l, unsigned end_r,
    unsigned coll, unsigned ljct_l, unsigned ljct_r)
{
  if(!activeTJcts) return UNDEF_ID;
  try
  {
    if(coll != UNDEF_ID && ljct_l != UNDEF_ID && ljct_r != UNDEF_ID)
      if(NoTJunctionYet(pole, end_p) && NoTJunctionYet(left, end_l) &&
         NoTJunctionYet(right, end_r))
      {
        double b, g;
        Vector2 inter = LineIntersection(
                Lines(pole)->point[end_p], Lines(pole)->tang[end_p],
                Lines(left)->point[end_l], Lines(left)->tang[end_l], &g, &b);
        unsigned new_t = NewGestalt(new TJunction(pole, left, right,
              end_p, end_l, end_r, g, inter));
        TJunctions(new_t)->coll = coll;
        TJunctions(new_t)->ljct[LEFT] = ljct_l;
        TJunctions(new_t)->ljct[RIGHT] = ljct_r;
      }
  }
  catch(Except &e)
  {
    // if tangents are parallel -> no L-jct
  }
  return UNDEF_ID;
}

/*
**	Split Line
*/
void FormJunctions::SplitLine(unsigned l, const Vector2 &inter,
    const Vector2 &dir, unsigned *l_left, unsigned *l_right,
    unsigned *end_left, unsigned *end_right, unsigned *c_new)
{
  unsigned l_new = SplitLine(l, inter, c_new);
  if(LeftOf(dir, Lines(l)->point[START] - inter))
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

/*
**	Split Line
*/
unsigned FormJunctions::SplitLine(unsigned l, const Vector2 &p, unsigned *c_new)
{
  unsigned idx_split = FindLineSplitIdx(l, p);
  // create new line
  unsigned l_new = NewGestalt(new Line(Lines(l)->seg, idx_split,
        Lines(l)->idx[END]));
  // this line has no vote lines associated, therefore does not vote itself but
  // defers vote to original line
  Lines(l_new)->defer_vote = l;
  // insert properly in the split chain
  Lines(l_new)->next = Lines(l)->next;
  Lines(l)->next = l_new;
  // shorten old line
  Lines(l)->idx[END] = idx_split;
  Lines(l)->Recalc();
  // copy END junctions of old line to END of new line
  MoveJunctions(l, l_new, END);
  // create new collinearity.
  // Note that if one of the split lines is very short and was near a
  // high-curvature part of the segment (i.e. near an L-jct), this line might
  // change orientation so much that a collinearity is rejected. In that case
  // form an L-jct.
  *c_new = NewC(l, l_new, END, START, false);
  // TODO: c_new actually can not be UNDEF
  if(*c_new == UNDEF_ID)
    NewL(l, l_new, END, START);

  // Update all following Gestalts with line l as property
  UpdateGestalts(l, l_new, *c_new);
  
  return l_new;
}

/**
 * Find index of edgel of line l which is closest to point p.
 * Is used to find split point for lines.
 */
unsigned FormJunctions::FindLineSplitIdx(unsigned l, const Vector2 &p)
{
  unsigned i, imin = UNDEF_ID;
  double d, dmin = HUGE;
  for(i = Lines(l)->idx[START]; i <= Lines(l)->idx[END]; i++)
  {
    d = DistanceSquare(Segments(Lines(l)->seg)->edgels[i].p, p);
    if(d < dmin)
    {
      dmin = d;
      imin = i;
    }
  }
  return imin;
}

void FormJunctions::MoveJunctions(unsigned l1, unsigned l2, unsigned end)
{
  // T-junctions
  if(Lines(l1)->t_jct[end] != UNDEF_ID)
  {
    unsigned t = Lines(l1)->t_jct[end];
    TJunctions(t)->line[POLE] = l2;
    TJunctions(t)->Recalc();
    Lines(l2)->t_jct[end] = t;
    Lines(l1)->t_jct[end] = UNDEF_ID;
  }

  // passive T-junctions
  for(unsigned side = LEFT; side <= RIGHT; side++)
  {
    for(unsigned i = 0; i < Lines(l1)->pt_jct[end][side].Size(); i++)
    {
      unsigned t = Lines(l1)->pt_jct[end][side][i];
      TJunctions(t)->line[side] = l2;
      TJunctions(t)->Recalc();
      Lines(l2)->AddPassiveTJunction(end, side, t);
    }
    Lines(l1)->pt_jct[end][side].Clear();
  }

  // Collinearities
  for(unsigned i = 0; i < Lines(l1)->coll[end].Size(); i++)
  {
    unsigned c = Lines(l1)->coll[end][i];
    unsigned w = Collinearities(c)->WhichLineIs(l1);
    Collinearities(c)->line[w] = l2;
    // near_point stays the same
    Collinearities(c)->Recalc();
    Lines(l2)->AddCollinearity(end, c);
  }
  Lines(l1)->coll[end].Clear();

  // L-junctions
  for(unsigned side = LEFT; side <= RIGHT; side++)
  {
    for(unsigned i = 0; i < Lines(l1)->l_jct[end][side].Size(); i++)
    {
      unsigned j = Lines(l1)->l_jct[end][side][i];
      LJunctions(j)->line[side] = l2;
      LJunctions(j)->Recalc();
      Lines(l2)->AddLJunction(end, side, j);
    }
    Lines(l1)->l_jct[end][side].Clear();
  }

  // E-Junctions															
  for(unsigned i = 0; i < Lines(l1)->e_jct[end].Size(); i++)
  {
    unsigned e = Lines(l1)->e_jct[end][i];
	EJunctions(e)->line = l2;
    Lines(l2)->AddEJunction(end, e);
  }
  Lines(l1)->e_jct[end].Clear();

}

/*
**	TODO ARI: Macht es Sinn diese Updates in die jeweiligen Form-Klassen zu 
**	verschieben?
*/
void FormJunctions::UpdateGestalts(unsigned l, unsigned l_new, unsigned c_new)
{
  UpdateClosures(l, l_new, c_new);
  UpdateExtEllipses(l, l_new);
  UpdateCylinders(l, l_new);
  UpdateCorners(l, l_new);
  UpdateCubes(l, l_new);														// TODO ARI: Update Cube implementieren!
}


/**
 * Line l1 has been split in to l1 and l2 with a colliinearity between them.	
 * Update all closures which l1 is part of accordingly.
 */
void FormJunctions::UpdateClosures(unsigned l1, unsigned l2, unsigned coll)
{
  for(unsigned i = 0; i < Lines(l1)->closures.Size(); i++)
  {
    unsigned cl = Lines(l1)->closures[i];
    unsigned j = Closures(cl)->lines.Find(l1);
    // note: depending on sense of l1, l2 is inserted before or after l2, but
    // the new collinearity is always inserted after the starting junction of l1
    if(Closures(cl)->senses[j] == SAME)
    {
      Closures(cl)->lines.InsertAfter(j, l2);
      Closures(cl)->senses.InsertAfter(j, Closures(cl)->senses[j]);
      Closures(cl)->jcts.InsertAfter(j, UNDEF_ID);
      Closures(cl)->colls.InsertAfter(j, coll);
    }
    else
    {
      Closures(cl)->lines.InsertBefore(j, l2);
      Closures(cl)->senses.InsertBefore(j, Closures(cl)->senses[j]);
      Closures(cl)->jcts.InsertAfter(j, UNDEF_ID);
      Closures(cl)->colls.InsertAfter(j, coll);
    }
    Lines(l2)->closures.PushBack(cl);
  }
}

/**
 * Line l1 has been split in to l1 and l2 with a colliinearity between them.
 * Update all corners which l1 is part of accordingly.
 */
void FormJunctions::UpdateCorners(unsigned l1, unsigned l2)
{
  Array<unsigned> erase;				// Corners to erase from line l1
	
  for(unsigned i = 0; i < Lines(l1)->corners.Size(); i++)
  {
	unsigned c = Lines(l1)->corners[i];	// Corner

	// search position of line in corner
	for (unsigned j=0; j<Corners(c)->lines.Size(); j++)
	{
	  if (Corners(c)->lines[j] == l1)
	  {
		// if corner->near_point on START => everything fine
		// if corner->near_point on END => change l1 to l2
		if (Corners(c)->near_point[j] == END)
		{
		  // replace l1 with l2 (near_point is the same)
		  Corners(c)->lines[j] = l2;	
		  
		  // enter l2->corner
		  Lines(l2)->corners.PushBack(c);

		  // delete l1->corner
		  erase.PushBack(i);			
		}
	  }
    }
  }

  // delete l1->extEllipses from erase (note: erase from highest to lowest)
  unsigned j=erase.Size()-1;
  for (int i=j; i>=0; i--)
	Lines(l1)->corners.Erase(erase[i]);
}

/**
 * Line l1 has been split in to l1 and l2 with a collinearity between them.
 * Update all ExtEllipses which l1 is part of accordingly.
 */
void FormJunctions::UpdateExtEllipses(unsigned l1, unsigned l2)
{
  Array<unsigned> erase;				// ExtEllipses to erase from line l1
	
  for(unsigned i=0; i<Lines(l1)->extEllipses.Size(); i++)
  {
	unsigned eE = Lines(l1)->extEllipses[i];	// extEllipse

    // search position of line in ExtEllipse
	for (unsigned j=0; j<ExtEllipses(eE)->extLines.Size(); j++)
	{
	  if (ExtEllipses(eE)->extLines[j] == l1 &&
		  ExtEllipses(eE)->extLinesEnd[j] == END)
	  {		  
		// if extEllipse->extLinesNearPoint on START => everything fine
		// if corner->near_point on END => change l1 to l2
		  
		// replace l1 with l2 (near_point is the same)
		ExtEllipses(eE)->extLines[j] = l2;	
		// recalc ColLines
		ExtEllipses(eE)->CalculateCollLines();	

		// enter l2->extEllipses
		Lines(l2)->extEllipses.PushBack(eE);
			
		// delete l1->extEllipse
		erase.PushBack(i);
	  }
    }
	
	ExtEllipses(eE)->CalculateCollLines();
  }
  
  // delete l1->extEllipses from erase (note: erase from highest to lowest)
  unsigned j=erase.Size()-1;
  for (int i=j; i>=0; i--)
	Lines(l1)->extEllipses.Erase(erase[i]);
}

/**
 * Line l1 has been split in to l1 and l2 with a colliinearity between them.
 * Update all Cylinders (sharedLines) which l1 is part of accordingly.
 */
void FormJunctions::UpdateCylinders(unsigned l1, unsigned l2)
{
  for(unsigned i = 0; i < Lines(l1)->cylinders.Size(); i++)
  {
	unsigned c = Lines(l1)->cylinders[i];	// cylinders

	// search position of line in cylinder
	for (unsigned j=0; j<Cylinders(c)->sharedLines.Size(); j++)
	{
	  if (Cylinders(c)->sharedLines[j] == l1)
	  {
		// both lines are now sharedLines
		Cylinders(c)->sharedLines.PushBack(l2);  
		 
		// enter l2->cylinders
		Lines(l2)->cylinders.PushBack(c);
	  }
    }
  }
}


/**
 * Line l1 has been split in to l1 and l2 with a colliinearity between them.
 * Update all cubes, which l1 is part of accordingly.
 */
void FormJunctions::UpdateCubes(unsigned l1, unsigned l2)						// TODO ARI: Implementierung überarbeiten!
{
  																				// Cubes sind bei den Linien nicht eingetragen => Line::Cubes implementieren
  for(unsigned i = 0; i < NumCubes(); i++)										// TODO ARI: scans all existing cubes!!!
  {
	// l1 is an ljctLine? => Add l2 as first element in cLines
	if (Cubes(i)->jctLines[0] == l1)
	  Cubes(i)->cLines[0].InsertBefore(0, l2);
	else if (Cubes(i)->jctLines[1] == l1)
	  Cubes(i)->cLines[1].InsertBefore(0, l2);
	
	// l1 is an cLine? => Add l2 after l1-element in cLines
	if (Cubes(i)->cLines[0].Contains(l1))
	{
	  unsigned pos = Cubes(i)->cLines[0].Find(l1);
	  Cubes(i)->cLines[0].InsertAfter(pos, l2);
	}
	else if (Cubes(i)->cLines[1].Contains(l1))
	{
 	  unsigned pos = Cubes(i)->cLines[1].Find(l1);
	  Cubes(i)->cLines[1].InsertAfter(pos, l2);
	} 
  }

  // Update all cube canditates nicht notwendig (da linien der Kandidaten nicht
  // gespeichert werden!
}

}
