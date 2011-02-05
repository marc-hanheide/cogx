/**
 * @file FormConvexArcGroups.cc
 * @author Zillich
 * @date Feb 2007
 * @version 0.1
 * @brief Gestalt principle class FormConvexArcGroups.
 **/

#include <assert.h>
#include <algorithm>
#include "Math.hh"
#include "Draw.hh"
#include "VisionCore.hh"
#include "Segment.hh"
#include "Arc.hh"
#include "ConvexArcGroup.hh"
#include "FormConvexArcGroups.hh"
#include "Ellipse.hh"
#include "Segment.hh"
#include "Arc.hh"

namespace Z
{

/* Determine whether two tangents belong to different same arcs.
 * Note that arc i has tangents 2*i and 2*i+1 */
#define FROM_DIFF_ARCS(i,j) ((i)/2 != (j)/2)

/* Returns true if one tangent is even and the other odd.
 */
#define DIFF_ODDITY(i,j) ((i)%2 != (j)%2)

static const int SEARCH_EXHAUSTIVE = 0;
static const int SEARCH_GREEDY = 1;

static const int CRIT_COCURV = 0;
static const int CRIT_YUEN = 1;
static const int CRIT_ELL = 2;

// debug thingy: peek at a specific group
unsigned peek = UNDEF_ID;

VoteImage *FormConvexArcGroups::vote_img = 0;

static int CmpConvexArcs(const void *a, const void *b)
{
  if(ConvexArcGroups(*(unsigned*)a)->sig > ConvexArcGroups(*(unsigned*)b)->sig)
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

static int CmpFcnLinkSig(const void *a, const void *b)
{
  if( ((ArcLink*)a)->sig > ((ArcLink*)b)->sig )
    return -1;  // a is first
  else
    return 1;   // b is first
}

/**
 * Calculate significance between links i and j
 */
static double CalcSignificance(unsigned i, unsigned j)
{
  double r, theta, d_phi;
  double n = 2.*(double)NumArcs(), t;
  // make sure i is the link of the longer arc
  if(Arcs(i/2)->ArcLength() < Arcs(j/2)->ArcLength())
    swap(i, j);
  Vector2 v = Arcs(j/2)->point[j%2] - Arcs(i/2)->point[i%2];
  Vector2 ti =
    Normalise((Arcs(i/2)->point[i%2] - Arcs(i/2)->center).NormalClockwise());
  Vector2 tj =
    Normalise((Arcs(j/2)->point[j%2] - Arcs(j/2)->center).NormalClockwise());
  r = v.Length();
  if(r > 0.)
  {
    v /= r;
    t = Cross(ti, v);
    t = fmax(t, -1.);
    t = fmin(t, 1.);
    theta = fabs(asin(t));
  }
  else
  {
    theta = M_PI_2;
    // avoid r == 0
    r = 1.;
  }
  // to avoid dphi = 0 and thus acc = 0, assume a minium of 2°
  d_phi = fmax(fabs(acos(Dot(ti, tj))), 0.03);
  // proximity factor
  double f_prox = theta*r*r/(double)VisionCore::ImageArea();
  // orientation factor
  double f_ori = d_phi/M_PI;
  double acc = 1. - exp(-n*f_prox*f_ori);
  double sig = -log(acc);
  return sig;

  /*
  // TODO: this seems to be a better sig than the nice Poisson probability above
  // the probability is too often = 0.0000000 for any larger gap
  Vector2 v = Arcs(j/2)->point[j%2] - Arcs(i/2)->point[i%2];
  double r = max(1., v.Length());
  double sig = 1/r;
  return sig;*/

  /* old: i is the longer arc
  double s, g, l;
  Vector2 v = Arcs(j/2)->point[j%2] - Arcs(i/2)->point[i%2];
  s = fabs(Cross(v, ti));
  g = fabs(Dot(v, ti));
  l = Arcs(i/2)->ArcLength();
  double theta = acos(Dot(ti, tj));
  double sig = -log(theta*s*g/pow(l, 2.));*/
}

FormConvexArcGroups::FormConvexArcGroups(Config *cfg)
: GestaltPrinciple(cfg)
{
  // COCURV is the default of: COCURV, YUEN, ELL
  config->AddItem("CONVEXARCS_STRONG_CONVEXITY", "1");
  // GREEDY is the default of: EXHAUSTIVE, GREEDY
  config->AddItem("CONVEXARCS_SEARCH_METHOD", "GREEDY");
  // COCURV is the default of: COCURV, YUEN, ELL
  config->AddItem("CONVEXARCS_GROUP_CRITERION", "COCURV");
  vote_img = 0;
  done = false;
}

FormConvexArcGroups::~FormConvexArcGroups()
{
  delete vote_img;
}

void FormConvexArcGroups::Reset(const Image *img)
{
  done = false;
}

/**
 * We produce convex groups of arcs.
 */
void FormConvexArcGroups::Operate(bool incremental)
{
  StartRunTime();
 
 // note: we only want to run this once for repeated calls to Operate()
  if(!done)
  {
    Arc::STRICT_CONVEXITY = config->GetValueInt("CONVEXARCS_STRONG_CONVEXITY");
    if(config->GetValueString("CONVEXARCS_GROUP_CRITERION") == "COCURV")
      GROUP_CRITERION = CRIT_COCURV;
    else if(config->GetValueString("CONVEXARCS_GROUP_CRITERION") == "YUEN")
      GROUP_CRITERION = CRIT_YUEN;
    else if(config->GetValueString("CONVEXARCS_GROUP_CRITERION") == "ELL")
      GROUP_CRITERION = CRIT_ELL;
    else
    {
      fprintf(stderr, "* convex arcs: unkown criterion: %s\n",
          config->GetValueString("CONVEXARCS_GROUP_CRITERION").c_str());
      return;
    }
    if(config->GetValueString("CONVEXARCS_SEARCH_METHOD") == "EXHAUSTIVE")
      SEARCH_METHOD = SEARCH_EXHAUSTIVE;
    else if(config->GetValueString("CONVEXARCS_SEARCH_METHOD") == "GREEDY")
      SEARCH_METHOD = SEARCH_GREEDY;
    else
    {
      fprintf(stderr, "* convex arcs: unkown search method: %s\n",
          config->GetValueString("CONVEXARCS_SEARCH_METHOD").c_str());
      return;
    }
    Prepare();
    Create();
    //Rank();  // TODO: actually i dont need ranking here
    done = true;
  }
  StopRunTime();
}

/**
 * Allocate vote image.
 */
void FormConvexArcGroups::Prepare()
{
  if(vote_img != 0 && (vote_img->width != VisionCore::ImageWidth() ||
     vote_img->height != VisionCore::ImageHeight()))
  {
    delete vote_img;
    vote_img = 0;
  }
  if(vote_img == 0)
  {
    vote_img = new VoteImage(VisionCore::ImageWidth(),
        VisionCore::ImageHeight());
    assert(vote_img != 0);
  }
  vote_img->Clear();
}

/**
 * TODO: use normal LEFT start, normal RIGHT start (dito end)
 */
void FormConvexArcGroups::DrawSearchLines(unsigned arc)
{
  // some degenerate arcs might have center = point[START] and thus throw
  // an ex in Normalise()
  try
  {
    unsigned sline = arc*8;  // search line base index for this arc
    Vector2 r; // vector of length r pointing inwards from start or end point
    // length of tangent, limited to reasonable size for very flat arcs
    double l = min(Arcs(arc)->Radius(), Arcs(arc)->ArcLength());
    r = Normalise(Arcs(arc)->center - Arcs(arc)->point[START]);
    // tangent
    vote_img->DrawLine(Arcs(arc)->point[START],
        Arcs(arc)->point[START] + l*r.NormalAntiClockwise(),
        /*Gestalt::ARC,*/ sline + VOTE_TS);
    // radii
    vote_img->DrawLine(Arcs(arc)->point[START],
        Arcs(arc)->point[START] + l/2.*r,
        /*Gestalt::ARC,*/ sline + VOTE_NLS);
    vote_img->DrawLine(Arcs(arc)->point[START],
        Arcs(arc)->point[START] - l/2.*r,
        /*Gestalt::ARC,*/ sline + VOTE_NLS);
    r = Normalise(Arcs(arc)->center - Arcs(arc)->point[END]);
    // tangent
    vote_img->DrawLine(Arcs(arc)->point[END],
        Arcs(arc)->point[END] + l*r.NormalClockwise(),
        /*Gestalt::ARC,*/ sline + VOTE_TE);
    // radii
    vote_img->DrawLine(Arcs(arc)->point[END],
        Arcs(arc)->point[END] + l/2.*r,
        /*Gestalt::ARC,*/ sline + VOTE_NLE);
    vote_img->DrawLine(Arcs(arc)->point[END],
        Arcs(arc)->point[END] - l/2.*r,
        /*Gestalt::ARC,*/ sline + VOTE_NLE);
  }
  catch(Except &e)
  {
    printf("* in %s: %s\n", __FUNCTION__, e.what());
    // do nothing, just skip that arc
  }
}

void FormConvexArcGroups::CheckSearchLines(unsigned arc)
{
  // some degenerate arcs might have center = point[START] and thus throw
  // an ex in Normalise()
  try
  {
    unsigned sline = arc*8;  // search line base index for this arc
    Array<VoteImage::Elem> iscts;
    Vector2 r; // vector of length r pointing inwards from start or end point
    // length of tangent, limited to reasonable size for very flat arcs
    double l = min(Arcs(arc)->Radius(), Arcs(arc)->ArcLength());
    r = Normalise(Arcs(arc)->center - Arcs(arc)->point[START]);
    vote_img->CheckLine(Arcs(arc)->point[START],
        Arcs(arc)->point[START] + l*r.NormalAntiClockwise(),
        /*Gestalt::ARC,*/ sline + VOTE_TS, iscts);
    CreateArcJunctions(arc, START, iscts);
    r = Normalise(Arcs(arc)->center - Arcs(arc)->point[END]);
    iscts.Clear();
    vote_img->CheckLine(Arcs(arc)->point[END],
        Arcs(arc)->point[END] + l*r.NormalClockwise(),
        /*Gestalt::ARC,*/ sline + VOTE_TE, iscts);
    CreateArcJunctions(arc, END, iscts);
  }
  catch(Except &e)
  {
    printf("* in %s: %s\n", __FUNCTION__, e.what());
    // do nothing, just skip that arc
  }
}

void FormConvexArcGroups::CreateArcJunctions(unsigned arc, unsigned end,
    Array<VoteImage::Elem> &iscts)
{
  for(unsigned k = 0; k < iscts.Size(); k++)
  {
    unsigned i = arc;
    unsigned j = iscts[k].id/8, vtype_j = iscts[k].id%8;
		// TODO: START/END stuff could probably be done more elegantly
    if(vtype_j == VOTE_TS || vtype_j == VOTE_NLS)
      CreateIntersection(2*i + end, 2*j + START);
    else if(vtype_j == VOTE_TE || vtype_j == VOTE_NLE)
      CreateIntersection(2*i + end, 2*j + END);
  }
}

/**
 * Create an intersection between tangent i and tangent/radius j.
 * But only if there is no intersection between i and j yet.
 * Only intersections between convex arcs are stored.
 */
bool FormConvexArcGroups::CreateIntersection(unsigned i, unsigned j)
{
  // basic "daisy chain" test: connect only START <-> END of two arcs
  if(!DIFF_ODDITY(i, j))
    return false;
  // if tangent i has intersection with tangent j
  for(unsigned k = links[i].Size(); k >= 1; k--)
    if(links[i][k-1].id == j)
      return false;
  // if not convex
  if(!Arcs(i/2)->ConvexWith(Arcs(j/2)))
    return false;
  // now insert one tangent into the intersections list of the other
  double sig = CalcSignificance(i, j);
  links[i].PushBack(ArcLink(j, sig));
  links[j].PushBack(ArcLink(i, sig));
  return true;
}

/**
 * TODO: sometimes it would actually make sense to include lines into groups
 */
void FormConvexArcGroups::Create()
{
  Array<unsigned> arcs(1000), arcs_opt(1000);

  ClearHashTable();
  // each arc has 2 tangents
  links.Resize(2*NumArcs());
  for(unsigned i = 0; i < links.Size(); i++)
    links[i].Clear();
  for(unsigned rank = 0; rank < NumArcs(); rank++)
    DrawSearchLines(RankedGestalts(Gestalt::ARC, rank));
  for(unsigned rank = 0; rank < NumArcs(); rank++)
    CheckSearchLines(RankedGestalts(Gestalt::ARC, rank));
  /* stupid O(n^2) version, for benchmarking
  unsigned i,j,k,l;
  for(i = 0; i < rank_max; i++)
  {
    k = RankedGestalts(Gestalt::ARC, i);
    for(j = i+1; j < rank_max; j++)
    {
      l = RankedGestalts(Gestalt::ARC, j);
      if(Arcs(k)->ConvexWith(Arcs(l)))
      {
        // now insert one tangent into the intersections list of the other
        double sig = CalcSignificance(2*k, 2*l+1);
        links[2*k].PushBack(ArcLink(2*l+1, sig));
        links[2*l+1].PushBack(ArcLink(2*k, sig));
      }
    }
  }*/

  // sort intersections on tangents according to significance
  for(unsigned i = 0; i < links.Size(); i++)
    links[i].Sort(CmpFcnLinkSig);
  // now form groups of neighbouring arcs
  if(SEARCH_METHOD == SEARCH_EXHAUSTIVE)
  {
    for(unsigned rank = 0; rank < NumArcs(); rank++)
      ExhaustiveSearch(RankedGestalts(Gestalt::ARC, rank), arcs, arcs_opt);
  }
  else if(SEARCH_METHOD == SEARCH_GREEDY)
  {
    for(unsigned rank = 0; rank < NumArcs(); rank++)
      GreedySearch(RankedGestalts(Gestalt::ARC, rank), arcs);
  }
}

void FormConvexArcGroups::ExhaustiveSearch(unsigned arc, Array<unsigned> &arcs,
    Array<unsigned> &arcs_opt)
{
  unsigned l = arcs.Size()/2, u = l, hash, size_opt = 1;
  cands.Clear();
  arcs[l] = arc;
  arcs_opt[0] = arc;
  hash = Hash(arcs, l, u);
  NewCand(new ConvexArcGroup(arcs, l, u));
  hashtable[hash] = 1;
  GrowAll(arcs, l, u, arcs_opt, size_opt, Evaluate(arcs, l, u, CRIT_ELL));
  //InstantiateBestCand();
  // TODO: this more efficient version does not work yet
  //NewGestalt(new ConvexArcGroup(arcs_opt, 0, size_opt -1));
}

void FormConvexArcGroups::GreedySearch(unsigned arc, Array<unsigned> &arcs)
{
  double sig;
  unsigned l = arcs.Size()/2, u = l, hash;
  arcs[l] = arc;
  sig = GrowBest(arcs, l, u);
  hash = Hash(arcs, l, u);
  if(hashtable[hash] == 0)
  {
    NewGestalt(new ConvexArcGroup(arcs, l, u, sig));
    hashtable[hash] = 1;
  }
}

void FormConvexArcGroups::Rank()
{
  RankGestalts(Gestalt::CONVEX_ARC_GROUP, CmpConvexArcs);
}

/**
 * Draw the vote  image.
 * detail has no effect. 
 */
void FormConvexArcGroups::Draw(int detail)
{
  // TODO: fill in
}

static bool Convex(Array<unsigned> &arcs, unsigned l, unsigned u, unsigned arc)
{
  for(unsigned i = l; i <= u; i++)
    if(!Arcs(arcs[i])->ConvexWith(Arcs(arc)))
      return false;
  return true;
}

static bool Contains(Array<unsigned> &arcs, unsigned l, unsigned u,
    unsigned arc)
{
  for(unsigned i = l; i <= u; i++)
    if(arcs[i] == arc)
      return true;
  return false;
}

void FormConvexArcGroups::NextEnd(Array<unsigned> &arcs, unsigned l,
   unsigned u, unsigned &end, unsigned &next, int criterion)
{
  unsigned cand[2] = {UNDEF_ID, UNDEF_ID};
  double sig[2] = {-HUGE, -HUGE}, st;
  unsigned i, s = 2*arcs[l], e = 2*arcs[u]+1, t;
  // find best admissible arc at START
  for(i = 0; i < links[s].Size(); i++)
  {
    t = links[s][i].id/2;
    if(!Contains(arcs, l, u, t) && Convex(arcs, l+1, u, t))
    {
      if(criterion == CRIT_COCURV)
      {
        cand[START] = t;
        sig[START] = links[s][i].sig;
        break;  // links are sorted best sig first, so we are done
      }
      else
      {
        arcs[l-1] = t;
        st = Evaluate(arcs, l-1, u, criterion);
        if(st > sig[START])
        {
          sig[START] = st;
          cand[START] = t;
        }
      }
    }
  }
  // find best admissible arc at END
  for(i = 0; i < links[e].Size(); i++)
  {
    t = links[e][i].id/2;
    if(!Contains(arcs, l, u, t) && Convex(arcs, l, u-1, t))
    {
      if(criterion == CRIT_COCURV)
      {
        cand[END] = t;
        sig[END] = links[e][i].sig;
        break;  // links are sorted best sig first, so we are done
      }
      else
      {
        arcs[u+1] = t;
        st = Evaluate(arcs, l, u+1, criterion);
        if(st > sig[END])
        {
          sig[END] = st;
          cand[END] = t;
        }
      }
    }
  }
  if(cand[START] == UNDEF_ID && cand[END] == UNDEF_ID)
  {
    next = UNDEF_ID;
    end = STOP;
  }
  else
  {
    if(sig[START] > sig[END])
    {
      next = cand[START];
      end = START;
    }
    else
    {
      next = cand[END];
      end = END;
    }
  }
}

double FormConvexArcGroups::Evaluate(Array<unsigned> &arcs, unsigned l,
    unsigned u, int criterion)
{
  if(criterion == CRIT_ELL)
  {
    double x, y, a, b, phi, error;
    FitEllipse(arcs, l, u, x, y, a, b, phi);
    return EllipseSupport(arcs, l, u, x, y, a, b, phi, error);
  }
  else // CRIT_YUEN or CRIT_COCURV
    return ArcGroupSignificance(arcs, l, u);
}

/**
 * Grow end with best arc.
 */
double FormConvexArcGroups::GrowBest(Array<unsigned> &arcs, unsigned &l,
    unsigned &u)
{
  double q1;
  unsigned first, last, next, cont;
  first = arcs[l];
  last = arcs[u];
  q1 = Evaluate(arcs, l, u, GROUP_CRITERION);
  NextEnd(arcs, l, u, cont, next, GROUP_CRITERION);
  if(cont != STOP)
  {
    unsigned l_try = l, u_try = u;
    double q2;
    if(cont == START)
    {
      assert(l_try > 0);
      l_try--;
      arcs[l_try] = next;
    }
    else
    {
      assert(u_try < arcs.Size() - 1);
      u_try++;
      arcs[u_try] = next;
    }
    q2 = GrowBest(arcs, l_try, u_try);
    if(q2 >= q1)
    {
      l = l_try;
      u = u_try;
      return q2;
    }
  }
  return q1;
}

void FormConvexArcGroups::GrowAll(Array<unsigned> &arcs, unsigned l, unsigned u,
    Array<unsigned> &arcs_opt, unsigned &size_opt, double q_opt)
{
  unsigned i, s = 2*arcs[l], e = 2*arcs[u]+1, t, hash;
  unsigned MAX_HYP = 1000;
  // add all admissible arcs at START
  for(i = 0; i < min((unsigned)MAX_HYP,links[s].Size()); i++)
  {
    t = links[s][i].id/2;
    if(!Contains(arcs, l, u, t) && Convex(arcs, l+1, u, t))
    {
      assert(l > 0);
      arcs[l-1] = t;
      hash = Hash(arcs, l-1, u);
      if(hashtable[hash] == 0)
      {
        // TODO: this more efficient version does not work yet
        /*q = Evaluate(arcs, l-1, u, CRIT_ELL);
        if(q > q_opt)
        {
          for(j = l-1, k = 0; j <= u; j++, k++)
            arcs_opt[k] = arcs[j];
          size_opt = u - l + 2;
          q_opt = q;
        }*/
        NewCand(new ConvexArcGroup(arcs, l-1, u));
        hashtable[hash] = 1;
        GrowAll(arcs, l-1, u, arcs_opt, size_opt, q_opt);
      }
    }
  }
  // add all admissible arcs at END
  for(i = 0; i < min((unsigned)MAX_HYP,links[e].Size()); i++)
  {
    t = links[e][i].id/2;
    if(!Contains(arcs, l, u, t) && Convex(arcs, l, u-1, t))
    {
      assert(u < arcs.Size() - 1);
      arcs[u+1] = t;
      hash = Hash(arcs, l, u+1);
      if(hashtable[hash] == 0)
      {
        // TODO: this more efficient version does not work yet
        /*q = Evaluate(arcs, l, u+1, CRIT_ELL);
        if(q > q_opt)
        {
          for(j = l, k = 0; j <= u + 1; j++, k++)
            arcs_opt[k] = arcs[j];
          size_opt = u - l + 2;
          q_opt = q;
        }*/
        NewCand(new ConvexArcGroup(arcs, l, u+1));
        hashtable[hash] = 1;
        GrowAll(arcs, l, u+1, arcs_opt, size_opt, q_opt);
      }
    }
  }
}

void FormConvexArcGroups::NewCand(ConvexArcGroup *g)
{
  //cands.PushBack(g);
  //cands_created++;
  // forget about candidate list and instantiating best candidate, just create
  // all groups
  NewGestalt(g);
}

void FormConvexArcGroups::InstantiateBestCand()
{
  if(cands.Size() > 0)
  {
    ConvexArcGroup *g_opt = cands[0];
    double q, q_opt = Evaluate(g_opt->arcs, 0, g_opt->arcs.Size()-1, CRIT_ELL);
    for(unsigned i = 1; i < cands.Size(); i++)
    {
      q = Evaluate(cands[i]->arcs, 0, cands[i]->arcs.Size()-1, CRIT_ELL);
      if(q > q_opt)
      {
        q_opt = q;
        g_opt = cands[i];
      }
    }
    NewGestalt(g_opt);
    for(unsigned i = 0; i < cands.Size(); i++)
      if(cands[i] != g_opt)
        delete cands[i];
    cands.Clear();
  }
}

unsigned FormConvexArcGroups::Hash(Array<unsigned> &arcs)
{
  if(arcs.Size() > 0)
    return Hash(arcs, 0, arcs.Size()-1);
  else
    return 0;
}

/**
 * Hash function.
 * The standard reference from Knuth's "The Art of Computer Programming",
 * volume 3 "Sorting and Searching", chapter 6.4.
 */
unsigned FormConvexArcGroups::Hash(Array<unsigned> &arcs, unsigned l,
    unsigned u)
{
  // first find one distinct arc in the group: maximum arc id
  unsigned i, i_max = l, max = arcs[l], hash;
  for(i = l+1; i <= u; i++)
    if(arcs[i] > max)
    {
      i_max = i;
      max = arcs[i];
    }
  hash = u - l + 1;
  for(i = i_max; i <= u; i++) 
    hash = ((hash<<5)^(hash>>27))^arcs[i];
  for(i = l; i < i_max; i++) 
    hash = ((hash<<5)^(hash>>27))^arcs[i];
  hash = hash % HASH_SIZE;
  return hash;
}

void FormConvexArcGroups::ClearHashTable()
{
  for(int i = 0; i < HASH_SIZE; i++)
    hashtable[i] = 0;
}

}
