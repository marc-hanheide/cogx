/**
 * @file FormClosures.hh
 * @author Zillich
 * @date April 2007
 * @version 0.1
 * @brief Form convex polygons from lines and junctions.
 **/

#include <algorithm>
#include <vector>
#include "Vector2.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Closure.hh"
#include "Rectangle.hh"
#include "FormClosures.hh"
#include "HCF.hh"

namespace Z
{
// en/disable printfs
// #define DEBUG

// try to also find non-convex paths
// TODO: this still has bugs
//#define FIND_NON_CONVEX
	
#define COST_INF HUGE

/**
 * Comparison function object for heap.
 * Note that the default STL heap is a max-heap, i.e. the first element is the
 * maximum. The default STL heap uses the less-than operator '<'.
 * By having heap use this LargerThan operator we get a min-heap, which is
 * required by the Dijkstra algorithm.
 */
class CompCostLargerThan
{
public:
  vector<double> &costs;
  CompCostLargerThan(vector<double> &c) : costs(c) {}
  bool operator ()(const unsigned &a, const unsigned &b)
  {
    return costs[a] > costs[b];
  }
};

/**
 * Comparison function for sorting inner angles of a closure, smallerst to
 * largest.
 */
static int CmpAngles(const void *a, const void *b)
{
  // if a is undefined, move it to end
  if(*(unsigned*)a == UNDEF_ID)
    return 1;   // b is first
  // if b is undefined, move it to end
  else if(*(unsigned*)b == UNDEF_ID)
    return -1;  // a is first
  // both are defined, check their angles, move larger to end
  else if( LJunctions(*(unsigned*)a)->OpeningAngle() <
           LJunctions(*(unsigned*)b)->OpeningAngle() )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

/**
 * 																				TODO ARI: never used
 * Check whether lines i and j are already in same closure.
 */
/*static bool InSameClosure(unsigned i, unsigned j)
{
  return Lines(i)->closures.Intersect(Lines(j)->closures);
}
*/

/**
 * Initiate a new closure candidate from a junction and its two lines.
 */
FormClosures::ClosCand::ClosCand(unsigned line0, unsigned line1,
    unsigned open0, unsigned open1, Bending b)
{
  lines.push_back(line0);
  lines.push_back(line1);
  open[START] = open0;
  open[END] = open1; 
  bend = b;
}

/**
 * Extending a given closure candidate.
 * If extending at START, add left line of jct to front.
 * If extending at END, add right line of jct to back.
 */
FormClosures::ClosCand::ClosCand(ClosCand *old)
{
  lines = old->lines;
  open[START] = old->open[START];
  open[END] = old->open[END]; 
  bend = old->bend;
}

unsigned FormClosures::ClosCand::FirstLine()
{
  //return LJunctions(jcts.front())->line[LEFT];
  return lines.front();
}

unsigned FormClosures::ClosCand::LastLine()
{
  //return LJunctions(jcts.back())->line[RIGHT];
  return lines.back();
}

unsigned FormClosures::ClosCand::FirstOpen()
{
  //return Other(LJunctions(jcts.front())->near_point[LEFT]);
  return open[START];
}

unsigned FormClosures::ClosCand::LastOpen()
{
  //return Other(LJunctions(jcts.back())->near_point[RIGHT]);
  return open[END];
}


/**
 * Check whether adding jct at START/END would lead to a self-intersection.
 * TODO: check
 */
bool FormClosures::ClosCand::Intersects(unsigned line)
{
/* HACK: disable check
  if(end == START)
  {
    unsigned left = LJunctions(jct)->line[LEFT];
    for(list<unsigned>::reverse_iterator j = jcts.rbegin(); j != jcts.rend(); ++j)
      if(LJunctions(*j)->line[LEFT] == left)
        return true;
  }
  else  // END
  {
    unsigned right = LJunctions(jct)->line[RIGHT];
    for(list<unsigned>::iterator j = jcts.begin(); j != jcts.end(); ++j)
      if(LJunctions(*j)->line[RIGHT] == right)
        return true;
  }*/
  return false;
}

static int CmpClosures(const void *a, const void *b)
{
  if( Closures(*(unsigned*)a)->sig > Closures(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

FormClosures::FormClosures(Config *cfg)
: GestaltPrinciple(cfg)
{
}

void FormClosures::Rank()
{
  RankGestalts(Gestalt::CLOSURE, CmpClosures);
}

void FormClosures::Reset(const Image *img)
{
  for(unsigned i = 0; i < cands.Size(); i++)
    delete cands[i];
  cands.Clear();
}

void FormClosures::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  StartRunTime();
  switch(type)
  {
    case Gestalt::L_JUNCTION:
      HaveNewLJunction(idx);
      break;
    case Gestalt::COLLINEARITY:
      HaveNewCollinearity(idx);
      break;
//    case Gestalt::T_JUNCTION:													// TODO ARI: never used
//      HaveNewTJunction(idx);
//      break;
    default:
      break;
  }
  StopRunTime();
}

void FormClosures::HaveNewLJunction(unsigned ljct)
{
  VisionCore::stats.attempt_path_search++;
  // only attempt shortest path search if the L-jct is not isolated
  if(!IsIsolatedL(ljct))
  {
    ShortestPath(ljct, UNDEF_ID,
                 LJunctions(ljct)->line[RIGHT],
                 LJunctions(ljct)->line[LEFT],
                 Other(LJunctions(ljct)->near_point[RIGHT]),
                 Other(LJunctions(ljct)->near_point[LEFT]));
  }
//  UpdateLNeighbors(ljct);
}

void FormClosures::HaveNewCollinearity(unsigned coll)
{
  VisionCore::stats.attempt_path_search++;
  // only attempt shortest path search if the collinearity is not isolated
  if(!IsIsolatedC(coll))
  {
    ShortestPath(UNDEF_ID, coll,
                 Collinearities(coll)->line[0],
                 Collinearities(coll)->line[1],
                 Other(Collinearities(coll)->near_point[0]),
                 Other(Collinearities(coll)->near_point[1]));
  }
//  UpdateCNeighbors(coll);
}

void FormClosures::HaveNewTJunction(unsigned tjct)								// TODO ARI: never used
{
//  UpdateTNeighbors(tjct);
}

void FormClosures::UpdateLNeighbors(unsigned ljct)								// TODO ARI: never used
{
}

void FormClosures::UpdateCNeighbors(unsigned coll)								// TODO ARI: never used
{
}

void FormClosures::UpdateTNeighbors(unsigned tjct)								// TODO ARI: never used
{
}

/**
 * Checks whether the given L-junction is isolated, i.e. can not form a closure.
 * Only if the open ends of LEFT and RIGHT arm have at least one continuation,
 * the junction could possibly form a closure.
 */
bool FormClosures::IsIsolatedL(unsigned ljct)
{
  unsigned left = LJunctions(ljct)->line[LEFT];
  unsigned right = LJunctions(ljct)->line[RIGHT];
  int open_l = Other(LJunctions(ljct)->near_point[LEFT]);
  int open_r = Other(LJunctions(ljct)->near_point[RIGHT]);
  // number of possible continuing junctions at open end of LEFT line
  int cont_l = Lines(left)->l_jct[open_l][RIGHT].Size() +
               Lines(left)->coll[open_l].Size();
  // same for right line
  int cont_r = Lines(right)->l_jct[open_r][LEFT].Size() +
               Lines(right)->coll[open_r].Size();
  // if one end has no continuation, this junction is isolated
  return cont_l == 0 || cont_r == 0;
}

/**
 * Checks whether the given Collinearity is isolated, i.e. can not form a
 * closure.  Only if the open ends of first and second line have at least one
 * continuation, the collinearity could possibly form a closure.
 */
bool FormClosures::IsIsolatedC(unsigned coll)
{
  unsigned a = Collinearities(coll)->line[0];
  unsigned b = Collinearities(coll)->line[1];
  int open_a = Other(Collinearities(coll)->near_point[0]);
  int open_b = Other(Collinearities(coll)->near_point[1]);
  // number of possible continuing junctions at open end of first line
  int cont_a = Lines(a)->l_jct[open_a][LEFT].Size() +
               Lines(a)->l_jct[open_a][RIGHT].Size() +								
               Lines(a)->coll[open_a].Size();
  // same for second line
  int cont_b = Lines(b)->l_jct[open_b][LEFT].Size() +
               Lines(b)->l_jct[open_b][RIGHT].Size() +
               Lines(b)->coll[open_b].Size();
  // if one end has no continuation, this junction is isolated
  return cont_a == 0 || cont_b == 0;
}

/**
 * 																				TODO ARI: never used
 * Return whether lines a and b already are part of the same closure.
 * This can only happen if there is first e.g. a collinearity and then an
 * L-junction created between the same lines.
 */
bool FormClosures::InSameClosure(unsigned a, unsigned b)
{
  return Lines(a)->closures.Intersect(Lines(b)->closures);
}

void FormClosures::NewCandidateL(unsigned jct)
{
  unsigned new_c = cands.Size();
  cands.PushBack(new ClosCand(
          LJunctions(jct)->line[LEFT],
          LJunctions(jct)->line[RIGHT],
          Other(LJunctions(jct)->near_point[LEFT]),
          Other(LJunctions(jct)->near_point[RIGHT]),
          ClosCand::BEND_LEFT));
  is_end_of[LJunctions(jct)->line[LEFT]].push_back(new_c);
  is_end_of[LJunctions(jct)->line[RIGHT]].push_back(new_c);
  printf("clos cands: %d\n", cands.Size());
}

void FormClosures::NewCandidateC(unsigned coll)
{
}

/**
 * TODO: beautify
 *       check auf uberkreuzung von linien
 */
void FormClosures::ExtendClosuresL(unsigned jct)
{
  for(unsigned side = LEFT; side <= RIGHT; side++)
  {
    unsigned line = LJunctions(jct)->line[side];
    unsigned other = LJunctions(jct)->line[Other(side)];
    for(list<unsigned>::iterator ended = is_end_of[line].begin();
      ended != is_end_of[line].end(); ++ended)
    {
      ClosCand *cand = cands[*ended];
      if(line == cand->FirstLine() &&
         cand->FirstOpen() == LJunctions(jct)->near_point[side])
         // && !cands[*ended]->Intersects(jct, END)) TODO
      {
        if(cand->bend == ClosCand::BEND_NONE ||
           (side == RIGHT && cand->bend == ClosCand::BEND_LEFT) ||
           (side == LEFT && cand->bend == ClosCand::BEND_RIGHT))
        {
          // if the other line is the other end, we have a complete closure
          if(other == cand->LastLine())
          {
            NewClosure(cand);
          }
          // else just a new candidate
          else
          {
            unsigned new_id = cands.Size();
            ClosCand *new_c = new ClosCand(cand);
            cands.PushBack(new_c);
            new_c->lines.push_front(other);
            new_c->open[START] =
              Other(LJunctions(jct)->near_point[Other(side)]);
            // if has no bending direction yet, new we can set it
            if(new_c->bend == ClosCand::BEND_NONE)
            {
              if(side == RIGHT)
                new_c->bend = ClosCand::BEND_LEFT;
              else // side == LEFT
                new_c->bend = ClosCand::BEND_RIGHT;
            }
            is_end_of[new_c->FirstLine()].push_back(new_id);
            is_end_of[new_c->LastLine()].push_back(new_id);
          }
        }
      }
      else if(line == cand->LastLine() &&
              cand->LastOpen() == LJunctions(jct)->near_point[side])
              // && !cands[*ended]->Intersects(jct, END)) TODO
      {
        if(cand->bend == ClosCand::BEND_NONE ||
           (side == LEFT && cand->bend == ClosCand::BEND_LEFT) ||
           (side == RIGHT && cand->bend == ClosCand::BEND_RIGHT))
        {
          // if the other line is the other end, we have a complete closure
          if(other == cand->FirstLine())
          {
            NewClosure(cand);
          }
          else
          {
            unsigned new_id = cands.Size();
            ClosCand *new_c = new ClosCand(cand);
            cands.PushBack(new_c);
            new_c->lines.push_back(other);
            new_c->open[END] =
              Other(LJunctions(jct)->near_point[Other(side)]);
            // if has no bending direction yet, new we can set it
            if(new_c->bend == ClosCand::BEND_NONE)
            {
              if(side == RIGHT)
                new_c->bend = ClosCand::BEND_RIGHT;
              else // side == LEFT
                new_c->bend = ClosCand::BEND_LEFT;
            }
            is_end_of[new_c->FirstLine()].push_back(new_id);
            is_end_of[new_c->LastLine()].push_back(new_id);
          }
        }
      }
    }
  }
}

void FormClosures::ExtendClosuresC(unsigned coll)								// TODO ARI: never used
{
}

void FormClosures::JoinClosuresL(unsigned jct)									// TODO ARI: never used
{
}

void FormClosures::NewClosure(ClosCand *cand)
{
  unsigned new_c = NewGestalt(new Closure());
  unsigned n = cand->lines.size(), i = 0;
  //Closures(new_c)->jcts.Resize(n);
  Closures(new_c)->lines.Resize(n);
  for(list<unsigned>::iterator j = cand->lines.begin();
      j != cand->lines.end(); ++j)
  {
    Closures(new_c)->lines[i] = *j;
    // make closure known to constituent line
    Lines(*j)->closures.PushBack(new_c);
    i++;
  }

  // if BEND_RIGHT, reverse above insert order
  // if BEND_NONE (e.g. collinearities forming a big circle), find bending via
  // summation if bending angles

  // TODO: remove cand from is_left_of and is_right_of lists
}

/**
 * Dijkstra shortest path algorithm
 * Find a path from right to left line taking only left turns.
 *
 * @param s  start line, which is the RIGHT arm of an L-jct
 * @param t  target line, which is the LEFT arm of that L-jct
 * @param end_s  the open end of line s (opposite to the joined end)
 * @param end_t  the open end of line t (opposite to the joined end)
 *
 * TODO: - book_s1.png: right rectangle is not found (jct 411)
 *       - find a better distance function, properly taking into account sig.
 *         e.g. accumulated (relative) gap size
 */
void FormClosures::ShortestPath(unsigned ljct, unsigned coll,
    unsigned s, unsigned t, int end_s, int end_t)
{
  unsigned n = NumLines();
  unsigned n_visited = 0;
  unsigned u = UNDEF_ID;
  bool have_connected_vertices = true;
  CompCostLargerThan comp(dist);
  int nsteps = 0;
  int cycles = 0; // HACK

  ClearPaths(n);
  // Distance from s to s is 0.
  dist[s] = 0.;
  // Note prev[s] is UNDEF_ID, not t. This serves as stopping criterion when
  // traversing the path backwards.
  prev[s] = UNDEF_ID;
  // But s does have an open end
  ends[s] = end_s;
  // and a junction (or collinearity), namely that with t
  jcts[s] = ljct;
  colls[s] = coll;
  // iff we have an L-jct, we already know the bending direction
  if(jcts[s] != UNDEF_ID)
    bend[s] = LEFT;
  // prepare min-heap (priority queue)
  make_heap(Q.begin(), Q.end(), comp);
  while(n_visited < n && have_connected_vertices)
  {
    // Remove best vertex from priority queue
    u = Q.front();
    // TODO: keep whole heap??
    pop_heap(Q.begin(), Q.end() - n_visited, comp);
    n_visited++;
    // If the best vertex has infinite distance (i.e. there is no path to it)
    // then there is no point in continuing
    if(dist[u] != COST_INF)
    {
      // If we reached the target and the ends interlock (u's open end along the
      // path is opposite the open end of the original L-jct)
      if(u == t && ends[u] == Other(end_t))
      {
        if(!ClosingLineIntersectingPath(u, s, ljct, coll))
        {
          NewClosure(s, t);
          /* find more than just one shortest path: TODO: think properly
          unsigned l = t;
          while(l != s)
          {
            dist[l] = COST_INF;
            l = prev[l];
          }*/
          cycles++;  // HACK
        }
      }
      else
      {
        int cnt = 0;
        // For each edge (u,v) outgoing from u and not intersecting the path so
        // far
        cnt += ExtendLJunctions(u);
        cnt += ExtendCollinearities(u);
#ifdef FIND_NON_CONVEX
        if(cnt == 0)
          cnt += ExtendNonConvexLJunctions(u);
#endif
        // TODO: take whole heap??
        if(cnt > 0)
          make_heap(Q.begin(), Q.end() - n_visited, comp);
      }
    }
    else
      have_connected_vertices = false;
    nsteps++;
  }
  
  // HACK: gather search statistics												// TODO ARI: notwendig?
  VisionCore::stats.path_max_visited =
    max(VisionCore::stats.path_max_visited, (int)n_visited);
  VisionCore::stats.path_avg_visited =
    (double)(VisionCore::stats.path_search*VisionCore::stats.path_avg_visited +
     (int)n_visited)/(double)(VisionCore::stats.path_search + 1);
  VisionCore::stats.path_search++;
  // HACK END
  
  // HACK: only clear used nodes
  //for(unsigned i = Q.size() - n_visited; i < Q.size(); i++)
  //  ClearNode(Q[i]);
  // HACK END
  if(cycles > 1)
    printf("%d cycles\n", cycles);  // HACK
}

/**
 * Clear all temporary path stuff for new search.
 * @param n  number of nodes in the search graph (= number of lines)
 */
void FormClosures::ClearPaths(unsigned n)
{
  //int old_n = Q.size();  // HACK: only clear used nodes

  dist.resize(n);
  // TODO: use fill(dist.begin(), dist.end(), COST_INF);
  prev.resize(n);
  ends.resize(n);
  jcts.resize(n);
  colls.resize(n);
  bend.resize(n);
  Q.resize(n);
  //for(unsigned i = old_n; i < n; i++)  // HACK: clear only used nodes
  for(unsigned i = 0; i < n; i++)
  {
    dist[i] = COST_INF;
    prev[i] = UNDEF_ID;
    ends[i] = NONE;
    jcts[i] = UNDEF_ID;
    colls[i] = UNDEF_ID;
    bend[i] = NONE;
    // enter each node 0..n-1 into the priority queue
    Q[i] = i;
  }
}

void FormClosures::ClearNode(unsigned i)
{
  dist[i] = COST_INF;
  prev[i] = UNDEF_ID;
  ends[i] = NONE;
  jcts[i] = UNDEF_ID;
  colls[i] = UNDEF_ID;
  bend[i] = NONE;
}

/**
 * Extend path using L-junctions.
 * @param u  last line of path so far
 */
int FormClosures::ExtendLJunctions(unsigned u)
{
  int cnt = 0;
  if(bend[u] == NONE)
  {
    cnt += ExtendLJunctions(u, LEFT);
    cnt += ExtendLJunctions(u, RIGHT);
  }
  else if(bend[u] == LEFT)
  {
    cnt += ExtendLJunctions(u, LEFT);
  }
  else if(bend[u] == RIGHT)
  {
    cnt += ExtendLJunctions(u, RIGHT);
  }
  return cnt;
}

int FormClosures::ExtendNonConvexLJunctions(unsigned u)
{
  int cnt = 0;
  if(bend[u] == LEFT)
    cnt += ExtendLJunctions(u, RIGHT);
  else if(bend[u] == RIGHT)
    cnt += ExtendLJunctions(u, LEFT);
  return cnt;
}

/**
 * Extend path using L-junctions.
 * TODO: decide for a proper cost function
 * @param u  last line of path so far
 * @param side  extend LEFT or RIGHT L-junctions at the end of u
 * Returns the number of extensions.
 */
int FormClosures::ExtendLJunctions(unsigned u, int side)
{
  int cnt = 0;
  int other = Other(side);
  // note: If side is LEFT:
  // s is the RIGHT arm of an L-jct. So for a convex continuation s
  // must be the LEFT arm of the next L-jct. Same for all subsequent u's.
  for(unsigned i = 0; i < Lines(u)->l_jct[ends[u]][side].Size(); i++)
  {
    unsigned j = Lines(u)->l_jct[ends[u]][side][i];
    unsigned v = LJunctions(j)->line[other];
    // if the extended path to v would be consistent and shorter than the path
    // to v so far
    if(!LineIntersectingPath(u, j, UNDEF_ID) &&
       dist[u] + LJunctions(j)->acc < dist[v])
       //dist[u] + 1. < dist[v])
       //dist[u] + SIG_OFFSET - LJunctions(j)->sig < dist[v])
       //dist[u] + 1./LJunctions(j)->sig < dist[v])
    {
      dist[v] = dist[u] + LJunctions(j)->acc; 
      //dist[v] = dist[u] + 1.; 
      //dist[v] = dist[u] + SIG_OFFSET - LJunctions(j)->sig; 
      //dist[v] = dist[u] + 1./LJunctions(j)->sig; 
      prev[v] = u;
      ends[v] = Other(LJunctions(j)->near_point[other]);
      jcts[v] = j;
      colls[v] = UNDEF_ID;
      // If side is LEFT (u is the LEFT arm of an L-jct), then v is the RIGHT,
      // and we have a LEFT bending curve. RIGHT accordingly.
      bend[v] = side;
      cnt++;
    }
  }
  return cnt;
}

/**
 * Extend path using collinearities.
 * @param u  last line of path so far
 * Returns the number of extensions.
 */
int FormClosures::ExtendCollinearities(unsigned u)
{
  int cnt = 0;
  for(unsigned i = 0; i < Lines(u)->coll[ends[u]].Size(); i++)
  {
    unsigned c = Lines(u)->coll[ends[u]][i];
    unsigned v = Collinearities(c)->OtherLine(u);
    // if the extended path to v would be consistent and shorter than the path
    // to v so far
    if(!LineIntersectingPath(u, UNDEF_ID, c) &&
       dist[u] + Collinearities(c)->acc < dist[v])
       //dist[u] + 1. < dist[v])
       //dist[u] + SIG_OFFSET - Collinearities(c)->sig < dist[v])
       //dist[u] + 1./Collinearities(c)->sig < dist[v])
    {
      dist[v] = dist[u] + Collinearities(c)->acc; 
      //dist[v] = dist[u] + 1.; 
      //dist[v] = dist[u] + SIG_OFFSET - Collinearities(c)->sig; 
      //dist[v] = dist[u] + 1./Collinearities(c)->sig; 
      prev[v] = u;
      ends[v] = Other(Collinearities(c)->WhichEndIs(v));
      colls[v] = c;
      jcts[v] = UNDEF_ID;
      // collinearities don't change the bending direction
      bend[v] = bend[u];
      cnt++;
    }
  }
  return cnt;
}

/**
 * Form a new closure along path from first to last line.
 * Note: path formation made sure that the resulting polygon in non-intersecting
 * and (mostly) convex.
 * TODO: use a proper constructor
 */
void FormClosures::NewClosure(unsigned first, unsigned last)
{
  Array<unsigned> l;	// lines
  Array<unsigned> lj;	// l-junctions
  Array<unsigned> c;	// collinearities
  Array<unsigned> s;	// sense

  unsigned u = last;
  int b = bend[u];

  // put lines (and jcts) into closure in reverse order (last in path is first
  // in list)
  while(u != UNDEF_ID)
  {
    l.PushBack(u);
    lj.PushBack(jcts[u]);
    c.PushBack(colls[u]);
    u = prev[u];
  }	
  // if bend is left, we have to reverse the list to put lines into counter
  // clockwise order.
  if(b == LEFT)
  {
    l.Reverse();
    lj.Reverse();
    c.Reverse();
  }
  // if bend is right, the above reverse order already put the lines in
  // counter clockwise orderi, but we have to shift junctions one forward
  else
  {
    unsigned l_last = lj.Last();
    unsigned c_last = c.Last();
    for(unsigned i = l.Size() - 1; i > 0; i--)
    {
      lj[i] = lj[i-1];
      c[i] = c[i-1];
    }
    lj[0] = l_last;
    c[0] = c_last;
  }
  // lines are now in counter-clockwise order, now find their senses
  // note:  jct i is the jct between line i and prev[i]
  s.Resize(l.Size());
  for(unsigned i = 0; i < l.Size(); i++)
  {
    if(lj[i] != UNDEF_ID)
    {
      // line is the RIGHT end of the L-jct
      if(LJunctions(lj[i])->near_point[RIGHT] == START)
        s[i] = SAME;
      else
        s[i] = OPP;
    }
    else
    {
      if(Collinearities(c[i])->WhichEndIs(l[i]) == START)
        s[i] = SAME;
      else
        s[i] = OPP;
    }
  }
  
  // make new closure 
  NewGestalt(new Closure(l, lj, c, s));
  // ARI: Ranking closures
  //Rank();
	
	

  // TODO: does this really belong here...?
//  if(VisionCore::IsEnabledGestaltPrinciple(GestaltPrinciple::FORM_RECTANGLES))
//    CreateNAngles(new_c); 
  // TODO: does this really belong here...?
  //if(VisionCore::IsEnabledGestaltPrinciple(GestaltPrinciple::FORM_SURFACES))
  //  SolveHCF();
  // HACK: collect statistics
//  VisionCore::stats.avg_clos_sig =
//   (VisionCore::stats.new_clos*VisionCore::stats.avg_clos_sig +
//     Closures(new_c)->sig)/(double)(VisionCore::stats.new_clos + 1);
//  VisionCore::stats.new_clos++;
  // HACK END
}


/**
 * HACK: this is still a bit hacky, using thresholds etc.
 * TODO: for now we only check for quadrilaterals
 * TODO: use different criterion, enable jumping over e.g. two small 45deg
 * angles to get one 90deg angle. how much of the closure is still covered when
 * taking out such small lines in order to get rectangle?
 */
/*
void FormClosures::CreateNAngles(unsigned clos)									// TODO ARI: unused
{
  // if closure has 4 L-junctions, it definitely is a quadrilateral
  if(Closures(clos)->NumLJunctions() == 4)
  {
    unsigned ljcts[4];
    for(unsigned i = 0, j = 0; i < Closures(clos)->jcts.Size(); i++)
      if(Closures(clos)->jcts[i] != UNDEF_ID)
        ljcts[j++] = Closures(clos)->jcts[i];
    NewGestalt(new Rectangle(clos, ljcts));
  }
  // else check whether the 4 smallest inner angles sum up to 2pi
  // Note that this only makes sense for more than 4 lines
  else if(Closures(clos)->NumLines() > 4)
  {
    const double delta = M_PI/8.;  // HACK: ad-hoc threshold
    double sum_angles = 0.;
    Array<unsigned> ordered_ljcts(Closures(clos)->jcts);

    ordered_ljcts.Sort(CmpAngles);
    for(unsigned i = 0; i < 4; i++)
      if(ordered_ljcts[i] != UNDEF_ID)
        sum_angles += LJunctions(ordered_ljcts[i])->OpeningAngle();
    if(fabs(2*M_PI - sum_angles) <= delta)
    {
      // note that junctions in ordered_ljcts are not in counter-clockwise order
      // -> go through original junction list and collect those which are the
      // first 4 in ordered_ljcts
      unsigned ljcts[4];
      for(unsigned i = 0, j = 0; i < Closures(clos)->jcts.Size() && j < 4; i++)
      {
        // if junction i is among the first 4
        // TODO: note that this is a bit inefficient..
        // TODO: sometimes j can become > 3 (hence the check in for(..)
        if(ordered_ljcts.Find(Closures(clos)->jcts[i]) < 4)
          ljcts[j++] = Closures(clos)->jcts[i];
      }
      NewGestalt(new Rectangle(clos, ljcts));
    }
  }
}
*/

/**
 * Helper function to get the vertex point of whatever there is between two
 * lines.
 * @param j  L-jct between lines or UNDEF_ID
 * @param c  collinearity between lines or UNDEF_ID
 */
inline static Vector2 GetVertex(unsigned j, unsigned c)
{
  if(j != UNDEF_ID && c != UNDEF_ID)
    throw Except(__HERE__, "need either L-jct or collinearity");
  else if(j != UNDEF_ID)
    return LJunctions(j)->isct;
  else if(c != UNDEF_ID)
    return Collinearities(c)->vertex;
  else
    throw Except(__HERE__, "need one of L-jct or collinearity");
}

/**
 * Helper function to get the vertex point of whatever there is between line
 * l and its previous line.
 * @param l  line of interest
 * @param jcts  array of L-junctions of lines, containing some UNDEF_IDs
 * @param colls  array of collinearities of lines, containing some UNDEF_IDs
 */
inline static Vector2 GetVertex(unsigned l, vector<unsigned> jcts,
    vector<unsigned> colls)
{
  return GetVertex(jcts[l], colls[l]);
}

/**
 * Check whether a new line with its new L-jct or collinearity intersects the
 * path so far.
 * @param l_last  new line
 * @param j_new  the L-jct which connects l_last to the path
 * @param c_new  the coll. which connects l_last to the path
 *               (one and only one of j_new, c_new must be != UNDEF)
 * The last line must not intersect geometrically with the previous lines in the
 * path.
 */
bool FormClosures::LineIntersectingPath(unsigned l_last, unsigned j_new,
    unsigned c_new)
{
  // note that the first line we have to check is the next to last of the path
  unsigned l = prev[l_last];
  Vector2 a, b, p, q;

  p = GetVertex(l_last, jcts, colls);
  q = GetVertex(j_new, c_new);
  while(l != UNDEF_ID && prev[l] != UNDEF_ID)
  {
    try
    {
      a = GetVertex(prev[l], jcts, colls);
      b = GetVertex(l, jcts, colls);
      if(LinesIntersecting(p, q, a, b))
        return true;
    }
    catch(Except &e)
    {
      // lines do not intersect, carry on
    }
    l = prev[l];
  }
  return false;
}

/**
 * Check whether the closing line of a possible closure intersects the
 * path so far.
 * @param l_close  the closing line = last on path
 * @param l_open  the opening line = first on path
 * @param j_init  the initialising L-jct between l_open and l_close, or UNDEF
 * @param c_init  the initialising coll. between l_open and l_close, or UNDEF
 * Note that one of j_init, c_init must be defined.
 * This function differs from LineIntersectingPath() by the fact that we do not
 * check against the opening line (which by definition must intersect with the
 * closing line).
 */
bool FormClosures::ClosingLineIntersectingPath(unsigned l_close,
    unsigned l_open, unsigned j_init, unsigned c_init)
{
  unsigned l = prev[l_close];
  Vector2 a, b, p, q;

  p = GetVertex(l_close, jcts, colls);
  q = GetVertex(j_init, c_init);
  while(l != UNDEF_ID && prev[l] != UNDEF_ID && prev[l] != l_open)
  {
    try
    {
      a = GetVertex(prev[l], jcts, colls);
      b = GetVertex(l, jcts, colls);
      if(LinesIntersecting(p, q, a, b))
        return true;
    }
    catch(Except &e)
    {
      // lines do not intersect, carry on
    }
    l = prev[l];
  }
  return false;
}

/* TODO: check complete polygon for simple
bool FormClosures::SimplePolygon(unsigned l_close,
    unsigned l_open, unsigned j_init, unsigned c_init)
{
  if(!ClosingLineIntersectingPath(l_close, l_open, j_init, c_init))
  {
  }
}
*/

}
