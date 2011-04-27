/**
 * $Id: FormClosures.cc,v 1.16 2007/04/14 20:50:59 mxz Exp mxz $
 */

#include <algorithm>
#include <vector>
#include "Vector2.hh"
#include "VisionCore.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "TJunction.hh"
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

static int CmpClosures(const void *a, const void *b)
{
  if( (*(Closure**)a)->sig > (*(Closure**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

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
  bool operator ()(const Line*& a, const Line*& b)
  {
    return costs[a->ID()] > costs[b->ID()];
  }
  bool operator ()(Line*& a, Line*& b)
  {
    return costs[a->ID()] > costs[b->ID()];
  }
};

/**
 * Comparison function for sorting LJunctions of a closure according to openeing
 * angles, smallest to largest.
 */
static int CmpAngles(const void *a, const void *b)
{
  // if a is undefined, move it to end
  if(*(LJunction**)a == 0)
    return 1;   // b is first
  // if b is undefined, move it to end
  else if(*(LJunction**)b == 0)
    return -1;  // a is first
  // both are defined, check their angles, move larger to end
  else if( (*(LJunction**)a)->OpeningAngle() <
           (*(LJunction**)b)->OpeningAngle() )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

/**
 * Initiate a new closure candidate from a junction and its two lines.
 */
FormClosures::ClosCand::ClosCand(Line *line0, Line *line1,
    int open0, int open1, Bending b)
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

Line *FormClosures::ClosCand::FirstLine()
{
  //return LJunctions(jcts.front())->line[LEFT];
  return lines.front();
}

Line *FormClosures::ClosCand::LastLine()
{
  //return LJunctions(jcts.back())->line[RIGHT];
  return lines.back();
}

int FormClosures::ClosCand::FirstOpen()
{
  //return Other(LJunctions(jcts.front())->near_point[LEFT]);
  return open[START];
}

int FormClosures::ClosCand::LastOpen()
{
  //return Other(LJunctions(jcts.back())->near_point[RIGHT]);
  return open[END];
}


/**
 * Check whether adding jct at START/END would lead to a self-intersection.
 * TODO: check
 */
bool FormClosures::ClosCand::Intersects(Line *line)
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

FormClosures::FormClosures(VisionCore *vc)
: GestaltPrinciple(vc)
{
}

void FormClosures::Reset()
{
  for(unsigned i = 0; i < cands.Size(); i++)
    delete cands[i];
  cands.Clear();
}

void FormClosures::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  switch(type)
  {
    case Gestalt::L_JUNCTION:
      HaveNewLJunction(idx);
      break;
    case Gestalt::COLLINEARITY:
      HaveNewCollinearity(idx);
      break;
    case Gestalt::T_JUNCTION:
      HaveNewTJunction(idx);
      break;
    default:
      break;
  }
}

void FormClosures::HaveNewLJunction(unsigned idx)
{
  LJunction *ljct = LJunctions(core, idx);
  core->stats.attempt_path_search++;
  // only attempt shortest path search if the L-jct is not isolated
  if(!IsIsolatedL(ljct))
  {
    ShortestPath(ljct, 0,
                 ljct->line[RIGHT],
                 ljct->line[LEFT],
                 Other(ljct->near_point[RIGHT]),
                 Other(ljct->near_point[LEFT]));
  }
  UpdateLNeighbors(ljct);
}

void FormClosures::HaveNewCollinearity(unsigned idx)
{
  Collinearity *coll = Collinearities(core, idx);
  core->stats.attempt_path_search++;
  // only attempt shortest path search if the collinearity is not isolated
  if(!IsIsolatedC(coll))
  {
    ShortestPath(0, coll,
                 coll->line[0],
                 coll->line[1],
                 Other(coll->near_point[0]),
                 Other(coll->near_point[1]));
  }
  UpdateCNeighbors(coll);
}

void FormClosures::HaveNewTJunction(unsigned idx)
{
  TJunction *tjct = TJunctions(core, idx);
  UpdateTNeighbors(tjct);
}

void FormClosures::UpdateLNeighbors(LJunction *ljct)
{
}

void FormClosures::UpdateCNeighbors(Collinearity *coll)
{
}

void FormClosures::UpdateTNeighbors(TJunction *tjct)
{
}

/**
 * Checks whether the given L-junction is isolated, i.e. can not form a closure.
 * Only if the open ends of LEFT and RIGHT arm have at least one continuation,
 * the junction could possibly form a closure.
 */
bool FormClosures::IsIsolatedL(LJunction *ljct)
{
  Line *left = ljct->line[LEFT];
  Line *right = ljct->line[RIGHT];
  int open_l = Other(ljct->near_point[LEFT]);
  int open_r = Other(ljct->near_point[RIGHT]);
  // number of possible continuing junctions at open end of LEFT line
  int cont_l = left->l_jct[open_l][RIGHT].Size() +
               left->coll[open_l].Size();
  // same for right line
  int cont_r = right->l_jct[open_r][LEFT].Size() +
               right->coll[open_r].Size();
  // if one end has no continuation, this junction is isolated
  return cont_l == 0 || cont_r == 0;
}

/**
 * Checks whether the given Collinearity is isolated, i.e. can not form a
 * closure.  Only if the open ends of first and second line have at least one
 * continuation, the collinearity could possibly form a closure.
 */
bool FormClosures::IsIsolatedC(Collinearity *coll)
{
  Line *a = coll->line[0];
  Line *b = coll->line[1];
  int open_a = Other(coll->near_point[0]);
  int open_b = Other(coll->near_point[1]);
  // number of possible continuing junctions at open end of first line
  int cont_a = a->l_jct[open_a][LEFT].Size() +
               a->l_jct[open_a][RIGHT].Size() +
               a->coll[open_a].Size();
  // same for second line
  int cont_b = b->l_jct[open_b][LEFT].Size() +
               b->l_jct[open_b][RIGHT].Size() +
               b->coll[open_b].Size();
  // if one end has no continuation, this junction is isolated
  return cont_a == 0 || cont_b == 0;
}

/**
 * Return whether lines a and b already are part of the same closure.
 * This can only happen if there is first e.g. a collinearity and then an
 * L-junction created between the same lines.
 */
bool FormClosures::InSameClosure(Line *a, Line *b)
{
  return a->closures.Intersect(b->closures);
}

void FormClosures::NewCandidateL(LJunction *jct)
{
  unsigned new_c = cands.Size();
  cands.PushBack(new ClosCand(
          jct->line[LEFT],
          jct->line[RIGHT],
          Other(jct->near_point[LEFT]),
          Other(jct->near_point[RIGHT]),
          ClosCand::BEND_LEFT));
  is_end_of[jct->line[LEFT]->ID()].push_back(new_c);
  is_end_of[jct->line[RIGHT]->ID()].push_back(new_c);
}

void FormClosures::NewCandidateC(Collinearity *coll)
{
}

#if 0
/**
 * TODO: beautify
 *       check auf uberkreuzung von linien
 */
void FormClosures::ExtendClosuresL(unsigned jct)
{
  for(int side = LEFT; side <= RIGHT; side++)
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

void FormClosures::ExtendClosuresC(unsigned coll)
{
}

void FormClosures::JoinClosuresL(unsigned jct)
{
 
}

void FormClosures::NewClosure(ClosCand *cand)
{
  unsigned new_c = core->NewGestalt(new Closure(core));
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
#endif

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
void FormClosures::ShortestPath(LJunction *ljct, Collinearity *coll,
    Line *s, Line *t, int end_s, int end_t)
{
  unsigned n = NumLines(core);
  unsigned n_visited = 0;
  Line *u = 0;
  bool have_connected_vertices = true;
  CompCostLargerThan comp(dist);
  int nsteps = 0;
  int cycles = 0; // HACK

  ClearPaths(n);
  // Distance from s to s is 0.
  dist[s->ID()] = 0.;
  // Note prev[s] is UNDEF_ID, not t. This serves as stopping criterion when
  // traversing the path backwards.
  prev[s->ID()] = 0;
  // But s does have an open end
  ends[s->ID()] = end_s;
  // and a junction (or collinearity), namely that with t
  jcts[s->ID()] = ljct;
  colls[s->ID()] = coll;
  // iff we have an L-jct, we already know the bending direction
  if(jcts[s->ID()] != 0)
    bend[s->ID()] = LEFT;
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
    if(dist[u->ID()] != COST_INF)
    {
      // If we reached the target and the ends interlock (u's open end along the
      // path is opposite the open end of the original L-jct)
      if(u == t && ends[u->ID()] == Other(end_t))
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
  // HACK: gather search statistics
  core->stats.path_max_visited =
    max(core->stats.path_max_visited, (int)n_visited);
  core->stats.path_avg_visited =
    (double)(core->stats.path_search*core->stats.path_avg_visited +
     (int)n_visited)/(double)(core->stats.path_search + 1);
  core->stats.path_search++;
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
    prev[i] = 0;
    ends[i] = NONE;
    jcts[i] = 0;
    colls[i] = 0;
    bend[i] = NONE;
    // enter each node 0..n-1 into the priority queue
    Q[i] = Lines(core, i);
  }
}

void FormClosures::ClearNode(unsigned i)
{
  dist[i] = COST_INF;
  prev[i] = 0;
  ends[i] = NONE;
  jcts[i] = 0;
  colls[i] = 0;
  bend[i] = NONE;
}

/**
 * Extend path using L-junctions.
 * @param u  last line of path so far
 */
int FormClosures::ExtendLJunctions(Line *u)
{
  int cnt = 0;
  if(bend[u->ID()] == NONE)
  {
    cnt += ExtendLJunctions(u, LEFT);
    cnt += ExtendLJunctions(u, RIGHT);
  }
  else if(bend[u->ID()] == LEFT)
  {
    cnt += ExtendLJunctions(u, LEFT);
  }
  else if(bend[u->ID()] == RIGHT)
  {
    cnt += ExtendLJunctions(u, RIGHT);
  }
  return cnt;
}

int FormClosures::ExtendNonConvexLJunctions(Line *u)
{
  int cnt = 0;
  if(bend[u->ID()] == LEFT)
    cnt += ExtendLJunctions(u, RIGHT);
  else if(bend[u->ID()] == RIGHT)
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
int FormClosures::ExtendLJunctions(Line *u, int side)
{
  int cnt = 0;
  int other = Other(side);
  // note: If side is LEFT:
  // s is the RIGHT arm of an L-jct. So for a convex continuation s
  // must be the LEFT arm of the next L-jct. Same for all subsequent u's.
  for(unsigned i = 0; i < u->l_jct[ends[u->ID()]][side].Size(); i++)
  {
    LJunction *j = u->l_jct[ends[u->ID()]][side][i];
    Line *v = j->line[other];
    // if the extended path to v would be consistent and shorter than the path
    // to v so far
    if(!LineIntersectingPath(u, j, 0) &&
       dist[u->ID()] + j->acc < dist[v->ID()])
       //dist[u] + 1. < dist[v])
       //dist[u] + SIG_OFFSET - LJunctions(j)->sig < dist[v])
       //dist[u] + 1./LJunctions(j)->sig < dist[v])
    {
      dist[v->ID()] = dist[u->ID()] + j->acc; 
      //dist[v] = dist[u] + 1.; 
      //dist[v] = dist[u] + SIG_OFFSET - LJunctions(j)->sig; 
      //dist[v] = dist[u] + 1./LJunctions(j)->sig; 
      prev[v->ID()] = u;
      ends[v->ID()] = Other(j->near_point[other]);
      jcts[v->ID()] = j;
      colls[v->ID()] = 0;
      // If side is LEFT (u is the LEFT arm of an L-jct), then v is the RIGHT,
      // and we have a LEFT bending curve. RIGHT accordingly.
      bend[v->ID()] = side;
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
int FormClosures::ExtendCollinearities(Line *u)
{
  int cnt = 0;
  for(unsigned i = 0; i < u->coll[ends[u->ID()]].Size(); i++)
  {
    Collinearity *c = u->coll[ends[u->ID()]][i];
    Line *v = c->OtherLine(u);
    // if the extended path to v would be consistent and shorter than the path
    // to v so far
    if(!LineIntersectingPath(u, 0, c) &&
       dist[u->ID()] + c->acc < dist[v->ID()])
       //dist[u] + 1. < dist[v])
       //dist[u] + SIG_OFFSET - Collinearities(c)->sig < dist[v])
       //dist[u] + 1./Collinearities(c)->sig < dist[v])
    {
      dist[v->ID()] = dist[u->ID()] + c->acc; 
      //dist[v] = dist[u] + 1.; 
      //dist[v] = dist[u] + SIG_OFFSET - Collinearities(c)->sig; 
      //dist[v] = dist[u] + 1./Collinearities(c)->sig; 
      prev[v->ID()] = u;
      ends[v->ID()] = Other(c->WhichEndIs(v));
      colls[v->ID()] = c;
      jcts[v->ID()] = 0;
      // collinearities don't change the bending direction
      bend[v->ID()] = bend[u->ID()];
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
void FormClosures::NewClosure(Line *first, Line *last)
{
  Line *u = last;
  int b = bend[u->ID()];
  Closure *new_c = new Closure(core);

  // At this point jct i is the jct between line i and prev[i] (same with coll).
  // Put lines (and jcts) into closure in reverse order (last in path is first
  // in list).
  // So after that jct i is the jct between line i-1 and line i.
  while(u != 0)
  {
    new_c->lines.PushBack(u);
    new_c->jcts.PushBack(jcts[u->ID()]);
    new_c->colls.PushBack(colls[u->ID()]);
    u->closures.PushBack(new_c);
    u = prev[u->ID()];
  }
  // if bend is left, we have to reverse the list to put lines into counter
  // clockwise order.
  if(b == LEFT)
  {
    new_c->lines.Reverse();
    new_c->jcts.Reverse();
    new_c->colls.Reverse();
  }
  // if bend is right, the above reverse order already put the lines in
  // counter clockwise order, but we have to shift junctions one forward
  else
  {
    LJunction *l_last = new_c->jcts.Last();
    Collinearity *c_last = new_c->colls.Last();
    for(unsigned i = new_c->lines.Size() - 1; i > 0; i--)
    {
      new_c->jcts[i] = new_c->jcts[i-1];
      new_c->colls[i] = new_c->colls[i-1];
    }
    new_c->jcts[0] = l_last;
    new_c->colls[0] = c_last;
  }
  // lines are now in counter-clockwise order, now find their senses
  // note:  jct i is the jct between line i and prev[i]
  new_c->senses.Resize(new_c->lines.Size());
  for(unsigned i = 0; i < new_c->lines.Size(); i++)
  {
    if(new_c->jcts[i] != 0)
    {
      // line is the RIGHT end of the L-jct
      if(new_c->jcts[i]->near_point[RIGHT] == START)
        new_c->senses[i] = SAME;
      else
        new_c->senses[i] = OPP;
    }
    else
    {
      if(new_c->colls[i]->WhichEndIs( new_c->lines[i]) == START)
        new_c->senses[i] = SAME;
      else
        new_c->senses[i] = OPP;
    }
  }
  new_c->CalculateSignificance();

  // TODO: does this really belong here...?
  if(core->IsEnabledGestaltPrinciple(GestaltPrinciple::FORM_RECTANGLES))
    CreateNAngles(new_c);

  // TODO: does this really belong here...?
  //if(core->IsEnabledGestaltPrinciple(GestaltPrinciple::FORM_SURFACES))
  //  SolveHCF();
  // HACK: collect statistics
  core->stats.avg_clos_sig =
    (core->stats.new_clos*core->stats.avg_clos_sig +
     new_c->sig)/(double)(core->stats.new_clos + 1);
  core->stats.new_clos++;
  // HACK END

  core->NewGestalt(new_c);
  // now re-rank all closure
  // TODO: this has O(n^2 log n) complexity, which is bad ... However typically
  // the number of closures tends to be small. So we are fine for a while.
  RankGestalts(Gestalt::CLOSURE, CmpClosures);
  Mask();
}

/**
 * HACK: this is still a bit hacky, using thresholds etc.
 * TODO: for now we only check for quadrilaterals
 * TODO: use different criterion, enable jumping over e.g. two small 45deg
 * angles to get one 90deg angle. how much of the closure is still covered when
 * taking out such small lines in order to get rectangle?
 */
void FormClosures::CreateNAngles(Closure *clos)
{
  // if closure has 4 L-junctions, it definitely is a quadrilateral
  if(clos->NumLJunctions() == 4)
  {
    LJunction *ljcts[4];
    for(unsigned i = 0, j = 0; i < clos->jcts.Size(); i++)
      if(clos->jcts[i] != 0)
        ljcts[j++] = clos->jcts[i];
    core->NewGestalt(new Rectangle(core, clos, ljcts));
  }
  // else check whether the 4 smallest inner angles sum up to 2pi
  // Note that this only makes sense for more than 4 lines
  else if(clos->NumLines() > 4)
  {
    const double delta = M_PI/8.;  // HACK: ad-hoc threshold
    double sum_angles = 0.;
    Array<LJunction*> ordered_ljcts(clos->jcts);

    ordered_ljcts.Sort(CmpAngles);
    for(unsigned i = 0; i < 4; i++)
      if(ordered_ljcts[i] != 0)
        sum_angles += ordered_ljcts[i]->OpeningAngle();
    if(fabs(2*M_PI - sum_angles) <= delta)
    {
      // note that junctions in ordered_ljcts are not in counter-clockwise order
      // -> go through original junction list and collect those which are the
      // first 4 in ordered_ljcts
      LJunction *ljcts[4];
      for(unsigned i = 0, j = 0; i < clos->jcts.Size() && j < 4; i++)
      {
        // if junction i is among the first 4
        // TODO: note that this is a bit inefficient..
        // TODO: sometimes j can become > 3 (hence the check in for(..)
        if(ordered_ljcts.Find(clos->jcts[i]) < 4)
          ljcts[j++] = clos->jcts[i];
      }
      core->NewGestalt(new Rectangle(core, clos, ljcts));
    }
  }
}

/**
 * Helper function to get the vertex point of whatever there is between two
 * lines.
 * @param j  L-jct between lines or UNDEF_ID
 * @param c  collinearity between lines or UNDEF_ID
 */
inline static Vector2 GetVertex(LJunction *j, Collinearity *c)
{
  if(j != 0 && c != 0)
    throw Except(__HERE__, "need either L-jct or collinearity");
  else if(j != 0)
    return j->isct;
  else if(c != 0)
    return c->vertex;
  else
    throw Except(__HERE__, "need one of L-jct or collinearity");
}

/**
 * Helper function to get the vertex point of whatever there is between line
 * l and its previous line.
 * @param l_idx  line of interest
 * @param jcts  array of L-junctions of lines, containing some UNDEF_IDs
 * @param colls  array of collinearities of lines, containing some UNDEF_IDs
 */
inline static Vector2 GetVertex(Line *line, vector<LJunction*> &jcts,
    vector<Collinearity*> &colls)
{
  return GetVertex(jcts[line->ID()], colls[line->ID()]);
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
bool FormClosures::LineIntersectingPath(Line *l_last, LJunction *j_new,
    Collinearity *c_new)
{
  // note that the first line we have to check is the next to last of the path
  Line *l = prev[l_last->ID()];
  Vector2 a, b, p, q;

  p = GetVertex(l_last, jcts, colls);
  q = GetVertex(j_new, c_new);
  while(l != 0 && prev[l->ID()] != 0)
  {
    try
    {
      a = GetVertex(prev[l->ID()], jcts, colls);
      b = GetVertex(l, jcts, colls);
      if(LinesIntersecting(p, q, a, b))
        return true;
    }
    catch(Except &e)
    {
      // lines do not intersect, carry on
    }
    l = prev[l->ID()];
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
bool FormClosures::ClosingLineIntersectingPath(Line *l_close,
    Line *l_open, LJunction *j_init, Collinearity *c_init)
{
  Line *l = prev[l_close->ID()];
  Vector2 a, b, p, q;

  p = GetVertex(l_close, jcts, colls);
  q = GetVertex(j_init, c_init);
  while(l != 0 && prev[l->ID()] != 0 && prev[l->ID()] != l_open)
  {
    try
    {
      a = GetVertex(prev[l->ID()], jcts, colls);
      b = GetVertex(l, jcts, colls);
      if(LinesIntersecting(p, q, a, b))
        return true;
    }
    catch(Except &e)
    {
      // lines do not intersect, carry on
    }
    l = prev[l->ID()];
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

void FormClosures::Mask()
{
  // array containing for each line the ID of the highest ranking closure which
  // contains that line
  Array<unsigned> owner(NumLines(core));
  owner.Set(UNDEF_ID);
  for(unsigned j = 0; j < NumClosures(core); j++)
  {
    Closure *clos = (Closure*)core->RankedGestalts(Gestalt::CLOSURE, j);
    clos->Mask(UNDEF_ID);
    for(unsigned l = 0; l < clos->lines.Size(); l++)
    {
      // first check if any previous (i.e. stronger) closure uses one of my
      // lines. if yes, let that closure mask me
      if(owner[clos->lines[l]->ID()] != UNDEF_ID)
      {
        // only mask if closures use line in same sense (one closure inside the
        // other), so first get line senses
        Closure *clos_j = Closures(core, owner[clos->lines[l]->ID()]);
        unsigned pos_i = l;
        unsigned pos_j = clos_j->lines.Find(clos->lines[pos_i]);
        bool same_sense = clos->senses[pos_i] == clos_j->senses[pos_j];
        if(same_sense)
          clos->Mask(owner[clos->lines[l]->ID()]);
      }
      // now the line belongs to me (and I might mask even lesser closures)
      owner[clos->lines[l]->ID()] = clos->ID();
    }
  }
}

}

