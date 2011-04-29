/**
 * $Id$
 * Michael Zillich
 */

#include <float.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "Segment.hh"
#include "Arc.hh"
#include "FormArcs.hh"
#include "rosin_arcs.cc"

namespace P 
{

static const int FIT_ARCS_SPLIT  = 0;
static const int FIT_ARCS_GROW   = 1;
static const int FIT_ARCS_RANSAC = 2;

const double FormArcs::MAX_CIRCLE_DIST = 1.;  // max dist of edgel from circle
const double FormArcs::RANSAC_DIST = 1.;  // max dist of edgel from circle
const unsigned FormArcs::MIN_LENGTH = 3;  // min number of pixels in arc
const double FormArcs::MIN_RADIUS = 2.;    // min radius
const double FormArcs::MAX_RADIUS = 10000000.;  // max radius

static int CmpArcs(const void *a, const void *b)
{
  if(((Arc*)a)->sig > ((Arc*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

FormArcs::FormArcs()
{
  // RANSAC is the default method of: SPLIT, GROW, RANSAC
  FIT_METHOD = FIT_ARCS_RANSAC;
}

/**
 * We produce arcs from generic segments.
 * TODO: prepare for incremental mode!
 */
void FormArcs::Operate(Array<Segment*> &segments, Array<Arc*> &arcs)
{
  for(unsigned i=0; i<segments.Size(); i++)
  {
    if(FIT_METHOD == FIT_ARCS_SPLIT)
    {
      FitArcsToSegment(segments[i],arcs);
    }
    else if(FIT_METHOD == FIT_ARCS_GROW)
    {
      unsigned cur_seg_pos = 0;
      while(FitArc(segments[i], &cur_seg_pos, arcs));
    }
    else if(FIT_METHOD == FIT_ARCS_RANSAC)
    {
      FitArcRANSAC(segments[i], 0, segments[i]->edgels.Size() - 1, arcs);
    }
  }

}

/**
 * Check whether edgels i+1 to k-1 lie on circle defined by points i, j and k.
 * As a byproduct returns center and radius of circle.
 */
bool FormArcs::EdgelsOnCircle(Array<Edgel> &edgels, unsigned i, unsigned j,
    unsigned k)
{
  try
  {
    Vector2 center = CircleCenter(edgels[i].p, edgels[j].p, edgels[k].p);
    double radius = Distance(center, edgels[j].p);
    for(unsigned l = i+1; l < k; l++)
      if(fabs(radius - Distance(center, edgels[l].p)) > MAX_CIRCLE_DIST)
        return false;
    return true;
  }
  catch(Except &e)
  {
    // note: if we cannot calculate a circle center, this does not mean that
    // we have no arc: the current part might be too short and thus straight
    // to allow calculation of the circle center.
    // -> report true, let's try growing larger and see if we get an arc
    // eventually
    return true;
  }
}

/**
 * Inscribe circular arc into edge segment.
 * A minimal arc is grown until it no longer fits a circle.
 */
bool FormArcs::FitArc(Segment *seg, unsigned *cur_pos, Array<Arc*> &arcs)
{
  Array<Edgel> &edgels =  seg->edgels;
  unsigned i = *cur_pos;
  unsigned k = i + MIN_LENGTH - 1;
  unsigned j = (i+k)/2;
  bool is_arc = false;
  while(k < edgels.Size() && EdgelsOnCircle(edgels, i, j, k))
  {
    is_arc = true;
    k++;
    j = (i+k)/2;
  }
  *cur_pos = k - 1;
  if(is_arc)
  {
    try
    {
      // note: up to the last iteration the arc was ok, so edgels up to k-1
      // are part of the circular arc
      k--;
      j = (i+k)/2;
      // arcs smaller 6 pixels would make problems later when fitting ellipses
      if(k-i+1 >= 6)
      {
        Vector2 center = CircleCenter(edgels[i].p, edgels[j].p, edgels[k].p);
        double radius = Distance(center, edgels[j].p);
        if(radius > MIN_RADIUS && radius < MAX_RADIUS)
          arcs.PushBack(new Arc(seg, i, j, k, center, radius));
      }
    }
    catch(Except &e)
    {
      // circle center could not be found
    }
  }
  return *cur_pos < edgels.Size();
}

/**
 * Returns the number of supporting pixels for circle defined by points i, j, k.
 * As a byproduct returns center and radius of circle.
 */
int FormArcs::Support(Array<Edgel> &edgels, int l, int u, int &i, int &j,
    int &k, Vector2 &center, double &radius)
{
  try
  {
    center = CircleCenter(edgels[i].p, edgels[j].p, edgels[k].p);
    radius = Distance(center, edgels[j].p);
    // all pixels between i and k must lie on arc
    for(int s = i+1; s < k; s++)
      if(fabs(radius - Distance(center, edgels[s].p)) > RANSAC_DIST)
        return 0;
    // extend arc beyond i and k
    for(i--; i >= l; i--)
      if(fabs(radius - Distance(center, edgels[i].p)) > RANSAC_DIST)
        break;
    if(i < l) i = l;
    for(k++; k <= u; k++)
      if(fabs(radius - Distance(center, edgels[k].p)) > RANSAC_DIST)
        break;
    if(k > u) k = u;
    return k - i + 1;
  }
  catch(Except &e)
  {
    // note: if we cannot calculate a circle center, this does not mean that
    // we have no arc: the current part might be too short and thus straight
    // to allow calculation of the circle center.
    return 0;
  }
}

/**
 * TODO: maybe base probability of split point on curvature.
 * TODO: at the end, small pieces remain with < 6 pixels where no fittiing is
 * possible. collect these pixels and attribute to the nearerest fitted arc if
 * this does not reverse the arcs direction.
 */
void FormArcs::FitArcRANSAC(Segment *seg, int l, int u, Array<Arc*> &arcs)
{
  static int trials = 10;
  int t;
  int i, j, k, len = u - l + 1;
  int i_opt = -1, k_opt = -1;
  int sup, sup_max = 0;
  Array<Edgel> &edgels = seg->edgels;
  Vector2 center, center_opt;
  double radius, radius_opt=-HUGE;

  // HACK
  //if(seg == 11)
  //  printf("seg 11\n");
  if(len < 6)
    return;
  for(t = 0; t < trials; t++)
  {
    i = l + RandInt()%len;
    k = l + RandInt()%len;

    if(i > k)
      swap(i, k);
    j = (i+k)/2;
    sup = Support(edgels, l, u, i, j, k, center, radius);

    if(sup > sup_max)
    {
      sup_max = sup;
      i_opt = i;
      k_opt = k;
      center_opt = center;
      radius_opt = radius;
    }
  }
  if(sup_max > 0)
  {
    i = i_opt;
    k = k_opt;
    j = (i+k)/2;
    if(k - i + 1 >= 6 && radius_opt > MIN_RADIUS && radius_opt < MAX_RADIUS)
    {
      if(center_opt!=edgels[i].p && center_opt!=edgels[j].p && center_opt!=edgels[k].p)
      {
        arcs.PushBack(new Arc(seg, i, j, k, center_opt, radius_opt));
        arcs.Last()->support = sup_max;
      }
    }
    if(i > l)
      FitArcRANSAC(seg, l, i-1, arcs);
    if(k < u)
      FitArcRANSAC(seg, k+1, u, arcs);
  }
  else
  {
    // no arc could be found. split segment at random point
    j = l + 1 + RandInt()%(len - 2);
    FitArcRANSAC(seg, l, j, arcs);
    FitArcRANSAC(seg, j+1, u, arcs);
  }
}



}

