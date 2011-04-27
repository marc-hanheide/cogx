/**
 * $Id: ConvexArcGroup.cc,v 1.18 2006/11/24 13:47:03 mxz Exp mxz $
 */

#include "Draw.hh"
#include "Arc.hh"
#include "ConvexArcGroup.hh"

namespace Z
{

static bool Inside(Array<unsigned> &arcs, unsigned l, unsigned u, Vector2 &p)
{
  for(unsigned i = l; i <= u; i++)
    if(!Arcs(arcs[i])->HasInside(p))
      return false;
  return true;
}

ConvexArcGroup::ConvexArcGroup()
  : Gestalt(CONVEX_ARC_GROUP)
{
}

ConvexArcGroup::ConvexArcGroup(Array<unsigned> &a, unsigned l, unsigned u,
    double s)
  : Gestalt(CONVEX_ARC_GROUP)
{
  if(u >= l)
  {
    arcs.Resize(u - l + 1);
    for(unsigned i = 0; l <= u; i++, l++)
      arcs[i] = a[l];
  }
  else
    throw(Except(__HERE__, "invalid array bounds %u-%u", l, u));
  ang_cover = ArcGroupAngularCoverage(arcs);
  closedness = ArcGroupClosedness(arcs);
  if(s >= 0.)
    sig = s;
  else
    CalculateSignificance();
}

static Vector2 cmp_mean;
static int CmpCenters(const void *a, const void *b)
{
  if(Distance(cmp_mean, *((Vector2*)a)) < Distance(cmp_mean, *((Vector2*)b)))
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

/**
 * Draw the convex arc group.
 * If detail >= 1 also draw stuff of the forming gestalt principle.
 */
void ConvexArcGroup::Draw(int detail)
{
  for(unsigned i = 0; i < arcs.Size(); i++)
    Arcs(arcs[i])->Draw(detail - 1);
  {
    unsigned i, j, k, n = 2*arcs.Size();
    Vector2 p[n], t[n], c[n];
    Vector2 m1, t1, m2, t2;
    Vector2 mean(0., 0.);
    double weight[n], sum_weight = 0., swd = 0., pp = 1.;
    double point_dens = 2.*NumArcs()/(double)VisionCore::ImageArea();
    double r, r_max = 0., alpha;
    double prob, s, s_max = -HUGE;
    for(i = 0; i < arcs.Size(); i++)
    {
      p[2*i] = Arcs(arcs[i])->tang_pt[0];
      t[2*i] = Arcs(arcs[i])->tang[0];
      p[2*i+1] = Arcs(arcs[i])->tang_pt[1];
      t[2*i+1] = Arcs(arcs[i])->tang[1];
    }
    for(i = 0; i < n; i++)
    {
      try
      {
        j = (i + n/3)%n;
        k = (i + (2*n)/3)%n;
        weight[i] = 1.;
        m1 = MidPoint(p[i], p[j]);
        t1 = LineIntersection(p[i], t[i], p[j], t[j]);
        weight[i] *= fabs(Cross(t[i], t[j]));
        //DrawLine2D(p[i].x, p[i].y, p[j].x, p[j].y, RGBColor::white);
        //DrawLine2D(m1.x, m1.y, t1.x, t1.y, RGBColor::yellow);
        t1 = Normalise(t1 - m1);  // now is direction vector
        m2 = MidPoint(p[j], p[k]);
        t2 = LineIntersection(p[j], t[j], p[k], t[k]);
        weight[i] *= fabs(Cross(t[j], t[k]));
        //DrawLine2D(p[j].x, p[j].y, p[k].x, p[k].y, RGBColor::white);
        //DrawLine2D(m2.x, m2.y, t2.x, t2.y, RGBColor::yellow);
        t2 = Normalise(t2 - m2);
        c[i] = LineIntersection(m1, t1, m2, t2);
        weight[i] *= fabs(Cross(t1, t2));
        DrawPoint2D(c[i].x, c[i].y, RGBColor::magenta);
        mean += c[i]*weight[i];
        sum_weight += weight[i];
      }
      catch(Except &e)
      {
      }
    }
    if(sum_weight > 0.)
    {
      mean /= sum_weight;
      for(i = 0; i < n; i++)
        swd += Distance(mean, c[i])*weight[i];
      swd /= sum_weight;
      for(i = 0; i < n; i++)
        pp *= (1. - exp(-CircleArea(fmax(Distance(mean, c[i]),1.)*point_dens)));
      // proper Poisson formalism
      // sort points according to distance
      cmp_mean = mean;
      qsort(c, n, sizeof(Vector2), CmpCenters);
      for(i = 0; i < n; i++)
      {
        r = fmax(Distance(mean, c[i]), 1.);
        alpha = CircleArea(r)*point_dens;
        // TODO: shouln't it be PoissonCDF(i-1, alpha)?
        prob = PoissonCDF(i, alpha);
        s = -log(1. - prob);
        if(s > s_max)
        {
          s_max = s;
          r_max = r;
        }
      }
      //FillRect2D(mean.x-2, mean.y-2, mean.x+2, mean.y+2, RGBColor::blue);
      TransparentArc2D(mean.x, mean.y, r_max, 0, 2*M_PI, RGBColor::blue);
    }
  }
}

const char* ConvexArcGroup::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size,
      "%sarcs: %d\ncoverage: %f (gap: %f), closedness: %f\n",
      Gestalt::GetInfo(), arcs.Size(), ang_cover,
      ArcGroupLargestAngularGap(arcs),
      closedness);
  return info_text;
}

bool ConvexArcGroup::IsAtPosition(int x, int y)
{
  for(unsigned i = 0; i < arcs.Size(); i++)
    if(Arcs(arcs[i])->IsAtPosition(x, y))
      return true;
  return false;
}

bool ConvexArcGroup::HasArc(unsigned arc)
{
  return arcs.Contains(arc);
}

/**
 * Returns true if the group is still convex when the given arc is added.
 * TODO: When appending an arc it is sufficient to check for convecity with the
 * first arc of the group. When prepending, with the last arc of the group.
 */
bool ConvexArcGroup::StillConvex(unsigned arc)
{
  for(unsigned i = 0; i < arcs.Size(); i++)
    if(!Arcs(arcs[i])->ConvexWith(Arcs(arc)))
      return false;
  return true;
}

/**
 * TODO: significance of a single arc should be > 0. if arc coverage is large
 * enough
 */
double ArcGroupSignificance(Array<unsigned> &arcs, unsigned l, unsigned u)
{
  if(u - l > 0)
  {
    unsigned i, j, k, n = 2*(u - l + 1);
    Vector2 p[n], t[n], c[n];
    Vector2 m1, t1, m2, t2;
    Vector2 mean(0., 0.);
    double weight[n], sum_weight = 0., r, alpha, sig_max = -HUGE;
    // estimation of center point density = num. points/area
    // Note: 2*NumArcs() is just an estimate of the number of center points
    // which however comes remarkably close to the true number (which can be
    // calculated once all groups have been created: e.g.
    // true: 3114, estimated: 2898;  true: 5678, estimated: 5492)
    double point_dens = 2.*NumArcs()/(double)VisionCore::ImageArea();

    for(i = l, j = 0; i <= u; i++, j++)
    {
      p[2*j] = Arcs(arcs[i])->tang_pt[0];
      t[2*j] = Arcs(arcs[i])->tang[0];
      p[2*j+1] = Arcs(arcs[i])->tang_pt[1];
      t[2*j+1] = Arcs(arcs[i])->tang[1];
    }
    for(i = 0; i < n; i++)
    {
      try
      {
        j = (i + n/3)%n;
        k = (i + (2*n)/3)%n;
        weight[i] = 1.;
        m1 = MidPoint(p[i], p[j]);
        t1 = LineIntersection(p[i], t[i], p[j], t[j]);
        weight[i] *= fabs(Cross(t[i], t[j]));
        t1 = Normalise(t1 - m1);  // now is direction vector
        m2 = MidPoint(p[j], p[k]);
        t2 = LineIntersection(p[j], t[j], p[k], t[k]);
        weight[i] *= fabs(Cross(t[j], t[k]));
        t2 = Normalise(t2 - m2);
        c[i] = LineIntersection(m1, t1, m2, t2);
        weight[i] *= fabs(Cross(t1, t2));
        mean += c[i]*weight[i];
        sum_weight += weight[i];
      }
      catch(Except &e)
      {
        // lines did not intersect: ignore this point
        c[i] = Vector2(HUGE/2., HUGE/2.);
        weight[i] = 0.;
        // leave mean and sum_weight untouched
      }
    }
    if(sum_weight > 0.)
    {
      mean /= sum_weight;
      // sort points according to distance
      cmp_mean = mean;
      qsort(c, n, sizeof(Vector2), CmpCenters);
      for(i = 0; i < n; i++)
      {
        r = fmax(Distance(mean, c[i]), 1.);
        alpha = CircleArea(r)*point_dens;
        // to be precise: we want PoissonCDF(i-1,alpha) for i=1..n
        // which is the same as PoissonCDF(i,alpha) for i=0..n-1
        sig_max = fmax(sig_max, -log(1. - PoissonCDF(i, alpha)));
      }
      return sig_max;
    }
    else
      return 0.;
  }
  else
    return 0.;
}

/**
 * Alternative significance.
 * Just sum up significances of pairwise good continuations of arcs.
 */
double ArcGroupSignificance2(Array<unsigned> &arcs, unsigned l, unsigned u)
{
  // TODO: CONTINUE
}

void ConvexArcGroup::CalculateSignificance()
{
  sig = ArcGroupSignificance(arcs, 0, arcs.Size() - 1);
}

double ArcGroupClosedness(Array<unsigned> &arcs)
{
  unsigned i, j;
  double sum_len = 0., sum_gap = 0.;
  for(i = 0; i < arcs.Size(); i++)
  {
    j = (i != arcs.Size() - 1 ? i + 1 : 0);
    sum_len += Arcs(arcs[i])->ArcLength();
    sum_gap += Distance(Arcs(arcs[i])->point[END], Arcs(arcs[j])->point[START]);
  }
  return sum_len/(sum_len + sum_gap);
}

/**
 * Calculates the angular coverage (= total angular span) of a group of arcs.
 * The angular span is just two pi minus the largest gap in the group of arcs.
 * Arcs must be sorted in ascending start angle order.
 */
double ArcGroupAngularCoverage(Array<unsigned> &arcs)
{
  return 2*M_PI - ArcGroupLargestAngularGap(arcs);
}

/**
 * Calculates the largest angular gap in the group of arcs.
 * Arcs must be sorted in ascending start angle order.
 */
double ArcGroupLargestAngularGap(Array<unsigned> &arcs)
{
  if(arcs.Size() == 1)
  {
    return 2*M_PI - Arcs(arcs[0])->angular_span;
  }
  else
  {
    double max_gap = 0., es, ss;
    unsigned j;
    // Note that since arcs are in ascending order, the differences
    // between end[i-1] and start[i] are always positive -> differences are
    // unique.
    // There might however be a slight overlap between end[i-1] and start[i]
    // (note that the arcs are only - sometimes coarse - approximations!).
    // If so start[i] - end[i-1] would be larger (e.g. 350 deg) than
    // start[i] - start[i-1]. Overlaps are treated as zero gaps and ignored.
    // Note: there are pathologic cases when even start[i] - start[i-1] is
    // "negative", e.g. 355 deg. Can happen for very flat parts of an ellipse.
    // In that case the daisy chain order of linking the arcs does not
    // correspond to an ascending start angle order.
    for(unsigned i = 0; i < arcs.Size(); i++)
    {
      j = (i == 0 ? arcs.Size() - 1 : i - 1);
      es = DiffAngle_0_2pi(Arcs(arcs[i])->start_angle,
          Arcs(arcs[j])->end_angle);
      ss = DiffAngle_0_2pi(Arcs(arcs[i])->start_angle,
          Arcs(arcs[j])->start_angle);
      // if no overlap
      //if(ss < M_PI && es < ss) // TODO: why did I require ss < M_PI?
      if(es < ss)
        max_gap = Max(max_gap, es);
    }
    return max_gap;
  }
}

double ArcGroupArea(Array<unsigned> &arcs)
{
  // TODO: continue
  return 0.;
}

double ArcGroupGapAreaRatio(Array<unsigned> &arcs)
{
  unsigned i, j;
  double sum_gap = 0.;
  for(i = 0; i < arcs.Size(); i++)
  {
    j = (i != arcs.Size() - 1 ? i + 1 : 0);
    sum_gap += Distance(Arcs(arcs[i])->point[END], Arcs(arcs[j])->point[START]);
  }
  return sum_gap/ArcGroupArea(arcs);
}

}

