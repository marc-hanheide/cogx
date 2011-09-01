/**
 * $Id$
 * Michael Zillich
 */

#include <stdio.h>
#include <algorithm>
extern "C"
{
#include "fitcircle.h"
}
#include "Math.hh"
#include "Segment.hh"
#include "Arc.hh"

namespace P 
{

int Arc::STRICT_CONVEXITY = 1;
 
Arc::Arc(Segment *seg, unsigned i, unsigned j, unsigned k,
    const Vector2 &cent, double rad)
{
  this->seg = seg;
  idx[START] = i;
  idx[MID] = j;
  idx[END] = k;
  idx[ONE_THIRD] = i + (k-i)/3;
  idx[TWO_THIRD] = i + (2*(k-i))/3;

  point[START] = seg->edgels[idx[START]].p;
  point[MID] = seg->edgels[idx[MID]].p;
  point[END] = seg->edgels[idx[END]].p;
  point[ONE_THIRD] = seg->edgels[idx[ONE_THIRD]].p;
  point[TWO_THIRD] = seg->edgels[idx[TWO_THIRD]].p;
  center = cent;
  radius = rad;
  // TODO: this sometimes returns odd results, disable for now
  // CalculateParams();
  norm[START] = Normalise(center - point[START]);
  norm[MID] = Normalise(center - point[MID]);
  norm[END] = Normalise(center - point[END]);

  CalculateTangents();
  // Note: START and END points might be swapped according to angles!

  CalculateAngles();

  CalculateSignificance();
}

extern unsigned peek;

/**
 * Draw the arc.
 * Details are:
 * 1: draw translucent "pie"
 * 2: draw edgels of arc and tangents
 * 3: draw arc id
 * 4: draw the complete originating segment
 */
void Arc::Draw(IplImage *img, int detail)
{
  if(detail >= 2)
    seg->Draw(img);
  if(detail >= 1)
  {
    for(unsigned i = idx[START]+1; i < idx[END]; i++)
      SDraw::DrawLine(img, seg->edgels[i-1].p.x, seg->edgels[i-1].p.y, seg->edgels[i].p.x, seg->edgels[i].p.y, CV_RGB(0,255,0));
  }
  if (radius<20000)   //OpenCV can not draw bigger circles
    SDraw::DrawArc(img, center.x, center.y, radius, start_angle, angular_span,CV_RGB(255,0,0));
}

bool Arc::IsAtPosition(int x, int y)
{
  double xd = (double)x, yd = (double)y;
  for(unsigned i = idx[START]; i <= idx[END]; i++)
    if(IsEqual(xd, seg->edgels[i].p.x) &&
       IsEqual(yd, seg->edgels[i].p.y))
      return true;
  return false;
}

void Arc::CalculateParams()
{
  double *x = new double[NumEdgels()];
  double *y = new double[NumEdgels()];
  for(unsigned i = idx[START], j = 0; i <= idx[END]; i++, j++)
  {
    x[j] = seg->edgels[i].p.x;
    y[j] = seg->edgels[i].p.y;
  }
  fit_circle(x, y, NumEdgels(), &center.x, &center.y, &radius);
  delete[] x;
  delete[] y;
}

/*
 * Order the given angles such that a,b,c are in counter clockwise order (i.e.
 * ascending in mathematical sense). The angles are either clockwise already
 * (then b - a > 0) or counterclockwise (then b - a < 0). If so, swap a and c.
 * Returns true if angles were already clockwise, false otherwise.
 * Note that since the sum of all angles is <= 2 pi and b is in the middle of
 * a and c, differences b - a and c - b can only be <= pi -> differences are
 * unique.
 */
bool Arc::OrderAnglesCounterClockwise(double &a, double &b, double &c)
{
  if(DiffAngle_mpi_pi(b, a) < 0.)
  {
    swap(a, c);
    return false;
  }
  else
    return true;
}

/**
 * Calculate start, mid and end angle.
 * Note: START and END points (and normals etc.) might be swapped according to
 * angles!
 */
void Arc::CalculateAngles()
{
  double mid_angle;
  Vector2 v1(point[START] - center);
  Vector2 v2(point[MID] - center);
  Vector2 v3(point[END] - center);
  start_angle = PolarAngle(v1);
  mid_angle = PolarAngle(v2);
  end_angle = PolarAngle(v3);
  if(!OrderAnglesCounterClockwise(start_angle, mid_angle, end_angle))
  {
    // if angles were swapped, also swap the corresponding points
    swap(point[START], point[END]);
    swap(point[ONE_THIRD], point[TWO_THIRD]);
    swap(norm[START], norm[END]);
    swap(tang_pt[0], tang_pt[1]);
    swap(tang[0], tang[1]);
  }
  // using the center angle we get the correct span also for spans > pi
  // note: for arcs spanning the whole circle, mid - start angle might be + or
  // -pi, therefore use absolute value
  angular_span = fabs(DiffAngle_mpi_pi(end_angle, mid_angle)) +
                 fabs(DiffAngle_mpi_pi(mid_angle, start_angle));
}

/**
 * Calculate tangents at two points at 1/3 and 2/3 of arc length.
 */
void Arc::CalculateTangents()
{
  tang_pt[0] = seg->edgels[idx[ONE_THIRD]].p;
  tang[0] = seg->Tangent(idx[ONE_THIRD], idx[START], idx[END]);
  tang_pt[1] = seg->edgels[idx[TWO_THIRD]].p;
  tang[1] = seg->Tangent(idx[TWO_THIRD], idx[START], idx[END]);
}

/**
 * Returns true if arc b lies "inside" this arc a, false otherwise
 *  b inside a:    b not inside a:
 *
 *   /    \          \   \
 *  |     |          |   |
 *   \   /          /   /
 *  a   b          a    b
 *
 * Calculation: v is a vector from point p on arc a to the circle center. If
 * dotproduct v.(x - p) for every point x on arc b is positive (angle<90) then b
 * lies on the "inside" of a. 
 */
bool Arc::HasInside(Arc *b)
{
  if(STRICT_CONVEXITY)
    return HasInsideStrong(b);
  else
    return HasInsideWeak(b);
}

/*
 * Strong convexity criterion.
 * Checks against half-planes at end points.
 */
bool Arc::HasInsideStrong(Arc *b)
{
  Vector2 b1s = b->point[ONE_THIRD] - point[START];
  Vector2 b1e = b->point[ONE_THIRD] - point[END];
  Vector2 b2s = b->point[TWO_THIRD] - point[START];
  Vector2 b2e = b->point[TWO_THIRD] - point[END];
  // check tangent halfplanes
  if(Dot(norm[START], b1s) < 0.)
    return false;
  if(Dot(norm[END], b1e) < 0.)
    return false;
  if(Dot(norm[START], b2s) < 0.)
    return false;
  if(Dot(norm[END], b2e) < 0.)
    return false;
  // check overlap
  if(Length(b->point[ONE_THIRD] - center) > radius)  // outside circle
  {
    if(angular_span < M_PI)
    {
      if(RightOf(norm[START], b1s) && LeftOf(norm[END], b1e))
        return false;
    }
    else
    {
      if(RightOf(norm[START], b1s) || LeftOf(norm[END], b1e))
        return false;
    }
  }
  if(Length(b->point[TWO_THIRD] - center) > radius)  // outside circle
  {
    if(angular_span < M_PI)
    {
      if(RightOf(norm[START], b2s) && LeftOf(norm[END], b2e))
        return false;
    }
    else
    {
      if(RightOf(norm[START], b2s) || LeftOf(norm[END], b2e))
        return false;
    }
  }
  return true;
}

/**
 * Weak convexity criterion.
 * Checks against half-plane at midpoint.
 */
bool Arc::HasInsideWeak(Arc *b)
{
  Vector2 b1s = b->point[ONE_THIRD] - point[START];
  Vector2 b1e = b->point[ONE_THIRD] - point[END];
  Vector2 b1m = b->point[ONE_THIRD] - point[MID];
  Vector2 b2s = b->point[TWO_THIRD] - point[START];
  Vector2 b2e = b->point[TWO_THIRD] - point[END];
  Vector2 b2m = b->point[TWO_THIRD] - point[MID];
  // check halfplane
  if(Dot(norm[MID], b1m) < 0.)
    return false;
  if(Dot(norm[MID], b2m) < 0.)
    return false;
  // check overlap
  if(Length(b->point[ONE_THIRD] - center) > radius)  // outside circle
  {
    if(angular_span < M_PI)
    {
      if(RightOf(norm[START], b1s) && LeftOf(norm[END], b1e))
        return false;
    }
    else
    {
      if(RightOf(norm[START], b1s) || LeftOf(norm[END], b1e))
        return false;
    }
  }
  if(Length(b->point[TWO_THIRD] - center) > radius)  // outside circle
  {
    if(angular_span < M_PI)
    {
      if(RightOf(norm[START], b2s) && LeftOf(norm[END], b2e))
        return false;
    }
    else
    {
      if(RightOf(norm[START], b2s) || LeftOf(norm[END], b2e))
        return false;
    }
  }
  return true;
}

bool Arc::HasInside(Vector2 &p)
{
  if(STRICT_CONVEXITY)
  {
    if(Dot(norm[START], p - point[START]) < 0.)
      return false;
    if(Dot(norm[END], p - point[END]) < 0.)
      return false;
  }
  else
  {
    if(Dot(norm[MID], p - point[MID]) < 0.)
      return false;
  }
  return true;
}

/**
 * Returns true if two arcs are convex (form a convex pair), false otherwise
 *  convex:        not convex:
 *
 *   /    \          \    /
 *  |     |          |   |
 *   \   /          /     \
 *
 */
bool Arc::ConvexWith(Arc *b)
{
  return this->HasInside(b) && b->HasInside(this);
}

/**
 * Attempt at a "soft" inside criterion.
 */
double Arc::Insideness(Arc *b)
{
  // vectors pointing inside
  Vector2 v[2] = {center - point[START], center - point[END]};
  unsigned inside = 0;
  for(unsigned i = b->idx[START]; i <= b->idx[END]; i++)
    if(Dot(v[START], b->seg->edgels[i].p - point[START]) > 0. &&
        Dot(v[END], b->seg->edgels[i].p - point[END]) > 0.)
      inside++;
  return (double)inside/(double)b->NumEdgels();
}

double Arc::Convexity(Arc *b)
{
  return fmax(this->Insideness(b), b->Insideness(this));
}

/**
 * Returns true if point p lies inside the "wedge" defined by center and
 * endpoints.
 */
bool Arc::HasInsideWedge(Arc *b)
{
  Vector2 v1 = (point[START] - center).NormalAntiClockwise();
  Vector2 v3 = (point[END] - center).NormalClockwise();
  Vector2 s = b->point[START] - center;
  Vector2 c = b->center - center;
  Vector2 e = b->point[END] - center;
  if(s.Norm() <= Radius() && Dot(s, v1) >= 0. && Dot(s, v3) >= 0.)
    return true;
  if(c.Norm() <= Radius() && Dot(c, v1) >= 0. && Dot(c, v3) >= 0.)
    return true;
  if(e.Norm() <= Radius() && Dot(e, v1) >= 0. && Dot(e, v3) >= 0.)
    return true;
  return false;
}

bool Arc::HasInsideBeam(Arc *b)
{
  Vector2 m0 = point[MID] - center;

  m0.Normalise();
  Vector2 s = point[START] - center;
  Vector2 sn = s - m0*Dot(s, m0);
  Vector2 e = point[END] - center;
  Vector2 en = e - m0*Dot(e, m0);
  if(Dot(b->point[START] - point[START], sn) <= 0 &&
     Dot(b->point[START] - point[END], en) <= 0)
    return true;
  if(Dot(b->point[END] - point[START], sn) <= 0 &&
     Dot(b->point[END] - point[END], en) <= 0)
    return true;
  return false;
}

/**
 * Returns true if there is a coaxial overlap between the two arcs.
 */
bool Arc::CoaxialOverlap(Arc *b)
{
  return this->HasInsideWedge(b) || b->HasInsideWedge(this);
  //return this->HasInsideBeam(b) || b->HasInsideBeam(this);
}

void Arc::CalculateSignificance()
{
  //int l = NumEdgels();//(int)(2.*M_PI*Radius());
  //sig = Significance(3, NumEdgels(), l, VisionCore::p_e);
  sig = NumEdgels()/(2.*M_PI*Radius());
}




/**
 * Detete an array of segments
 */
void DeleteArcs(P::Array<Arc *> &arcs)
{
  for (unsigned i=0; i<arcs.Size(); i++)
    delete arcs[i];
  arcs.Clear();
}





}

