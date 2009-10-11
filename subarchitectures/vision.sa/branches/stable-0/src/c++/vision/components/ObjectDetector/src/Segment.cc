/**
 * $Id: Segment.cc,v 1.10 2006/11/24 13:47:03 mxz Exp mxz $
 */

#include <algorithm>
#include "Draw.hh"
#include "Segment.hh"
#include "FormSegments.hh"

namespace Z
{

/**
 * Construct a segment with a given number of edgels.
 */
Segment::Segment(const Array<Edgel> &arr)
  : Gestalt(SEGMENT), edgels(arr.Size())
{
  edgels = arr;
  CalculateSignificance();
}

/**
 * Draw the segment.
 * The detail has no effect here.
 */
void Segment::Draw(int detail)
{
  for(unsigned i = 0; i < edgels.Size(); i++)
    DrawPoint2D(edgels[i].p.x, edgels[i].p.y, RGBColor::green);
}

const char* Segment::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size, "%sedgels: %d\n", Gestalt::GetInfo(),
      edgels.Size());
  return info_text;
}

/**
 * Returns true if any part of the Gestalt is at pixel position (x,y).
 */
bool Segment::IsAtPosition(int x, int y)
{
  double xd = (double)x, yd = (double)y;
  for(unsigned i = 0; i < edgels.Size(); i++)
    if(IsEqual(xd, edgels[i].p.x) && IsEqual(yd, edgels[i].p.y))
      return true;
  return false;
}

void Segment::CalculateSignificance()
{
  /*if(edgels.Size() <= 1)
    sig = 0.;
  else
  {
    // gap missing for closed contour
    int g = (int)floor(Distance(edgels[0].p, edgels[edgels.Size()-1].p) + 0.5);
    sig = Significance(0, edgels.Size(), edgels.Size() + g, VisionCore::p_e);
  }*/
  //!!! HACK
  sig = edgels.Size();
}

/**
 * Returns the normalised tangent direction at the specified pixel.
 * Range of points over which tangent is estimated is adapted automatically.
 * Starting with points i-1 to i+1, extends range until line through first and
 * last point no longer fits the points between.
 * Note: direction points "forward" in terms of counting pixels on the edge.
 */
Vector2 Segment::Tangent(int i, int l, int u)
{
  const double MAX_DIST = 1.; // TODO: nasty threshold
  if(u == (int)UNDEF_ID)
    u = edgels.Size() - 1;
  int j, start = max(l, i - 1);
  int end = min(i + 1, u);
  bool on_line = true;
  Vector2 d, d_out;
  d_out = Normalise(edgels[end].p - edgels[start].p);
  while(on_line && (start >= l && end <= u))
  {
    d = Normalise(edgels[end].p - edgels[start].p);
    for(j = start + 1; j <= end - 1 && on_line; j++)
      if(AbsDistPointToLine(edgels[j].p, edgels[start].p, d) > MAX_DIST)
        on_line = false;
    if(on_line)
    {
      d_out = d;
      start--;
      end++;
    }
  }
  return d_out;
}

}

