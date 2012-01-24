/**
 * $Id$
 */

#include <algorithm>
#include "Segment.hh"

namespace P 
{

/**
 * Construct a segment with a given number of edgels.
 */
Segment::Segment(const Array<Edgel> &arr)
 : edgels(arr.Size())
{
  edgels = arr;
}

Segment::Segment()
{
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

/**
 * Draw segments
 */
void Segment::Draw(IplImage *img, CvScalar col)
{
  for (unsigned i=1; i<edgels.Size(); i++)
    SDraw::DrawLine(img, edgels[i-1].p.x, edgels[i-1].p.y, edgels[i].p.x, edgels[i].p.y, col);
}



/**
 * Detete an array of segments
 */
void DeleteSegments(P::Array<Segment *> &segments)
{
  for (unsigned i=0; i<segments.Size(); i++)
    delete segments[i];
  segments.Clear();
}


}

