//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Michael Zillich,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#include "syDebug.hpp"

#include <algorithm>
#include "sySegment.hpp"

NAMESPACE_CLASS_BEGIN( RTE )


////////////////////////////////////////////////////////////////////////////////
CzSegment::CzSegment()
{
	end = NULL;
	start = NULL;
	s = e = false;
	closable = false;
	closed = false;
}

////////////////////////////////////////////////////////////////////////////////
// Construct a segment with a given number of edgels.
CzSegment::CzSegment(const CzArray<CzEdgel> &arr) 
   : edgels(arr.Size())
{
   edgels = arr;
   end = NULL;
   start = NULL;
   s = e = false;
   closable = false;
   closed = false;
}

////////////////////////////////////////////////////////////////////////////////
// Returns true if any part of the Segment is at pixel position (x,y).
bool CzSegment::IsAtPosition(int x, int y)
{
   double xd = (double)x, 
          yd = (double)y;
   for (unsigned i = 0; i < edgels.Size(); i++) {
      if (IsEqual(xd, edgels[i].p.x) && IsEqual(yd, edgels[i].p.y)) {
         return true;
      }
   }
   return false;
}

////////////////////////////////////////////////////////////////////////////////
// Returns the normalised tangent direction at the specified pixel.
// Range of points over which tangent is estimated is adapted automatically.
// Starting with points i-1 to i+1, extends range until line through first and
// last point no longer fits the points between.
// Note: direction points "forward" in terms of counting pixels on the edge.
CzVector2 CzSegment::Tangent(int i, int l, int u)
{
   const double MAX_DIST = 1.; // nasty threshold
   if (u == (int)UNDEF_ID)
      u = edgels.Size() - 1;
   int j, start = max(l, i - 1);
   int end = min(i + 1, u);
   bool on_line = true;
   CzVector2 d, d_out;
   d_out = Normalise(edgels[end].p - edgels[start].p);
   while (on_line && (start >= l && end <= u)) {
      d = Normalise(edgels[end].p - edgels[start].p);
      for (j = start + 1; j <= end - 1 && on_line; j++) {
         if (AbsDistPointToLine(edgels[j].p, edgels[start].p, d) > MAX_DIST)
            on_line = false;
      }
      if (on_line) {
         d_out = d;
         start--;
         end++;
      }
   }
   return d_out;
}

NAMESPACE_CLASS_END()
