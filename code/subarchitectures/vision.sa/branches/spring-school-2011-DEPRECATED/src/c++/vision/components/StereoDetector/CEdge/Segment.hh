/**
 * $Id$
 * Michael Zillich!
 */

#ifndef P_SEGMENT_HH
#define P_SEGMENT_HH

#include "Array.hh"
#include "Edgel.hh"
#include "PNamespace.hh"
#include "SDraw.hh"

namespace P 
{
  
class Segment
{
private:

public:
  Array<Edgel> edgels;

  Segment();
  Segment(const Array<Edgel> &arr);

  Vector2 Tangent(int i, int l = 0, int u = UINT_MAX);
  bool IsAtPosition(int x, int y);
  void Draw(IplImage *img, CvScalar col=CV_RGB(0,0,255));
};



void DeleteSegments(P::Array<Segment *> &segments);

inline void DeleteSegment(P::Array<Segment *> &segments, int idx);





/******************************** INLINE METHODES **********************************/
inline void DeleteSegment(P::Array<Segment *> &segments, int idx)
{
  delete segments[idx];
  segments.Erase(idx);
}

}





#endif

