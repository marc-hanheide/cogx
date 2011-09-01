//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef SYEDGE_HPP
#define SYEDGE_HPP

#include "multiplatform.hpp"

#include <iostream>
#include "syExcept.hpp"
#include "syEdgel.hpp"
#include "sySegment.hpp"
#include "ImgUtils.h"
#include "SMath.h"

// include Steinbichler libraries
#include "ipAOI.hpp"

// include OpenCV libraries
#include OCV_CV_H

NAMESPACE_CLASS_BEGIN( RTE )


//begin class///////////////////////////////////////////////////////////////////
//
//    CLASS DESCRIPTION:
//       RTE Edge Detection
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzEdge
{
private:
  int apertureSize;

  // Trace edge gradient
  void TraceEdge(IplImage *edgels, IplImage *tmp, short u, short v, CzArray<CzEdgel> &arr);

  // returns maximum
  inline short Max(short a, short b, short c);
  // test edgel neighbourhood
  inline bool TestEdgel(IplImage *edge, IplImage *tmp, short &x, short &y);

public:
  CzEdge();
  ~CzEdge();

  // Applies a sobel filter on img to both directions and returns the both gradient images in dx and dy
  void Sobel(IplImage *img, IplImage *dx, IplImage *dy);
  // Get the norm of edgels according to dx and dy
  void Norm(IplImage *dx, IplImage *dy, IplImage *edge);
  // Canny edge detection 
  void Canny(IplImage *indx, IplImage *indy, IplImage *idst, double lowThr, double highThr);
  // Link canny edgels to segments
  void LinkEdge(IplImage *src, 
                CzArray<CzSegment *> &segments, 
                CzArray<CzSegment *> &segments_mem, 
                IplImage *dx, IplImage *dy, 
                int iLinkingGrowCount);

  // set aperture size for sobel filter
  void Set(int ap){apertureSize=ap;}
};
//end class/////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// returns maximum
inline short CzEdge::Max(short a, short b, short c)
{
  return ( abs(a)>abs(b)?(abs(a)>abs(c)?a:c):abs(b)>abs(c)?b:c );
}

////////////////////////////////////////////////////////////////////////////////
// test edgel neighbourhood
inline bool CzEdge::TestEdgel(IplImage *edge, IplImage *tmp, short &x, short &y)
{
  //test 4-neighbourhood
  if (GetPx8UC1(edge,x+1,y)==255 && GetPx8UC1(tmp,x+1,y)==0)
    { x++; return true; }
  if (GetPx8UC1(edge,x,y+1)==255 && GetPx8UC1(tmp,x,y+1)==0)
    { y++; return true; }
  if (GetPx8UC1(edge,x-1,y)==255 && GetPx8UC1(tmp,x-1,y)==0)
    { x--; return true; }
  if (GetPx8UC1(edge,x,y-1)==255 && GetPx8UC1(tmp,x,y-1)==0)
    { y--; return true; }

  //test 8-neighbourhood
  if (GetPx8UC1(edge,x+1,y+1)==255 && GetPx8UC1(tmp,x+1,y+1)==0)
    { x++; y++; return true; }
  if (GetPx8UC1(edge,x+1,y-1)==255 && GetPx8UC1(tmp,x+1,y-1)==0)
    { x++; y--; return true; }
  if (GetPx8UC1(edge,x-1,y+1)==255 && GetPx8UC1(tmp,x-1,y+1)==0)
    { x--; y++; return true; }
  if (GetPx8UC1(edge,x-1,y-1)==255 && GetPx8UC1(tmp,x-1,y-1)==0)
    { x--; y--; return true; }

  return false;
}

NAMESPACE_CLASS_END()

#endif

