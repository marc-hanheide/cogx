/**
 * $Id$
 */

#ifndef P_LOG_POLAR_HH
#define P_LOG_POLAR_HH

#include <opencv/cv.h>
#include <iostream>
#include "v4r/CEdge/PNamespace.hh"
#include "v4r/CEdge/Except.hh"
#include "v4r/CEdge/Vector2.hh"

namespace P
{


class LogPolar
{
private:
  CvMat *mapx, *mapy;

public:
  LogPolar();
  ~LogPolar();
  void ComputeMaps(CvSize ssize, CvSize dsize, CvPoint2D32f center, double m, int flags);
  void MapImage(IplImage *src, IplImage *dst, int flags, int bgcol=0);
};

/*********************** INLINE METHODES **************************/
/**
 * map a point back to cartesian space
 */
inline void LogPolar2Cart(Vector2 in, Vector2 center, Vector2 size, double M, Vector2 &out)
{
  double r = std::exp(in.x/M);
  out.x = r*cos(in.y*2*CV_PI/(size.x-1)) + center.x;
  out.y = r*sin(in.y*2*CV_PI/(size.y-1)) + center.y;
}




}

#endif

