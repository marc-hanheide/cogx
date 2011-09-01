//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#include "syDebug.hpp"

#include "SMath.h"


NAMESPACE_CLASS_BEGIN( RTE )

/**
 * Clip line to rectangle (0,0)-(xmin, xmax) (including xmin and xmax).
 * Returns true if at least part of the line is inside, false if line is
 * completely outside.
 */
bool ClipLine(int xmax, int ymax, int *x1, int *y1, int *x2, int *y2)
{
  int xmin = 0, ymin = 0;
  // left
  if(*x1 >= xmin)    // don't clip point 1
  {
    if(*x2 < xmin)   // clip point 2
    {
      *y2 = *y2 + ((xmin - *x2)*(*y1 - *y2))/(*x1 - *x2);
      *x2 = xmin;
    }
  }
  else   // clip point 1
  {
    if(*x2 >= xmin)
    {
      *y1 = *y1 + ((xmin - *x1)*(*y2 - *y1))/(*x2 - *x1);
      *x1 = xmin;
    }
    else // both points are out of region
    {
      return false;
    }
  }
  // right
  if(*x1 <= xmax)    // don't clip point 1
  {
    if(*x2 > xmax)   // clip point 2
    {
      *y2 = *y2 - ((*x2 - xmax)*(*y2 - *y1))/(*x2 - *x1);
      *x2 = xmax;
    }
  }
  else   // clip point 1
  {
    if(*x2 <= xmax)
    {
      *y1 = *y1 - ((*x1 - xmax)*(*y1 - *y2))/(*x1 - *x2);
      *x1 = xmax;
    }
    else // both points are out of region
    {
      return false;
    }
  }
  // top
  if(*y1 >= ymin)    // don't clip point 1
  {
    if(*y2 < ymin)   // clip point 2
    {
      *x2 = *x2 + ((ymin - *y2)*(*x1 - *x2))/(*y1 - *y2);
      *y2 = ymin;
    }
  }
  else   // clip point 1
  {
    if(*y2 >= ymin)
    {
      *x1 = *x1 + ((ymin - *y1)*(*x2 - *x1))/(*y2 - *y1);
      *y1 = ymin;
    }
    else // both points are out of region
    {
      return false;
    }
  }
  // bottom
  if(*y1 <= ymax)    // don't clip point 1
  {
    if(*y2 > ymax)   // clip point 2
    {
      *x2 = *x2 - ((*y2 - ymax)*(*x2 - *x1))/(*y2 - *y1);
      *y2 = ymax;
    }
  }
  else   // clip point 1
  {
    if(*y2 <= ymax)
    {
      *x1 = *x1 - ((*y1 - ymax)*(*x1 - *x2))/(*y1 - *y2);
      *y1 = ymax;
    }
    else // both points are out of region
    {
      return false;
    }
  }
  return true;
}

/**
 * Returns timespecs x - y as double.
 */
double timespec_diff(struct timespec *x, struct timespec *y)
{
  if(x->tv_nsec < y->tv_nsec)
  {
    long nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
    y->tv_nsec -= 1000000000 * nsec;
    y->tv_sec += nsec;
  }
  if(x->tv_nsec - y->tv_nsec > 1000000000)
  {
    long nsec = (y->tv_nsec - x->tv_nsec) / 1000000000;
    y->tv_nsec += 1000000000 * nsec;
    y->tv_sec -= nsec;
  }
  double diff = (double)(x->tv_sec - y->tv_sec) +
                (double)(x->tv_nsec - y->tv_nsec)/1000000000.;
  return diff;
}

NAMESPACE_CLASS_END()

