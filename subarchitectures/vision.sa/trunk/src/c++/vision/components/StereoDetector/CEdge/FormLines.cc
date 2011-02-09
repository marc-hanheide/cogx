/**
 * $Id$
 * Michael Zillich
 */

#include <float.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "FormLines.hh"

namespace P 
{

static int CmpLines(const void *a, const void *b)
{
  if(((Line*)a)->sig > ((Line*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

FormLines::FormLines()
{
}

/**
 * interpolate
 */
void FormLines::GetSubPx(IplImage *dx, IplImage *dy, Line *l, Array<FVector2> &subPx)
{
  float ax, ay, bx, by, cx, cy;
  float fa, fb, fc;
  float minx, miny;

  FVector2 dir = FVector2(l->dir.x, l->dir.y);
  FVector2 normal = FVector2(-dir.y, dir.x);

  subPx.Clear();

  for(unsigned i = l->idx[START]; i <= l->idx[END]; i++)
  {
    bx = l->seg->edgels[i].p.x;
    by = l->seg->edgels[i].p.y;
    ax = bx - cvRound(normal.x);
    ay = by - cvRound(normal.y);
    cx = bx + cvRound(normal.x);
    cy = by + cvRound(normal.y);

    fa = sqrt(Sqr((float)GetPx16SC1(dx, (int)ax,(int)ay)) + Sqr((float)GetPx16SC1(dy, (int)ax,(int)ay)));
    fb = sqrt(Sqr((float)GetPx16SC1(dx, (int)bx,(int)by)) + Sqr((float)GetPx16SC1(dy, (int)bx,(int)by)));
    fc = sqrt(Sqr((float)GetPx16SC1(dx, (int)cx,(int)cy)) + Sqr((float)GetPx16SC1(dy, (int)cx,(int)cy)));
    minx = bx - .5*( ((bx-ax)*(bx-ax)*(fb-fc) - (bx-cx)*(bx-cx)*(fb-fa)) / ((bx-ax) * (fb-fc) - (bx-cx)*(fb-fa)) );
    miny = by - .5*( ((by-ay)*(by-ay)*(fb-fc) - (by-cy)*(by-cy)*(fb-fa)) / ((by-ay) * (fb-fc) - (by-cy)*(fb-fa)) );

    if(cvRound(normal.x) == 0)
      minx = bx;
    if(cvRound(normal.y) == 0)
      miny = by;

    subPx.PushBack(FVector2(minx,miny));
  }
}

/**
 * least square fit of a line
 */
bool FormLines::FitLine(Array<FVector2> &subPx, float params[4])
{
  if (subPx.Size()<3)
    return false;

  CvMat points;

  cvInitMatHeader(&points, 1, subPx.Size(), CV_32FC2, &subPx[0]);

  cvFitLine( &points, CV_DIST_L2, 0, 0.01, 0.01, params );
  
  return true;
}






/**
 * Find lines
 */
void FormLines::Operate(Array<Segment*> &segments, Array<Line*> &lines)
{
  LineSegmenter ls;

  DeleteLines(lines);

  for (unsigned i=0; i<segments.Size(); i++)
  {
    ls.RosinFitLinesToSegment(segments[i],lines);
  }
 
  for (unsigned i=0; i<lines.Size(); i++)
    lines[i]->id=i;
}

/**
 * Refine lines with gradient interpolation (parabola)
 */
void FormLines::GetSubPixel(IplImage *dx, IplImage *dy, Array<Line*> &lines, double minLineLength)
{
  Line *l;
  float params[4];
  Array<FVector2> subPx;
  double dist;
  Vector2 p, d, n, dq;

  for (unsigned i=0; i<lines.Size(); i++)
  {
    if (lines[i]->len >= minLineLength)
    {
      l = lines[i];
      GetSubPx(dx, dy, l, subPx);

      if (subPx.Size()>=2)
      {
        if (FitLine(subPx, params))
        {
          p = Vector2(params[2], params[3]);
          d = Vector2(params[0], params[1]); 
          n = d.Normal();

          dist = DistPointToLine(l->point[0], p, d);
          dq = dist * n;
          l->point[0] = l->point[0] + dq;

          dist = DistPointToLine(l->point[1], p, d);
          dq = dist * n;
          l->point[1] = l->point[1] + dq;
        }
        else
        {
          l->point[0] = Vector2(subPx[0].x, subPx[0].y);
          l->point[1] = Vector2(subPx.Last().x, subPx.Last().y);
        }
      }

      l->CalculateParameters();
    }
  }
}


}





