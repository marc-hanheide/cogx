/**
 * $Id$
 */

#ifndef P_FORM_LINES_HH
#define P_FORM_LINES_HH

#include "Array.hh"
#include "Edgel.hh"
#include "Line.hh"
#include "Segment.hh"
#include "rosin_lines.hh"
#include "ImgUtils.hh"

namespace P 
{

class FVector2
{
public:
  float x;
  float y;

  FVector2(){}
  FVector2(float xx, float yy) : x(xx), y(yy) {}
};

class FormLines
{
private:
  void GetSubPx(IplImage *dx, IplImage *dy, Line *l, Array<FVector2> &subPx);
  bool FitLine(Array<FVector2> &subPx, float params[4]);


public:
  FormLines();
  void Operate(Array<Segment*> &segments, Array<Line*> &lines);
  void GetSubPixel(IplImage *dx, IplImage *dy, Array<Line*> &lines, double minLineLength=2.);
};


 
}

#endif

