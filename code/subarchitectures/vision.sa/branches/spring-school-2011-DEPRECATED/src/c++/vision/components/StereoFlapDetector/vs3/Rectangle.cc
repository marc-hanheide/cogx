/**
 * $Id: Rectangle.cc,v 1.7 2007/02/04 23:53:03 mxz Exp mxz $
 */

#include "Draw.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Closure.hh"
#include "Rectangle.hh"

namespace Z
{

Rectangle::Rectangle(VisionCore *c, Closure *cl, LJunction *js[4])
  : Gestalt(c, RECTANGLE)
{
  clos = cl;
  for(unsigned i = 0; i < 4; i++)
    jcts[i] = js[i];
}

void Rectangle::Draw(int detail)
{
  if(detail == 0)
  {
    for(unsigned i = 0; i < 4; i++)
      DrawLine2D(jcts[i]->isct.x, jcts[i]->isct.y,
          jcts[(i<3?i+1:0)]->isct.x, jcts[(i<3?i+1:0)]->isct.y,
          RGBColor::yellow);
  }
  if(detail >= 1)
    clos->Draw(detail - 1);
}

const char* Rectangle::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n = 0;
  n += snprintf(info_text, info_size, "%sclosure: %u, junctions %u %u %u %u\n",
      Gestalt::GetInfo(), clos->ID(), jcts[0]->ID(), jcts[1]->ID(),
      jcts[2]->ID(), jcts[3]->ID());
  for(unsigned i = 0; i < 4; i++)
    n += snprintf(info_text + n, info_size - n, "(%.2f %.2f) ",
        jcts[i]->isct.x, jcts[i]->isct.y);
  return info_text;
}

bool Rectangle::IsAtPosition(int x, int y)
{
  return clos->IsAtPosition(x, y);;
}

void Rectangle::CalculateSignificance()
{
  //sig = -log(fmax(1., SumGaps())/fmax(1., Area()));
  //sig = SumGaps()/Circumference();
}

}

