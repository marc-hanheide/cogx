/**
 * $Id: Color.cc,v 1.10 2007/01/21 22:23:03 mxz Exp mxz $
 */

#include <string.h>
#include "Math.hh"
#include "Color.hh"

namespace Z
{

static const int NAME_LENGTH = 40;
static const char names[][NAME_LENGTH] = {
  "GRAY8",
  "RGB24",
  "BGR24",
  "MAX_COLOR_FMT"
   };

const char *ColorFormat2String(ColorFormat col)
{
  return names[col];
}

ColorFormat String2ColorFormat(const char *str)
{
  for(int i = 0; i < MAX_COLOR_FMT; i++)
    if(strncmp(str, names[i], NAME_LENGTH) == 0)
      return (ColorFormat)i;
  return MAX_COLOR_FMT;
}

const RGBColor RGBColor::black(0, 0, 0);
const RGBColor RGBColor::white(255, 255, 255);
const RGBColor RGBColor::red(255, 0, 0);
const RGBColor RGBColor::green(0, 255, 0);
const RGBColor RGBColor::blue(0, 0, 255);
const RGBColor RGBColor::cyan(0, 255, 255);
const RGBColor RGBColor::magenta(255, 0, 255);
const RGBColor RGBColor::yellow(255, 255, 0);
const RGBColor RGBColor::dark_yellow(128, 128, 0);

int DistSqr(RGBColor x, RGBColor y)
{
  return Sqr((int)x.r - (int)y.r) + Sqr((int)x.g - (int)y.g) +
    Sqr((int)x.b - (int)y.b);
}

double Dist(RGBColor x, RGBColor y)
{
  return sqrt((double)DistSqr(x, y));
}

}

