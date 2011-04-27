/**
 * $Id: Color.hh,v 1.11 2007/02/04 23:53:03 mxz Exp mxz $
 */

#ifndef Z_COLOR_HH
#define Z_COLOR_HH

#include "Namespace.hh"

namespace Z
{

enum ColorFormat
{
  GRAY8,
  RGB24,
  BGR24,
  MAX_COLOR_FMT
};

/// returns the name (string) of the given color format enum
extern const char* ColorFormat2String(ColorFormat col);

/// returns the color format enum of the given string
extern ColorFormat String2ColorFormat(const char *str);

class RGBColor
{
public:
  static const RGBColor black;
  static const RGBColor white;
  static const RGBColor red;
  static const RGBColor green;
  static const RGBColor blue;
  static const RGBColor cyan;
  static const RGBColor magenta;
  static const RGBColor yellow;
  static const RGBColor dark_yellow;
  unsigned char r, g, b;
  RGBColor() {}
  RGBColor(unsigned char _r, unsigned char _g, unsigned char _b)
    : r(_r), g(_g), b(_b) {}
  bool operator==(RGBColor c) {return r == c.r && g == c.g && b == c.b;}
  unsigned char Grey()
  {return (unsigned char)(((int)r + (int)g + (int)b)/3);}
};

extern int DistSqr(RGBColor x, RGBColor y);
extern double Dist(RGBColor x, RGBColor y);

}

#endif

