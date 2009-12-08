/**
 * $Id: Draw_small.cc,v 1.3 2006/11/24 13:47:03 mxz Exp mxz $
 * Global functions for drawing points, lines, text, status text.
 * This file implements these functions for the QT GUI.
 */

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "Draw.hh"

namespace Z
{

/**
 * Draw printf style text in the status bar.
 */
void PrintStatus(const char *format, ...)
{
  static char msg[1024];
  va_list arg_list;
  va_start(arg_list, format);
  vsnprintf(msg, 1024, format, arg_list);
  va_end(arg_list);
  printf("%s\n", msg);
}

void DrawImageRGB24(const char *rgb24, int width, int height){}
void DrawText2D(const char *text, double x, double y, RGBColor col){}
void DrawPoint2D(double x, double y, RGBColor col){}
void DrawLine2D(double x1, double y1, double x2, double y2,
    RGBColor col){}
void DrawRect2D(double x1, double y1, double x2, double y2,
    RGBColor col){}
void FillRect2D(double x1, double y1, double x2, double y2,
    RGBColor col){}
void TransparentRect2D(double x1, double y1, double x2, double y2,
    RGBColor col){}
void DrawArc2D(double x, double y, double r, double start, double span,
    RGBColor col){}
void FillArc2D(double x, double y, double r, double start, double span,
    RGBColor col){}
void TransparentArc2D(double x, double y, double r, double start,
    double span, RGBColor col){}
void DrawEllipse2D(double x, double y, double a, double b, double phi,
    RGBColor col){}
void FillEllipse2D(double x, double y, double a, double b, double phi,
    RGBColor col){}
void TransparentEllipse2D(double x, double y, double a, double b,
    double phi, RGBColor col){}

}

