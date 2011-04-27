/**
 * $Id$
 * Global functions for drawing points, lines, text, status text.
 */

#ifndef P_DRAW_HH
#define P_DRAW_HH

#include <opencv/cv.h>
#include "Color.hh"
#include "PoseCv.hh"
#include "PNamespace.hh"

namespace P 
{

extern void SetFatLines(bool set);
extern void SetBlackWhite(bool set);
/// Draw printf style text in the status bar.
extern void SetClearImagesWhite();
extern void SetClearImagesBlack();
extern void DrawText2D(const char *text, double x, double y, RGBColor col);
extern void DrawPoint2D(double x, double y, RGBColor col);
extern void DrawLine2D(double x1, double y1, double x2, double y2,
    RGBColor col);
extern void DrawCross2D(double x, double y, RGBColor col);
extern void DrawRect2D(double x1, double y1, double x2, double y2,
    RGBColor col);
extern void FillRect2D(double x1, double y1, double x2, double y2,
    RGBColor col);
extern void TransparentRect2D(double x1, double y1, double x2, double y2,
    RGBColor col);
extern void DrawArc2D(double x, double y, double r, double start, double span,
    RGBColor col);
extern void FillArc2D(double x, double y, double r, double start, double span,
    RGBColor col);
extern void TransparentArc2D(double x, double y, double r, double start,
    double span, RGBColor col);
extern void DrawEllipse2D(double x, double y, double a, double b, double phi,
    RGBColor col);
extern void FillEllipse2D(double x, double y, double a, double b, double phi,
    RGBColor col);
extern void TransparentEllipse2D(double x, double y, double a, double b,
    double phi, RGBColor col);
extern void DrawArrow2D(double x, double y, double phi, double size, RGBColor col);

//3D
extern void DrawPoint3D(double x, double y, double z, RGBColor col);
extern void DrawLine3D(double x1, double y1, double z1, 
                       double x2, double y2, double z2, RGBColor col);
extern void DrawText3D(const char *text, double x, double y, double z, RGBColor col);
extern void DrawCross3D(double x, double y, double z, double size, RGBColor col);
extern void DrawSphere3D(double x, double y, double z, double r, RGBColor col, bool filled=false);
extern void DrawCam3D(PoseCv &p, double scale=0.005, RGBColor col=RGBColor(255,255,255));
}

#endif

