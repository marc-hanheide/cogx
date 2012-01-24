/**
 * @file Draw.hh
 * @author Richtsfeld
 * @date May 2009
 * @version 0.1
 * @brief Drawing Gestalts to the OpenCv IplImage.
 **/


#ifndef Z_DRAW_HH
#define Z_DRAW_HH

#include "Color.hh"
#include "Vector2.hh"

namespace Z
{

extern void SetFatLines(bool set);
extern void SetBlackWhite(bool set);
/// Draw printf style text in the status bar.
extern void PrintStatus(const char *format, ...);
extern void DrawImageRGB24(const char *rgb24, int width, int height);
extern void SetDrawImagesSolid();
extern void SetDrawImagesLight();
extern void SetDrawImagesDark();
extern void SetClearImagesWhite();
extern void SetClearImagesBlack();
extern void DrawText2D(const char *text, double x, double y, RGBColor col);
extern void DrawPoint2D(double x, double y, RGBColor col);
extern void DrawLine2D(double x1, double y1, double x2, double y2, RGBColor col);
extern void DrawLine2D(double x1, double y1, double x2, double y2, RGBColor col, unsigned char transparency);
extern void DrawArrow(Vector2 start, Vector2 end, RGBColor col);
extern void DrawRect2D(double x1, double y1, double x2, double y2, RGBColor col);
extern void FillRect2D(double x1, double y1, double x2, double y2, RGBColor col);
extern void TransparentRect2D(double x1, double y1, double x2, double y2, RGBColor col);
extern void TransparentQuadrilateral2D(double x1, double y1, double x2, double y2,
		double x3, double y3, double x4, double y4, RGBColor col, bool fill, unsigned char transparency);
extern void DrawArc2D(double x, double y, double r, double start, double span, RGBColor col);
extern void FillArc2D(double x, double y, double r, double start, double span, RGBColor col);
extern void TransparentArc2D(double x, double y, double r, double start, double span, RGBColor col);
extern void DrawEllipse2D(double x, double y, double a, double b, double phi, RGBColor col);
extern void FillEllipse2D(double x, double y, double a, double b, double phi, RGBColor col);
extern void TransparentEllipse2D(double x, double y, double a, double b, double phi, RGBColor col);

}

#endif

