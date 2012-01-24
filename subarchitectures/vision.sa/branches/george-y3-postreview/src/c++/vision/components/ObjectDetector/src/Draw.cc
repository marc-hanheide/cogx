/**
 * @file Draw.cc
 * @author Richtsfeld
 * @date May 2009
 * @version 0.1
 * @brief Drawing Gestalts to the OpenCv IplImage.
 **/


#include "Draw.hh"
#include <opencv/highgui.h>

// #include <stdio.h>
// #include <stdarg.h>
#include <math.h>
// #include <GL/glut.h>
// #include <qstatusbar.h>
// #include "Image.hh"
// #include "MainWin.hh"

namespace Z
{

IplImage *iplImage;

// // active drawing area
// // static QGLWidget *draw_area = 0;

// // static bool THICK_LINES = false;
// // static bool BLACK_WHITE = false;
// // static bool draw_bw = false;

/**
 * Set/unset fat lines and dots for all further drawing.
 * Note: only affects the main draw area;
 */
void SetFatLines(bool set)
{
// //   THICK_LINES = set;
}

/**
 * Set/unset black and white drawing of lines, dots etc. for all further
 * drawing.
 * Note: only affects the main draw area.
 */
void SetBlackWhite(bool set)
{
// //   BLACK_WHITE = set;
}

/**
 * Set where to draw to: main or info draw area.
 */
// void SetActiveDrawArea(QGLWidget *da)
// {
//   draw_area = da;
//   // note: only main draw area is affected by fat lines and black/white
//   if(draw_area == MainWin::main_win->draw_area)
//   {
//     if(THICK_LINES)
//     {
//       glLineWidth(2.);
//       glPointSize(2.);
//     }
//     else
//     {
//       glLineWidth(1.);
//       glPointSize(1.);
//     }
//     draw_bw = BLACK_WHITE;
//   }
//   else
//   {
//     glLineWidth(1.);
//     glPointSize(1.);
//     draw_bw = false;
//   }
// }

/**
 * @brief Set the IplImage as active draw area.
 * @param iI OpenCv IplImage for drawings.
 */
void SetActiveDrawArea(IplImage *iI)
{
	iplImage = iI;
}


void SaveDrawArea(const char *filename)
{
//   Image img(NULL, draw_area->width(), draw_area->height(), 3, RGB24, true);
//   SetDrawImagesSolid();
//   glReadPixels(0, 0, img.Width(), img.Height(), GL_RGB, GL_UNSIGNED_BYTE,
//       img.Data());
//   img.SavePPM_FlipVert(filename);
}

/**
 * Draw printf style text in the status bar.
 */
void PrintStatus(const char *format, ...)
{
//   static char msg[1024];
//   va_list arg_list;
//   va_start(arg_list, format);
//   vsnprintf(msg, 1024, format, arg_list);
//   va_end(arg_list);
//   MainWin::main_win->statusBar()->message(msg);
}

/**
 * Draw an image given as RGB24 char array.
 */
void DrawImageRGB24(const char *rgb24, int width, int height)
{
//   glRasterPos2i(0, 0);
//   glDrawPixels(width, height, GL_RGB, GL_UNSIGNED_BYTE, rgb24);
}

/**
 * Draw images solid.
 */
void SetDrawImagesSolid()
{
//   glPixelTransferf(GL_RED_SCALE, 1.);
//   glPixelTransferf(GL_RED_BIAS, 0.);
//   glPixelTransferf(GL_GREEN_SCALE, 1.);
//   glPixelTransferf(GL_GREEN_BIAS, 0.);
//   glPixelTransferf(GL_BLUE_SCALE, 1.);
//   glPixelTransferf(GL_BLUE_BIAS, 0.);
}

/**
 * Draw images light transparent.
 * Good as background for displaying edges etc.
 */
void SetDrawImagesLight()
{
//   float f = 0.4;
//   glPixelTransferf(GL_RED_SCALE, f);
//   glPixelTransferf(GL_RED_BIAS, 1. - f);
//   glPixelTransferf(GL_GREEN_SCALE, f);
//   glPixelTransferf(GL_GREEN_BIAS, 1. - f);
//   glPixelTransferf(GL_BLUE_SCALE, f);
//   glPixelTransferf(GL_BLUE_BIAS, 1. - f);
}

/**
 * Draw images dark transparent.
 * Good as background for displaying edges etc.
 */
void SetDrawImagesDark()
{
//   float f = 0.4;
//   glPixelTransferf(GL_RED_SCALE, f);
//   glPixelTransferf(GL_RED_BIAS, 0.);
//   glPixelTransferf(GL_GREEN_SCALE, f);
//   glPixelTransferf(GL_GREEN_BIAS, 0.);
//   glPixelTransferf(GL_BLUE_SCALE, f);
//   glPixelTransferf(GL_BLUE_BIAS, 0.);
}

void SetClearImagesWhite()
{
//   glClearColor(1., 1., 1., 1.);
}

void SetClearImagesBlack()
{
//   glClearColor(0., 0., 0., 1.);
}

void SetColor(unsigned char r, unsigned char g, unsigned char b,
    unsigned char a = 255)
{
//   if(!draw_bw)
//     glColor4ub(r, g, b, a);
//   else
//   {
//     // white stays white
//     if(r == 255 && g == 255 && b == 255)
//       glColor4ub(r, g, b, a);
//     // everyting else become black
//     else
//       glColor4ub(0, 0, 0, a);
//   }
}

/**
 * @brief Draw 2D text.
 * @param text Text
 * @param x x-position
 * @param y y-position
 * @param col Color of Text
 */
void DrawText2D(const char *text, double x, double y, RGBColor col)
{
	CvScalar cvCol = cvScalar(col.r, col.g, col.b);
	CvFont font;
	double hScale=0.5;
	double vScale=0.5;
	int    lineWidth=1;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);

	cvPutText (iplImage, text, cvPoint(x, y), &font, cvCol);
}

/**
 * @brief Draw a 2D point.
 */
void DrawPoint2D(double x, double y, RGBColor col)
{
	CvScalar cvCol = cvScalar(col.r, col.g, col.b);
	cvLine(iplImage, cvPoint(x, y), cvPoint(x,y), cvCol, 1);
}

/**
 * @brief Draw a 2D line.
 */
void DrawLine2D(double x1, double y1, double x2, double y2, RGBColor col)
{
	CvScalar cvCol = cvScalar(col.r, col.g, col.b);
	cvLine(iplImage, cvPoint(x1, y1), cvPoint(x2,y2), cvCol, 1);
}

/**
 * @brief Draw a 2D line.
 * @TODO Without transparency
 */
void DrawLine2D(double x1, double y1, double x2, double y2, RGBColor col, unsigned char transparency)
{
	DrawLine2D(x1, y1, x2, y2, col);
}

/**
 * @brief Draws an arrow at the middle of a line.
 * @param start Start point of line.
 * @param end End point of line.
 * @param color Color of Arrow.
 */
void DrawArrow(Vector2 start, Vector2 end, RGBColor col)
{
//   double len_2 = 4.; // length/2
//   double wid_2 = 3.; // width/2
// 	Vector2 dir = end - start;
// 	if (dir.y != 0.) dir = Normalise(dir);
//   Vector2 tip = (start + end)/2. + dir*len_2;
//   Vector2 left = (start + end)/2. - dir*len_2 +
//     dir.NormalAntiClockwise()*wid_2;
//   Vector2 right = (start + end)/2. - dir*len_2 +
//     dir.NormalClockwise()*wid_2;
//   DrawLine2D(left.x, left.y, tip.x, tip.y, col);
//   DrawLine2D(right.x, right.y, tip.x, tip.y, col);
}

/**
 * @brief Draw a 2D rectangle.
 * @TODO Without transparency
 */
static void Rect2D(double x1, double y1, double x2, double y2,
    RGBColor col, bool fill, unsigned char transparency)
{
	CvScalar cvCol = cvScalar(col.r, col.g, col.b);
	int thickness = 1;
	if(fill) thickness = -1;

	cvLine(iplImage, cvPoint(x1, y1), cvPoint(x2,y1), cvCol, thickness);
	cvLine(iplImage, cvPoint(x2, y1), cvPoint(x2,y2), cvCol, thickness);
	cvLine(iplImage, cvPoint(x2, y2), cvPoint(x1,y2), cvCol, thickness);
	cvLine(iplImage, cvPoint(x1, y2), cvPoint(x1,y1), cvCol, thickness);
}

/**
 * @brief Draw a 2D rectangle.
 */
void DrawRect2D(double x1, double y1, double x2, double y2,
    RGBColor col)
{
  Rect2D(x1, y1, x2, y2, col, false, 255);
}

/**
 * @brief Draw a filled 2D rectangle.
 */
void FillRect2D(double x1, double y1, double x2, double y2,
    RGBColor col)
{
  Rect2D(x1, y1, x2, y2, col, true, 255);
}

/**
 * @brief Draw a transparent 2D rectangle.
 */
void TransparentRect2D(double x1, double y1, double x2, double y2,
    RGBColor col)
{
  Rect2D(x1, y1, x2, y2, col, true, 64);
}

/**
 * @brief Draw a quadrilateral with transparency
 * @TODO Without transparency
 */
void TransparentQuadrilateral2D(double x1, double y1, double x2, double y2,		// HACK ARI
    double x3, double y3, double x4, double y4, RGBColor col, bool fill, unsigned char transparency)
{
	int thickness = 1;
	if(fill) thickness = -1; 	// negative values => filling
	CvScalar cvCol = cvScalar(col.r, col.g, col.b);
	cvLine(iplImage, cvPoint(x1, y1), cvPoint(x2,y2), cvCol, thickness);
	cvLine(iplImage, cvPoint(x2, y2), cvPoint(x3,y3), cvCol, thickness);
	cvLine(iplImage, cvPoint(x3, y3), cvPoint(x4,y4), cvCol, thickness);
	cvLine(iplImage, cvPoint(x4, y4), cvPoint(x1,y1), cvCol, thickness);
}


/**
 * @brief Draw an arc with center (x,y), radius r from angle start to end.
 * @TODO Without transparency
 */
static void Arc2D(double x, double y, double r, double start, double span,
    RGBColor col, bool fill, unsigned char transparency)
{
	int thickness = 1;
	if(fill) thickness = -1; 	// negative values => filling
	CvScalar cvCol = cvScalar(col.r, col.g, col.b);

	if(r > 30000.) return;

// printf("Arc2D: x-y: %4.2f-%4.2f / r: %4.1f / start: %4.1f / span: %6.6f\n", x, y, r, start, span);
	cvEllipse(iplImage, cvPoint(x, y), cvSize(r, r), (-start)*360/(2*M_PI), 0, (-span)*360/(2*M_PI), cvCol, thickness);

}

/**
 * @brief Draw an arc with center (x,y), radius r from angle start to end.
 */
void DrawArc2D(double x, double y, double r, double start, double span,
    RGBColor col)
{
  Arc2D(x, y, r, start, span, col, false, 255);
}

/**
 * @brief Fill an arc with center (x,y), radius r from angle start to end.
 */
void FillArc2D(double x, double y, double r, double start, double span,
    RGBColor col)
{
  Arc2D(x, y, r, start, span, col, true, 255);
}

/**
 * @brief Fill an arc with center (x,y), radius r from angle start to end using
 * transparent color.
 */
void TransparentArc2D(double x, double y, double r, double start, double span,
    RGBColor col)
{
  Arc2D(x, y, r, start, span, col, true, 64);
}

/**
 * @brief Approximate ellipse with 100 lines.
 * Note: fill and transparency ignored for now
 */
static void Ellipse2D(double x, double y, double a, double b, double phi,
    RGBColor col, bool fill, unsigned char transparency)
{
	int thickness = 1;
	if(fill) thickness = -1; 	// negative values => filling
	CvScalar cvCol = cvScalar(col.r, col.g, col.b);

	cvEllipse(iplImage, cvPoint(x, y), cvSize(a, b), -phi*360/(2*M_PI), 0, 360, cvCol, thickness);
}

/**
 * @brief Draw a 2D Ellipse.
 */
void DrawEllipse2D(double x, double y, double a, double b, double phi,
    RGBColor col)
{
  Ellipse2D(x, y, a, b, phi, col, false, 255);
}


/**
 * @brief Draw a 2D Ellipse.
 */
void FillEllipse2D(double x, double y, double a, double b, double phi,
    RGBColor col)
{
  Ellipse2D(x, y, a, b, phi, col, true, 255);
}

/**
 * @brief Draw a 2D Ellipse.
 */
void TransparentEllipse2D(double x, double y, double a, double b, double phi,
    RGBColor col)
{
  Ellipse2D(x, y, a, b, phi, col, true, 64);
}

}

