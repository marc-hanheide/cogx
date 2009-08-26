/**
 * @file Draw.cc
 * @author Zillich
 * @date 2007
 * @version 0.1
 * @brief Drawing Gestalts to the drawing area.
 **/


#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <GL/glut.h>
#include <qstatusbar.h>
#include "Image.hh"
#include "Draw.hh"
#include "MainWin.hh"

namespace Z
{

// active drawing area
static QGLWidget *draw_area = 0;

static bool THICK_LINES = false;
static bool BLACK_WHITE = false;
static bool draw_bw = false;

/**
 * Set/unset fat lines and dots for all further drawing.
 * Note: only affects the main draw area;
 */
void SetFatLines(bool set)
{
  THICK_LINES = set;
}

/**
 * Set/unset black and white drawing of lines, dots etc. for all further
 * drawing.
 * Note: only affects the main draw area.
 */
void SetBlackWhite(bool set)
{
  BLACK_WHITE = set;
}

/**
 * Set where to draw to: main or info draw area.
 */
void SetActiveDrawArea(QGLWidget *da)
{
  draw_area = da;
  // note: only main draw area is affected by fat lines and black/white
  if(draw_area == MainWin::main_win->draw_area)
  {
    if(THICK_LINES)
    {
      glLineWidth(2.);
      glPointSize(2.);
    }
    else
    {
      glLineWidth(1.);
      glPointSize(1.);
    }
    draw_bw = BLACK_WHITE;
  }
  else
  {
    glLineWidth(1.);
    glPointSize(1.);
    draw_bw = false;
  }
}

void SaveDrawArea(const char *filename)
{
  Image img(NULL, draw_area->width(), draw_area->height(), 3, RGB24, true);
  SetDrawImagesSolid();
  glReadPixels(0, 0, img.Width(), img.Height(), GL_RGB, GL_UNSIGNED_BYTE,
      img.Data());
  img.SavePPM_FlipVert(filename);
}

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
  MainWin::main_win->statusBar()->message(msg);
}

/**
 * Draw an image given as RGB24 char array.
 */
void DrawImageRGB24(const char *rgb24, int width, int height)
{
  glRasterPos2i(0, 0);
  glDrawPixels(width, height, GL_RGB, GL_UNSIGNED_BYTE, rgb24);
}

/**
 * Draw images solid.
 */
void SetDrawImagesSolid()
{
  glPixelTransferf(GL_RED_SCALE, 1.);
  glPixelTransferf(GL_RED_BIAS, 0.);
  glPixelTransferf(GL_GREEN_SCALE, 1.);
  glPixelTransferf(GL_GREEN_BIAS, 0.);
  glPixelTransferf(GL_BLUE_SCALE, 1.);
  glPixelTransferf(GL_BLUE_BIAS, 0.);
}

/**
 * Draw images light transparent.
 * Good as background for displaying edges etc.
 */
void SetDrawImagesLight()
{
  float f = 0.4;
  glPixelTransferf(GL_RED_SCALE, f);
  glPixelTransferf(GL_RED_BIAS, 1. - f);
  glPixelTransferf(GL_GREEN_SCALE, f);
  glPixelTransferf(GL_GREEN_BIAS, 1. - f);
  glPixelTransferf(GL_BLUE_SCALE, f);
  glPixelTransferf(GL_BLUE_BIAS, 1. - f);
}

/**
 * Draw images dark transparent.
 * Good as background for displaying edges etc.
 */
void SetDrawImagesDark()
{
  float f = 0.4;
  glPixelTransferf(GL_RED_SCALE, f);
  glPixelTransferf(GL_RED_BIAS, 0.);
  glPixelTransferf(GL_GREEN_SCALE, f);
  glPixelTransferf(GL_GREEN_BIAS, 0.);
  glPixelTransferf(GL_BLUE_SCALE, f);
  glPixelTransferf(GL_BLUE_BIAS, 0.);
}

void SetClearImagesWhite()
{
  glClearColor(1., 1., 1., 1.);
}

void SetClearImagesBlack()
{
  glClearColor(0., 0., 0., 1.);
}

void SetColor(unsigned char r, unsigned char g, unsigned char b,
    unsigned char a = 255)
{
  if(!draw_bw)
    glColor4ub(r, g, b, a);
  else
  {
    // white stays white
    if(r == 255 && g == 255 && b == 255)
      glColor4ub(r, g, b, a);
    // everyting else become black
    else
      glColor4ub(0, 0, 0, a);
  }
}

/**
 * Draw 2D text.
 * String must be '\0' delimited.
 */
void DrawText2D(const char *text, double x, double y, RGBColor col)
{
  SetColor(col.r, col.g, col.b);
  glRasterPos2d(x, y);
  while(*text != '\0')
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *text++);
}

/**
 * @brief Draw a 2D point.
 */
void DrawPoint2D(double x, double y, RGBColor col)
{
  SetColor(col.r, col.g, col.b);
  glBegin(GL_POINTS);
  glVertex2d(x, y);
  glEnd();
}

/**
 * @brief Draw a 2D line.
 */
void DrawLine2D(double x1, double y1, double x2, double y2, RGBColor col)
{
	DrawLine2D(x1, y1, x2, y2, col, 255);
}

/**
 * @brief Draw a 2D line.
 */
void DrawLine2D(double x1, double y1, double x2, double y2, RGBColor col, unsigned char transparency)
{
  SetColor(col.r, col.g, col.b, transparency);
  glBegin(GL_LINES);
  glVertex2d(x1, y1);
  glVertex2d(x2, y2);
  glEnd();
}

/**
 * @brief Draw a 2D rectangle.
 */
static void Rect2D(double x1, double y1, double x2, double y2,
    RGBColor col, bool fill, unsigned char transparency)
{
  SetColor(col.r, col.g, col.b, transparency);
  if(fill)
    glBegin(GL_QUADS);
  else
    glBegin(GL_LINE_LOOP);
  glVertex2d(x1, y1);
  glVertex2d(x2, y1);
  glVertex2d(x2, y2);
  glVertex2d(x1, y2);
  glEnd();
}

/**
 * @brief Draws an arrow at the middle of a line.
 * @param start Start point of line.
 * @param end End point of line.
 * @param color Color of Arrow.
 */
void DrawArrow(Vector2 start, Vector2 end, RGBColor col)
{
  double len_2 = 4.; // length/2
  double wid_2 = 3.; // width/2
	Vector2 dir = end - start;
	if (dir.y != 0.) dir = Normalise(dir);
  Vector2 tip = (start + end)/2. + dir*len_2;
  Vector2 left = (start + end)/2. - dir*len_2 +
    dir.NormalAntiClockwise()*wid_2;
  Vector2 right = (start + end)/2. - dir*len_2 +
    dir.NormalClockwise()*wid_2;
  DrawLine2D(left.x, left.y, tip.x, tip.y, col);
  DrawLine2D(right.x, right.y, tip.x, tip.y, col);
}

void DrawRect2D(double x1, double y1, double x2, double y2,
    RGBColor col)
{
  Rect2D(x1, y1, x2, y2, col, false, 255);
}

void FillRect2D(double x1, double y1, double x2, double y2,
    RGBColor col)
{
  Rect2D(x1, y1, x2, y2, col, true, 255);
}

void TransparentRect2D(double x1, double y1, double x2, double y2,
    RGBColor col)
{
  Rect2D(x1, y1, x2, y2, col, true, 64);
}

void TransparentQuadrilateral2D(double x1, double y1, double x2, double y2,		// HACK ARI
    double x3, double y3, double x4, double y4, RGBColor col, bool fill, unsigned char transparency)
{
  SetColor(col.r, col.g, col.b, transparency);
  if(fill)
    glBegin(GL_QUADS);
  else
    glBegin(GL_LINE_LOOP);
  glVertex2d(x1, y1);
  glVertex2d(x2, y2);
  glVertex2d(x3, y3);
  glVertex2d(x4, y4);
  glEnd();
}


/**
 * Draw an arc with center (x,y), radius r from angle start to end.
 */
static void Arc2D(double x, double y, double r, double start, double span,
    RGBColor col, bool fill, unsigned char transparency)
{
  static double eps = M_PI/50.;  // maximum angle step
  int n = (int)ceil(span/eps);
  double delta = span/(double)n;
  SetColor(col.r, col.g, col.b, transparency);
  if(fill)
  {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2d(x, y);
  }
  else
    glBegin(GL_LINE_STRIP);
  for(int i = 0; i <= n; i++)
    glVertex2d(x + r*cos(start + (double)i*delta),
        y + r*sin(start + (double)i*delta));
  glEnd();
}

/**
 * Draw an arc with center (x,y), radius r from angle start to end.
 */
void DrawArc2D(double x, double y, double r, double start, double span,
    RGBColor col)
{
  Arc2D(x, y, r, start, span, col, false, 255);
}

/**
 * Fill an arc with center (x,y), radius r from angle start to end.
 */
void FillArc2D(double x, double y, double r, double start, double span,
    RGBColor col)
{
  Arc2D(x, y, r, start, span, col, true, 255);
}

/**
 * Fill an arc with center (x,y), radius r from angle start to end using
 * transparent color.
 */
void TransparentArc2D(double x, double y, double r, double start, double span,
    RGBColor col)
{
  Arc2D(x, y, r, start, span, col, true, 64);
}

/**
 * Approximate ellipse with 100 lines.
 * Note: fill and transparency ignored for now
 */
static void Ellipse2D(double x, double y, double a, double b, double phi,
    RGBColor col, bool fill, unsigned char transparency)
{
  int npoints = 100, i;
  double ang = 0., delta_ang = 2.*M_PI/npoints;
  double tx, ty;
  SetColor(col.r, col.g, col.b, transparency);
  if(fill)
    glBegin(GL_POLYGON);
  else
    glBegin(GL_LINE_LOOP);
  for(i = 0; i < npoints; i++)
  {
    tx = a*cos(ang - atan2(a*sin(phi),b*cos(phi)));
    ty = b*sin(ang - atan2(a*sin(phi),b*cos(phi)));
    glVertex2d(x + tx*cos(phi) - ty*sin(phi),
             y + tx*sin(phi) + ty*cos(phi));
    ang += delta_ang;
  }
  glEnd();
}

void DrawEllipse2D(double x, double y, double a, double b, double phi,
    RGBColor col)
{
  Ellipse2D(x, y, a, b, phi, col, false, 255);
}

void FillEllipse2D(double x, double y, double a, double b, double phi,
    RGBColor col)
{
  Ellipse2D(x, y, a, b, phi, col, true, 255);
}

void TransparentEllipse2D(double x, double y, double a, double b, double phi,
    RGBColor col)
{
  Ellipse2D(x, y, a, b, phi, col, true, 64);
}

}

