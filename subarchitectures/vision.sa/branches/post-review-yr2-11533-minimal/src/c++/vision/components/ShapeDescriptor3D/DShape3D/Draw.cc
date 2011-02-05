/**
 * $Id$
 * Global functions for drawing points, lines, text, status text.
 * This file implements these functions for the QT GUI.
 */

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <GL/glut.h>
#include "Draw.hh"
#include "Vector2.hh"

namespace P 
{

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
    glColor4ub(r,g,b, a);
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
 * Draw a 2D point.
 */
void DrawPoint2D(double x, double y, RGBColor col)
{
  SetColor(col.r, col.g, col.b);
  glBegin(GL_POINTS);
  glVertex2d(x, y);
  glEnd();
}

/**
 * Draw a 2D line.
 */
void DrawLine2D(double x1, double y1, double x2, double y2, RGBColor col)
{
  SetColor(col.r, col.g, col.b);
  glBegin(GL_LINES);
  glVertex2d(x1, y1);
  glVertex2d(x2, y2);
  glEnd();
}

void DrawCross2D(double x, double y, RGBColor col)
{
  DrawLine2D(x-3, y-3, x+3, y+3, col);
  DrawLine2D(x+3, y-3, x-3, y+3, col);
}


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

void DrawArrow2D(double x1, double y1, double x2, double y2, RGBColor col)
{
  Vector2 p1=Vector2(x1, y1);
  Vector2 p2=Vector2(x2, y2);
  Vector2 d=p2-p1;
  double phi=PolarAngle(d);
  
  Vector2 p3=Vector2(-5, 2);
  Vector2 p4=Vector2(-5, -2);

  p3=Rotate(p3,phi);
  p4=Rotate(p4,phi);
  p3=p3+p2;
  p4=p4+p2;
  DrawLine2D(x1,y1,x2,y2,col);
  DrawLine2D(p3.x,p3.y,x2,y2, col);
  DrawLine2D(p4.x,p4.y,x2,y2, col);
}

/**
 * Draw a 3D point.
 */
void DrawPoint3D(double x, double y, double z, RGBColor col)
{
  SetColor(col.r, col.g, col.b);
  glBegin(GL_POINTS);
  glVertex3d(x, y, z);
  glEnd();
}

/**
 * Draw a 3D line.
 */
void DrawLine3D(double x1, double y1, double z1, 
                double x2, double y2, double z2, RGBColor col)
{
  SetColor(col.r, col.g, col.b);
  glBegin(GL_LINES);
  glVertex3d(x1, y1, z1);
  glVertex3d(x2, y2, z2);
  glEnd();
}

/**
 * Draw 2D text.
 * String must be '\0' delimited.
 */
void DrawText3D(const char *text, double x, double y, double z, RGBColor col)
{
  SetColor(col.r, col.g, col.b);
  glRasterPos3d(x, y, z);
  while(*text != '\0')
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *text++);
}

void DrawCross3D(double x, double y, double z, double size, RGBColor col)
{
  DrawLine3D(x-size,y,z , x+size,y,z, col);
  DrawLine3D(x,y-size,z , x,y+size,z, col);
  DrawLine3D(x,y,z-size , x,y,z+size, col);
}

void DrawSphere3D(double x, double y, double z, double r, RGBColor col, bool filled)
{
  GLuint glist = glGenLists(1);
  glNewList(glist, GL_COMPILE);
  glColor3ub(col.r, col.g, col.b);

  glPushMatrix();
  glTranslated(x, y, z);
	GLUquadric* mySphere = gluNewQuadric();
  if(filled)
    gluSphere(mySphere, r, 16, 16);
  else
    gluSphere(mySphere, r, 16, 16);
  glPopMatrix();
  glEndList();

  glCallList(glist);
	gluDeleteQuadric(mySphere);
}

void DrawCam3D(PoseCv &p, double scale, RGBColor col)
{
  CvMat *v0 = cvCreateMat( 3, 1, CV_32F );
  CvMat *v1 = cvCreateMat( 3, 1, CV_32F );

  cvZero(v0);
  cvmSet(v1,0,0,0.); cvmSet(v1,1,0,0.); cvmSet(v1,2,0,scale);
  
  PoseCv pose;
  InvPoseCv(p, pose);
  
  cvMatMulAdd(pose.R, v0, pose.t, v0 );
  cvMatMulAdd(pose.R, v1, pose.t, v1 );
  //cvSub(v0,pose.t,v0); cvGEMM( pose.R, v0, 1, 0, 1, v0);//, CV_GEMM_A_T );
  //cvSub(v1,pose.t,v1); cvGEMM( pose.R, v1, 1, 0, 1, v0);//, CV_GEMM_A_T );

  //DrawSphere3D(cvmGet(v0,0,0), cvmGet(v0,1,0), cvmGet(v0,2,0), scale/5., col, true);
  DrawCross3D(cvmGet(v0,0,0), cvmGet(v0,1,0), cvmGet(v0,2,0),scale/2.,col);
  DrawLine3D(cvmGet(v0,0,0), cvmGet(v0,1,0), cvmGet(v0,2,0), 
             cvmGet(v1,0,0), cvmGet(v1,1,0), cvmGet(v1,2,0), col);

  cvReleaseMat(&v0);
  cvReleaseMat(&v1);
}

}

