/**
 * A somewhat hacky glut based version of stereoflaps.
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <list>
#include <string>
#include <GL/glut.h>
#include "StereoCore.hh"
#include "OpenCvImgSeqVideo.hh"

using namespace Z;

#define LEFT  0
#define RIGHT 1
#define DEFAULT_RUNTIME  250

class Opts
{
public:
  int runtime;
  Opts() : runtime(DEFAULT_RUNTIME) {}
} opts;

class Vector3D
{
public:
  double m_x, m_y, m_z;
  Vector3D() {}
  Vector3D(double x, double y, double z) : m_x(x), m_y(y), m_z(z) {}
};

class Pose3D
{
public:
  Vector3D m_position;
  Vector3D m_orientation;
};

// actual processing stuff
OpenCvImgSeqVideo *video;
StereoCore *score;
Pose3D ground_pose;

// display stuff
int main_win, left_win, right_win, stereo_win;
double cam_trans[3];
double cam_rot[2];
int mouse_x, mouse_y;
int mouse_butt;
int butt_state;
Vector3 view_point, view_dir, view_up, view_normal;
GLfloat col_background[4];
GLfloat col_surface[4];
GLfloat col_overlay[4];
GLfloat col_highlight[4];

bool THICK_LINES = false;
bool BLACK_WHITE = false;
bool draw_bw = false;

namespace Z
{

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

/**
 * Draw an image given as RGB24 char array.
 */
void DrawImageRGB24(const char *rgb24, int width, int height)
{
  glRasterPos2i(0, 0);
  glDrawPixels(width, height, GL_BGR, GL_UNSIGNED_BYTE, rgb24);
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

}  // namespace Z


inline Vector3D operator-(const Vector3D &p)
{
  Vector3D q;
  q.m_x = -p.m_x;
  q.m_y = -p.m_y;
  q.m_z = -p.m_z;
  return q;
}

double Length(const Vector3D &v)
{
  return sqrt(v.m_x*v.m_x + v.m_y*v.m_y + v.m_z*v.m_z);
}

void Identity3x3(double M[3][3])
{
  int i, j;
  for(i = 0; i < 3; i++)
    for(j = 0; j < 3; j++)
      M[i][j] = 0.;
  M[0][0] = M[1][1] = M[2][2] = 1.;
}

void RotationAxisAngleToMatrix(const Vector3D &r, double R[3][3])
{
  double th = Length(r);
  if(th != 0)
  {
    double x = r.m_x/th, y = r.m_y/th, z = r.m_z/th;
    double co = cos(th), si = sin(th);
    R[0][0] = x*x*(1. - co) + co;
    R[0][1] = x*y*(1. - co) - z*si;
    R[0][2] = x*z*(1. - co) + y*si;
    R[1][0] = x*y*(1. - co) + z*si;
    R[1][1] = y*y*(1. - co) + co;
    R[1][2] = y*z*(1. - co) - x*si;
    R[2][0] = x*z*(1. - co) - y*si;
    R[2][1] = y*z*(1. - co) + x*si;
    R[2][2] = z*z*(1. - co) + co;
  }
  else
  {
    Identity3x3(R);
  }
}

Vector3D Rotate(const Vector3D &r, const Vector3D &p)
{
  double R[3][3];
  Vector3D q;
  RotationAxisAngleToMatrix(r, R);
  q.m_x = R[0][0]*p.m_x + R[0][1]*p.m_y + R[0][2]*p.m_z;
  q.m_y = R[1][0]*p.m_x + R[1][1]*p.m_y + R[1][2]*p.m_z;
  q.m_z = R[2][0]*p.m_x + R[2][1]*p.m_y + R[2][2]*p.m_z;
  return q;
}

Vector3D Translate(const Vector3D &t, const Vector3D p)
{
  Vector3D v;
  v.m_x = p.m_x + t.m_x;
  v.m_y = p.m_y + t.m_y;
  v.m_z = p.m_z + t.m_z;
  return v;
}

/**
 * w = R o + t
 * o = R^T w - R^T t
 */
Pose3D InvertPose3D(const Pose3D &pose)
{
  Pose3D inv;
  inv.m_orientation = -pose.m_orientation;
  inv.m_position = -Rotate(inv.m_orientation, pose.m_position);
  return inv;
}

Vector3D TransformPointToGlobal(const Pose3D &pose, const Vector3D &p)
{
  return Translate(pose.m_position, Rotate(pose.m_orientation, p));
}

istream& operator>>(istream &is, Vector3D &v)
{
  return is >> v.m_x >> v.m_y >> v.m_z;
}

void ReadPose(const string &filename, Pose3D &pose)
{
  ifstream file(filename.c_str());
  file >> pose.m_position >> pose.m_orientation;
}

void DrawZAxis(char axis)
{
  static const double l = 0.20;

  glBegin(GL_LINES);
  glVertex3d(0., 0., 0.);
  glVertex3d(0., 0., l);
  glEnd();
  glRasterPos3d(0., 0., 1.2*l);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, axis);
}

void DrawCoordFrame()
{
  glColor4fv(col_overlay);
  glPushMatrix();
  DrawZAxis('z');
  glRotated(-90, 1., 0., 0.);
  DrawZAxis('y');
  glRotated(90, 0., 1., 0.);
  DrawZAxis('x');
  glPopMatrix();
}

void DrawWorldGrid()
{
  const double d = 1.0;
  const int nx = 4, ny = 4, nz = 4;

  glColor4f(col_overlay[0]/8.0, col_overlay[1]/8.0, col_overlay[2]/8.0, 1.0);
  glPushMatrix();
  glTranslated(-d*(double)nx/2.0, -d*(double)ny/2.0, -d*(double)nz/2.0);
  for(int i = -nx/2; i <= nx/2; i++)
  {
    glTranslated(d, 0.0, 0.0);
    glPushMatrix();
    for(int j = -ny/2; j <= ny/2; j++)
    {
      glTranslated(0.0, d, 0.0);
      glPushMatrix();
      for(int k = -nz/2; k <= nz/2; k++)
      {
        glTranslated(0.0, 0.0, d);
        glutWireCube(d);
      }
      glPopMatrix();
    }
    glPopMatrix();
  }
  glPopMatrix();

  // HACK: draw ground
  {
    const double GROUND_GRID_SIZE = 0.150;
    const int GROUND_NUMDOTS_X = 5;
    const int GROUND_NUMDOTS_Y = 5;
    const double GROUND_HEIGHT = 0.000;
    
    double grid_size = GROUND_GRID_SIZE;
    int nx = GROUND_NUMDOTS_X;
    int ny = GROUND_NUMDOTS_Y;
    double z = GROUND_HEIGHT;

    glNormal3d(0., 0., 1.);
    glLineWidth(1.f);
    glColor3d(1.0, 1.0, 0.0); // yellow
    glBegin(GL_LINES);
    // draw lines parallel y-axis
    for(int i = 0; i < nx; i++)
    {
      Vector3D p = TransformPointToGlobal(ground_pose,
          Vector3D(i*grid_size, 0, z));
      Vector3D q = TransformPointToGlobal(ground_pose,
          Vector3D(i*grid_size, (ny-1)*grid_size, z));
      glVertex3d(p.m_x, p.m_y, p.m_z);
      glVertex3d(q.m_x, q.m_y, q.m_z);
    }
    // draw lines parallel x-axis
    for(int i = 0; i < ny; i++)
    {
      Vector3D p = TransformPointToGlobal(ground_pose,
          Vector3D(0, i*grid_size, z));
      Vector3D q = TransformPointToGlobal(ground_pose,
          Vector3D((nx-1)*grid_size, i*grid_size, z));
      glVertex3d(p.m_x, p.m_y, p.m_z);
      glVertex3d(q.m_x, q.m_y, q.m_z);
    }
    glEnd();
  }
  // HACK END
}

void DrawOverlays()
{
  //DrawCoordFrame();
  DrawWorldGrid();
}

void DrawStereoSurface(const Surface *surf)
{
  double l = 0.05;  // length of normal vector for drawing in [m]

  glBegin(GL_POLYGON);
  for(unsigned i = 0; i < surf->vertices.Size(); i++)
  {
    glNormal3d(surf->vertices[i].normal.x, surf->vertices[i].normal.y,
      surf->vertices[i].normal.z);
    glVertex3d(surf->vertices[i].pos.x, surf->vertices[i].pos.y,
      surf->vertices[i].pos.z);
  }
  glEnd();
  glBegin(GL_LINES);
  for(unsigned i = 0; i < surf->vertices.Size(); i++)
  {
    glVertex3d(surf->vertices[i].pos.x, surf->vertices[i].pos.y,
      surf->vertices[i].pos.z);
    glVertex3d(
      surf->vertices[i].pos.x + l*surf->vertices[i].normal.x,
      surf->vertices[i].pos.y + l*surf->vertices[i].normal.y,
      surf->vertices[i].pos.z + l*surf->vertices[i].normal.z);
  }
  glEnd();
}

void DrawStereoSurfaces()
{
  int num_surfs = score->NumSurfaces();
  for(int i = 0; i < num_surfs; i++)
  {
    //if((int)i == selected_surface)
    //  glColor4fv(col_highlight);
    //else
      glColor4fv(col_surface);
    DrawStereoSurface(score->Surfaces(i));
  }
}

void DrawObjects()
{
  DrawStereoSurfaces();
}

void displayStereoWin()
{
  GLfloat light_position[] = {2.0, -2.0, 1.0, 1.0};

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  /*glTranslated(cam_trans[0], cam_trans[1], cam_trans[2]);
  gluLookAt(1.8, 1.8, 1.8,  0., 0., 0.,  0., 0., 1.);
  glRotated(cam_rot[0], 0., 0., 1.);*/
  //glTranslated(cam_trans[0], cam_trans[1], cam_trans[2]);
  //gluLookAt(0., 0., -dist,  0., 0., 0.,  0., -1., 0.);
  //glRotated(180., 1., 0., 0);
  //glRotated(-cam_rot[0], 0., 1., 0.);
  // move so that y-axis around which we rotate is in center of image
  //glTranslated(-(double)score->ImageWidth()/2.,
  //    -(double)score->ImageHeight()/2., 0.);
  gluLookAt(view_point.x, view_point.y, view_point.z,
      view_point.x + view_dir.x, view_point.y + view_dir.y,
      view_point.z + view_dir.z,
      view_up.x, view_up.y, view_up.z);
  glRotated(cam_rot[0], 0., 1., 0.);
  glRotated(-cam_rot[1], 1., 0., 0.);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glDisable(GL_LIGHTING);
  DrawOverlays();
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  DrawObjects();
  glDisable(GL_COLOR_MATERIAL);

  glutSwapBuffers();
}

void mouseMoveStereoWin(int x, int y)
{
  double trans_scale = 0.005, rot_scale = 1.;
  double delta_x = (double)(x - mouse_x);
  double delta_y = (double)(y - mouse_y);

  if(mouse_butt == GLUT_LEFT_BUTTON)
  {
    view_point += (view_up*delta_y - view_normal*delta_x)*trans_scale;
  }
  else if(mouse_butt == GLUT_MIDDLE_BUTTON)
  {
    view_point -= view_dir*delta_y*trans_scale;
  }
  else if(mouse_butt == GLUT_RIGHT_BUTTON)
  {
    cam_rot[0] += (GLfloat)delta_x/rot_scale;
    cam_rot[1] += (GLfloat)delta_y/rot_scale;
  }
  mouse_x = x;
  mouse_y = y;
  glutSetWindow(stereo_win);
  glutPostRedisplay();
}

void mousePressStereoWin(int button, int state, int x, int y)
{
  mouse_x = x;
  mouse_y = y;
  mouse_butt = button;
  butt_state = state;
}

void initStereoWin()
{
  GLfloat light_ambient[] = {0.4, 0.4, 0.4, 1.0};
  GLfloat light_diffuse[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat light_specular[] = {0.0, 0.0, 0.0, 1.0};
  //GLfloat mat_specular[] = {0.5, 0.5, 0.5, 1.0};
  //GLfloat mat_shininess[] = {100.0};

  glClearColor(col_background[0], col_background[1], col_background[2],
      col_background[3]);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glShadeModel(GL_SMOOTH);

  // setup lighting
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_ambient);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
  //glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  //glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  // setup view point stuff
  cam_trans[0] = cam_trans[1] = cam_trans[2] = 0.;
  cam_rot[0] = cam_rot[1] = 0.;
  mouse_x = mouse_y = 0;
  mouse_butt = 0;
  butt_state = 0;

  // look in z direcction with y pointing downwards
  view_point = Vector3(0.0, 0.0, 0.0);
  view_dir = Vector3(0.0, 0.0, 1.0);
  view_up = Vector3(0.0, -1.0, 0.0);
  view_normal = Cross(view_dir, view_up);
 
  // black background
  col_background[0] = 0.0;
  col_background[1] = 0.0;
  col_background[2] = 0.0;
  col_background[3] = 1.0;

  // surfaces in white
  col_surface[0] = 1.0;
  col_surface[1] = 1.0;
  col_surface[2] = 1.0;
  col_surface[3] = 1.0;

  // highlighted things in light blue
  col_highlight[0] = 0.2;
  col_highlight[1] = 0.2;
  col_highlight[2] = 1.0;
  col_highlight[3] = 1.0;

  // overlay thingies (e.g. coordinate axes) in yellow
  col_overlay[0] = 1.0;
  col_overlay[1] = 1.0;
  col_overlay[2] = 0.0;
  col_overlay[3] = 1.0;
}

void resizeStereoWin(int w, int h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45., (double)w/(double)h, 0.001, 10000.);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void DrawMonoSurface(const TmpSurf &surf)
{
  glBegin(GL_LINE_LOOP);
  for(unsigned i = 0; i < surf.p.size(); i++)
    glVertex2d(surf.p[i].x, surf.p[i].y);
  glEnd();
}

void DrawMonoSurfaces(int side)
{
  int num_surfs = score->NumSurfaces2D(side);
  for(int i = 0; i < num_surfs; i++)
  {
    if(i >= score->NumMatches())
    {
      glColor3d(1.0, 0.0, 0.0);
      DrawMonoSurface(score->Surfaces2D(side, i));
    }
  }
  for(int i = 0; i < num_surfs; i++)
  {
    if(i < score->NumMatches())
    {
      //if(i == selected_surface)
      //  glColor3d(0.0, 0.0, 1.0);
      //else
        glColor3d(0.0, 1.0, 0.0);
      DrawMonoSurface(score->Surfaces2D(side, i));
    }
  }
}

void initMonoWin(int side)
{
  glDisable(GL_DEPTH_TEST);
  glClearColor(0.0, 0.0, 0.0, 1.0);
}

void initLeftWin()
{
  initMonoWin(LEFT);
}

void initRightWin()
{
  initMonoWin(RIGHT);
}

void resizeMonoWin(int side, int w, int h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // y points down
  glScaled(1., -1., 1);
  gluOrtho2D(0, score->ImageWidth(), 0, score->ImageHeight());
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  // y points down
  glPixelZoom((double)w/(double)score->ImageWidth(),
      -(double)h/(double)score->ImageHeight());
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void resizeLeftWin(int w, int h)
{
  resizeMonoWin(LEFT, w, h);
}

void resizeRightWin(int w, int h)
{
  resizeMonoWin(RIGHT, w, h);
}

void displayMonoWin(int side)
{
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glClear(GL_COLOR_BUFFER_BIT);

  if(score->ImageRGB24(side) != 0)
  {
    DrawImageRGB24(score->ImageRGB24(side), score->ImageWidth(),
        score->ImageHeight());
  }
  else
  {
    glRasterPos2d(score->ImageWidth()/2.0, score->ImageHeight()/2.0);
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, 'N');
  }

  DrawMonoSurfaces(side);

  glutSwapBuffers();
}

void displayLeftWin()
{
  displayMonoWin(LEFT);
}

void displayRightWin()
{
  displayMonoWin(RIGHT);
}

void keyPressAllWin(unsigned char key, int x, int y)
{
  switch(key)
  {
    case 'p':  // process current image
    {
      score->ProcessStereoImage2(opts.runtime);
      break;
    }
    case 'n':  // next image
    {
      // advance if possible
      if(!score->NextImage())
        // if at end, rewind
        score->FirstImage();
      break;
    }
    default:
      break;
  }
  glutSetWindow(left_win);
  glutPostRedisplay();
  glutSetWindow(right_win);
  glutPostRedisplay();
  glutSetWindow(stereo_win);
  glutPostRedisplay();
}

void PrintUsage(const char *argv0)
{
  cout << argv0 <<
    " <stereo config file> <ground pose file> <left img> <right img>\n";
}

int main(int argc, char **argv)
{
  // the main window of size 2w x 3h is devided into 3 subwindows, w and h are
  // some basic width, height units (e.g. the video image width and height or a
  // fraction thereof)
  // top left sub win:  w x h
  // top right sub win: w x h
  // bottom sub win:    2 w x 2 h
  int base_width, base_height;

  glutInit(&argc, argv);

  if(argc < 5)
  {
    PrintUsage(argv[0]);
    exit(EXIT_FAILURE);
  }

  ReadPose(argv[2], ground_pose);
  ground_pose = InvertPose3D(ground_pose);

  // the rest of the arguments are left/right pairs
  vector<string> image_files;
  for(int i = 3; i < argc; )
  {
    image_files.push_back(argv[i++]);
    image_files.push_back(argv[i++]);
  }

  video = new OpenCvImgSeqVideo(image_files);
  score = new StereoCore(video, argv[1]);
  score->FirstImage();

  base_width = score->ImageWidth()/2;
  base_height = score->ImageHeight()/2;

  glutInitWindowSize(2*base_width, 3*base_height);
  main_win = glutCreateWindow("glstereoflaps");
  glutKeyboardFunc(keyPressAllWin);

  left_win = glutCreateSubWindow(main_win, 0, 0, base_width, base_height);
  initLeftWin();
  glutKeyboardFunc(keyPressAllWin);
  glutReshapeFunc(resizeLeftWin);
  glutDisplayFunc(displayLeftWin);

  right_win = glutCreateSubWindow(main_win, base_width, 0,
      base_width, base_height);
  initRightWin();
  glutKeyboardFunc(keyPressAllWin);
  glutReshapeFunc(resizeRightWin);
  glutDisplayFunc(displayRightWin);

  stereo_win = glutCreateSubWindow(main_win, 0, base_height,
      2*base_width, 2*base_height);
  initStereoWin();
  glutKeyboardFunc(keyPressAllWin);
  glutMouseFunc(mousePressStereoWin);
  glutMotionFunc(mouseMoveStereoWin);
  glutReshapeFunc(resizeStereoWin);
  glutDisplayFunc(displayStereoWin);

  glutMainLoop();

  // note that glutMainLoop() actually never exits ...
  delete score;
  glutDestroyWindow(main_win);
  exit(EXIT_SUCCESS);
}

