/**
 * A somewhat less hacky glut based version of stereoflaps.
 * Uses ObjectTracker class.
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <list>
#include <string>
#include <GL/glut.h>
#include <opencv/highgui.h>
#include "ObjectTracker.hh"
#include "Draw.hh"

using namespace Z;


class Options
{
public:
  VideoType vid_type;
  string img_template[2];
  int cap_device_class;
  string stereoconfig_file;
  int imgcnt_start;
  int imgcnt_stop;
  int runtime;
  Options()
  {
    runtime = 0;
    vid_type = VIDEO_TYPE_IMG_SEQ;
    cap_device_class = CV_CAP_ANY;
    imgcnt_start = imgcnt_stop = 0;
    img_template[LEFT] = img_template[RIGHT] = "";
  }
  void Parse(int argc, char **argv);
} opts;

ObjectTracker *tracker;

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
bool log_images = false;
int log_image_cnt = 0;
const char *log_image_basename_L = "img%03d-L.jpg";
const char *log_image_basename_R = "img%03d-R.jpg";

static void PrintUsage(const char *argv0);
static void PrintKeys();
static void ProcessImage();
static void GrabAndProcessImage();
static void LogCurrentFrame();

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

void SaveDrawArea(const string &filename)
{
  int dims[4];
  int pad;

  // get viewport x, y, width, height
  glGetIntegerv(GL_VIEWPORT, dims);
  IplImage *img = cvCreateImage(cvSize(dims[2], dims[3]), IPL_DEPTH_8U, 3);
  SetDrawImagesSolid();
  // check what pixel alignment we need
  pad = img->widthStep - img->width*img->nChannels;
  if(pad < 4)
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
  else if(pad < 8)
    glPixelStorei(GL_UNPACK_ALIGNMENT, 8);
  else
    throw Except(__HERE__,
        "failed ot save image, can not handle image alignement");
  glReadPixels(0, 0, img->width, img->height, GL_RGB, GL_UNSIGNED_BYTE,
      img->imageData);
  cvSaveImage(filename.c_str(), img);
  cvReleaseImage(&img);
}

}  // namespace Z

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
  glPushMatrix();
  glColor4f(0., 0., 1., 1.);  // z = BLUE
  DrawZAxis('z');
  glRotated(-90, 1., 0., 0.);
  glColor4f(0., 1., 0., 1.);  // y = GREEN
  DrawZAxis('y');
  glRotated(90, 0., 1., 0.);
  glColor4f(1., 0., 0., 1.);  // x = RED
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
}

void DrawOverlays()
{
  DrawCoordFrame();
  DrawWorldGrid();
}

void DrawSurface(const Surface &surf)
{
  double l = 0.05;  // length of normal vector for drawing in [m]
  Vector3 c(0., 0., 0.);  // center of surface

  // draw surface
  glBegin(GL_POLYGON);
  for(unsigned i = 0; i < surf.vertices.size(); i++)
  {
    Vector3 pos = surf.vertices[i].pos;
    Vector3 normal = surf.vertices[i].normal;
    glNormal3d(normal.x, normal.y, normal.z);
    glVertex3d(pos.x, pos.y, pos.z);
    c += pos;
  }
  glEnd();

  // draw the normal vectors
  /*c /= (double)surf.vertices.size();
  glBegin(GL_LINES);
  glVertex3d(c.x, c.y, c.z);
  glVertex3d(c.x + l*surf.normal.x,
             c.y + l*surf.normal.y,
             c.z + l*surf.normal.z);
  glEnd();*/
}

void DrawObject(const Object &obj)
{
  double M[4][4];
  obj.pose.ToMat44(M);
  TransposeMat44(M);
  glPushMatrix();
  glMultMatrixd((GLdouble*)M);
  DrawCoordFrame();
  for(size_t i = 0; i < obj.surfs.size(); i++)
  {
    glColor4fv(col_surface);
    DrawSurface(obj.surfs[i]);
  }
  glPopMatrix();
}

void DrawObjects()
{
  for(size_t i = 0; i < tracker->GetObjects().size(); i++)
    DrawObject(tracker->GetObjects()[i]);
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
  glEnable(GL_CULL_FACE);
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
  // assuming that left and right image are of course of the same size
  gluOrtho2D(0, tracker->GetLeftImage()->width,
      0, tracker->GetLeftImage()->height);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  // y points down
  glPixelZoom((double)w/(double)tracker->GetLeftImage()->width,
      -(double)h/(double)tracker->GetLeftImage()->height);
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

  const IplImage *img =
    (side == LEFT ? tracker->GetLeftImage() : tracker->GetRightImage());
  DrawImageRGB24(img->imageData, img->width, img->height);

  if(side == LEFT)
  {
    char buf[1000];
    snprintf(buf, 1000, "runtime per frame: %d ms",
        tracker->GetPerFrameRuntime());
    DrawText2D(buf, 20, 20, RGBColor::red);
    snprintf(buf, 1000, "have %d objects", (int)tracker->GetObjects().size());
    DrawText2D(buf, 20, 60, RGBColor::red);
  }

  if(side == LEFT)
    tracker->DebugDrawLeftImage();
  else
    tracker->DebugDrawRightImage();
  char name[100];
  snprintf(name, 100, "debug-%c.png", side == LEFT ? 'L' : 'R');
  Z::SaveDrawArea(name);

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
      ProcessImage();
      break;
    }
    case 'n':  // next image pair
    {
      tracker->CaptureFrame();
      if(log_images)
        LogCurrentFrame();
      break;
    }
    case 'r':
    {
      tracker->Clear();
      break;
    }
    case '+':
    {
      tracker->SetPerFrameRuntime(tracker->GetPerFrameRuntime() + 50);
      break;
    }
    case '-':
    {
      tracker->SetPerFrameRuntime(tracker->GetPerFrameRuntime() - 50);
      break;
    }
    case 'l':
    {
      log_images = !log_images;
      cout << "logging is turned " << (log_images ? "on" : "off") << endl;
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

void ProcessImage()
{
  tracker->ProcessFrame();

  cout << "number of objects: " << tracker->GetObjects().size() << endl;
  for(size_t i=0; i < tracker->GetObjects().size(); i++)
  {
    const Object &o = tracker->GetObjects()[i];
    cout << "  object " << i << ":" << endl;
    cout << "  pose (position/rotation vector): " << o.pose << endl;
    for(size_t j = 0; j < o.surfs.size(); j++)
    {
      cout << "    surface " << j << ":" << endl;
      for(size_t k = 0; k < o.surfs[j].vertices.size(); k++)
      {
        cout << "      vertex " << k << ":" << endl;
        cout << " in object coordinates:";
        cout << " position: " << o.surfs[j].vertices[k].pos;
        cout << " normal: " << o.surfs[j].vertices[k].normal << endl;
        cout << " in world coordinates:";
        cout << " position: " << o.pose.Transform(o.surfs[j].vertices[k].pos);
        cout << " normal: " << o.pose.TransformDir(o.surfs[j].vertices[k].normal) << endl;
      }
      cout << "    normal for surface in object coordinates: " <<
        o.surfs[j].normal << endl;
      cout << "    normal for surface in world coordinates: " <<
        o.pose.TransformDir(o.surfs[j].normal) << endl;
    }
    cout << endl;
  }
  cout << endl;
}

void GrabAndProcessImage()
{
  tracker->CaptureFrame();
  if(log_images)
    LogCurrentFrame();
  ProcessImage();

  glutSetWindow(left_win);
  glutPostRedisplay();
  glutSetWindow(right_win);
  glutPostRedisplay();
  glutSetWindow(stereo_win);
  glutPostRedisplay();
}

void LogCurrentFrame()
{
  char name[1000];
  snprintf(name, 1000, log_image_basename_L, log_image_cnt);
  cvSaveImage(name, tracker->GetLeftImage());
  snprintf(name, 1000, log_image_basename_R, log_image_cnt);
  cvSaveImage(name, tracker->GetRightImage());
  log_image_cnt++;
}

/**
 * Parse command line options.
 */
void Options::Parse(int argc, char **argv)
{
  bool ok = true;
  int c;
  while(1)
  {
    c = getopt(argc, argv, "c:r:p:fvh");
    if(c == -1)
      break;
    switch(c)
    {
      case 'h':
        PrintUsage(argv[0]);
        break;
      case 'c':
        if(optarg)
          stereoconfig_file = optarg;
        else
        {
          cerr << "-c: missing argument\n";
          ok = false;
        }
        break;
      case 'r':
        if(optarg)
          runtime = atoi(optarg);
        else
        {
          cerr << "-r: missing argument\n";
          ok = false;
        }
        break;
      case 'v':
        vid_type = VIDEO_TYPE_LIVE;
        if(cap_device_class == CV_CAP_ANY)
          cap_device_class = CV_CAP_V4L2;
        else
        {
          cerr << "device class can be set to either V4L2 or IEEE1394\ni";
          ok = false;
        }
        break;
      case 'f':
        vid_type = VIDEO_TYPE_LIVE;
        if(cap_device_class == CV_CAP_ANY)
          cap_device_class = CV_CAP_IEEE1394;
        else
        {
          cout << "device class can be set to either V4L2 or IEEE1394\n";
          ok = false;
        }
        break;
      default:
        cerr << "getopt returned character code " << c << endl;
    }
  }
  // remaining arguments are left and right image template filname, start and
  // stop image number
  if(argc - optind == 4)
  {
    img_template[LEFT] = argv[optind++];
    img_template[RIGHT] = argv[optind++];
    imgcnt_start = atoi(argv[optind++]);
    if(imgcnt_start < 0)
    {
      cerr << "invalid image start number: " << imgcnt_start <<
        " must be >= 0" << endl;
      ok = false;
    }
    imgcnt_stop = atoi(argv[optind++]);
    if(imgcnt_stop < 0)
    {
      cerr << "invalid image stop number: " << imgcnt_stop <<
        " must be >= " << imgcnt_start << endl;
    }
  }

  // check if combination of options is valid
  if(vid_type == VIDEO_TYPE_IMG_SEQ)
    if(img_template[LEFT].empty() || img_template[RIGHT].empty())
    {
      cerr << "Need left and right image templates when working from"
       " stored image sequences. Use -v or -f for live video.\n";
      ok = false;
    }
  if(!ok)
    exit(EXIT_FAILURE);
}

void PrintUsage(const char *argv0)
{
  cout << "usage: " << argv0 <<
  " [-h] [-v | -f] [-r <runtime>] -c <stereo config file> "
  "<left img template> <right img template> <start number> <stop number>\n"
  " -h .. this help\n"
  " -v .. use live V4L2 video (i.e. typically USB cameras) instead of stored"
  "       image sequences\n"
  " -f .. use live firewire (IEEE1394) video instead of stored image sequences\n"
  " -r <runtime> .. runtime for processing each stereo frame, default is "
  << 500 << " ms\n" <<
  " -c <stereo config file> .. SVS-like stereo configuration file, mandatory\n"
  " <left/right img template> .. e.g. img-L-%03d.jpg to load\n"
  "   img-L-001.jpg .. img-L-099.jpg\n"
  " <start/stop number> .. start and stop image number e.g. 1 and 99\n";
}

void PrintKeys()
{
  cout << "Hot keys:\n"
    "  p - process current image pair\n"
    "  n - capture next image pair (only if working from stored image\n"
    "      seqences\n"
    "  r - reset tracking\n"
    "  +/- - increase/decrease runtime by 50 ms\n"
    "  l - turn logging of images on/off, images will be written to\n"
    "      img000-L.jpg, img000-R.jpg ... img999-R.jpg in the current directory\n";
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

  opts.Parse(argc, argv);

  PrintKeys();

  if(opts.vid_type == Z::VIDEO_TYPE_LIVE)
  {
#ifdef HAVE_VIDERE
    tracker = new ObjectTracker(opts.stereoconfig_file);
#else
    tracker = new ObjectTracker(opts.stereoconfig_file,
            0, 1, opts.cap_device_class);
#endif
  }
  else if(opts.vid_type == Z::VIDEO_TYPE_IMG_SEQ)
  {
    tracker = new ObjectTracker(opts.stereoconfig_file,
        opts.img_template[0], opts.img_template[1],
        opts.imgcnt_start, opts.imgcnt_stop);
  }
  if(opts.runtime > 0)
    tracker->SetPerFrameRuntime(opts.runtime);

  base_width = tracker->GetLeftImage()->width/2;
  base_height = tracker->GetRightImage()->height/2;

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
  if(tracker->HaveLiveVideo())
    glutIdleFunc(GrabAndProcessImage);

  glutMainLoop();

  // note that glutMainLoop() actually never exits ...
  glutDestroyWindow(main_win);
  exit(EXIT_SUCCESS);
}


