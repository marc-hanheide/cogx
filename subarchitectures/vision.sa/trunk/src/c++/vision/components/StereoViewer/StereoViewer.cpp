/**
 * @author Michael Zillich
 * @date June 2009
 */

#include <limits.h>
#include <GL/freeglut.h>
#include <opencv/highgui.h>
#include <cogxmath.h>
#include <VideoUtils.h>
#include "StereoViewer.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::StereoViewer();
  }
}

namespace cast
{

using namespace std;
using namespace Stereo;
using namespace cogx;
using namespace cogx::Math;

static int win;
static double cam_trans[3];
static double cam_rot[2];
static int mouse_x, mouse_y;
static int mouse_butt;
static int butt_state;
static Vector3 view_point, view_dir, view_up, view_normal;
static GLfloat col_background[4];
static GLfloat col_surface[4];
static GLfloat col_overlay[4];
static GLfloat col_highlight[4];
static vector<Vector3> points;
static size_t selected_point = UINT_MAX;
static Vector3 img_cam_pos;

static void InitWin()
{
  GLfloat light_ambient[] = {0.4, 0.4, 0.4, 1.0};
  GLfloat light_diffuse[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat light_specular[] = {0.0, 0.0, 0.0, 1.0};

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
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  // setup view point stuff
  cam_trans[0] = cam_trans[1] = cam_trans[2] = 0.;
  cam_rot[0] = cam_rot[1] = 0.;
  mouse_x = mouse_y = 0;
  mouse_butt = 0;
  butt_state = 0;

  // look in z direcction with y pointing downwards
  view_point = vector3(0.0, 0.0, 0.0);
  view_dir = vector3(0.0, 0.0, 1.0);
  view_up = vector3(0.0, -1.0, 0.0);
  view_normal = cross(view_dir, view_up);
 
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

static void ResizeWin(int w, int h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45., (double)w/(double)h, 0.001, 10000.);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

static void DrawText3D(const char *text, double x, double y, double z)
{
  glRasterPos3d(x, y, z);
  while(*text != '\0')
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *text++);
}

/**
 * Draw things like coord frames
 */
static void DrawOverlays()
{
  glColor4fv(col_overlay);

  // draw coordinate axes
  glBegin(GL_LINES);
  glVertex3d(-1000., 0., 0.);
  glVertex3d(1000., 0., 0.);
  glVertex3d(0., -1000., 0.);
  glVertex3d(0., 1000., 0.);
  glVertex3d(0., 0., -1000.);
  glVertex3d(0., 0., 1000.);
  glEnd();
  DrawText3D("x", 0.1, 0.02, 0.);
  DrawText3D("y", 0., 0.1, 0.02);
  DrawText3D("z", 0.02, 0., 0.1);

  // draw tics every m, up to 10 m
  const double tic_size = 0.05;
  glBegin(GL_LINES);
  for(int i = -10; i < 10; i++)
  {
    if(i != 0)
    {
      glVertex3d((double)i, 0, 0.);
      glVertex3d((double)i, tic_size, 0.);
      glVertex3d(0., (double)i, 0.);
      glVertex3d(0., (double)i, tic_size);
      glVertex3d(0., 0., (double)i);
      glVertex3d(tic_size, 0., (double)i);
    }
  }
  glEnd();
  char buf[100];
  for(int i = -10; i < 10; i++)
  {
    if(i != 0)
    {
      snprintf(buf, 100, "%d", i);
      DrawText3D(buf, (double)i, 2.*tic_size, 0.);
      DrawText3D(buf, 0., (double)i, 2.*tic_size);
      DrawText3D(buf, 2.*tic_size, 0., (double)i);
    }
  }

  // draw z=0 plane
  glBegin(GL_LINE_LOOP);
  glColor4f(0., 1., 0., 1.);
  glVertex3d(-1., -1., 0.);
  glVertex3d(1., -1., 0.);
  glVertex3d(1., 1., 0.);
  glVertex3d(-1., 1., 0.);
  glEnd();

  // draw left camera position
  glColor4f(1., 1., 1., 1.);
  glPushMatrix();
  glTranslatef(img_cam_pos.x, img_cam_pos.y, img_cam_pos.z);
  glutWireSphere(0.01, 10, 10);
  glPopMatrix();
}

static void DrawPoints()
{
  glBegin(GL_POINTS);
  for(size_t i = 0; i < points.size(); i++)
  {
    if(points[i].z > 0.05)  // above the table plane, which is a z = 0
      glColor3ub(0, 255, 0);
    else
      glColor3ub(255, 255, 255);
    glVertex3d(points[i].x, points[i].y, points[i].z);
  }
  glEnd();
  if(selected_point < points.size())
  {
    size_t i = selected_point;
    char text[100];

    glColor3ub(255, 0, 0);

    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);
    snprintf(text, 100, "%.3f", length(points[i]));
    DrawText3D(text, points[i].x, points[i].y, points[i].z);
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);

    glPushMatrix();
    glTranslatef(points[i].x, points[i].y, points[i].z);
    glutWireSphere(0.01, 10, 10);
    glPopMatrix();
  }
}

static void DisplayWin()
{
  GLfloat light_position[] = {2.0, -2.0, 1.0, 1.0};

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
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
  DrawPoints();
  glDisable(GL_COLOR_MATERIAL);

  glutSwapBuffers();
}

static void KeyPress(unsigned char key, int x, int y)
{
  switch(key)
  {
    default:
      break;
  }
  glutPostRedisplay();
}

static void MousePress(int button, int state, int x, int y)
{
  mouse_x = x;
  mouse_y = y;
  mouse_butt = button;
  butt_state = state;
}

static void MouseMove(int x, int y)
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
  glutPostRedisplay();
}

/**
 * This assume that the left optical axis is equal to the global z axis, i.e.
 * that the left camera is the coordinate origin.
 */
static void selectPointNearLeftOpticalAxis()
{
  double min_dist = HUGE;
  Vector3 o = vector3(0., 0., 0.);
  Vector3 z = vector3(0., 0., 1.);
  for(size_t i = 0; i < points.size(); i++)
  {
    double dist = distPointToLine(points[i], o, z);
    if(dist < min_dist)
    {
      min_dist = dist;
      selected_point = i;
    }
  }
}

void StereoViewer::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  // first let the base classes configure themselves
  configureVideoCommunication(_config);
  configureStereoCommunication(_config);

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream str(it->second);
    str >> camId;
  }
}

void StereoViewer::start()
{
  startVideoCommunication(*this);
  startStereoCommunication(*this);

  int argc = 1;
  char argv0[] = "StereoViewer";
  char *argv[1] = {argv0};
  glutInit(&argc, argv);
  win = glutCreateWindow("points");
  InitWin();
  glutKeyboardFunc(KeyPress);
  glutMouseFunc(MousePress);
  glutMotionFunc(MouseMove);
  glutReshapeFunc(ResizeWin);
  glutDisplayFunc(DisplayWin);

  cvNamedWindow(getComponentID().c_str(), 1);
}

void StereoViewer::runComponent()
{
  while(isRunning())
  {
    Video::Image image;
    getImage(camId, image);
    IplImage *iplImage = convertImageToIpl(image);
    img_cam_pos = image.camPars.pose.pos;

    points.resize(0);
    getPoints(points);
    selectPointNearLeftOpticalAxis();

    for(size_t i = 0; i < points.size(); i++)
    {
      //if(points[i].z > 0.05)  // above the table plane, which is a z = 0
      {
        Vector2 p = projectPoint(image.camPars, points[i]);
        distortPoint(image.camPars, p, p);
        cvCircle(iplImage, cvPoint(p.x, p.y), 1, CV_RGB(255,0,0));
      }
    }

    /* image being the right camera image, this draws a nice epipolar line
    for(int i = 1; i <= 5; i++)
    {
      Vector2 p = projectPoint(image.camPars, vector3(0., 0., 0.5 + (double)i*0.2));
      cvCircle(iplImage, cvPoint(p.x, p.y), 3, CV_RGB(0,255,0));
      distortPoint(image.camPars, p, p);
      cvCircle(iplImage, cvPoint(p.x, p.y), 1, CV_RGB(255,0,0));
    }*/

    // draw points of the calibration pattern
    {
      Vector2 p;
      p = projectPoint(image.camPars, vector3(0.000, 0.000, 0.000));
      distortPoint(image.camPars, p, p);
      cvCircle(iplImage, cvPoint(p.x, p.y), 1, CV_RGB(0,255,0));
      p = projectPoint(image.camPars, vector3(0.225, 0.000, 0.000));
      distortPoint(image.camPars, p, p);
      cvCircle(iplImage, cvPoint(p.x, p.y), 1, CV_RGB(0,255,0));
      p = projectPoint(image.camPars, vector3(0.225, 0.113, 0.000));
      distortPoint(image.camPars, p, p);
      cvCircle(iplImage, cvPoint(p.x, p.y), 1, CV_RGB(0,255,0));
      p = projectPoint(image.camPars, vector3(0.188, 0.151, 0.000));
      distortPoint(image.camPars, p, p);
      cvCircle(iplImage, cvPoint(p.x, p.y), 1, CV_RGB(0,255,0));
      p = projectPoint(image.camPars, vector3(0.000, 0.151, 0.000));
      distortPoint(image.camPars, p, p);
      cvCircle(iplImage, cvPoint(p.x, p.y), 1, CV_RGB(0,255,0));
    }

    cvShowImage(getComponentID().c_str(), iplImage);

    // needed to make the window appear
    // (an odd behaviour of OpenCV windows!)
    cvWaitKey(10);
    cvReleaseImage(&iplImage);

    glutPostRedisplay();
    glutMainLoopEvent();

    // wait a bit so we don't hog the CPU
    sleepComponent(50);
  }
}

}

