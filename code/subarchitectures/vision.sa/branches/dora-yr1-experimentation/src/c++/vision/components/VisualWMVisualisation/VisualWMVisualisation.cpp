/**
 * @author Michael Zillich
 * @date February 2009
 */

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#endif

#include <ChangeFilterFactory.hpp>
#include "VisualWMVisualisation.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::VisualWMVisualisation();
  }
}

namespace cast
{

using namespace std;

static float white[4] = {1., 1., 1., 1.};
static float black[4] = {0., 0., 0., 1.};
static float grey[4] = {0.6, 0.6, 0.6, 1.};
static float yellow[4] = {1., 1., 0., 1.};
// overlay thingies (e.g. coordinate axes) in yellow
static float col_overlay[4] = {1.0, 1.0, 0.0, 1.0};

int glut_window = -1;
int mouse_x = -1;
int mouse_y = -1;
int mouse_butt = -1;
double cam_trans[3] = {0., 0., 0.};
double cam_rot[2] = {0., 0.};


VisualWMVisualisation::VisualWMVisualisation()
{
  // some useful default for window size
  windowWidth = 400;
  windowHeight = 400;
}

VisualWMVisualisation::~VisualWMVisualisation()
{
  if(glut_window != -1)
    glutDestroyWindow(glut_window);
}

void VisualWMVisualisation::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  if((it = _config.find("--windowWidth")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> windowWidth;
  }
  if((it = _config.find("--windowHeight")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> windowHeight;
  }
}

void VisualWMVisualisation::start()
{
  // we want to receive added, overwritten and deleted visual objects
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<VisualWMVisualisation>(this,
        &VisualWMVisualisation::newVisualObject));
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<VisualWMVisualisation>(this,
        &VisualWMVisualisation::changedVisualObject));
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::DELETE),
      new MemberFunctionChangeReceiver<VisualWMVisualisation>(this,
        &VisualWMVisualisation::deletedVisualObject));

  initGL();
}

void VisualWMVisualisation::runComponent()
{
  while(isRunning())
  {
    sleepComponent(100);
#ifdef __APPLE__
    glutMainLoop();
#else
    glutMainLoopEvent();
#endif 

  }
}

void VisualWMVisualisation::newVisualObject(
    const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::VisualObjectPtr obj =
    getMemoryEntry<VisionData::VisualObject>(_wmc.address);
  objects[_wmc.address.id] = obj;
}

void VisualWMVisualisation::changedVisualObject(
    const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::VisualObjectPtr obj =
    getMemoryEntry<VisionData::VisualObject>(_wmc.address);
  objects[_wmc.address.id] = obj;
}

void VisualWMVisualisation::deletedVisualObject(
    const cdl::WorkingMemoryChange & _wmc)
{
  objects.erase(_wmc.address.id);
}

void VisualWMVisualisation::updateDisplay()
{
  for(ObjectMap::iterator i = objects.begin(); i != objects.end(); i++)
  {
    println("object '%s' conf %f", i->second->label.c_str(),
        i->second->detectionConfidence);
  }
}

static VisualWMVisualisation *vis;

static void redrawGL_Helper()
{
  vis->redrawGL();
}

static void resizeGL_Helper(int width, int height)
{
  vis->resizeGL(width, height);
}

static void mousePressed_Helper(int button, int state, int x, int y)
{
  vis->mousePressed(button, state, x, y);
}

static void mouseMoved_Helper(int x, int y)
{
  vis->mouseMoved(x, y);
}

void VisualWMVisualisation::initGL()
{
  int argc = 1;
  char *argv[1] = {""};

  vis = this;

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_ALPHA);
  glutInitWindowSize(windowWidth, windowHeight);
  //glutInitWindowPosition(0, 0);
  glut_window = glutCreateWindow("Visual Working Memory");
  glutDisplayFunc(redrawGL_Helper);
  glutReshapeFunc(resizeGL_Helper);
  glutMouseFunc(mousePressed_Helper);
  glutMotionFunc(mouseMoved_Helper);

  float light_pos[4] = {1., 1., 0., 1.};  // light top right
  glClearColor(0., 0., 0., 0.);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glLightfv(GL_LIGHT0, GL_AMBIENT, grey);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
  glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
}

void VisualWMVisualisation::resizeGL(int width, int height)
{
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // viewing frustrum, note all values are in m.
  gluPerspective(60., (double)width/(double)height, 0.001, 100.);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void VisualWMVisualisation::redrawGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslated(cam_trans[0], cam_trans[1], cam_trans[2]);
  gluLookAt(1.8, 1.8, 1.8,  0., 0., 0.,  0., 0., 1.);
  glRotated(cam_rot[0], 0., 0., 1.);
  glRotated(cam_rot[1], 0., 1., 0.);
  drawCoordFrame();
  drawWorldGrid();
  //drawCameras();
  drawVisualObjects();
  glutSwapBuffers();
}

void VisualWMVisualisation::drawZAxis(char axis)
{
  static const double l = 0.20;

  glBegin(GL_LINES);
  glVertex3d(0., 0., 0.);
  glVertex3d(0., 0., l);
  glEnd();
  glRasterPos3d(0., 0., 1.2*l);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, axis);
}

void VisualWMVisualisation::drawCoordFrame()
{
  glPushMatrix();
  glColor4f(0., 0., 1., 1.);  // z = BLUE
  drawZAxis('z');
  glRotated(-90, 1., 0., 0.);
  glColor4f(0., 1., 0., 1.);  // y = GREEN
  drawZAxis('y');
  glRotated(90, 0., 1., 0.);
  glColor4f(1., 0., 0., 1.);  // x = RED
  drawZAxis('x');
  glPopMatrix();
}

void VisualWMVisualisation::drawWorldGrid()
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

void VisualWMVisualisation::drawVisualObject(const VisionData::VisualObjectPtr obj)
{
  glPushMatrix();
  glTranslated(obj->boundingSphere.pos.x,
               obj->boundingSphere.pos.y,
               obj->boundingSphere.pos.z);
  glutWireSphere(obj->boundingSphere.rad, 12, 8);
  glPopMatrix();
}

void VisualWMVisualisation::drawVisualObjects()
{
  for(ObjectMap::iterator i = objects.begin(); i != objects.end(); i++)
  {
    drawVisualObject(i->second);
  }
}

void VisualWMVisualisation::mousePressed(int button, int state, int x, int y)
{
  mouse_x = x;
  mouse_y = y;
  if(state == GLUT_DOWN)
    mouse_butt = button;
  else
    mouse_butt = -1;
}

void VisualWMVisualisation::mouseMoved(int x, int y)
{
  double trans_scale = 200., rot_scale = 1.;
  int delta_x = x - mouse_x;
  int delta_y = y - mouse_y;

  if(mouse_butt == GLUT_LEFT_BUTTON)
  {
    cam_trans[0] += ((GLfloat)delta_x)/trans_scale;
    cam_trans[1] -= ((GLfloat)delta_y)/trans_scale;
  }
  else if(mouse_butt == GLUT_MIDDLE_BUTTON)
  {
    cam_trans[2] -= ((GLfloat)delta_y)/trans_scale;
  }
  else if(mouse_butt == GLUT_RIGHT_BUTTON)
  {
    cam_rot[0] += (GLfloat)delta_x/rot_scale;
    cam_rot[1] += (GLfloat)delta_y/rot_scale;
  }
  mouse_x = x;
  mouse_y = y;
  redrawGL();
}

}

