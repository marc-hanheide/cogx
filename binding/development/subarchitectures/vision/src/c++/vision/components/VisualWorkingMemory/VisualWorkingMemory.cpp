/**
 * @author Michael Zillich
 * @date September 2006
 */

#include <stdio.h>
#include <cmath>
#include <csignal>
#include <vision/utils/VisionUtils.h>
#include "VisualWorkingMemory.h"

using namespace Vision;

// redraw interval in ms
#define REDRAW_INTERVAL 1000 

static float white[4] = {1., 1., 1., 1.};
static float black[4] = {0., 0., 0., 1.};
static float grey[4] = {0.6, 0.6, 0.6, 1.};
static float yellow[4] = {1., 1., 0., 1.};
//static float dark_yellow[4] = {0.4, 0.4, 0., 1.};

// size of the ground plane, i.e. tha part of it that will be drawn
static const double GROUND_SIZE_X = 1.7;
static const double GROUND_SIZE_Y = 1.8;
static const double GROUND_GRID_SIZE = 0.2;

int VisualWMVisualisation::glut_window = -1;
int VisualWMVisualisation::mouse_x = -1;
int VisualWMVisualisation::mouse_y = -1;
int VisualWMVisualisation::mouse_butt = -1;
double VisualWMVisualisation::cam_trans[3] = {0., 0., 0.};
double VisualWMVisualisation::cam_rot[2] = {0., 0.};
GLUquadric *VisualWMVisualisation::glu_quad = 0;
pthread_t VisualWMVisualisation::gl_thread;
VisualWorkingMemory *VisualWMVisualisation::visual_wm = 0;

static void *GLThread(void *vis)
{
  ((VisualWMVisualisation*)vis)->StartGLThread();
  return NULL;
}

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new VisualWorkingMemory(_id);
  }
}


VisualWorkingMemory::VisualWorkingMemory(const string &_id)
  : 
    SubarchitectureWorkingMemory(_id)    
{

  // visual wm should broadcast to other sub-architectures
  // (manipulation SA needs to know about objects)
  setSendXarchChangeNotifications(true);

  /*VIS = NEW VisualWMVisualisation(this);
  if(vis == 0)
    printf("* failed to create visualisation\n");*/

  //m_bDebugOuput = true;
}

VisualWorkingMemory::~VisualWorkingMemory() 
{
  delete vis;
}

void VisualWorkingMemory::GetSurfaces(vector< shared_ptr< WorkingMemoryItem<Surface> > > &surfs) 
{
  lockProcess();
  getItems<Surface>(typeName<Surface>(),surfs);
  unlockProcess();
}

void VisualWorkingMemory::GetCameras(vector< shared_ptr< WorkingMemoryItem<Camera> > > &cams) 
{
  lockProcess();  
  getItems<Camera>(typeName<Camera>(),cams);
  unlockProcess();
}

void VisualWorkingMemory::GetSceneObjects(vector< shared_ptr< WorkingMemoryItem<SceneObject> > > &objs) 
{
  lockProcess();
  getItems<SceneObject>(typeName<SceneObject>(),objs);
  unlockProcess();
}



void VisualWorkingMemory::redrawGraphics2D()
{
}

void VisualWorkingMemory::redrawGraphics3D()
{
  vector< shared_ptr< WorkingMemoryItem<SceneObject> > > objs;
  GetSceneObjects(objs);
  for(unsigned i = 0; i < objs.size(); i++)
  { 
    char label[100];
    
    drawBox3D(objs[i]->getData()->m_bbox.m_centroid.m_x,
	      objs[i]->getData()->m_bbox.m_centroid.m_y,
	      objs[i]->getData()->m_bbox.m_centroid.m_z,
	      objs[i]->getData()->m_bbox.m_size.m_x,
	      objs[i]->getData()->m_bbox.m_size.m_y,
	      objs[i]->getData()->m_bbox.m_size.m_z,
	      255, 255, 255,  0);
    
    string bubbbb(objs[i]->getData()->m_label.m_string);
    
    string id(objs[i]->getID());
   
    snprintf(label, 100, "%s / %.3f", bubbbb.c_str(),
	     objs[i]->getData()->m_label.m_confidence);
    
    drawText3D(objs[i]->getData()->m_bbox.m_centroid.m_x,
	       objs[i]->getData()->m_bbox.m_centroid.m_y,
	       objs[i]->getData()->m_bbox.m_centroid.m_z,
	       string(label),
	       255, 255, 255,  0);
  }
}

void VisualWorkingMemory::redrawGraphicsText()
{
  vector<string> ids;
  m_workingMemory.getIDsByType(typeName<SceneObject>(), 0, ids);
  printText("%d scene objects\n", ids.size());
  for(unsigned i = 0; i < ids.size(); i++) {
    printText("%s\n", ids[i].c_str());
  }
}

VisualWMVisualisation::VisualWMVisualisation(VisualWorkingMemory *wm)
{
  visual_wm = wm;
  if(pthread_create(&gl_thread, NULL, GLThread, NULL) != 0)
    printf("* failed to create GL thread");
}

VisualWMVisualisation::~VisualWMVisualisation()
{
  pthread_kill(gl_thread, SIGKILL);
  pthread_join(gl_thread, NULL);
  gluDeleteQuadric(glu_quad);
  if(glut_window != -1)
    glutDestroyWindow(glut_window);
}

void VisualWMVisualisation::StartGLThread()
{
  int argc = 1;
  char *argv[1] = {""};

  mouse_x = mouse_y = mouse_butt = -1;
  glutInit(&argc, argv);
  glu_quad = gluNewQuadric();
  //gluQuadricDrawStyle(glu_quad, GLU_SILHOUETTE);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_ALPHA);
  glutInitWindowSize(400, 400);
  //glutInitWindowPosition(0, 0);
  glut_window = glutCreateWindow("Visual Working Memory");
  glutDisplayFunc(&VisualWMVisualisation::DrawGL);
  glutReshapeFunc(&VisualWMVisualisation::ResizeGL);
  glutMouseFunc(&VisualWMVisualisation::MousePressed);
  glutMotionFunc(&VisualWMVisualisation::MouseMoved);
  glutTimerFunc(REDRAW_INTERVAL, &VisualWMVisualisation::TimerCallback, 0);
  InitGL();
  glutMainLoop();
}

void VisualWMVisualisation::InitGL()
{
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

void VisualWMVisualisation::ResizeGL(int width, int height)
{
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // viewing frustrum, note all values are in m.
  gluPerspective(60., (double)width/(double)height, 0.001, 100.);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void VisualWMVisualisation::TimerCallback(int val)
{
  DrawGL();
  // and reset the timer again
  glutTimerFunc(REDRAW_INTERVAL, &VisualWMVisualisation::TimerCallback, 0);
}

void VisualWMVisualisation::DrawGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslated(cam_trans[0], cam_trans[1], cam_trans[2]);
  gluLookAt(1.8, 1.8, 1.8,  0., 0., 0.,  0., 0., 1.);
  glRotated(cam_rot[0], 0., 0., 1.);
  DrawCoordFrame();
  DrawGroundPlane();
  DrawCameras();
  DrawSceneObjects();
  glutSwapBuffers();
}

void VisualWMVisualisation::DrawSurfaces()
{
  vector< shared_ptr< WorkingMemoryItem<Surface> > > surfs;
  
  visual_wm->GetSurfaces(surfs);
  for(unsigned i = 0; i < surfs.size(); i++)
  {
    DrawSurface(surfs[i]->getData());
  }
}

void VisualWMVisualisation::DrawSceneObjects()
{
  vector< shared_ptr< WorkingMemoryItem<SceneObject> > > objs;
  visual_wm->GetSceneObjects(objs);
  for(unsigned i = 0; i < objs.size(); i++)
  {
    string id = objs[i]->getID();
    DrawSceneObject(objs[i]->getData(), id);
  }
}

void VisualWMVisualisation::DrawGroundPlane()
{
  double grid_size = GROUND_GRID_SIZE;
  double size_x = GROUND_SIZE_X;
  double size_y = GROUND_SIZE_Y;
  double x, y;
  char buf[100];

  glNormal3d(0., 0., 1.);
  glLineWidth(1.);
  glColor4fv(yellow);
  glBegin(GL_LINES);
  for(x = 0.; x <= size_x/2.; x += grid_size)
  {
    glVertex3d(x, -size_y/2., 0.);
    glVertex3d(x, size_y/2., 0.);
  }
  for(x = -grid_size; x >= -size_x/2.; x -= grid_size)
  {
    glVertex3d(x, -size_y/2., 0.);
    glVertex3d(x, size_y/2., 0.);
  }
  for(y = 0.; y <= size_y/2.; y += grid_size)
  {
    glVertex3d(-size_x/2., y, 0.);
    glVertex3d(size_x/2., y, 0.);
  }
  for(y = -grid_size; y >= -size_y/2.; y -= grid_size)
  {
    glVertex3d(-size_x/2., y, 0.);
    glVertex3d(size_x/2., y, 0.);
  }
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3d(-size_x/2., -size_y/2., 0.);
  glVertex3d(size_x/2., -size_y/2., 0.);
  glVertex3d(size_x/2., size_y/2., 0.);
  glVertex3d(-size_x/2., size_y/2., 0.);
  glEnd();
  for(x = 0.; x <= size_x/2.; x += grid_size)
  {
    snprintf(buf, 100, "%.1f", x);
    DrawText3D(buf, x, size_y/2., 0.);
  }
  for(y = 0.; y <= size_y/2.; y += grid_size)
  {
    snprintf(buf, 100, "%.1f", y);
    DrawText3D(buf, size_x/2., y, 0.);
  }
}

void VisualWMVisualisation::DrawObjectLabel(shared_ptr<const SceneObject> obj, string &id)
{
  char text[200];
  string label = CORBA::string_dup(obj->m_label.m_string);
  snprintf(text, 200, "%s '%s' (%.2f)", id.c_str(), label.c_str(),
      obj->m_label.m_confidence);
  DrawText3D(text, 0., 0., 0.);
}

void VisualWMVisualisation::DrawSceneObject(shared_ptr<const SceneObject> obj, string &id)
{
  glColor4fv(white);
  glPushMatrix();
  glTranslated(obj->m_bbox.m_centroid.m_x, obj->m_bbox.m_centroid.m_y,
    obj->m_bbox.m_centroid.m_z);
  glScaled(obj->m_bbox.m_size.m_x, obj->m_bbox.m_size.m_y,
    obj->m_bbox.m_size.m_z);
  glutWireCube(1.);
  glColor4fv(white);
  DrawObjectLabel(obj, id);
  glPopMatrix();
  for(unsigned int i = 0; i < obj->m_surfaces.length(); i++) {
    //HACK time saving as michael is not using this in the future?
    DrawSurface(shared_ptr<const Surface>(&obj->m_surfaces[i]));
  }

}

void VisualWMVisualisation::DrawSurface(shared_ptr<const Surface> s)
{
  double l = 0.05;  // length of normal vector for drawing in [m]

  glColor4fv(white);
  glBegin(GL_LINE_LOOP);
  for(unsigned i = 0; i < s->m_vertices.length(); i++)
  {
    glVertex3d(s->m_vertices[i].m_pos.m_x, s->m_vertices[i].m_pos.m_y,
      s->m_vertices[i].m_pos.m_z);
  }
  glEnd();
  glBegin(GL_LINES);
  for(unsigned i = 0; i < s->m_vertices.length(); i++)
  {
    glVertex3d(s->m_vertices[i].m_pos.m_x, s->m_vertices[i].m_pos.m_y,
      s->m_vertices[i].m_pos.m_z);
    glVertex3d(
      s->m_vertices[i].m_pos.m_x + l*s->m_vertices[i].m_normal.m_x,
      s->m_vertices[i].m_pos.m_y + l*s->m_vertices[i].m_normal.m_y,
      s->m_vertices[i].m_pos.m_z + l*s->m_vertices[i].m_normal.m_z);
  }
  glEnd();
}

void VisualWMVisualisation::MousePressed(int button, int state, int x, int y)
{
  mouse_x = x;
  mouse_y = y;
  if(state == GLUT_DOWN)
    mouse_butt = button;
  else
    mouse_butt = -1;
}

void VisualWMVisualisation::MouseMoved(int x, int y)
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
  DrawGL();
}

/**
 * Draw an arrow pointing in z-axis.
 * Rotate as needed for other axes.
 * \param  axis  one of 'x', 'y', 'z'
 * \param  index '0', '1', '2', ... '9'
 */
void VisualWMVisualisation::DrawZAxis(char axis, float col[4])
{
  static const double l = 0.20, r = 0.010, a = 0.05, w = 0.030;

  glColor4fv(col);
  gluCylinder(glu_quad, r, r, l - a, 10, 1);
  glPushMatrix();
  glTranslated(0., 0., l - a);
  gluCylinder(glu_quad, w, 0., a, 10, 1);
  glPopMatrix();
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, col);
  glRasterPos3d(0., 0., 1.5*l);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, axis);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, black);
}

void VisualWMVisualisation::DrawCoordFrame()
{
  glPushMatrix();
  DrawZAxis('z', yellow);
  glRotated(-90, 1., 0., 0.);
  DrawZAxis('y', yellow);
  glRotated(90, 0., 1., 0.);
  DrawZAxis('x', yellow);
  glPopMatrix();
}

void VisualWMVisualisation::DrawCamera(shared_ptr<const Camera> cam)
{
  double cam_size = 0.10;
  double theta = Length(cam->m_pose.m_orientation)*180./M_PI;

  glColor4fv(yellow);
  glPushMatrix();
  glTranslated(cam->m_pose.m_position.m_x, cam->m_pose.m_position.m_y,
    cam->m_pose.m_position.m_z);
  if(theta > 0.)
  {
    Vector3D r = Normalise(cam->m_pose.m_orientation);
    glRotated(theta, r.m_x, r.m_y, r.m_z);
  }
  gluCylinder(glu_quad, 0.04, 0.04, 0.05, 10, 1);
  //glScaled(1., 1., 1.5);
  glTranslated(0., 0., -cam_size/2.);
  glutSolidCube(cam_size);
  glPopMatrix();
}

void VisualWMVisualisation::DrawCameras()
{
  vector< shared_ptr< WorkingMemoryItem<Camera> > > cams;
  visual_wm->GetCameras(cams);
  for(unsigned i = 0; i < cams.size(); i++)
  {
    DrawCamera(cams[i]->getData());   
  }
}

void VisualWMVisualisation::DrawText3D(const char *text, double x, double y,
  double z)
{
  glRasterPos3d(x, y, z);
  while(*text != '\0')
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *text++);
}
