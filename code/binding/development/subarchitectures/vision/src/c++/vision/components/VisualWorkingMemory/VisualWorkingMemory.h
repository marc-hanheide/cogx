#ifndef CAST_VISUAL_WORKING_MEMORY_H_
#define CAST_VISUAL_WORKING_MEMORY_H_


#ifdef __APPLE__

#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>

#else

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#endif

#include <pthread.h>
#include <cast/architecture/SubarchitectureWorkingMemory.hpp>
#include <vision/idl/Vision.hh>
#include <CoSyCommon/idl/Math.hh>


using namespace cast; using namespace std; using namespace boost; //default useful namespaces, fix to reflect your own code

class VisualWorkingMemory;

/**
 * 3D visualisation of the visual working memory content.
 * TODO: the static members are nasty, but DrawGL() must be static and hence all
 * data it references.
 */
class VisualWMVisualisation
{
private:
  static int glut_window;
  static int mouse_x, mouse_y, mouse_butt;
  static double cam_trans[3];
  static double cam_rot[2];
  static GLUquadric *glu_quad;
  static pthread_t gl_thread;
  static VisualWorkingMemory *visual_wm;

  static void InitGL();
  static void ResizeGL(int width, int height);
  static void TimerCallback(int val);
  static void DrawGL();
  static void MousePressed(int button, int state, int x, int y);
  static void MouseMoved(int x, int y);
  static void DrawZAxis(char axis, float col[4]);
  static void DrawCoordFrame();
  static void DrawGroundPlane();
  static void DrawSurfaces();
  static void DrawSurface(shared_ptr<const Vision::Surface> s);
  static void DrawObjectLabel(shared_ptr<const Vision::SceneObject> obj, string &id);
  static void DrawSceneObject(shared_ptr<const Vision::SceneObject> obj, string &id);
  static void DrawSceneObjects();
  static void DrawCamera(shared_ptr<const Vision::Camera> cam);
  static void DrawCameras();
  static void DrawText3D(const char *text, double x, double y, double z);

public:
  VisualWMVisualisation(VisualWorkingMemory *wm);
  ~VisualWMVisualisation();
  void StartGLThread();
};

class VisualWorkingMemory : public SubarchitectureWorkingMemory
{
private:
  VisualWMVisualisation *vis;

protected:
  virtual void redrawGraphics2D();
  virtual void redrawGraphics3D();
  virtual void redrawGraphicsText();

public:
  VisualWorkingMemory(const string & _id);
  virtual ~VisualWorkingMemory();
  

  //nah: replacing with SubarchitectureWorkingMemory template methods
 void GetSurfaces(vector< shared_ptr< WorkingMemoryItem<Vision::Surface> > > & surfs);
 void GetSceneObjects(vector< shared_ptr< WorkingMemoryItem<Vision::SceneObject> > > & objs);
 void GetCameras(vector< shared_ptr< WorkingMemoryItem<Vision::Camera> > > & cams);
};

#endif

