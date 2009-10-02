/**
 * @author Kai ZHOU
 * @date June 2009
 */

#include <GL/freeglut.h>
#include <cogxmath.h>
#include "PlanePopOut.h"
#include <stack>
#include <vector>
#include <VideoUtils.h>
#include <cast/architecture/ChangeFilterFactory.hpp>

#define Shrink_SOI 0.75
#define Upper_BG 1.8
#define Lower_BG 1.2	// 1.2-1.8 radius of BoundingSphere

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::PlanePopOut();
  }
}

namespace cast
{
using namespace std;
using namespace Stereo;
using namespace cogx;
using namespace cogx::Math;
using namespace VisionData;
using namespace cdl;

int win;
int objnumber = 0;
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

VisionData::SurfacePointSeq points;
VisionData::SurfacePointSeq pointsN;

vector <int> points_label;  //0->plane; 1~999->objects index; -1->discarded points
vector< Vector3 > v3size;
vector< Vector3 > v3center;
vector<double> vdradius;
vector< VisionData::SurfacePointSeq > SOIPointsSeq;
vector< VisionData::SurfacePointSeq > BGPointsSeq;
vector< VisionData::SurfacePointSeq > EQPointsSeq; //equivocal points
double A, B, C, D;
int N;  // 1/N points will be used
bool mbDrawWireSphere;
Vector3 v3dmax;
Vector3 v3dmin;


void InitWin()
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

  mbDrawWireSphere = true;
}

void ResizeWin(int w, int h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45., (double)w/(double)h, 0.001, 10000.);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void DrawText3D(const char *text, double x, double y, double z)
{
  glRasterPos3d(x, y, z);
  while(*text != '\0')
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *text++);
}
/**
 * Draw things like coord frames
 */
void DrawOverlays()
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
}

void DrawPlaneGrid()
{
	glLineWidth(1);
	glEnable(GL_BLEND);
	glEnable(GL_LINE_SMOOTH);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
///////////////////////////////////////////////////////////
	glBegin(GL_LINE_LOOP);
	glColor3f(1.0,1.0,1.0);
	glVertex3f(v3dmax.x, v3dmax.y, v3dmax.z);
	glVertex3f(v3dmin.x, v3dmax.y, v3dmax.z);
	glVertex3f(v3dmin.x, v3dmin.y, v3dmin.z);
	glVertex3f(v3dmax.x, v3dmin.y, v3dmin.z);
	glVertex3f(v3dmin.x, v3dmax.y, v3dmax.z);
	glVertex3f(v3dmin.x, v3dmin.y, v3dmin.z);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glColor3f(1.0,1.0,1.0);
	glVertex3f(v3dmax.x, v3dmax.y, v3dmax.z);
	glVertex3f(v3dmin.x, v3dmax.y, v3dmax.z);
	glVertex3f(v3dmin.x+A, v3dmin.y+B, v3dmin.z+C);
	glVertex3f(v3dmax.x+A, v3dmax.y+B, v3dmax.z+C);
	glVertex3f(v3dmin.x, v3dmax.y, v3dmax.z);
	glVertex3f(v3dmin.x+A, v3dmin.y+B, v3dmin.z+C);
	glEnd();

	glDisable(GL_BLEND);
}

void DrawPoints()
{ 
  glPointSize(2);
  glBegin(GL_POINTS);
  for(size_t i = 0; i < pointsN.size(); i++)
  {
	if (points_label.at(i) == 0)   glColor3f(1.0,1.0,0.0); // plane/*DrawPlaneGrid();*/

	else if (points_label.at(i) < 0)  glColor3f(0.0,0.0,0.0);  // discarded points
	else if (points_label.at(i) == 1)  glColor3f(0.0,0.0,1.0);  // 1st object
	else if (points_label.at(i) == 2)  glColor3f(0.0,0.0,1.0);  //
	else if (points_label.at(i) == 3)  glColor3f(0.0,0.0,1.0);  //
	else if (points_label.at(i) == 4)  glColor3f(0.0,0.0,1.0);  //
	else if (points_label.at(i) == 5)  glColor3f(0.0,0.0,1.0);  //
	else glColor3f(0.0,0.0,1.0);  //rest object
    glVertex3f(pointsN[i].p.x, pointsN[i].p.y, pointsN[i].p.z);
  }
  glEnd();
/*
glPointSize(20);
glColor3ub(255, 255, 255);
glBegin(GL_POINTS);
glVertex3f(0.0, 0.0, 0.0);
glEnd();
*/
}

void DrawOneCuboid(Vector3 Max, Vector3 Min)
{
	glLineWidth(1);
	glEnable(GL_BLEND);
	glEnable(GL_LINE_SMOOTH);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//////////////////////top/////////////////////////////////////
	glBegin(GL_LINE_LOOP);
	glColor3f(1.0,1.0,1.0);
	glVertex3f(Max.x, Max.y, Max.z);
	glVertex3f(Min.x, Max.y, Max.z);
	glVertex3f(Min.x, Min.y, Max.z);
	glVertex3f(Max.x, Min.y, Max.z);
	glEnd();
/////////////////////////////bottom///////////////////////
	glBegin(GL_LINE_LOOP);
	glColor3f(1.0,1.0,1.0);
	glVertex3f(Min.x, Min.y, Min.z);
	glVertex3f(Max.x, Min.y, Min.z);
	glVertex3f(Max.x, Max.y, Min.z);
	glVertex3f(Min.x, Max.y, Min.z);
	glEnd();
//////////////////////verticle lines//////////////////////////
	glBegin(GL_LINES);
	glVertex3f(Min.x, Min.y, Min.z);
	glVertex3f(Min.x, Min.y, Max.z);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(Max.x, Min.y, Min.z);
	glVertex3f(Max.x, Min.y, Max.z);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(Max.x, Max.y, Min.z);
	glVertex3f(Max.x, Max.y, Max.z);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(Min.x, Max.y, Min.z);
	glVertex3f(Min.x, Max.y, Max.z);
	glEnd();

	glDisable(GL_BLEND);
}

void DrawCuboids(VisionData::SurfacePointSeq &points, std::vector <int> &labels)
{
	VisionData::SurfacePointSeq Max;
	VisionData::SurfacePointSeq Min;
	Vector3 initial_vector;
	initial_vector.x = -9999;
	initial_vector.y = -9999;
	initial_vector.z = -9999;
	VisionData::SurfacePoint InitialStructure;
	InitialStructure.p = initial_vector;
	InitialStructure.c.r = InitialStructure.c.g = InitialStructure.c.b = 0;
	Max.assign(objnumber, InitialStructure);
	initial_vector.x = 9999;
	initial_vector.y = 9999;
	initial_vector.z = 9999;
	InitialStructure.p = initial_vector;
	Min.assign(objnumber, InitialStructure);

	for(unsigned int i = 0; i<points.size(); i++)
	{
		Vector3 v3Obj = points.at(i).p;
		int label = labels.at(i);
		if (label > 0)
		{
			if (v3Obj.x>Max.at(label-1).p.x) Max.at(label-1).p.x = v3Obj.x;
			if (v3Obj.x<Min.at(label-1).p.x) Min.at(label-1).p.x = v3Obj.x;
	
			if (v3Obj.y>Max.at(label-1).p.y) Max.at(label-1).p.y = v3Obj.y;
			if (v3Obj.y<Min.at(label-1).p.y) Min.at(label-1).p.y = v3Obj.y;
	
			if (v3Obj.z>Max.at(label-1).p.z) Max.at(label-1).p.z = v3Obj.z;
			if (v3Obj.z<Min.at(label-1).p.z) Min.at(label-1).p.z = v3Obj.z;
		}
	}
	v3size.clear();
	for (int i=0; i<objnumber; i++)
	{
		Vector3 s;
		s.x = (Max.at(i).p.x-Min.at(i).p.x)/2;
		s.y = (Max.at(i).p.y-Min.at(i).p.y)/2;
		s.z = (Max.at(i).p.z-Min.at(i).p.z)/2;
		v3size.push_back(s);
	}
/*	
	for (int i = 0; i<objnumber; i++)
	{
		DrawOneCuboid(Max.at(i),Min.at(i));
	}
*/
	Max.clear();
	Min.clear();
}

void DrawWireSphere(Vector3 center, double radius)
{
	glColor3f(1.0,1.0,1.0);
	glTranslatef(center.x, center.y, center.z);
	glutWireSphere(radius,10,10);
	glTranslatef(-center.x, -center.y, -center.z);
}

Vector3 ProjectOnDominantPlane(Vector3 InputP)
{
	Vector3 OutputP;
	OutputP.x = ((B*B+C*C)*InputP.x-A*(B*InputP.y+C*InputP.z+D))/(A*A+B*B+C*C);
	OutputP.y = ((A*A+C*C)*InputP.y-B*(A*InputP.x+C*InputP.z+D))/(A*A+B*B+C*C);
	OutputP.z = ((B*B+A*A)*InputP.z-C*(B*InputP.y+A*InputP.x+D))/(A*A+B*B+C*C);

	return OutputP;
}

void BoundingSphere(VisionData::SurfacePointSeq &points, std::vector <int> &labels)
{
	VisionData::SurfacePointSeq center;
	Vector3 initial_vector;
	initial_vector.x = 0;
	initial_vector.y = 0;
	initial_vector.z = 0;
	VisionData::SurfacePoint InitialStructure;
	InitialStructure.p = initial_vector;
	InitialStructure.c.r = InitialStructure.c.g = InitialStructure.c.b = 0;
	center.assign(objnumber,InitialStructure);
	VisionData::SurfacePointSeq pointsInOneSOI;
	SOIPointsSeq.clear();
	SOIPointsSeq.assign(objnumber, pointsInOneSOI);
	BGPointsSeq.clear();
	BGPointsSeq.assign(objnumber, pointsInOneSOI);
	EQPointsSeq.clear();
	EQPointsSeq.assign(objnumber, pointsInOneSOI);

	std::vector<int> amount;
	amount.assign(objnumber,0);
	std::vector<double> radius_world;
	radius_world.assign(objnumber,0);
////////////////////calculate the center of each object/////////////////////////
	for(unsigned int i = 0; i<points.size(); i++)
	{
		Vector3 v3Obj = points.at(i).p;
		int label = labels.at(i);
		if (label > 0)
		{
			center.at(label-1).p = center.at(label-1).p + v3Obj;
			amount.at(label-1) = amount.at(label-1) + 1;
		}
	}
	v3center.clear();
	if (amount.at(0) != 0)
	{
		for (unsigned int i=0; i<center.size(); i++)
		{
			center.at(i).p = center.at(i).p/amount.at(i);
			v3center.push_back(center.at(i).p);
		}
	////////////////////calculte radius in the real world//////////////////
		for(unsigned int i = 0; i<points.size(); i++)
		{
			Vector3 v3Obj = points.at(i).p;
			int label = labels.at(i);
			if (label > 0 && dist(v3Obj,center.at(label-1).p) > radius_world.at(label-1))
				radius_world.at(label-1) = dist(v3Obj,center.at(label-1).p);
		}
		vdradius.clear();
		for (int i=0; i<objnumber; i++)
			vdradius.push_back(radius_world.at(i));
	}

	for (int i = 0; i<objnumber; i++)
	{
		if (mbDrawWireSphere)	DrawWireSphere(center.at(i).p,radius_world.at(i));
		Vector3 Center_DP = ProjectOnDominantPlane(center.at(i).p);//cout<<" center on DP ="<<Center_DP<<endl;
		for (unsigned int j = 0; j<points.size(); j++)
		{
			VisionData::SurfacePoint PushStructure;
			PushStructure.p = points.at(j).p;
			PushStructure.c = points.at(j).c;	//cout<<"in BG"<<PushStructure.c.r<<PushStructure.c.g<<PushStructure.c.b<<endl;
			Vector3 Point_DP = ProjectOnDominantPlane(PushStructure.p);
			int label = labels.at(j);
			if (label > 0 && dist(Point_DP,Center_DP) < Shrink_SOI*radius_world.at(i))
			{
				SOIPointsSeq.at(label-1).push_back(PushStructure);
			}
			if (label == -1 && dist(Point_DP,Center_DP) < Lower_BG*radius_world.at(i)) // equivocal points
			{
				EQPointsSeq.at(i).push_back(PushStructure);
				glPointSize(2);
				glBegin(GL_POINTS);
				glColor3f(0.0,1.0,0.0);  //equivocal points
				glVertex3f(PushStructure.p.x, PushStructure.p.y, PushStructure.p.z);
				glEnd();
			}
			if (label == 0 && dist(Point_DP,Center_DP) < Upper_BG*radius_world.at(i) && dist(Point_DP,Center_DP) > Lower_BG*radius_world.at(i)) //BG nearby also required
			{
				BGPointsSeq.at(i).push_back(PushStructure);

				glPointSize(2);
				glBegin(GL_POINTS);
				glColor3f(1.0,0.0,0.0);  //nearby points
				glVertex3f(PushStructure.p.x, PushStructure.p.y, PushStructure.p.z);
				glEnd();
			}
		}
	}
	
	center.clear();
	amount.clear();
	radius_world.clear();
	pointsInOneSOI.clear();
}


void DisplayWin()
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
  if (objnumber != 0)
  {
	DrawCuboids(pointsN,points_label);
	BoundingSphere(pointsN,points_label);
  }
  else
  {
	v3size.clear();
	v3center.clear();
	vdradius.clear();
  }
  glDisable(GL_COLOR_MATERIAL);

  glutSwapBuffers();
}

void KeyPress(unsigned char key, int x, int y)
{
  switch(key)
  {/*
    case 'q':
      // a slightly harsh way to end a program ...
      exit(EXIT_SUCCESS);
      break;*/
    case 's':
	{if (mbDrawWireSphere) mbDrawWireSphere = false;
	 else mbDrawWireSphere = true;}
      break;
    default:
      break;
  }
  glutPostRedisplay();
}

void MousePress(int button, int state, int x, int y)
{
  mouse_x = x;
  mouse_y = y;
  mouse_butt = button;
  butt_state = state;
}

void MouseMove(int x, int y)
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


void PlanePopOut::configure(const map<string,string> & _config)
{
  // first let the base classes configure themselves
  configureStereoCommunication(_config);
}

void PlanePopOut::start()
{
  startStereoCommunication(*this);

  int argc = 1;
  char argv0[] = "PlanePopOut";
  char *argv[1] = {argv0};
  glutInit(&argc, argv);
  win = glutCreateWindow("points");
  InitWin();
  glutKeyboardFunc(KeyPress);
  glutMouseFunc(MousePress);
  glutMotionFunc(MouseMove);
  glutReshapeFunc(ResizeWin);
  glutDisplayFunc(DisplayWin);
}

void PlanePopOut::runComponent()
{	
  while(isRunning())
  {//cout<<"mbDrawWireSphere = "<<mbDrawWireSphere<<endl;
	VisionData::SurfacePointSeq tempPoints = points;
	points.resize(0);
	getPoints(points);
	if (points.size() == 0)
		points = tempPoints;
	else
	{
		tempPoints.clear();
		pointsN.clear();
		objnumber = 0;
		N = 5;
		for (VisionData::SurfacePointSeq::iterator it=points.begin(); it<points.end(); it+=N)
			pointsN.push_back(*it);
		points_label.clear();
		points_label.assign(pointsN.size(), -3);
	

		if (RANSAC(pointsN,points_label))
		{
			SplitPoints(pointsN,points_label);
			glutPostRedisplay();
			glutMainLoopEvent();
		}
	}
	if (para_a!=0.0 || para_b!=0.0 || para_c!=0.0 || para_d!=0.0)
	{
		CurrentObjList.clear();
		for(unsigned int i=0; i<v3center.size(); i++)  //create objects
		{
			ObjPara OP;
			OP.c = v3center.at(i);
			OP.s = v3size.at(i);
			OP.r = vdradius.at(i);
			OP.id = "";
			OP.pointsInOneSOI = SOIPointsSeq.at(i);
			OP.BGInOneSOI = BGPointsSeq.at(i);
			OP.EQInOneSOI = EQPointsSeq.at(i);
			CurrentObjList.push_back(OP);
		}
		if (PreviousObjList.empty())
		{
			for(unsigned int i=0; i<CurrentObjList.size(); i++)
			{
				CurrentObjList.at(i).id = newDataID();
				SOIPtr obj = createObj(CurrentObjList.at(i).c, CurrentObjList.at(i).s, CurrentObjList.at(i).r, CurrentObjList.at(i).pointsInOneSOI, CurrentObjList.at(i).BGInOneSOI, CurrentObjList.at(i).EQInOneSOI);
//cout<<"x in SOI after add in WM = "<<obj->boundingSphere.pos.x<<" ID of this SOI = "<<CurrentObjList.at(i).id<<endl;
				addToWorkingMemory(CurrentObjList.at(i).id, obj);	
			}
			PreviousObjList = CurrentObjList;
		}
		else
		{
			std::vector <int> newObjList; //store the serial number of new objects in CurrentObjList
			for(unsigned int i=0; i<CurrentObjList.size(); i++)
			{
				bool flag = false;
				for(unsigned int j=0; j<PreviousObjList.size(); j++)
				{
					if(Compare2SOI(CurrentObjList.at(i), PreviousObjList.at(j)))// if these two objects were the same one
					{
						flag = true;
						SOIPtr obj = createObj(CurrentObjList.at(i).c, CurrentObjList.at(i).s, CurrentObjList.at(i).r,CurrentObjList.at(i).pointsInOneSOI, CurrentObjList.at(i).BGInOneSOI, CurrentObjList.at(i).EQInOneSOI);
//cout<<"x in SOI after overwrite in WM = "<<obj->boundingSphere.pos.x<<" ID of this SOI = "<<PreviousObjList.at(j).id<<endl;
						overwriteWorkingMemory(PreviousObjList.at(j).id, obj);	
						break;
					}
				}
				if (!flag) //there might be some new objects
					newObjList.push_back(i);
			}
			if(!newObjList.empty())
			{
				for(unsigned int i=0; i<newObjList.size(); i++)// add all new objects
				{
					CurrentObjList.at(newObjList.at(i)).id = newDataID();
					SOIPtr obj = createObj(CurrentObjList.at(newObjList.at(i)).c, CurrentObjList.at(newObjList.at(i)).s, CurrentObjList.at(newObjList.at(i)).r,CurrentObjList.at(newObjList.at(i)).pointsInOneSOI, CurrentObjList.at(newObjList.at(i)).BGInOneSOI, CurrentObjList.at(newObjList.at(i)).EQInOneSOI);
//cout<<"x in SOI after add 2 in WM = "<<obj->boundingSphere.pos.x<<" ID of this SOI = "<<CurrentObjList.at(newObjList.at(i)).id<<endl;
					addToWorkingMemory(CurrentObjList.at(newObjList.at(i)).id, obj);	
					PreviousObjList.push_back(CurrentObjList.at(newObjList.at(i)));//update PreviousObjList
				}
			}
			if (PreviousObjList.size()!=CurrentObjList.size()) //need to delete the disappeared objects
			{
				std::vector <unsigned int> disappearedObjList; //store the serial number of disappeared objects in PreviousObjList
				for(unsigned int i=0; i<PreviousObjList.size(); i++)
				{
					bool flag = false;
					for(unsigned int j=0; j<CurrentObjList.size(); j++)
					{
						if(Compare2SOI(CurrentObjList.at(j), PreviousObjList.at(i)))// if these two objects were the same
						{
							flag = true;
							break;
						}
					}
					if (!flag) //this is a disappeared object
						disappearedObjList.push_back(i);
				}
				if(!disappearedObjList.empty())
				{
					for(unsigned int i=0; i<disappearedObjList.size(); i++)// delete all objects
					{
						deleteFromWorkingMemory(PreviousObjList.at(disappearedObjList.at(i)).id);
					}
					std::vector<ObjPara> new_PreviousList;
					new_PreviousList.reserve(PreviousObjList.size()-disappearedObjList.size());
					for(unsigned int i=0; i<PreviousObjList.size(); i++)
					{
						bool temp_flag = false;
						for (unsigned int j=0; j<disappearedObjList.size(); j++)
						{								
							if (i == disappearedObjList.at(j))
							{
								temp_flag = true;
								break;
							}									
						}
						if (!temp_flag)
							new_PreviousList.push_back( PreviousObjList.at(i) );
					}
					PreviousObjList = new_PreviousList;
					new_PreviousList.clear();
				}
			}
		
		}
	}
//cout<<"SOI in the WM = "<<PreviousObjList.size()<<endl;
    // wait a bit so we don't hog the CPU
    sleepComponent(50);
  }
}

bool PlanePopOut::RANSAC(VisionData::SurfacePointSeq &points, std::vector <int> &labels)
{
	VisionData::SurfacePointSeq R_points = points;
	unsigned int nPoints = R_points.size();
	if(nPoints < 10)
	{
		//std::cout << "  too few points to calc plane" << std::endl;
		return false;
	};
	int nRansacs = 100;
	int point1 = 0;
	int point2 = 0;
	int point3 = 0;
	Vector3 v3BestMean;
	Vector3 v3BestNormal;
	double dBestDistSquared = 9999999999999999.9;
	
	for(int i=0; i<nRansacs; i++)
	{
		int nA, nB, nC;
		Vector3 v3Normal;
		do
		{
			nA = rand()%nPoints;
			nB = nA;
			nC = nA;
			while(nB == nA)
				nB = rand()%nPoints;
			while(nC == nA || nC==nB)
				nC = rand()%nPoints;
			Vector3 v3CA = R_points.at(nC).p  - R_points.at(nA).p;
			Vector3 v3BA = R_points.at(nB).p  - R_points.at(nA).p;
			v3Normal = cross(v3CA, v3BA);
			if (norm(v3Normal) != 0)
				normalise(v3Normal);
			else v3Normal = 99999.9*v3Normal;
		} while (fabs(v3Normal.x/(dot(v3Normal,v3Normal)+1))>0.01); //the plane should parallel with the initialisation motion of camera

		Vector3 v3Mean = 0.33333333 * (R_points.at(nA).p + R_points.at(nB).p + R_points.at(nC).p);
		double dSumError = 0.0;
		for(unsigned int i=0; i<nPoints; i++)
		{
			Vector3 v3Diff = R_points.at(i).p - v3Mean;
			double dDistSq = dot(v3Diff, v3Diff);
			if(dDistSq == 0.0)
			continue;
			double dNormDist = fabs(dot(v3Diff, v3Normal));	
			if(dNormDist > 0.02)
			dNormDist = 0.02;
			dSumError += dNormDist;
		}
		if(dSumError < dBestDistSquared)
		{
			dBestDistSquared = dSumError;
			v3BestMean = v3Mean;
			v3BestNormal = v3Normal;
			point1 = nA;
			point2 = nB;
			point3 = nC;
		}
	}
////////////////////////////////use three points to cal plane////
	para_a = ( (R_points.at(point2).p.y-R_points.at(point1).p.y)*(R_points.at(point3).p.z-R_points.at(point1).p.z)-(R_points.at(point2).p.z-R_points.at(point1).p.z)*(R_points.at(point3).p.y-R_points.at(point1).p.y) );
	para_b = ( (R_points.at(point2).p.z-R_points.at(point1).p.z)*(R_points.at(point3).p.x-R_points.at(point1).p.x)-(R_points.at(point2).p.x-R_points.at(point1).p.x)*(R_points.at(point3).p.z-R_points.at(point1).p.z) );
	para_c = ( (R_points.at(point2).p.x-R_points.at(point1).p.x)*(R_points.at(point3).p.y-R_points.at(point1).p.y)-(R_points.at(point2).p.y-R_points.at(point1).p.y)*(R_points.at(point3).p.x-R_points.at(point1).p.x) );
	para_d = ( 0-(para_a*R_points.at(point1).p.x+para_b*R_points.at(point1).p.y+para_c*R_points.at(point1).p.z) );
/////////////////////////////////end parameters calculation/////////////
// Done the ransacs, now collect the supposed inlier set
	if (para_d<0)
	{
		para_a = -para_a;
		para_b = -para_b;
		para_c = -para_c;
		para_d = -para_d;
	}
	A = para_a;
	B = para_b;
	C = para_c;
	D = para_d;

	double dmin = 9999.0;
	double dmax = 0.0;

	if (v3BestMean.x != 0 || v3BestMean.y != 0 || v3BestMean.z != 0)
	{
		for(unsigned int i=0; i<nPoints; i++)
		{
			Vector3 v3Diff = R_points.at(i).p - v3BestMean;
			double dNormDist = fabs(dot(v3Diff, v3BestNormal));
			if(dNormDist < 0.02)
			{
				labels.at(i) = 0; // dominant plane
				double ddist = dot(R_points.at(i).p,R_points.at(i).p);
				if (ddist > dmax) {dmax = ddist; v3dmax = R_points.at(i).p;}
				else if (ddist < dmin) {dmin = ddist; v3dmin = R_points.at(i).p;}
			}
			else
			{
				double d_parameter = -(A*R_points.at(i).p.x+B*R_points.at(i).p.y+C*R_points.at(i).p.z);
				if (d_parameter > 0 && d_parameter < D && fabs(d_parameter-D) > fabs(D)/50)
					labels.at(i) = -2; // objects
				if (d_parameter > 0 && d_parameter < D && fabs(d_parameter-D) <= fabs(D)/50)
					labels.at(i) = -1; // cannot distingush
			}
		}
	}
	return true;
}

void PlanePopOut::SplitPoints(VisionData::SurfacePointSeq &points, std::vector <int> &labels)
{
	std::vector<int> candidants;
	std::vector <int> S_label = labels;
	for (unsigned int i=0; i<S_label.size(); i++)
	{
		if (S_label.at(i) == -2) //not belong to the dominant plane cluster
			candidants.push_back(i);
	}
//cout<<"candidants.size() =  "<<candidants.size()<<endl;
	std::vector<int> one_obj;
	objnumber = 1;
	unsigned int points_of_one_object = 0;
	std::stack <int> objstack;
	double split_threshold = Calc_SplitThreshold(points, labels);
	unsigned int obj_number_threshold;
	if (N == 1) obj_number_threshold = 400;
	if (N == 5) obj_number_threshold = 60;
	if (N == 10) obj_number_threshold = 20;
	while(!candidants.empty())
	{
		S_label.at(*candidants.begin()) = objnumber;
		objstack.push(*candidants.begin());
		points_of_one_object++;
		candidants.erase(candidants.begin());
		while(!objstack.empty())
		{
			int seed = objstack.top();
			objstack.pop();
			for(std::vector<int>::iterator it=candidants.begin(); it<candidants.end(); it++)
			{
				if (dist(points.at(seed).p, points.at(*it).p)<split_threshold)
				{
					S_label.at(*it) = S_label.at(seed);
					objstack.push(*it);
					points_of_one_object++;
					candidants.erase(it);
				}
			}
		}//cout<<"points_of_one_object =  "<<points_of_one_object<<endl;cout<<"candidants.size() =  "<<candidants.size()<<endl;
		if (points_of_one_object>obj_number_threshold)
		{
			labels = S_label;
			objnumber++;
		}
		else
			S_label = labels;
		points_of_one_object = 0;
	}
	objnumber--;
}

double PlanePopOut::Calc_SplitThreshold(VisionData::SurfacePointSeq &points, std::vector <int> &labels)
{
	double max_x = -99999.0;
	double min_x = 99999.0;
	double max_y = -99999.0;
	double min_y = 99999.0;
	double max_z = -99999.0;
	double min_z = 99999.0;

	for(unsigned int i=0; i<points.size(); i++)
	{
		if (labels.at(i) == -2)
		{
			Vector3 v3Obj = points.at(i).p;
			if (v3Obj.x>max_x) max_x = v3Obj.x;
			if (v3Obj.x<min_x) min_x = v3Obj.x;
			if (v3Obj.y>max_y) max_y = v3Obj.y;
			if (v3Obj.y<min_y) min_y = v3Obj.y;
			if (v3Obj.z>max_z) max_z = v3Obj.z;
			if (v3Obj.z<min_z) min_z = v3Obj.z;
		}
	}
	return sqrt((max_x-min_x)*(max_x-min_x)+(max_y-min_y)*(max_y-min_y)+(max_z-min_z)*(max_z-min_z))/40;
}

SOIPtr PlanePopOut::createObj(Vector3 center, Vector3 size, double radius, VisionData::SurfacePointSeq psIn1SOI, VisionData::SurfacePointSeq BGpIn1SOI, VisionData::SurfacePointSeq EQpIn1SOI)
{
	VisionData::SOIPtr obs = new VisionData::SOI;
	obs->boundingBox.pos.x = obs->boundingSphere.pos.x = center.x;
	obs->boundingBox.pos.y = obs->boundingSphere.pos.y = center.y;
	obs->boundingBox.pos.z = obs->boundingSphere.pos.z = center.z;
	obs->boundingBox.size.x = size.x;
	obs->boundingBox.size.y = size.y;
	obs->boundingBox.size.z = size.z;	//cout<<"radius in SOI = "<<radius<<endl;
	obs->boundingSphere.rad = radius;       //cout<<"radius in SOI (obs) = "<<obs->boundingSphere.rad<<endl;
	obs->time = getCASTTime();
	obs->points = psIn1SOI;//cout<<"points in 1 SOI = "<<obs->points.at(1).p<<obs->points.at(2).p<<obs->points.at(10).p<<endl;
	obs->BGpoints =	BGpIn1SOI;
	obs->EQpoints =	EQpIn1SOI;//cout<<"EQ points in 1 SOI = "<<obs->EQpoints.size()<<endl;

	return obs;
}

bool PlanePopOut::Compare2SOI(ObjPara obj1, ObjPara obj2)
{
	if (sqrt((obj1.c.x-obj2.c.x)*(obj1.c.x-obj2.c.x)+(obj1.c.y-obj2.c.y)*(obj1.c.y-obj2.c.y)+(obj1.c.z-obj2.c.z)*(obj1.c.z-obj2.c.z))<0.25*obj1.r && obj1.r/obj2.r>0.5 && obj1.r/obj2.r<2)
		return true; //the same object
	else	
		return false; //not the same one
}



}

