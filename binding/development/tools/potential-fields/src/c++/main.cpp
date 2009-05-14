#include <iostream>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <fstream>
#include <stdlib.h>
//#include <conio.h>


#include "myVector.h"
#include "potentialField.h"
#include "scene.h"
#include <math.h> //we need cos(..) and sin(..)

#include "ProximityMap.h"
#include "ProjectiveMap.h"

#ifdef __APPLE__

#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>

#else

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>


#endif


using namespace std;


/**********************************************************************
 ** graphics Functions
 **********************************************************************/

typedef myVector<float> vector_type;
typedef potentialField<float> pf_type;
typedef scene<float> scene_type;

pf_type* renderedField;
scene_type* scene_pf;

vector_type* viewerPosition;
vector_type* landmarkPosition;
vector_type* originPosition;

bool sweetSpot;
string sName;

int width;
int height;
  
const float DEG2RAD = 3.14159/180;

ProximityMap * proxMap;
//ProjectiveMap * proxMap;
 
void drawCircle(float radius, float x, float y)
{
  //   glBegin(GL_LINE_LOOP);
  glBegin(GL_LINES);
 
  for (int i=0; i < 361; i++)
    {
      float degInRad = i*DEG2RAD;
      glVertex2f(x, y);
      glVertex2f(x + cos(degInRad)*radius, y + sin(degInRad)*radius);
    }
 
  glEnd();
}

void renderScene(void) {

  float scene_row = renderedField->getRow();
  float scene_column = renderedField->getColumn();
	
  float** DrawingArray = renderedField->getPotentialField();

  if(sweetSpot) {
    for(int i = 0; i < 2; i++) {
      float x,y,val;
      
      proxMap->nextSweetSpot(x,y,val);
      
      vector_type* maxloc = new vector_type(x,y,0);
      
      cout << "sweet spot = ";
      maxloc->outputVector();
      cout << "val = " << val << endl;
      
      float topleftx = maxloc->getX() - 5;
      float toplefty = maxloc->getY() - 5;
      float botrightx = maxloc->getX() + 5;
      float botrighty = maxloc->getY() + 5;
      glColor3f(0.0, 0.0, 1.0);
			
      glRectf(topleftx, toplefty, botrightx, botrighty);
      //scene_pf->addAndApplyDistractor(scene_row,scene_column,maxloc,16);
      //
      renderedField->fillPF(1);
      //renderedField->mergeByMultiplication(scene_pf->getPotentialField());
      renderedField->mergeByMultiplication(proxMap->getSceneField()->getPotentialField());
    }
  }

  if(DrawingArray) {
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    ///clear the window
    glClear(GL_COLOR_BUFFER_BIT);

    int ri, ci;
    glPointSize(1.0);
    glBegin(GL_POINTS);
    cout << "r = " << scene_row << " c = " << scene_column << endl;
    for(float r = 0; r < scene_row; r++) {
      for(float c = 0; c < scene_column; c ++) {
	ri = (int)r;
	ci = (int)c;
	glColor3f(DrawingArray[ri][ci], DrawingArray[ri][ci], DrawingArray[ri][ci]);
	glVertex2f(r,c);
	
      }
    }
    glEnd();
  } else {
    printf("DrawingArray == NULL\n");
  }

	
  if(viewerPosition) {
    glColor3f(1.0, 1.0, 0.0);
    //		glRectf(viewerPosition->getX()-5, viewerPosition->getY()-5, viewerPosition->getX()+5, viewerPosition->getY()+5);
		
    glBegin(GL_TRIANGLES);						// Drawing Using Triangles
    glVertex3f(viewerPosition->getX(), viewerPosition->getY(), 0.0f);				// Top
    glVertex3f(viewerPosition->getX()-10,viewerPosition->getY()-10, 0.0f);				// Bottom Left
    glVertex3f(viewerPosition->getX()+10,viewerPosition->getY()-10, 0.0f);				// Bottom Right
    glEnd();			
  }

  if(landmarkPosition) {
    glColor3f(1.0, 0.0, 0.0);
    drawCircle(6,landmarkPosition->getX(),landmarkPosition->getY());
    glRectf(landmarkPosition->getX()-5, landmarkPosition->getY()-5, landmarkPosition->getX()+5, landmarkPosition->getY()+5);
  }

  vector<pf_type*> distractor_pf_ptr_vec = scene_pf->getDistractorsVec();
  int vec_size = distractor_pf_ptr_vec.size();
  vector_type* pf_origin_vec_ptr;
  for(int i = 0; i < vec_size; i++) {
    glColor3f(0.0, 1.0, 0.0);
    pf_origin_vec_ptr = distractor_pf_ptr_vec[i]->getOrigin();
    glRectf(pf_origin_vec_ptr->getX()-5, pf_origin_vec_ptr->getY()-5, pf_origin_vec_ptr->getX()+5, pf_origin_vec_ptr->getY()+5);
  }            


  //start	
  //  glColor3f(0.0, 0.0, 1.0);
  //glRectf(245, 245, 255, 255);
  //glRectf(245, 145, 255, 155);


  //end	
  //	glColor3f(0.0, 0.0, 1.0);
  //	glRectf(167, 195, 177, 205);
  //	glRectf(150, 195, 160, 205);
	
  // glColor3f(1.0, 0.0, 0.0);                                                                     
  //drawCircle(6,200,200);

  glFlush();
}

void initGlut(int argc, char **argv) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);
  glutInitWindowPosition(-1,-1);
  glutInitWindowSize(width,height);
  glutCreateWindow(sName.c_str());
	
  gluOrtho2D(0,width,0,height);
  //gluOrtho2D
  glutDisplayFunc(renderScene);
  glutMainLoop();
}

/*************************************************************************
 ** Main
 *************************************************************************/

int main(int argc, char **argv) {



  /*opengl coordinates:
   * (0, 0) = bottom left
   */

  sName = "Nicks Scene";
  sweetSpot = false;



  //the coordinates work with bottom left = 00 topright = row, column
  //this is primarily because thats the way opengl seems to draw it so
  //if you want the middle of your scene to be 0, 0 and position
  //everything else relative to that I'd suggest you add row/2 to your
  //x values and column/2 to your y values set up the extent of scene
  //- ignore scene origin just set it to 0,0,0
	
  int x_offset = 0; //width/2;
  int y_offset = 0; //height/2;

  float scale = 10;
  
  width = 100 * scale;
  height = 100 * scale;

  proxMap = new ProximityMap(width,height);
  // proxMap = new ProjectiveMap(width,height,ProjectiveMap::LEFT_PROJ);
  //proxMap = new ProjectiveMap(width,height,ProjectiveMap::RIGHT_PROJ);
  //proxMap = new ProjectiveMap(width,height,ProjectiveMap::FRONT_PROJ);
  //proxMap = new ProjectiveMap(width,height,ProjectiveMap::BACK_PROJ);
	
  //	
	
  renderedField = new pf_type(width, height, new vector_type(0, 0, 0));
  scene_pf = new scene_type(width, height, new vector_type(0, 0, 0));
	
  //viewer at (0,-200)

  float vX = 50 * scale;
  float vY = -50 * scale;
  
  viewerPosition = new vector_type(vX, vY, 0);	
  proxMap->setViewerPosition(vX, vY, 0);

  //landmark at (50,50)

  float obj0X = 55 * scale;
  float obj0Y = 45 * scale;
  
  landmarkPosition = new vector_type(obj0X, obj0Y, 0);
  scene_pf->setLandmark(width, height, landmarkPosition);
  proxMap->addObject("obj0", obj0X, obj0Y, 0);
	


  float obj1X = 45 * scale;
  float obj1Y = 45 * scale;

  scene_pf->addDistractor(width, height, new vector_type(obj1X, obj1Y, 0));
  proxMap->addObject("obj1", obj1X, obj1Y, 0);
	

  float obj2X = 45 * scale;
  float obj2Y = 55 * scale;

  scene_pf->addDistractor(width, height, new vector_type(obj2X, obj2Y, 0));
  proxMap->addObject("obj2", obj2X, obj2Y, 0);


  float obj3X = 45 * scale;
  float obj3Y = 48 * scale;

  scene_pf->addDistractor(width, height, new vector_type(obj3X, obj3Y, 0));
  proxMap->addObject("obj3", obj3X, obj3Y, 0);

  //	scene_pf->addDistractor(width, height, new vector_type(172+x_offset, 200+y_offset, 0));
  //	scene_pf->addDistractor(width, height, new vector_type(155+x_offset, 200+y_offset, 0));

  //with distractors
  //scene_pf->computeProximityWaypoint(0.095,0.9);
  
  //scene_pf->computeProjectiveWaypoint(viewerPosition, "back", 90, 0.9, 16);

  //cout<<"proxVal: "<<proxMap->proximityValue("obj3","obj0")<<endl;
 
  //  cout<<"proxVal 3 2: "<<proxMap->proximityValue("obj3","obj2")<<endl;
  //cout<<"proxVal 3 1: "<<proxMap->proximityValue("obj3","obj1")<<endl;
  //cout<<"proxVal 3 0: "<<proxMap->proximityValue("obj3","obj0")<<endl;
  //cout<<"proxVal 2 3: "<<proxMap->proximityValue("obj2","obj3")<<endl;
  //cout<<"proxVal 2 1: "<<proxMap->proximityValue("obj2","obj1")<<endl;
  //cout<<"proxVal 2 0: "<<proxMap->proximityValue("obj2","obj0")<<endl;
  //  cout<<"proxVal 1 3: "<<proxMap->proximityValue("obj1","obj3")<<endl;
  //cout<<"proxVal 1 2: "<<proxMap->proximityValue("obj1","obj2")<<endl;
  //cout<<"proxVal 1 0 : "<<proxMap->projectiveValue("obj1","obj0")<<endl;
  //cout<<"proxVal 0 3: "<<proxMap->proximityValue("obj0","obj3")<<endl;
  //cout<<"proxVal 0 2: "<<proxMap->proximityValue("obj0","obj2")<<endl;
  //cout<<"proxVal 0 1: "<<proxMap->proximityValue("obj0","obj1")<<endl;


  //  vector<string> distractors;
  //distractors.push_back("obj0");
  //distractors.push_back("obj2");
  //distractors.push_back("obj3");
  //distractors.push_back("obj1");
  //proxMap->makeMap("obj0", distractors, true);
  
  proxMap->makeMap("obj0");
  //  proxMap->makeMap("obj1");
  //proxMap->makeMap("obj2");
  //  proxMap->makeMap("obj3");

  //renderedField->mergeByMultiplication(scene_pf->getPotentialField());
  renderedField->mergeByMultiplication(proxMap->getSceneField()->getPotentialField());


 

  //  proxMap->makeMap("obj2");


  //  for(int i = 0; i < 10; i++){
  //float x=-1,y=-1,val=-1;
  
  //proxMap->nextSweetSpot(x,y,val);
    
  //cout << "sweet spot = "<<x<<","<<y<<" "<<val<<endl;
  //}

  //scene_pf->addAndApplyDistractor(width, height, new vector_type(150+x_offset, 150+y_offset, 0),15);
  //scene_pf->addAndApplyDistractor(width, height, new vector_type(150+x_offset, 250+y_offset, 0),15);
  //scene_pf->addAndApplyDistractor(width, height, new vector_type(250+x_offset, 250+y_offset, 0));


  //for non-distractors
  //scene_pf->computeProximityWaypoint(0.8,0.9);

  //	scene_pf->fillScene(0);
  //		scene_pf->addAndApplyDistractor(width, height, new vector_type(201+x_offset, 180+y_offset, 0),15);	
  //		scene_pf->addAndApplyDistractor(width, height, new vector_type(201+x_offset, 220+y_offset, 0),15);	
  //
  //scene_pf->addDistractor(width, height, new vector_type(150+x_offset, 150+y_offset, 0));
  //	scene_pf->addDistractor(width, height, new vector_type(150+x_offset, 250+y_offset, 0));


	
  //	scene_pf->computeProjectiveWaypoint(viewerPosition, "left", 90, 0.9, 15);


	


  //  cout<<"obj1: "<<proxMap->getFieldValue("obj1")<<endl;
  //cout<<"obj2: "<<proxMap->getFieldValue("obj2")<<endl;
  //cout<<"obj3: "<<proxMap->getFieldValue("obj3")<<endl;
  //cout<<"obj4: "<<proxMap->getFieldValue("obj4")<<endl;

  //	cout << "scene pf: 200,200 = "<<scene_pf->getPFValue(200+x_offset,200+y_offset)<<endl;
  //	cout << "scene pf: 150,150 = "<<scene_pf->getPFValue(150+x_offset,150+y_offset)<<endl;	
  //	cout << "scene pf: 100,100 = "<<scene_pf->getPFValue(100+x_offset,100+y_offset)<<endl;
  //		cout << "scene pf: 100,50 = "<<scene_pf->getPFValue(100+x_offset,50+y_offset)<<endl;
  //	cout << "scene pf: 50,50 = "<<scene_pf->getPFValue(50+x_offset,50+y_offset)<<endl;
  //	cout << "scene pf: 0,0 = "<<scene_pf->getPFValue(0+x_offset,0+y_offset)<<endl;
  //	
  //		cout << "scene pf: 390,462 = "<<scene_pf->getPFValue(390+x_offset,462+y_offset)<<endl;
  //		cout << "scene pf: 390,462 = "<<scene_pf->getPFValue(390,462)<<endl;

	
  //	
  //	scene_pf->fillScene(1);
  //	renderedField->fillPF(1);
  //
  //
  ////move landmakr landmark at (100,100)
  //	landmarkPosition = new vector_type(100+x_offset, 100+y_offset, 0);
  //	
  //	scene_pf->setLandmark(width, height, landmarkPosition);
  //	
  //	scene_pf->computeProjectiveWaypoint(viewerPosition, "right", 90, 0.9, 15);
  //
  //	scene_pf->addAndApplyDistractor(width, height, landmarkPosition, 15);
  //
  //
  //	renderedField->mergeByMultiplication(scene_pf->getPotentialField());
  //
  //	cout << "scene pf: 200,200 = "<<scene_pf->getPFValue(200+x_offset,200+y_offset)<<endl;
  //	cout << "scene pf: 150,150 = "<<scene_pf->getPFValue(150+x_offset,150+y_offset)<<endl;	
  //	cout << "scene pf: 100,100 = "<<scene_pf->getPFValue(100+x_offset,100+y_offset)<<endl;
  //		cout << "scene pf: 100,50 = "<<scene_pf->getPFValue(100+x_offset,50+y_offset)<<endl;
  //		cout << "scene pf: 50,50 = "<<scene_pf->getPFValue(50+x_offset,50+y_offset)<<endl;
  //	cout << "scene pf: 0,0 = "<<scene_pf->getPFValue(0+x_offset,0+y_offset)<<endl;

	
	
	
  //code to illustrate projective pf
  /*	if(landmarkPosition) {		
    scene_pf = new scene_type(row, column, originPosition);
    scene_pf->setLandmark(row, column, landmarkPosition);
    scene_pf->addDistractor(row, column, new vector_type(50, 256, 0));
    scene_pf->addDistractor(row, column, new vector_type(100, 256, 0));
    scene_pf->addDistractor(row, column, new vector_type(150, 256, 0));
    scene_pf->addDistractor(row, column, new vector_type(200, 256, 0));
    scene_pf->addDistractor(row, column, new vector_type(225, 256, 0));
    scene_pf->computeProjectiveWaypoint(viewerPosition, "left", 90, 0.9, 15);
    renderedField->mergeByMultiplication(scene_pf->getPotentialField());
    }*/
	 
  //code to illustrate proximity
  /*	 if(landmarkPosition) {		
    scene_pf = new scene_type(row, column, originPosition);
    scene_pf->setLandmark(row, column, landmarkPosition);
    //scene_pf->addDistractor(row, column, new vector_type(50, 256, 0));
    //scene_pf->addDistractor(row, column, new vector_type(100, 256, 0));
    scene_pf->addDistractor(row, column, new vector_type(400, 400, 0));
    scene_pf->addDistractor(row, column, new vector_type(200, 256, 0));
    //scene_pf->addDistractor(row, column, new vector_type(225, 256, 0));
    scene_pf->computeProximityWaypoint(0.1);
    renderedField->mergeByMultiplication(scene_pf->getPotentialField());
    }*/

	 
  initGlut(argc, argv);

  return 0;
}


