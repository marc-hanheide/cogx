/**
 * @file main
 * @author Johann Prankl
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/

#include <vector>
#include <time.h>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <limits.h>
#include "PNamespace.hh"
#include "DShapeCore.hh"
#include "TomGine/tgEngine.h"
#include "TomGine/tgTexture.h"
#include "Draw.hh"
#include "Scene3D.hh"
#include "RASDescriptor.hh"
#include "Config.hh"
#include "Except.hh"


//#define LOG_IMAGES

#define NAME "Box1"
#define FILENAME "log/testRAS.txt"



using namespace P;
using namespace TomGine;

void TestRAS();

void Draw3D(Scene3D &scene);
void* ThreadDrawing(void* c);

Scene3D scene;
int sharedKey;
pthread_mutex_t keyMutex, sceneMutex;



/******************************** main *********************************/
int main ( int argc, char** argv ) 
{
  if( argc != 2)
  {
    printf("USAGE: %s cfg/config.txt\n",argv[0]);
    return 0;
  }

  char file[PATH_MAX];
  Config cfg(argv[1]);
 
  strcpy(file,cfg.GetValueString("INTRINSIC1").c_str());
  CvMat *intrinsic1 = (CvMat*)cvLoad(file);
  strcpy(file,cfg.GetValueString("INTRINSIC2").c_str());
  CvMat *intrinsic2 = (CvMat*)cvLoad(file);
  strcpy(file,cfg.GetValueString("DISTORTION1").c_str());
  CvMat *distortion1 = (CvMat*)cvLoad(file);
  strcpy(file,cfg.GetValueString("DISTORTION2").c_str());
  CvMat *distortion2 = (CvMat*)cvLoad(file);
  strcpy(file,cfg.GetValueString("CAM12_ROT").c_str());
  CvMat *rod = (CvMat*)cvLoad(file);
  strcpy(file,cfg.GetValueString("CAM12_TRANS").c_str());
  CvMat *trans = (CvMat*)cvLoad(file);

  if (intrinsic1==0 || intrinsic2==0 || distortion1==0 || distortion2==0 || rod==0 || trans==0)
    throw Except(__HERE__,"Camera parameter file(s) not found!");

  double rot[9];
  CvMat matRot = cvMat(3,3,CV_64F,rot);
  cvRodrigues2(rod,&matRot);

  cvNamedWindow ( "Image1", 1 );
  cvNamedWindow ( "Image2", 1 );

  pthread_t thread;
  pthread_mutex_init(&keyMutex,NULL);
  pthread_mutex_init(&sceneMutex,NULL);
  pthread_create(&thread, NULL, ThreadDrawing, 0);

  IplImage *img1= 0;
  IplImage *img2= 0;
  IplImage *mask1 = 0;
  IplImage *dbg1=0, *dbg2=0;

  // ------------------ Init --------------------
  DShapeCore dshape;  
  dshape.SetCameraParameter(intrinsic1, intrinsic2, distortion1, distortion2, &matRot, trans);
  RASDescriptor ras;
  // --------------------------------------------

  int key;
  struct timespec start1, end1;

  for (;;)
  {
    strcpy(file,cfg.GetValueString("IMAGE1").c_str());
    cout<<file<<endl;
    img1 = cvLoadImage ( file,CV_LOAD_IMAGE_GRAYSCALE );
    strcpy(file,cfg.GetValueString("IMAGE2").c_str());
    cout<<file<<endl;
    img2 = cvLoadImage ( file,CV_LOAD_IMAGE_GRAYSCALE );
    strcpy(file,cfg.GetValueString("MASK1").c_str());
    cout<<file<<endl;
    mask1 = cvLoadImage ( file,CV_LOAD_IMAGE_GRAYSCALE );

    if (img1==0 || img2==0 || mask1==0)
      throw Except(__HERE__,"Image file(s) not found!");

    if ( dbg1==0 ) 
    {
        dbg1 = cvCreateImage ( cvGetSize ( img1 ), 8, 3 );
        dbg2 = cvCreateImage ( cvGetSize ( img2 ), 8, 3 );
    }

    cvConvertImage(img1, dbg1);
    cvConvertImage(img2, dbg2);

    clock_gettime(CLOCK_REALTIME, &start1); //CLOCK_THREAD_CPUTIME_ID

    //------------------- process images ----------------------
      
    dshape.SetDebugImage(dbg1, dbg2);
    dshape.Operate(img1, img2, ras, mask1);

    cout<<"--"<<endl;
    for (unsigned i=0; i<ras.Size(); i++)
      cout<<ras.data[i]<<" ";
    cout<<endl<<"--"<<endl;
    
    // --------------------------------------------------------   
    clock_gettime(CLOCK_REALTIME, &end1);

    cout<<"Time [s]: "<<P::timespec_diff(&end1, &start1)<<endl;

    #ifdef LOG_IMAGES
    char file[1024];
    snprintf(file,1024,"log/result-left.jpg");
    cvSaveImage(file,dbg1);
    snprintf(file,1024,"log/result-right.jpg");
    cvSaveImage(file,dbg2);
    #endif

    /*ofstream output(FILENAME,ios_base::app);
    ras.SaveDescriptor(output,&ras, NAME);
    output.close();
    TestRAS();*/
    
    cvShowImage ( "Image1", dbg1 );
    cvShowImage ( "Image2", dbg2 );

    //copy 3d stuff
    pthread_mutex_lock(&sceneMutex);
    dshape.GetScene(scene);
    pthread_mutex_unlock(&sceneMutex);
      
    do{
        key = cvWaitKey ( 10 );
        pthread_mutex_lock(&keyMutex);
        sharedKey = key;
        pthread_mutex_unlock(&keyMutex);
    }while ((((char)key)!=' ') && (((char)key) != 27));
    if (((char)key) == 27) break; 

    cvReleaseImage ( &img1 );
    cvReleaseImage ( &img2 );
    cvReleaseImage ( &mask1 );
  }


  cvWaitKey(500);

  if (dbg1!=0)
  {
    cvReleaseImage ( &dbg1 );
    cvReleaseImage ( &dbg2 );
  }

  cvDestroyWindow ( "Image1" );
  cvDestroyWindow ( "Image2" );

  cvReleaseMat(&intrinsic1), cvReleaseMat(&intrinsic2);
  cvReleaseMat(&distortion1), cvReleaseMat(&distortion2);
  cvReleaseMat(&rod), cvReleaseMat(&trans);

  pthread_mutex_destroy(&keyMutex);
  pthread_mutex_destroy(&sceneMutex);

  return 0;
}



/************************** SOME METHODES *************************/

RGBColor col;
static map<unsigned, RGBColor> cols;
/**
 * Draw all the opengl stuff.....
 */
void Draw3D(Scene3D &scene)
{
  glDisable(GL_CULL_FACE);
  glDisable(GL_LIGHTING);

  for (unsigned i=0; i<scene.ids.Size(); i++)
  {
    map<unsigned,RGBColor>::iterator it = cols.find(scene.ids[i]);
    if( it == cols.end() )
    {
      col = RGBColor(rand()%255,rand()%255,rand()%255);
      cols[scene.ids[i]] = col;
    }
    else
      col = cols[scene.ids[i]];

    for (unsigned j=0; j<scene.cs[i].Size(); j++)
    {
      DrawCross3D(scene.cs[i][j].x,scene.cs[i][j].y, scene.cs[i][j].z, 0.003, col);
    }
    for (unsigned j=0; j<scene.contours[i].Size(); j++)
    {
      Vector3 &p1 = scene.contours[i][j];
      Vector3 &p2 = (j+1<scene.contours[i].Size()?scene.contours[i][j+1]:scene.contours[i][0]);
      DrawLine3D(p1.x,p1.y,p1.z,p2.x,p2.y,p2.z,col);
    }
  }
}


/**
 * opengl drawing
 */
void* ThreadDrawing(void* c)
{
  cout<<"INIT 3D RENDERING"<<endl;    // then wie can use 3d rendering
  tgEngine render;
  render.Init(640,480, 10.0, 0.01, "TomGine Render Engine", true);
  bool end=false;

  while (!end)
  {
    pthread_mutex_lock(&sceneMutex);
    Draw3D(scene);
    render.Update();
    pthread_mutex_unlock(&sceneMutex);

    clock_t goal = 10 + clock();  //just sleep 10ms
    while (goal > clock());

    pthread_mutex_lock(&keyMutex);
    if ((char)sharedKey == 27)
      end=true;
    pthread_mutex_unlock(&keyMutex);
  }

  return((void *)0);
}




void TestRAS()
{
  string name;
  Array<RASDescriptor> rass;
  Array<string> nn;

  int cnt;

  ifstream input(FILENAME);
  for (cnt=0;;cnt++)
  {
    if (input.eof())
      break;

    rass.PushBack(RASDescriptor());
    RASDescriptor::LoadDescriptor(input, &rass.Last(), name);

    nn.PushBack(name);

    if (input.eof())
      break;
  }
  input.close();
 
  for (int i=0; i<cnt; i++)
  {
    cout<<nn[i]<<" ";
    for (int j=0; j<cnt; j++)
    {
      cout<<RASDescriptor::Compare(&rass[i], &rass[j])<<" ";
    }
    cout<<endl;
  } 
}









