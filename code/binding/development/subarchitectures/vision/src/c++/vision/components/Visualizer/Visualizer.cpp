
#include <iostream>
#include <sstream>

#include <opencv/highgui.h>

#include <vision/VisionGoals.h>
#include <vision/idl/Vision.hh>
#include <vision/utils/VisionUtils.h>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "Visualizer.h"


extern "C" 
{
  FrameworkProcess* newComponent(const string &_id) 
 {
    return new Visualizer(_id);
 }
}


Visualizer::Visualizer(const string &_id) :
  WorkingMemoryAttachedComponent(_id),
  VideoClientProcess(_id) 
{
}


Visualizer::~Visualizer() 
{
 cvReleaseImage(&img);
}


void Visualizer::configure(map<string,string> & _config) 
{
  VideoClientProcess::configure(_config);
  ostringstream outStream;

  if(_config["-c"] != "") 
  {
   istringstream configStream(_config["-c"]);
   configStream >> m_camera;
   outStream.str("");
   outStream<<"setting camera: "<<m_camera;
   log(outStream.str());
  }

  if(_config["-f"] != "") 
  {
   istringstream configStream(_config["-f"]);
   configStream >> frameSkip;
   outStream.str("");
   outStream<<"reading WM every "<<m_camera<<" frames";
   log(outStream.str());
  }
  else
  {
   frameSkip=4;
  }


}


//draw objects from working memmory on visualization layer
void Visualizer::drawObjectsOnLayer()
{
 long i,v,h;
 long p1,p2,p3,p4;
 float x,y;

 h=lay->width;
 v=lay->height;

 //clear visualization layer
 for(i=0; i<h*v*3; i++) lay->imageData[i]=0;

 //write basic text data on image surface
 CvFont font;
 double hScale=0.5;
 double vScale=0.5;
 int    lineWidth=1;
 cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
 cvPutText (lay,"test text",cvPoint(10,25), &font, cvScalar(255,255,0));

 //DRAW ROIs
 //get the ROI list from working memmory
 std::vector<shared_ptr<const CASTData<Vision::ROI> > > roisInWM;
 getWorkingMemoryEntries<ROI>(0, roisInWM);
 // Initialize the table of ROIs
 int nr = roisInWM.size();
 float center_x[nr]; 
 float center_y[nr]; 
 float size_x[nr]; 
 float size_y[nr]; 
 string wmRaddr[nr];
 string wmSaddr[nr];
 // Fill in the table of ROIs  
 i=0;
 for (  std::vector<shared_ptr<const CASTData<Vision::ROI> > >::const_iterator
        wit = roisInWM.begin(), wit_e = roisInWM.end();
        wit != wit_e; wit++)
 {
  center_x[i]=(*wit)->getData().get()->m_bbox.m_center.m_x;
  center_y[i]=(*wit)->getData().get()->m_bbox.m_center.m_y;
  size_x[i]=(*wit)->getData().get()->m_bbox.m_size.m_x;
  size_y[i]=(*wit)->getData().get()->m_bbox.m_size.m_y;
  wmRaddr[i]=(*wit)->getData().get()->m_address;
  wmSaddr[i]=(*wit)->getData().get()->m_objId;
  i++;
 }
 //write ROI data on visualization layer
 for(i=0; i<nr; i++)
 {
  x=center_x[i]-(size_x[i]/2);
  y=center_y[i]-(size_y[i]/2);
  p1=((long)y*h)+(long)x;
  x=center_x[i]+(size_x[i]/2);
  y=center_y[i]-(size_y[i]/2);
  p2=((long)y*h)+(long)x;
  x=center_x[i]-(size_x[i]/2);
  y=center_y[i]+(size_y[i]/2);
  p3=((long)y*h)+(long)x;
  x=center_x[i]+(size_x[i]/2);
  y=center_y[i]+(size_y[i]/2);
  p4=((long)y*h)+(long)x;
  cvLine(lay, cvPoint(p1%h,p1/h), cvPoint(p2%h,p2/h), cvScalar(0,255,0), 1); //p1,p2
  cvLine(lay, cvPoint(p1%h,p1/h), cvPoint(p3%h,p3/h), cvScalar(0,255,0), 1); //p1,p4
  cvLine(lay, cvPoint(p3%h,p3/h), cvPoint(p4%h,p4/h), cvScalar(0,255,0), 1); //p3,p4
  cvLine(lay, cvPoint(p4%h,p4/h), cvPoint(p2%h,p2/h), cvScalar(0,255,0), 1); //p3,p2
 }



}


//merge visualization layer with live image
void Visualizer::drawLayerOnImage()
{
 unsigned char r,g,b;
 long h,v,i,j;

 h=lay->width;
 v=lay->height;

 //merge visualization layer with live image
 for(i=0; i<h*v; i++) 
 {
  j=3*i;
  b=lay->imageData[j];
  g=lay->imageData[j+1];
  r=lay->imageData[j+2];
  if((b!=0)||(g!=0)||(r!=0))
  {
   img->imageData[j]=lay->imageData[j];
   img->imageData[j+1]=lay->imageData[j+1];
   img->imageData[j+2]=lay->imageData[j+2];
  }
 }
}



void Visualizer::ShowResultingWindow() 
{
  int key;

  //pull the image from the videoserver
  getImage(m_camera,Image);
  //convert image
  for(long i=0; i<Image.m_width*Image.m_height*3; i++)
    img->imageData[i]=Image.m_image[i];

  //merge layer with live image
  drawLayerOnImage();

  //display image
  cvNamedWindow("Visualizer",CV_WINDOW_AUTOSIZE);
  cvShowImage("Visualizer", img);
  key=cvWaitKey(100);  
}



void Visualizer::runComponent()
{
  long num_of_frames;

  sleepProcess(2000);

  //capture first image
  getImage(m_camera,Image);
  
  //create image for live picture
  img = cvCreateImage(cvSize(Image.m_width, Image.m_height), IPL_DEPTH_8U, 3);  //color image

  //create image for rendering objects
  lay = cvCreateImage(cvSize(Image.m_width, Image.m_height), IPL_DEPTH_8U, 3);  //color image

  num_of_frames=0;
  while(true) 
  {
   //read data from WM
   if(num_of_frames%frameSkip==0) drawObjectsOnLayer(); //skip frameSkip frames for drawing on visualization layer

   //show window
   ShowResultingWindow();
  }

}

void Visualizer::start() 
{
 ManagedProcess::start();
}


void Visualizer::taskAdopted(const string &_taskID) 
{
}



void Visualizer::taskRejected(const string &_taskID)
{
}

