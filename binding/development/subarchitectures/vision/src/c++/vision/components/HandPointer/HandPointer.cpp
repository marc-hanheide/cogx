
#include <iostream>
#include <sstream>
#include <math.h>

#include <opencv/highgui.h>

#include <vision/VisionGoals.h>
#include <vision/idl/Vision.hh>
#include <vision/utils/VisionUtils.h>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "HandPointer.h"

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new HandPointer(_id);
  }
}


HandPointer::HandPointer(const string &_id) :
  
  WorkingMemoryAttachedComponent(_id),
  VideoClientProcess(_id)
{

  //initialization of parameter stack
  pstack.nmbr=0;
  pstack.size=2;
  pstack.position_hor = new long [pstack.size];
  pstack.position_ver = new long [pstack.size];
  pstack.direction    = new long [pstack.size];  

  //prepare first frame
  first_frame = true;

  //do not show opencv window if it is not configured
  showOpenCVWindow=false;

  //default minimum number of hand features in scene
  h_minPresent = 33;

  //default location of configuration file
  file_loc="config/handpointer/color.txt";

}


void HandPointer::configure(map<string,string> & _config) 
{
  ostringstream outStream;
  VideoClientProcess::configure(_config);

  if(_config["-c"] != "") 
  {
    istringstream configStream(_config["-c"]);
    configStream >> m_camera;

    outStream.str("");
    outStream<<"setting camera: "<<m_camera;
    log(outStream.str());
  }

  if(_config["-b"] != "")
  {
    file_loc = _config["-b"];
    outStream.str("");
    outStream<<"setting color configuration file: "<<file_loc;
    log(outStream.str());
  }

  if(_config["-w"] != "") 
  {
    showOpenCVWindow = true;
  }

}


void HandPointer::start() 
{
  ManagedProcess::start();
  
  addChangeFilter(createLocalTypeFilter<SceneChanged>(cdl::OVERWRITE),
   new MemberFunctionChangeReceiver<HandPointer>(this, &HandPointer::HandleSceneChanging));
}


void HandPointer::HandleSceneChanging(const cdl::WorkingMemoryChange &change)
{
  //get SceneChanged IDL structure from working memmory
  shared_ptr<const CASTData<Vision::SceneChanged> > pSceneChangedData
    = getWorkingMemoryEntry<Vision::SceneChanged>(change.m_address);
  shared_ptr<const Vision::SceneChanged> sceneChanged = pSceneChangedData->getData();
  
  // Get the ID for the next task
  string taskID = newTaskID();
  
  //test if scene is changing or has changed
  if(sceneChanged->m_sceneChanging)    
  {
    //propose a task to find salient roi.
    m_tasks[taskID] = VisionGoals::GET_POINTED_ROI_TASK;  //task for HandDetector 
    proposeInformationProcessingTask(taskID, VisionGoals::GET_POINTED_ROI_TASK);
    //remember the SceneChanged memory ID
    m_sceneStatusID = string(change.m_address.m_id);
  }
  
}


void HandPointer::taskAdopted(const string &_taskID) 
{
  // First, see what the task ID is about.
  std::map<string, string>::iterator tit =
    m_tasks.find(_taskID);

  if (tit != m_tasks.end()) 
  {
    // Get the task name and remove it from the map.
    string task = m_tasks[_taskID];
    m_tasks.erase(tit);

    // Now, decide what to do with this task.
    if (task == VisionGoals::GET_POINTED_ROI_TASK) 
    {
      GetPointedROI(_taskID);
    } 
  }

  // and now we're finished, tell the goal manager that the task is
  // over successfully (assuming it is... naughty!)
  taskComplete(_taskID, cdl::PROCESSING_COMPLETE_SUCCESS);
}

 
void HandPointer::GetPointedROI(const string &_taskID)
{
  //Capture and process the image
  grabAndProcessImage(m_camera); 

  //add handpointer results structure to working memory
  Vision::HandPointingResults *hpo = new Vision::HandPointingResults(); 

  //check if hand is present in scene
  hpo->m_handInScene=false;
  if(proc->hfeatures > h_minPresent) 
  {  
   hpo->m_handInScene=true;
   log("Hand is in scene"); 
  }

  //update handpointer results strucuture
  hpo->m_pointerDetected = detectedroi.is_detected;   
  hpo->m_ROIcenterX = detectedroi.cent_x;       
  hpo->m_ROIcenterY = detectedroi.cent_y;       
  hpo->m_ROIsizeX = detectedroi.roi_sx;       
  hpo->m_ROIsizeY = detectedroi.roi_sy;       
  hpo->m_PNTcenterX = hpointer.pos_x;             
  hpo->m_PNTcenterY = hpointer.pos_y;             
  hpo->m_PNTdirectX = hpointer.ang_x;             
  hpo->m_PNTdirectY = hpointer.ang_y;             
  hpo->m_ROIAddress = CORBA::string_dup(detectedroi.wm_addr.c_str()); 
  hpo->m_SceneObjAddress = CORBA::string_dup(detectedroi.wm_obj.c_str());
  hpo->m_cam = m_camera;

  //update handpointing results in working memory structure
  this->overwriteWorkingMemory<HandPointingResults>(this->m_memoryID, hpo);

  //if configured show the image window
  if(showOpenCVWindow==true)
  {
   IplImage *showimg = cvCreateImage(cvSize(frameW, frameH), IPL_DEPTH_8U, 1);
   for(long i=0; i<frameW*frameH; i++) showimg->imageData[i]=proc->image_se[i];  
   cvNamedWindow("Handpointer segmentation",CV_WINDOW_AUTOSIZE);
   cvShowImage("Handpointer segmentation", showimg);
   cvWaitKey(10);
   cvReleaseImage(&showimg);
  }
}

void HandPointer::grabAndProcessImage(int camNum)
{
  getImage(camNum,Image);
  if(first_frame==true)
  { 
   frameW=Image.m_width;
   frameH=Image.m_height;
   unsigned char* pimg=(unsigned char *)&(Image.m_image[0]);
   proc = new HandDetector(frameW,frameH,pimg,&file_loc[0]); 
   first_frame=false;
  }

  //Process the image
  proc->Run();

  //Check if image contains the model
  if((proc->res_mod.st_modelov>0)&&(proc->res_mod.koef_u_mod[0]>210))
  {
   //load the pointing parameters
   pstack.position_hor[pstack.nmbr]=proc->res_mod.polozaj_hor[0];
   pstack.position_ver[pstack.nmbr]=proc->res_mod.polozaj_ver[0];
   pstack.direction[pstack.nmbr]=proc->res_mod.kot[0];
   ++pstack.nmbr;
  }
  else //if model is not contained 
  {
   pstack.nmbr=0;  //clear the stack
  }

  //if stack is full
  if(pstack.nmbr==pstack.size)
  {
   //get pointer parameters
   getPointer();
   pstack.nmbr=0;  //clear the stack
   //get pointed ROI and update it in memmory
   if(hpointer.exist==true) findPointedROI();
  } 
}


void HandPointer::getPointer()   //calculating pointer parameters
{
  long i;
  float d_x=0.0,d_y=0.0;
  float a,a_x=0.0,a_y=0.0;
  float dd_x=0.0,dd_y=0.0;
  float da_x=0.0,da_y=0.0;

  //calculate the means
  for(i=0; i<pstack.size; i++)
  {
   d_x = d_x + pstack.position_hor[i];
   d_y = d_y + pstack.position_ver[i];
   a = M_PI*(float)pstack.direction[i]/500;
   a_x = a_x + cos(a);
   a_y = a_y + sin(a);
  }
  d_x=d_x/pstack.size;
  d_y=d_y/pstack.size;
  a_x=a_x/pstack.size;
  a_y=a_y/pstack.size;

  //calculate the deviations
  for(i=0; i<pstack.size; i++)
  {
   dd_x = dd_x + ((pstack.position_hor[i]-d_x)*(pstack.position_hor[i]-d_x));
   dd_y = dd_y + ((pstack.position_ver[i]-d_y)*(pstack.position_ver[i]-d_y));
   a = M_PI*(float)pstack.direction[i]/500;
   da_x = da_x + ((cos(a)-a_x)*(cos(a)-a_x));
   da_y = da_y + ((sin(a)-a_y)*(sin(a)-a_y));
  }
  dd_x=sqrt(dd_x/pstack.size);
  dd_y=sqrt(dd_y/pstack.size);
  da_x=sqrt(da_x/pstack.size);
  da_y=sqrt(da_y/pstack.size);

  //check the conditions for presence of pointer
  hpointer.exist=false;
  if((dd_x<5)&&(dd_y<5)&&(da_x<0.4)&&(da_y<0.4))
  {
   hpointer.exist=true;
   hpointer.pos_x=d_x;
   hpointer.pos_y=d_y;
   //correct the initial pointer rotation
   hpointer.ang_x = (a_x*cos(1.156))  -  (a_y*sin(1.156));
   hpointer.ang_y = (a_x*sin(1.156))  +  (a_y*cos(1.156));
   log("Detected pointer: position:(x=%f, y=%f)  direction:(x=%f, y=%f)",hpointer.pos_x,hpointer.pos_y,hpointer.ang_x,hpointer.ang_y); 
  }
}   

  
void HandPointer::findPointedROI()     
{
  int i;

  // Get the list of current ROIs in the working memory.
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

  //Find pointed ROI
  int startx = (int)hpointer.pos_x;
  int starty = (int)hpointer.pos_y;
  int deltax = (int)(hpointer.ang_x*10);
  int deltay = (int)(hpointer.ang_y*10);
  if(fabs((hpointer.ang_x*10)-(float)deltax)>0.5) 
  { 
    if(deltax>=0) ++deltax;
    if(deltax<0)  --deltax;
  }
  if(fabs((hpointer.ang_y*10)-(float)deltay)>0.5) 
  {
    if(deltay>=0) ++deltay;
    if(deltay<0)  --deltay;
  }

  bool detected = false;
  while((startx>0)&&(startx<frameW)&&(starty>0)&&(starty<frameH))  
  {
   for(i=0; i<nr; i++)
   {
    float minx = center_x[i] - (size_x[i]/2);
    float maxx = center_x[i] + (size_x[i]/2);
    float miny = center_y[i] - (size_y[i]/2);
    float maxy = center_y[i] + (size_y[i]/2);
    if((startx>minx)&&(startx<maxx)&&(starty>miny)&&(starty<maxy))
    {
     detected=true;
     break;
    } //if
   } //for
   if(detected==true) break;  //break if pointed ROI is detected 
   //increment pointer values according to its direction
   startx = startx + deltax;
   starty = starty + deltay;
  } //while

  //update resulting structure
  detectedroi.is_detected=false;
  if(detected==true)
  {
   detectedroi.is_detected=true;  
   detectedroi.cent_x=center_x[i];       
   detectedroi.cent_y=center_y[i];       
   detectedroi.roi_sx=size_x[i];       
   detectedroi.roi_sy=size_y[i];
   detectedroi.wm_addr=wmRaddr[i];
   detectedroi.wm_obj=wmSaddr[i];       
   log("Detected salient ROI: ROIaddress:%s  SceneObjectAddress:%s",wmRaddr[i].c_str(),wmSaddr[i].c_str());
  }
}


void HandPointer::runComponent()
{
 sleepProcess(2500); 
 //add handpointer results structure to working memory
 Vision::HandPointingResults *hpo = new Vision::HandPointingResults(); 
 hpo->m_cam = m_camera;
 hpo->m_pointerDetected=false;
 hpo->m_handInScene=false;
 this->m_memoryID = this->newDataID();
 this->addToWorkingMemory<HandPointingResults>(m_memoryID, hpo); 
}


HandPointer::~HandPointer()
{
 delete proc;
 delete [] pstack.position_hor;
 delete [] pstack.position_ver;
 delete [] pstack.direction;   
}


void HandPointer::taskRejected(const string &_taskID)
{
 //do nothing
}

