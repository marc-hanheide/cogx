
#include <iostream>
#include <sstream>

#include <opencv/highgui.h>

#include <vision/VisionGoals.h>
#include <vision/idl/Vision.hh>
#include <vision/utils/VisionUtils.h>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "CarClassifier.h"

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new CarClassifier(_id);
  }
}

CarClassifier::CarClassifier(const string &_id): 
    WorkingMemoryAttachedComponent(_id),
    VideoClientProcess(_id),
    camera_(0),
    lastframe_(0),_hogjbcw(0),
    image_classes(0) { //ManagedProcess(_id) {
//  setOntology(VisionOntologyFactory::getOntology()); // <- what is this good for ? A: nothing, it's obsolete
  cout << "CarClassifier::[put constructor here]"<<endl;
  cvNamedWindow( "ROI", 1);
  cvNamedWindow( "classes",1);
  cvNamedWindow( "crop",1 );
  image_cropped = cvCreateImage (cvSize(228,212),8,3); //fixed for current descriptor size
}

CarClassifier::~CarClassifier() {
  cout << "CarClassifier::[put destructor here]"<<endl;
  if(lastframe_) 
    delete lastframe_;
  if(_hogjbcw)
    delete _hogjbcw;
  if(image_classes);
   cvReleaseImage(&image_classes);
}
void CarClassifier::configure(map<string,string> & _config) {
  ManagedProcess::configure(_config);
  cout << "CarClassifier::[put configuration here]"<<endl;
  std::map<std::string,std::string>::const_iterator i;
  std::string confFile;
  std::string modelFile;
  std::string cvimgFile;
  std::string ilFile;

  bool trainModel=false;
  int theta=50;
  int boostRounds=100;

  bool modelInitDone = true;
  i=_config.find("-hconf");
  if(i!=_config.end()){
    confFile = i->second;
    cout << "CarClassifier::HOGconfig: " <<confFile<<endl;  
  } else{
    cout << "you mus specify a HoG config file!"<<endl;
  }
  
  i=_config.find("-jbmodel");
  if(i!=_config.end()){
    modelFile = i->second;
    cout << "CarClassifier::JBModel: " <<modelFile<<endl;
  } else {
      i=_config.find("-trainjb");
      if(i!=_config.end()){
         trainModel=true;
	 cout << "TRAINING"<<endl;
	 i=_config.find("-il");
         if(i!=_config.end()){
           ilFile = i->second;
	   cout << "CarClassifier::ilFile: " <<ilFile<<endl;
	 } else {
	   cout << "No image annotation file specified for training!"<<endl;;
	   //exit();
	 }
         i=_config.find("-modelout");
         if(i!=_config.end()){
  	   modelFile = i->second;
           cout << "CarClassifier::JBModelOut: " <<modelFile<<endl;
	 }else{
	   cout << "No model filename given. TRAINED MODEL WILL NOT BE SAVED!"<<endl; 
	 }
         i=_config.find("-theta");
         if(i!=_config.end()){
            theta = atoi(i->second.c_str());
            cout << "CarClassifier::theta: " <<theta<<endl;
         }
	 i=_config.find("-boostrounds");
	 if(i!=_config.end()){
	    boostRounds = atoi(i->second.c_str());
	    cout << "CarClassifier::boostrounds: " <<boostRounds<<endl;
	 }
      }
  }

  i=_config.find("-cvimg");
  if(i !=_config.end()){
    cvimgFile = i->second;
    cout << "CarClassifier::ClassVisImage: " <<cvimgFile<<endl;
    cvNamedWindow( "classes",1);
    //image_classes =  cvLoadImage(cvimgFile.c_str()); 
    //cvShowImage( "classes", image_classes );
    _hogjbcw = new CarHOGJBWrapper(theta, cvimgFile.c_str());
   } else 
    _hogjbcw = new CarHOGJBWrapper(theta, 0);
  _hogjbcw->loadConfig(confFile.c_str());
  if(trainModel){
     cout << "TRAIN"<<endl;
    _hogjbcw->trainModel(ilFile.c_str(),modelFile.empty()?0:modelFile.c_str(),boostRounds);
  } else {
     cout <<"MODEL LOAD"<<endl;
    _hogjbcw->loadModel(modelFile.c_str());
  }
 
}
/*
void CarClassifier::configure(map<string,string> & _config) {
  ManagedProcess::configure(_config);
  cout << "CarClassifier::[put configuration here]"<<endl;
  std::map<std::string,std::string>::const_iterator i;
  std::string confFile;
  std::string modelFile;
  std::string cvimgFile;
  bool modelInitDone = true;
  i=_config.find("-hconf");
  if(i!=_config.end()){
    confFile = i->second;
    cout << "CarClassifier::HOGconfig: " <<confFile<<endl;  
  }
  i=_config.find("-jbmodel");
  if(i!=_config.end()){
    modelFile = i->second;
    cout << "CarClassifier::JBModel: " <<modelFile<<endl;
  }
  i=_config.find("-cvimg");
  if(i !=_config.end()){
    cvimgFile = i->second;
    cout << "CarClassifier::ClassVisImage: " <<cvimgFile<<endl;
    //cvNamedWindow( "classes",1);
    //image_classes =  cvLoadImage(cvimgFile.c_str()); 
    //cvShowImage( "classes", image_classes );
    _hogjbcw = new CarHOGJBWrapper(100, cvimgFile.c_str());
    _hogjbcw->loadConfig(confFile.c_str());
    _hogjbcw->loadModel(modelFile.c_str());
  } 
}*/

void CarClassifier::runComponent(){cout << "CarClassifier::sleeping..."<<endl; sleepProcess(1000);}

void CarClassifier::start() {
  ManagedProcess::start();
  //addChangeFilter(VisionOntology::SCENE_CHANGED_TYPE, cdl::OVERWRITE, true,
  //		  new MemberFunctionChangeReceiver<CarClassifier>(this,
  //							      &CarClassifier::HandleROIChange));
  addChangeFilter(createLocalTypeFilter<SceneChanged>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<CarClassifier>(this,
                                                              &CarClassifier::HandleROIChange));
}

void CarClassifier::taskAdopted(const string &_taskID) {}
void CarClassifier::taskRejected(const string &_taskID) {}                     
void CarClassifier::HandleROIChange(const cdl::WorkingMemoryChange &change) {
  shared_ptr<const CASTData<Vision::SceneChanged> > pSceneChangedData
    = getWorkingMemoryEntry<Vision::SceneChanged>(change.m_address);
  shared_ptr<const Vision::SceneChanged> sceneChanged = pSceneChangedData->getData();
  cout << "CarClassifier::HandleROIChange:" << endl << \
          "m_sceneChanging =" << sceneChanged->m_sceneChanging << endl << \
          "m_sceneChanged  =" << sceneChanged->m_sceneChanged << endl << \
          "m_sceneProcessed=" << sceneChanged->m_sceneProcessed <<endl;
  if(sceneChanged->m_sceneProcessed || sceneChanged->m_sceneChanged) {
    //grab image. should be ok since a fixed scene is assumed after m_sceneProcessed=true;
    delete lastframe_;
    lastframe_ = GetImage(camera_);

    unsigned char* data = &(lastframe_->m_image[0]);
    unsigned w = lastframe_->m_width;
    unsigned h = lastframe_->m_height;
    unsigned rowstep = lastframe_->m_image.length() / lastframe_->m_height;

    IplImage* tmp = 0;
    IplImage* iplImage = 0;

    iplImage = cvCreateImageHeader(cvSize(w,h), IPL_DEPTH_8U, 3);
    cvSetData(iplImage, data, rowstep);
    
    std::vector<shared_ptr<const CASTData<Vision::ROI> > > roisInWM;
    //getWorkingMemoryEntries(VisionOntology::ROI_TYPE, 0, roisInWM);
    // is this change correct ?
    getWorkingMemoryEntries<ROI>(0, roisInWM);
    cout << "SGMT: Num Input ROIs: " << roisInWM.size() << "\n";
    
    CvMat* tplate=0;
    //CvMat* tplate_cropped=0;
    
    for( std::vector<shared_ptr<const CASTData<Vision::ROI> > >::const_iterator
	 rit = roisInWM.begin(), rit_e = roisInWM.end();
         rit != rit_e; rit++) {
         const  Vision::ROI  *pROI = (*rit)->getData().get();  
         //get & display ROIs stored in WM
         IplImage* iplimage = cvCreateImageHeader(cvSize(pROI->m_region.m_width, pROI->m_region.m_height), IPL_DEPTH_8U, 3);
         cout << "w: " << pROI->m_region.m_width  << "h: " << pROI->m_region.m_height << " len: " << pROI->m_region.m_image.length()<<endl;
         cvSetData(iplimage, (unsigned char*)&(pROI->m_region.m_image[0]), pROI->m_region.m_image.length()/pROI->m_region.m_height);
         cvShowImage("ROI", iplimage);
         //cvWaitKey(1000);
         
         CvPoint pt1, pt2;
         //pt1.x = track_window.x; pt2.x = track_window.x+track_window.width;
         //pt1.y = track_window.y; pt2.y = track_window.y+track_window.height;
            
         pt1.x = (int)(pROI->m_bbox.m_center.m_x-pROI->m_bbox.m_size.m_x/2.0);    //size is width
         pt2.x = (int)(pROI->m_bbox.m_center.m_x+pROI->m_bbox.m_size.m_x/2.0);    
         pt1.y = (int)(pROI->m_bbox.m_center.m_y-pROI->m_bbox.m_size.m_y/2.0);    //size is height
         pt2.y = (int)(pROI->m_bbox.m_center.m_y+pROI->m_bbox.m_size.m_y/2.0);
         
         float gw=226.0; float gh=210.0;  float aspect=gw/gh;
         //float aspect_tracked = track_window.width/ track_window.height;
         float aspect_tracked = pROI->m_bbox.m_size.m_x / pROI->m_bbox.m_size.m_y;
         if(aspect_tracked < aspect){ // add width
           float new_width = aspect*  pROI->m_bbox.m_size.m_y;//track_window.height;
           int new_width_half = (int) ((new_width-pROI->m_bbox.m_size.m_x)/2.0);//track_window.width)/2.0;
           pt1.x = pt1.x-new_width_half;
           pt2.x = pt2.x+new_width_half;
              
         } else {  // add height
           float new_height = pROI->m_bbox.m_size.m_x/aspect;//track_window.width/aspect;
           int new_height_half = (int) ((new_height-pROI->m_bbox.m_size.m_y)/2.0);//track_window.height)/2.0;
           pt1.y = pt1.y-new_height_half;
           pt2.y = pt2.y+new_height_half; 
         }
            
         int width_new = pt2.x-pt1.x;
         int height_new = pt2.y-pt1.y;
         //image_tmp = cvCreateImage (cvSize(new_width,new_height),8,3);
         cout <<"crop x1,x2,y1,y2: " <<pt1.x <<" " <<pt2.x <<" " <<pt1.y <<" " <<pt2.y <<endl;
         if(pt1.x > 0 && pt2.x >0 && pt1.y > 0 && pt2.y >0 && pt1.x < w && pt2.x <w && pt1.y <h && pt2.y <h) {
           CvRect croprect = cvRect(pt1.x, pt1.y, width_new, height_new);
	   
           //create a template extracted from source image
           if(tplate) 
              cvReleaseMat(&tplate);
	   tplate = cvCreateMat(width_new, height_new, CV_8UC3);

           cout <<"getsubrect ..."<<endl;
	   cvGetSubRect(iplImage, tplate, croprect );
           cout <<"resize ..."<<endl;   
           cvResize(tplate, image_cropped);
           cout <<"show ..."<<endl;  
           //cvShowImage("crop", image_cropped);
           vector <float> cresults;
           _hogjbcw->classifyRawColorInterleaved((unsigned char*)image_cropped->imageData, image_cropped->width, image_cropped->height, image_cropped->widthStep, cresults);
	   float maxval=1e-31;
	   int maxclass=0;
           for(unsigned int c=0;c<cresults.size();c++){
	     if (cresults[c]>maxval){ maxval=cresults[c]; maxclass=c;}
             cout <<cresults[c]<<" ";
           }
	   cout << "class: "<<maxclass<<" score: "<<std::fixed<<maxval;
	   CvFont font;
	   double hScale=0.4;
	   double vScale=0.4;
	   int    lineWidth=1;
	   cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
           stringstream sstr;
	   sstr <<"class: "<<maxclass<<" score: "<<std::fixed<<maxval;
	   cvPutText (image_cropped, sstr.str().c_str(),cvPoint(10,50), &font, cvScalar(0,255,0));
	   cvShowImage("crop", image_cropped);
           cout <<endl;
           cvWaitKey(1000);
         } else {
           cout << "ROI boundaries out of frame ?!"<<endl;
         }
       }
  }
}


/*  
void CarClassifier::HandleROIChange(const cdl::WorkingMemoryChange &change) {
  shared_ptr<const CASTData<Vision::SceneChanged> > pSceneChangedData
    = getWorkingMemoryEntry<Vision::SceneChanged>(change.m_address);
  shared_ptr<const Vision::SceneChanged> sceneChanged = pSceneChangedData->getData();
  cout << "CarClassifier::HandleROIChange:" << endl << \
          "m_sceneChanging =" << sceneChanged->m_sceneChanging << endl << \
          "m_sceneChanged  =" << sceneChanged->m_sceneChanged << endl << \
          "m_sceneProcessed=" << sceneChanged->m_sceneProcessed <<endl;
  if(sceneChanged->m_sceneChanged || sceneChanged->m_sceneProcessed) {
    //grab image. should be ok since a fixed scene is assumed after m_sceneProcessed=true;
    delete lastframe_;
    lastframe_ = GetImage(camera_);

    unsigned char* data = &(lastframe_->m_image[0]);
    unsigned w = lastframe_->m_width;
    unsigned h = lastframe_->m_height;
    unsigned rowstep = lastframe_->m_image.length() / lastframe_->m_height;

    IplImage* tmp = 0;
    IplImage* iplImage = 0;

    iplImage = cvCreateImageHeader(cvSize(w,h), IPL_DEPTH_8U, 3);
    cvSetData(iplImage, data, rowstep);
    
    std::vector<shared_ptr<const CASTData<Vision::ROI> > > roisInWM;
    //getWorkingMemoryEntries(VisionOntology::ROI_TYPE, 0, roisInWM);
    // is this change correct ?
    getWorkingMemoryEntries<ROI>(0, roisInWM);
    cout << "SGMT: Num Input ROIs: " << roisInWM.size() << "\n";
    
    CvMat* tplate=0;
    //CvMat* tplate_cropped=0;
    
    for( std::vector<shared_ptr<const CASTData<Vision::ROI> > >::const_iterator
	 rit = roisInWM.begin(), rit_e = roisInWM.end();
         rit != rit_e; rit++) {
         const  Vision::ROI  *pROI = (*rit)->getData().get();  
         //get & display ROIs stored in WM
         IplImage* iplimage = cvCreateImageHeader(cvSize(pROI->m_region.m_width, pROI->m_region.m_height), IPL_DEPTH_8U, 3);
         cout << "w: " << pROI->m_region.m_width  << "h: " << pROI->m_region.m_height << " len: " << pROI->m_region.m_image.length()<<endl;
         cvSetData(iplimage, (unsigned char*)&(pROI->m_region.m_image[0]), pROI->m_region.m_image.length()/pROI->m_region.m_height);
         cvShowImage("ROI", iplimage);
         //cvWaitKey(1000);
         
         CvPoint pt1, pt2;
         //pt1.x = track_window.x; pt2.x = track_window.x+track_window.width;
         //pt1.y = track_window.y; pt2.y = track_window.y+track_window.height;
            
         pt1.x = (int)(pROI->m_bbox.m_center.m_x-pROI->m_bbox.m_size.m_x/2.0);    //size is width
         pt2.x = (int)(pROI->m_bbox.m_center.m_x+pROI->m_bbox.m_size.m_x/2.0);    
         pt1.y = (int)(pROI->m_bbox.m_center.m_y-pROI->m_bbox.m_size.m_y/2.0);    //size is height
         pt2.y = (int)(pROI->m_bbox.m_center.m_y+pROI->m_bbox.m_size.m_y/2.0);
         
         float gw=226.0; float gh=206.0;  float aspect=gw/gh;
         //float aspect_tracked = track_window.width/ track_window.height;
         float aspect_tracked = pROI->m_bbox.m_size.m_x / pROI->m_bbox.m_size.m_y;
         if(aspect_tracked < aspect){ // add width
           float new_width = aspect*  pROI->m_bbox.m_size.m_y;//track_window.height;
           int new_width_half = (int) ((new_width-pROI->m_bbox.m_size.m_x)/2.0);//track_window.width)/2.0;
           pt1.x = pt1.x-new_width_half;
           pt2.x = pt2.x+new_width_half;
              
         } else {  // add height
           float new_height = pROI->m_bbox.m_size.m_x/aspect;//track_window.width/aspect;
           int new_height_half = (int) ((new_height-pROI->m_bbox.m_size.m_y)/2.0);//track_window.height)/2.0;
           pt1.y = pt1.y-new_height_half;
           pt2.y = pt2.y+new_height_half; 
         }
            
         int width_new = pt2.x-pt1.x;
         int height_new = pt2.y-pt1.y;
         //image_tmp = cvCreateImage (cvSize(new_width,new_height),8,3);
         cout <<"crop x1,x2,y1,y2: " <<pt1.x <<" " <<pt2.x <<" " <<pt1.y <<" " <<pt2.y <<endl;
         if(pt1.x > 0 && pt2.x >0 && pt1.y > 0 && pt2.y >0 && pt1.x < w && pt2.x <w && pt1.y <h && pt2.y <h) {
           CvRect croprect = cvRect(pt1.x, pt1.y, width_new, height_new);
	   
           //create a template extracted from source image
           if(tplate) 
              cvReleaseMat(&tplate);
	   tplate = cvCreateMat(width_new, height_new, CV_8UC3);

           cout <<"getsubrect ..."<<endl;
	   cvGetSubRect(iplImage, tplate, croprect );
           cout <<"resize ..."<<endl;   
           cvResize(tplate, image_cropped);
           cout <<"show ..."<<endl;  
           cvShowImage("crop", image_cropped);
           vector <float> cresults;
           _hogjbcw->classifyRawColorInterleaved((unsigned char*)image_cropped->imageData, image_cropped->width, image_cropped->height, image_cropped->widthStep, cresults);
           for(unsigned int c=0;c<cresults.size();c++){
            cout <<cresults[c]<<" ";
           }
           cout <<endl;
           cvWaitKey(1000);
         } else {
           cout << "ROI boundaries out of frame ?!"<<endl;
         }
       }
  }
}
*/
