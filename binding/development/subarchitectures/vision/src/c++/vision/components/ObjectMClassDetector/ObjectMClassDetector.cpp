#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>

#include <opencv/highgui.h>

#include <vision/VisionGoals.h>
#include <vision/idl/Vision.hh>
#include <vision/utils/VisionUtils.h>
#include <cast/architecture/ChangeFilterFactory.hpp>

#include <libAnnotation/annotationlist.h>
#include <libPPProcess/nonmaxsuppression.h>

#include "ObjectMClassDetector.h"

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new ObjectMClassDetector(_id);
  }
}

ObjectMClassDetector::ObjectMClassDetector(const string &_id): 
    WorkingMemoryAttachedComponent(_id),
    VideoClientProcess(_id),
    camera_(0),
    lastframe_(0),
    imgdet_(0),
    fixwidth_(0),
    frameno_(0), 
    framestosave_(0),
    alwayson_(false),
    visualize_(false),
    nonmax_(false),
    saveone_(false),
    saveidl_(false){ 
    //ManagedProcess(_id) {
    //setOntology(VisionOntologyFactory::getOntology()); // <- what is this good for ?   
    
    cout << "ObjectMClassDetector::[constructor]"<<endl;
    std::ostringstream oss;
    srand((unsigned)time(0));
    oss << rand();
    outprefix = oss.str();
    numsavedidls_=0;
}

ObjectMClassDetector::~ObjectMClassDetector() {
  cout << "ObjectMClassDetector::[destructor]"<<endl;
  if(lastframe_) 
    delete(lastframe_);    
  if(imgdet_)
    cvReleaseImage(&imgdet_); 
}

void ObjectMClassDetector::configure(map<string,string> & _config) {
  ManagedProcess::configure(_config);
  cout << "ObjectMClassDetector::[configuration]"<<endl;
  std::map<std::string,std::string>::const_iterator i;
  std::string confFile;

  bool trainModel=false;
  bool modelInitDone = true;

  i=_config.find("-conf");
  if(i!=_config.end()){
    confFile = i->second;
    cout << "loading config file: " <<confFile<<endl;
    loadConfFile(confFile.c_str());
    cout << "conf loading done"<<endl;
  } else{
    cout << "you must specify a config file with config,model touples!"<<endl;
  }
  i=_config.find("-fixwidth");
  if(i!=_config.end()){
    fixwidth_ = atoi(i->second.c_str());
  }
  i=_config.find("-alwayson");
  if(i!=_config.end()){
    alwayson_ = true;
  }
  i=_config.find("-vis");
  if(i!=_config.end()){
    visualize_ = true;
  }
  i=_config.find("-nonmax");
  if(i!=_config.end()){
    nonmax_ = true;
  }
  i=_config.find("-saveidl");
  if(i!=_config.end()){
    saveidl_=true;
    framestosave_= atof(i->second.c_str());
    cout <<"saving detection hypotheses every "<<framestosave_<< " frames "<<endl;
    int numclasses = swMULTHOG_.m_vHOGSVM.size();
    if(numclasses > 0) {
      classannolists_.resize(numclasses);
      if(!nonmax_)
        classannolistsnms_.resize(numclasses);
    }
  }
}

void ObjectMClassDetector::loadConfFile(const char* file) {
  std::ifstream infile(file, std::ios_base::in);
  std::string line1;
  if(infile.good()) {
  while(getline(infile, line1, '\n')) {
    std::string line2;
    std::string line3;
    if(getline(infile, line2, '\n') && getline(infile, line3, '\n')) {
      cout <<"CONF: "<<line1<<" MODEL: "<<line2<<endl;
      swMULTHOG_.loadConfigAndModel(line1.c_str(),line2.c_str(), line3.c_str());
      
    }else {
      cout << "line1: "<<line1<<endl<< "line2: "<<line2<<endl<<"line3: "<<line3<<endl;
      cout << "is model file ok ?"<<endl;
      //throw BALTException(__HERE__, "check model configuration file");
    }
  }
  }else {
    std::ostringstream oss;
    oss << "Failed to open "<<file<<". Check ObjectMClassDetector component parameters.";
    throw BALTException(__HERE__, oss.str().c_str());
    //cout << "Failed to open "<<file<<". Check ObjectMClassDetector component parameters. Quitting."<<endl;
    //exit(0);
  }
}
int pal16[48]={
  255,  255,  255,
  255,    0,    0,
  255,  255,    0,
    0,  255,    0,
    0,  255,  255,
    0,    0,  255,
  255,    0,  255,
    0,    0,    0,
  128,  128,  128,
  128,    0,    0,
  128,  128,    0,
    0,  128,    0,
    0,  128,  128,
    0,    0,  128,
  128,    0,  128,
  191,  191,  191};
  
void ObjectMClassDetector::processImageFromCamera(){
  delete lastframe_;
  
  lastframe_ = GetImage(camera_);  
  
  unsigned char* data = &(lastframe_->m_image[0]);
  unsigned w = lastframe_->m_width;
  unsigned h = lastframe_->m_height;
  unsigned rowstep = lastframe_->m_image.length() / lastframe_->m_height;

  IplImage* iplImageResized = 0;
  IplImage* iplImage = 0;
  IplImage* iplImageFinal =0;
  iplImage = cvCreateImageHeader(cvSize(w,h), IPL_DEPTH_8U, 3);
  cvSetData(iplImage, data, rowstep);
  
  if(fixwidth_ >1){
    float downfactor_ = 1.0 * w / fixwidth_;
    iplImageResized = cvCreateImage(cvSize(static_cast<int>(w/downfactor_), static_cast<int>(h/downfactor_)), IPL_DEPTH_8U, 3);
    cvResize(iplImage, iplImageResized, CV_INTER_LINEAR );
    data=(unsigned char*)iplImageResized->imageData;
    w=iplImageResized->width;
    h=iplImageResized->height;
    rowstep=iplImageResized->widthStep;
    iplImageFinal = iplImageResized;
  } else {
    iplImageFinal = iplImage;
  }
  std::ostringstream oss;
  if(saveidl_ || saveone_) {
    
    oss << "/tmp/"<<outprefix<<"-frame-"<< frameno_<<".png";
    cvSaveImage(oss.str().c_str(),iplImageFinal);
  }

  if(swMULTHOG_.m_vHOGSVM.size() > 0) {
  Annotation anno;
  
  swMULTHOG_.testImageMultiScale(iplImageFinal,anno , 1, 1.0, 100.0, 0.1, 1);
  anno.sortByScore();
  for(int numc=0;numc< swMULTHOG_.m_vHOGSVM.size(); numc++){
    AnnotationList finalList; 
    std::vector<float> bandwidth(3);
    bandwidth[0] = swMULTHOG_.m_vHOGSVM.at(numc).ppParams.smoothingBandWidthX;
    bandwidth[1] = swMULTHOG_.m_vHOGSVM.at(numc).ppParams.smoothingBandWidthY;
    bandwidth[2] = swMULTHOG_.m_vHOGSVM.at(numc).ppParams.smoothingBandWidthScale;
    NonMaxSuppresion nms(bandwidth);
    
    Annotation classAnno;
    for(unsigned int j = 0; j < anno.size(); j++) {
      if(anno[j].score() >= swMULTHOG_.m_vHOGSVM.at(numc).ppParams.minNMSScore && anno[j].silhouetteID() == numc) {
        anno[j].setSilhouetteID(-1);
        classAnno.addAnnoRect(anno[j]);     
        if(visualize_ && nonmax_) {
          int pi=(numc % 16)*3;
          cvRectangle(iplImageFinal,cvPoint(anno[j].x1(),anno[j].y1()),cvPoint(anno[j].x2(),anno[j].y2()),
                      cvScalar(pal16[pi],pal16[pi+1],pal16[pi+2]),1);
        }
        
      }
    }
    if(saveidl_){
      classAnno.setImageName(oss.str().c_str());
      classannolists_.at(numc).addAnnotation(classAnno);
    }
    if(!nonmax_) {  //non max suppression by default 
      nms.addDataPoints(classAnno);
      Annotation finalAnno;
      nms.getModes(finalAnno, swMULTHOG_.m_vHOGSVM.at(numc).ppParams.minDetectionsPerMode, 0.01, 100, swMULTHOG_.m_vHOGSVM.at(numc).ppParams.scoreMode);
      if(visualize_){
        cout <<swMULTHOG_.m_vHOGSVM.at(numc).modelname<<" scores: ";
        for(unsigned int j = 0; j < finalAnno.size(); j++) {
          int pi=(numc % 16)*3;
	  cout <<finalAnno[j].score()<<" ";
	  //if(finalAnno[j].score() > 1.0)
          cvRectangle(iplImageFinal,cvPoint(finalAnno[j].x1(),finalAnno[j].y1()),cvPoint(finalAnno[j].x2(),finalAnno[j].y2()),
                      cvScalar(pal16[pi],pal16[pi+1],pal16[pi+2]),2); 
        }
	cout <<endl;
      }
      if(saveidl_) { //add name and pushback to annotation list
        finalAnno.setImageName(oss.str().c_str());
        classannolistsnms_.at(numc).addAnnotation(finalAnno);
      }
   //finalAnno.setImageName("/local/mmarinov/test8-0000000002.png");
   
   /*finalAnno.setImageName(tstimg);//"/local/mmarinov/test8-0000000002.png");
   finalList.addAnnotation(classAnno);
   std::ostringstream oss;
   oss <<"test8-class-"<<numc<<".idl";
   
   finalList.save(oss.str().c_str());
   finalList.clear();
   
   finalList.addAnnotation(finalAnno);
   std::ostringstream os;
   os <<"test8-class-"<<numc<<"-nmm.idl"; 
   finalList.save(os.str().c_str());*/
    }
  }
  } else {
    cout << "ObjectMClassDetector::[processImageFromCamera]: NO MODELS LOADED ! CHECK PATHS !"<<endl;
  }
    if(visualize_) {
      cvShowImage( "cam", iplImageFinal); 
      char t='s';
      int k=cvWaitKey(10); //allow opencv to breathe
      if(k > 0) {
	      cout <<"KEY:" <<k << " s: "<<((int)t) <<" k=='s': "<< (k==(int)t);
	      if(k == 1048691)
	        saveone_=-1;
      }
    }
    if((saveone_==1) || (saveone_ == -1)) {
      std::ostringstream ossd;
      ossd << "/tmp/"<<outprefix<<"-frame-"<< frameno_<<"-detections.png";
      //cout << "::::::::::::::::FRAME: "<<oss.str();
      cvSaveImage(ossd.str().c_str(),iplImageFinal);
     if(saveone_)
      saveone_=0;
     if(saveone_== -1)
      saveone_=1;
    } 
    frameno_++;
    cout << "SAVEIDL: "<< saveidl_<<" FRAMENO: "<< frameno_<<" FRAMESTOSAVE: "<<framestosave_<<" F%F: "<< (frameno_ % framestosave_)<<endl;
    if(saveidl_ && (frameno_ % framestosave_)==0 ){ //writeout time
      for(int numc=0;numc< swMULTHOG_.m_vHOGSVM.size(); numc++){
         std::ostringstream idloutname;
         idloutname <<"/tmp/class-"<<(numc+1)<<"-"<<swMULTHOG_.m_vHOGSVM.at(numc).modelname<<"-frames-"<<(frameno_- framestosave_)<<"-"<<frameno_<<".idl";
         classannolists_.at(numc).save(idloutname.str().c_str());
         classannolists_.at(numc).clear();
         if(!nonmax_){
          std::ostringstream nmsidloutname;
          idloutname <<"/tmp/class-"<<(numc+1)<<"-"<<swMULTHOG_.m_vHOGSVM.at(numc).modelname<<"-nms-frames-"<<(frameno_- framestosave_)<<"-"<<frameno_<<".idl";
          classannolistsnms_.at(numc).save(nmsidloutname.str().c_str());
          classannolistsnms_.at(numc).clear();
         }
      }
    }
    cout <<"FRAMENO: "<<frameno_<<endl;;
    //frameno_++;

  if(iplImage) cvReleaseImageHeader(&iplImage);
  if(iplImageResized) cvReleaseImage(&iplImageResized);
}
void ObjectMClassDetector::runComponent(){
  cout << "ObjectMClassDetector::[runComponent]"<<endl;
  if(visualize_)
    cvNamedWindow( "cam",1 );
  
  if(alwayson_)
    while(true) {
      processImageFromCamera();
      sleepProcess(10);  //do we need this?
    }
}

void ObjectMClassDetector::start() {
  cout << "ObjectMClassDetector::[start]"<<endl; 
  ManagedProcess::start();
  //listen to all or type filtered wm events ...
  //

  //addChangeFilter(createLocalTypeFilter<SceneChanged>(cdl::OVERWRITE),  //set appropriate type filter ...
  //                new MemberFunctionChangeReceiver<ObjectMClassDetector>(this,
  //                &ObjectMClassDetector::HandleWMChange));
}

void ObjectMClassDetector::taskAdopted(const string &_taskID) {}

void ObjectMClassDetector::taskRejected(const string &_taskID) {}

void ObjectMClassDetector::HandleWMChange(const cdl::WorkingMemoryChange &change) {
  //add some WM change event listener 
  //shared_ptr<const CASTData<Vision::SceneChanged> > pSceneChangedData
  //  = getWorkingMemoryEntry<Vision::SceneChanged>(change.m_address);
  
  //trigger processImageFromCamera()
  //
  //writeout new object detection hypotheses to wm (delete old hypos first?)
  //

}



