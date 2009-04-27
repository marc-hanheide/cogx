/**
 * @author Thomas Mörwald
 * @date April 2009
 */

#include "ObjectTracker.h"
#include <VideoUtils.h>
#include <ChangeFilterFactory.hpp>

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ObjectTracker();
  }
}



using namespace cast;
using namespace std;
using namespace VisionData;


ObjectTracker::ObjectTracker(){
  camId = 0;
  track = false;
}

ObjectTracker::~ObjectTracker(){
  delete(g_Resources);
}


// *** Working Memory Listeners ***

void ObjectTracker::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc){
	VisualObjectPtr obj = getMemoryEntry<VisualObject>(_wmc.address);
	log("adding VisualObject '%s'", obj->label.c_str());
	
	// Convert GeometryModel to Model for tracker
	Model* model = new Model();
	if(!convertGeometryModel(obj->model, model)){
		delete(model);
		return;
	}
	
	// Get IDs of working memory object and resources object
	IDList ids;
	ids.resources_ID = g_Resources->AddModel(model, obj->label.c_str());
	ids.cast_AD = _wmc.address;
	
	// add IDs and visual object to lists
	m_modelID_list.push_back(ids);
}

void ObjectTracker::receiveTrackingCommand(const cdl::WorkingMemoryChange & _wmc){
	TrackingCommandPtr track_cmd = getMemoryEntry<TrackingCommand>(_wmc.address);
	
	log("received tracking command ...");
	switch(track_cmd->cmd){
		case VisionData::START:
			if(track){
				log("allready started tracking");
			}else{
				log("starting tracking");
				track = true;
			}
			break;
		case VisionData::STOP:
			if(track){
				log("stopping tracking");
				track = false;
			}else{
				log("allready stopped tracking");
			}
			break;
		case VisionData::RELEASEMODELS:
			log("releasing all models");
			g_Resources->ReleaseModel();
			break;
		default:
			log("unknown tracking command received, doing nothing");
			break;
	}	
}


// *** base functions *** (configure, start, runcomponent)

void ObjectTracker::configure(const map<string,string> & _config){
  map<string,string>::const_iterator it;
 
  // first let the base classes configure themselves
  configureVideoCommunication(_config);

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> camId;
  }
}

void ObjectTracker::start(){
  startVideoCommunication(*this);
  
  addChangeFilter(createLocalTypeFilter<VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectTracker>(this,
        &ObjectTracker::receiveVisualObject));
  
  addChangeFilter(createLocalTypeFilter<TrackingCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectTracker>(this,
        &ObjectTracker::receiveTrackingCommand));
}

void ObjectTracker::runComponent(){
  
  // *** Initialisation of Tracker ***
  Tracker tracker;
  Video::Image image;
  IplImage* cvImage;
  m_trackpose = Particle(0.0);
  
  // Set pathes of resource manager
  g_Resources->SetModelPath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/model/");
  g_Resources->SetTexturePath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/texture/");
  g_Resources->SetShaderPath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/shader/");

  // Grab one image from VideoServer for initialisation
  getImage(camId, image);
  cvImage = convertImageToIpl(image);
    
  // Initialize SDL screen
  g_Resources->InitScreen(cvImage->width, cvImage->height);
 
  // Initialize tracking (parameters for edge-based tracking)
  if(!tracker.init(	cvImage->width, cvImage->height,
					1000,
					49.0,
					0.25, 0.165, 0.25,
					0.0, 40.0,
					0.0, 0.1,
					17.0))
	log("Initialisation failed!");

  cvReleaseImage(&cvImage);
  
  // *** Tracking Loop ***
  Model* model;
  VisualObjectPtr obj;
  float dTimeImage;
  float dTimeTracker;
  double dTimeStamp;
  int i;
  
  while(isRunning())
  {
  	if(track){
  	// * Tracking *
  	  m_timer.Update();
  	  dTimeStamp = m_timer.GetApplicationTime();
  	  
  	  // Grab image from VideoServer
  	  getImage(camId, image);
  	  cvImage = convertImageToIpl(image);
  	  cvConvertImage(cvImage, cvImage, CV_CVTIMG_FLIP);
  	  
	  // Track all models
	  for(i=0; i<m_modelID_list.size(); i++){
	    model = g_Resources->GetModel(m_modelID_list[i].resources_ID);
	    obj = getMemoryEntry<VisualObject>(m_modelID_list[i].cast_AD);
	    obj->time = convertTime(dTimeStamp);
	    
	    // conversion from CogX.vision coordinates to ObjectTracker coordinates
	    convertPose2Particle(obj->pose, m_trackpose);
	    
		// Track model
		tracker.trackEdge((unsigned char*)cvImage->imageData, model, &m_trackpose, &m_trackpose);
		
		// conversion from ObjectTracker coordinates to ObjectTracker CogX.vision coordinates
		convertParticle2Pose(m_trackpose, obj->pose);
		
		obj->detectionConfidence = m_trackpose.w;
		obj->time = convertTime(dTimeStamp);
		
		overwriteWorkingMemory(m_modelID_list[i].cast_AD.id, obj);
	  }
	  //printf("TimeImage:   %.0f ms\n", fTimeImage*1000.0);
	  //printf("TimeTracker: %.0f ms\n\n", fTimeTracker*1000.0);
	  
	  cvReleaseImage(&cvImage);
	  sleepComponent(10);
	}else{
	  // * Idle *
      sleepComponent(1000);
	}
  }
 
  // Releasing Resources here cause ~ObjectTracker() not called when [STRG+C]
  tracker.release();
  delete(g_Resources);
}


