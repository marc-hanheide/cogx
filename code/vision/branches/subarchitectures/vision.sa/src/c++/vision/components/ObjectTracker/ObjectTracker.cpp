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

namespace cast
{

using namespace std;
using namespace VisionData;


ObjectTracker::ObjectTracker(){
  camId = 0;
  m_model = 0;
  track = false;
}

ObjectTracker::~ObjectTracker(){
  delete(g_Resources);
}

// *** Working Memory Listeners ***

void ObjectTracker::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc){
	VisualObjectPtr obj = getMemoryEntry<VisualObject>(_wmc.address);
	log("adding VisualObject '%s'", obj->label.c_str());
	
	ModelData* md = new ModelData();
	if(!convert_GeometryModel_to_ModelData(obj->model, md))
		return;
	
	// Generate new Model using ModelData
	Model* model = new Model();
	model->load(*md);
	delete(md);
	model->computeEdges();
	model->computeNormals();
	
	// Get IDs of working memory object and resources object
	IDList ids;
	ids.resources_ID = g_Resources->AddModel(model, obj->label.c_str());
	istringstream istr(_wmc.address.id);
	istr >> ids.cast_ID;
	
	// add IDs and visual object to lists
	m_model_list.push_back(ids);
	m_visobj_list.push_back(obj);
	
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
  Tracker tracker;
  Video::Image image;
  IplImage* cvImage;
  Model* model;
  Particle m_result = Particle(0.0);
  
  // Set pathes of resource manager
  g_Resources->SetModelPath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/model/");
  g_Resources->SetTexturePath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/texture/");
  g_Resources->SetShaderPath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/shader/");

  // Grab one image from video server for initialisation
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
	
  // Load model with resource manager
  int id;
  if((id = g_Resources->AddModel("box_blender.ply")) == -1)
	log("failed loading model!");
  m_model = g_Resources->GetModel(id);
  
  cvReleaseImage(&cvImage);
  
  float fTimeImage;
  float fTimeTracker;
  
  while(isRunning())
  {
  	if(track){
  	  m_timer.Update();
  	  
  	  getImage(camId, image);
  	  cvImage = convertImageToIpl(image);
  	  cvConvertImage(cvImage, cvImage, CV_CVTIMG_FLIP);
  	  fTimeImage = m_timer.Update();
	  
	  tracker.trackEdge((unsigned char*)cvImage->imageData, m_model, &m_result, &m_result);
	  glFlush();
	  fTimeTracker = m_timer.Update();
	  
	  //printf("TimeImage:   %.0f ms\n", fTimeImage*1000.0);
	  //printf("TimeTracker: %.0f ms\n\n", fTimeTracker*1000.0);
	  
	  cvReleaseImage(&cvImage);
	  sleepComponent(10);
	}else{
      sleepComponent(1000);
	}
  }
 
  // Releasing Resources here cause ~ObjectTracker() not called when [STRG+C]
  tracker.release();
  delete(g_Resources);
}

}

