/**
 * @author Thomas Mörwald
 * @date April 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "ObjectTracker.h"
#include <VideoUtils.h>


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
  running = true;
  testmode = false;
}

ObjectTracker::~ObjectTracker(){
  delete(g_Resources);
}

void ObjectTracker::initTracker(){
  // *** Initialisation of Tracker ***
  m_trackpose = Particle(0.0);
  int id = 0;
  
  // Set pathes of resource manager
  g_Resources->SetModelPath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/model/");
  g_Resources->SetTexturePath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/texture/");
  g_Resources->SetShaderPath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/shader/");

  // Grab one image from VideoServer for initialisation
  getImage(camId, m_image);
    
  // Initialize SDL screen
  g_Resources->InitScreen(m_image.width, m_image.height);
 
  // Initialize tracking (parameters for edge-based tracking)
  if(!m_tracker.init(	m_image.width, m_image.height,		// image size in pixels
											3000,															// maximum number of particles (=storage size of particle list)
											20.0,															// standard deviation of rotational noise in degree
											0.05,															// standard deviation of translational noise in meter
											20.0,															// edge matching tolerance in degree
											0.05)){														// goal tracking time in seconds
		log("Initialisation failed!");
		running = false;
  }
  
  // Load extrensic Camera (has to be done AFTER tracker initialisation
  if((id = g_Resources->AddCamera("cam_extrinsic")) == -1)
  	running = false;
  m_camera = g_Resources->GetCamera(id);
  m_camera->Set(	0.2, 0.2, 0.2,											// position of camera	
									0.0, 0.0, 0.0,											// point camera looks at
									0.0, 1.0, 0.0,											// up-vector of camera
									49, m_image.width, m_image.height,	// vield of view in degree
									0.1, 10.0,
									GL_PERSPECTIVE);
				
  log("initialisation successfull!");		
}

void ObjectTracker::runTracker(){
	// *** Tracking Loop ***
	Model* model;
	VisualObjectPtr obj;
	float fTimeImage;
	float fTimeTracker;
	double dTimeStamp;
	int i;
  
	// * Tracking *
	m_timer.Update();
	//dTimeStamp = m_timer.GetApplicationTime();

	// Grab image from VideoServer
	getImage(camId, m_image);
	fTimeImage = m_timer.Update();
	if(!testmode){
		// TODO Camera parameters from file
		// TODO Camera parameters from image
		m_camera->Set(	0.2,
										0.2,
										0.2,
										0.0, 0.0, 0.0,
										0.0, 1.0, 0.0,
										45, m_image.width, m_image.height,
										0.1, 10.0,
										GL_PERSPECTIVE);
		//log("Cam_pos: %f %f %f", m_image.camPars.pose.pos.x, m_image.camPars.pose.pos.y, m_image.camPars.pose.pos.z);
	}
	
	

	// Track all models
	for(i=0; i<m_modelID_list.size(); i++){
		model = g_Resources->GetModel(m_modelID_list[i].resources_ID);
		obj = getMemoryEntry<VisualObject>(m_modelID_list[i].cast_AD);

		// conversion from CogX.vision coordinates to ObjectTracker coordinates
		convertPose2Particle(obj->pose, m_trackpose);
		m_trackpose.w = obj->detectionConfidence;

		// Track model
		running = m_tracker.trackEdge((unsigned char*)(&m_image.data[0]), model, m_camera, m_trackpose, m_trackpose);

		// conversion from ObjectTracker coordinates to ObjectTracker CogX.vision coordinates
		convertParticle2Pose(m_trackpose, obj->pose);

		// Send new data to working memory
		obj->detectionConfidence = m_trackpose.w;
		obj->time = convertTime(dTimeStamp);
		overwriteWorkingMemory(m_modelID_list[i].cast_AD.id, obj);
	}
	  
	fTimeTracker = m_timer.Update();
	//log("TimeImage:   %.0f ms", fTimeImage*1000.0);
	//log("TimeTracker: %.0f ms", fTimeTracker*1000.0);
}


// *** Working Memory Listeners ***

void ObjectTracker::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc){
	VisualObjectPtr obj = getMemoryEntry<VisualObject>(_wmc.address);
	
	// Test if model is valid
	if(!obj->model || obj->model->vertices.size()<=0){
		log("receive VisualObject: no valid model received, adding nothing");
		return;
	}
	
	
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
	
	log("receive VisualObject: model added");
}

void ObjectTracker::receiveTrackingCommand(const cdl::WorkingMemoryChange & _wmc){
	TrackingCommandPtr track_cmd = getMemoryEntry<TrackingCommand>(_wmc.address);
	
	log("received tracking command ...");
	switch(track_cmd->cmd){
		case VisionData::START:
			if(track){
				log("start tracking: I'm allready tracking");
			}else{
				if(g_Resources->GetNumModels()<=0)
					log("start tracking: no model to track in memory");
				else{
					log("start tracking: ok");
					track = true;
				}
			}
			break;
		case VisionData::STOP:
			if(track){
				log("stop tracking: ok");
				track = false;
			}else{
				log("stop tracking: I'm not tracking");
			}
			break;
		case VisionData::TESTMODE:
			if(testmode){
				log("switching from testmode to normal tracking");
				testmode = false;
			}else{
				log("switching to testmode");
				testmode = true;
			}
			break;
		case VisionData::RELEASEMODELS:
			log("release models: releasing all models");
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
  
  if((it = _config.find("--log")) != _config.end())
  	g_Resources->ShowLog(true);
  else
  	g_Resources->ShowLog(false);
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
  
  // Initialize Tracker
  initTracker();
  
  while(running)
  {
  	if(track){
  	  // Run Tracker
  	  runTracker();
  	  sleepComponent(10);
	}else{
	  // * Idle *
      sleepComponent(1000);
      //log("sleeping");
	}
  }
  
  // Release Tracker
  delete(g_Resources);
  m_tracker.release();
  log("stop");
  
}


