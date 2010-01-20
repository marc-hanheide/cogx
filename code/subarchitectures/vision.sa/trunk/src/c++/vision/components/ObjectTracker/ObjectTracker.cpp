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
using namespace Tracking;


ObjectTracker::ObjectTracker(){
	m_tracker = 0;
  m_camId = 0;
  m_track = false;
  m_running = true;
  m_testmode = false;
  m_bfc = true;
}

ObjectTracker::~ObjectTracker(){
  for(int i=0; i<m_trackinglist.size(); i++){
		if(m_trackinglist[i].tracker) delete(m_trackinglist[i].tracker);
	}
	delete(g_Resources);
	log("[ObjectTracker::~ObjectTracker()]");
}

// *** Working Memory Listeners ***
void ObjectTracker::receiveTrackingCommand(const cdl::WorkingMemoryChange & _wmc){
	TrackingCommandPtr track_cmd = getMemoryEntry<TrackingCommand>(_wmc.address);
	
	log("received tracking command ...");
	switch(track_cmd->cmd){
		case VisionData::START:
			if(m_track){
				log("start tracking: I'm allready tracking");
			}else{
				log("start tracking: ok");
				m_track = true;
			}
			break;
		case VisionData::STOP:
			if(m_track){
				log("stop tracking: ok");
				m_track = false;
			}else{
				log("stop tracking: I'm not tracking");
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

void ObjectTracker::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc){
	log("receiving VisualObject");
	
	if((int)m_trackinglist.size()>=m_maxModels){
		log("warning: maximum trackable models set to %d", m_maxModels);
		return;
	}

	VisualObjectPtr obj = getMemoryEntry<VisualObject>(_wmc.address);
	TrackingEntry newTrackingEntry;
	
	newTrackingEntry.valid = false;
	newTrackingEntry.obj = obj;
	newTrackingEntry.castWMA = _wmc.address;

	m_trackinglist.push_back(newTrackingEntry);

	// Standard outputs
	log("VisualObject received: '%s'", obj->label.c_str());
}

void ObjectTracker::changeVisualObject(const cdl::WorkingMemoryChange & _wmc){
	log("[ObjectTracker::changeofVisualObject] WARNING: function not implemented");
}

void ObjectTracker::removeVisualObject(const cdl::WorkingMemoryChange & _wmc){
	log("[ObjectTracker::removeVisualObject] WARNING: function not implemented");
}

// *** base functions *** (configure, start, runcomponent)
void ObjectTracker::configure(const map<string,string> & _config){
  map<string,string>::const_iterator it;
  
  if((it = _config.find("--videoname")) != _config.end())
  {
    m_videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> m_camId;
  }
  
	if((it = _config.find("--testmode")) != _config.end()){
		m_testmode = true;
		log("test mode enabled (only for ObjectTrackerTest component)");
	}
	
	if((it = _config.find("--BFC_disabled")) != _config.end()){
		m_bfc = false;
	}
	if(m_bfc)
		log("Backface culling enabled: suitable for objects with closed surface (i.e. a cube)");
	else
		log("Backface culling disabled: suitable for non closed surfaces (i.e. a polyflap)");

	if((it = _config.find("--maxModels")) != _config.end())
	{
    istringstream istr(it->second);
    istr >> m_maxModels;
  }else{
		m_maxModels = 3;
	}
	
	log("Maximal number of trackable models: %d", m_maxModels);
	
		
 // if((it = _config.find("--log")) != _config.end())
 // 	g_Resources->ShowLog(true);
 // else
  	g_Resources->ShowLog(true);
  	
}

void ObjectTracker::start(){
  // get connection to the video server
  m_videoServer = getIceServer<Video::VideoInterface>(m_videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

  addChangeFilter(createLocalTypeFilter<VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectTracker>(this,
        &ObjectTracker::receiveVisualObject));
  
  addChangeFilter(createLocalTypeFilter<TrackingCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectTracker>(this,
        &ObjectTracker::receiveTrackingCommand));
}

void ObjectTracker::destroy(){
	for(int i=0; i<m_trackinglist.size(); i++){
		if(m_trackinglist[i].tracker) delete(m_trackinglist[i].tracker);
	}
  delete(g_Resources);
}

void ObjectTracker::receiveImages(const std::vector<Video::Image>& images){
/*
  assert(images.size() > 0);
  m_image = images[0];
  runTracker(m_image);
  */
}

void ObjectTracker::runComponent(){
  
  // Initialize Tracker
  // Grab one image from VideoServer for initialisation
  m_videoServer->getImage(m_camId, m_image);
  initTracker(m_image);
  
  while(m_running)
  {
    if(m_track){
      m_videoServer->getImage(m_camId, m_image);
      runTracker(m_image);
    }
    else if(!m_track){
			// * Idle *
			sleepComponent(1000);
		}
		// Query keyboard input
		m_running = inputsControl(&m_trackinglist, fTimeTracker);

  }
  
  destroy();
  log("stop");
}

// Tracking
void ObjectTracker::initTracker(const Video::Image &image){
  
  log("Initialising Tracker");
  // *** Initialisation of Tracker ***
  int id = 0;
  
  m_ImageWidth = image.width;
  m_ImageHeight = image.height;
  
  m_params = LoadParametersFromINI("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/ObjectTracker.ini");
  
  // Set pathes for resource manager
	g_Resources->SetModelPath(m_params.modelPath.c_str());
	g_Resources->SetTexturePath(m_params.texturePath.c_str());
	g_Resources->SetShaderPath(m_params.shaderPath.c_str());

  // initialize SDL screen
  g_Resources->InitScreen(m_ImageWidth, m_ImageHeight, "ObjectTracker");

  // load camera parameters from Video::Image.camPars to OpenGL camera 'm_camera'
  loadCameraParameters(&m_camera, image.camPars, 0.1, 10.0);
  
	if(m_params.mode==1)
		m_tracker = new TextureTracker();
	else
		m_tracker = new EdgeTracker();
	
	if(!m_tracker->init(m_ImageWidth, m_ImageHeight,			// image size in pixels
											m_params.edgeMatchingTol,						// edge matching tolerance in degree
											m_params.minTexGrabAngle,						// goal tracking time in seconds (not implemented yet)
											Particle(0.0)))										// initial pose (where to reset when pressing 'z')
	{														
		log("  error: initialisation of tracker failed!");
		m_running = false;
	}
	m_tracker->setCamPerspective(&m_camera);
	 
  log("... initialisation successfull!");
}

void ObjectTracker::initTrackingEntry(int i){
	log("initialising new tracking entry '%s'", m_trackinglist[i].obj->label.c_str());
	
	int id;
	m_trackinglist[i].bfc = m_bfc;
	m_trackinglist[i].textured = m_params.mode;
	m_trackinglist[i].recursions = m_params.recursions;
	m_trackinglist[i].particles = m_params.particles;
	// --------------------------------
	// init tracker
	if(m_params.mode==1)
		m_trackinglist[i].tracker = new TextureTracker();
	else
		m_trackinglist[i].tracker = new EdgeTracker();
	
	if(!m_trackinglist[i].tracker->init(m_ImageWidth, m_ImageHeight,			// image size in pixels
																			m_params.edgeMatchingTol,					// edge matching tolerance in degree
																			m_params.minTexGrabAngle,					// goal tracking time in seconds (not implemented yet)
																			Particle(0.0),										// initial pose (where to reset when pressing 'z')
																			m_params.constraints));
	{														
		log("  error: initialisation of tracker failed!");
		m_running = false;
	}

	// --------------------------------
	// init model
	// Test if model is valid
	if(!m_trackinglist[i].obj->model || m_trackinglist[i].obj->model->vertices.size()<=0){
		log("  error: no valid model received, adding nothing");
		return;
	}
	m_trackinglist[i].model = new TrackerModel();
	if((id = g_Resources->AddModel(m_trackinglist[i].model, m_trackinglist[i].obj->label.c_str())) == -1)
		return;
	m_trackinglist[i].model = g_Resources->GetModel(id);
	if(!convertGeometryModel(m_trackinglist[i].obj->model, m_trackinglist[i].model)){
		delete(m_trackinglist[i].model);
		log("  error can not convert VisualObject to tracker model");
		return;
	}
	m_trackinglist[i].model->computeFaceNormals();
	m_trackinglist[i].model->computeEdges();
	m_trackinglist[i].model->Update();
	//m_trackinglist[i].model->print();

	m_trackinglist[i].camera = &m_camera;
	
	m_trackinglist[i].constraints = m_params.constraints;
	convertPose2Particle(m_trackinglist[i].obj->pose, m_trackinglist[i].detectpose);
	m_trackinglist[i].trackpose = m_trackinglist[i].detectpose;
	m_trackinglist[i].tracker->setInitialPose(m_trackinglist[i].detectpose);
	m_trackinglist[i].tracker->reset();

	m_trackinglist[i].valid = true;
	
	log("  initialisation of new tracking entry successfull");
}

void ObjectTracker::runTracker(const Video::Image &image){

	// *** Tracking Loop ***
	double dTimeStamp;
	fTimeTracker=0.0;
	int i;
	
	
	
	// * Tracking *
	m_timer.Update();
	dTimeStamp = m_timer.GetApplicationTime();

// 	if(m_testmode){
// 		m_camera->Set(	0.2, 0.2, 0.2,											// Position of camera relative to Object
// 										0.0, 0.0, 0.0,											// Point where camera looks at (world origin)
// 										0.0, 1.0, 0.0,											// Up vector (y-axis)
// 										45, image.width, image.height,		  // field of view angle, image width and height
// 										0.1, 10.0,													// camera z-clipping planes (far, near)
// 										GL_PERSPECTIVE);										// Type of projection (GL_ORTHO, GL_PERSPECTIVE)
// 	}

	m_tracker->image_processing((unsigned char*)(&image.data[0]));
	m_tracker->drawImage(NULL);

	// Track all models
	for(i=0; i<(int)m_trackinglist.size(); i++){
		if(!m_trackinglist[i].valid){
			initTrackingEntry(i);
			return;
		}else{			
			// Track model
// 			m_trackinglist[i].track();
			
			// conversion from ObjectTracker coordinates to ObjectTracker CogX.vision coordinates
			convertParticle2Pose(m_trackinglist[i].trackpose, m_trackinglist[i].obj->pose);
		
			// Send new data to working memory
			m_trackinglist[i].obj->detectionConfidence = m_trackinglist[i].trackpose.c;
			m_trackinglist[i].obj->time = convertTime(dTimeStamp);
			overwriteWorkingMemory(m_trackinglist[i].castWMA.id, m_trackinglist[i].obj);
		}
	}
	
	m_tracker->drawCoordinates();
	for(int id=0; id<i; id++){
			m_trackinglist[id].tracker->drawResult(&m_trackinglist[id].trackpose, m_trackinglist[id].model);
	}
	
	m_tracker->swap();

	fTimeTracker = m_timer.Update();
}



