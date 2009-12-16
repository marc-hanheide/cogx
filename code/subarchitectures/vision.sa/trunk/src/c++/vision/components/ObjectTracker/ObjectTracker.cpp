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
  m_camId = 0;
  m_track = false;
  m_running = true;
  m_testmode = false;
  m_bfc = true;
  m_trackTexture = false;
}

ObjectTracker::~ObjectTracker(){
  for(int i=0; i<m_trackinglist.size(); i++){
		if(m_trackinglist[i].tracker) delete(m_trackinglist[i].tracker);
	}
	delete(g_Resources);
	log("Resources released");
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
  
/*
  log("**    ObjectTracker by Thomas Moerwald    **");
  log("**       moerwald@acin.tuwien.ac.at       **");
  log("**                                        **");
  log("** please lookup the README file in the   **");
  log("** component directory for usage          **");
 */
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

	if((it = _config.find("--textured")) != _config.end()){
		m_trackTexture = true;
		log("Texture tracking enabled!");
	}

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
  log("Resources released");
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
  
  log("stop");
}

// Tracking
void ObjectTracker::initTracker(const Video::Image &image){
  
  log("Initialising Tracker");
  // *** Initialisation of Tracker ***
  int id = 0;
  
  m_ImageWidth = image.width;
  m_ImageHeight = image.height;
  
  // Set pathes for resource manager
  g_Resources->SetModelPath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/model/");
  g_Resources->SetTexturePath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/texture/");
  g_Resources->SetShaderPath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/shader/");

  // initialize SDL screen
  g_Resources->InitScreen(m_ImageWidth, m_ImageHeight);

  // initialize camera
  if((id = g_Resources->AddCamera("cam_extrinsic")) == -1)
  	m_running = false;
  m_camera = g_Resources->GetCamera(id);

  // load camera parameters from Video::Image.camPars to OpenGL camera 'm_camera'
  loadCameraParameters(m_camera, image.camPars, 0.1, 10.0);

  // setting spatial constraints
	float rot = 15.0 * PIOVER180;
	float rotp = 2.0;
	float trans = 0.01;
	float transp = 0.2;
	float zoom = 0.01;
	float zoomp = 0.5;
	
	m_constraints.r.x = rot;
	m_constraints.r.y = rot;
	m_constraints.r.z = rot;
	m_constraints.rp.x = rotp;
	m_constraints.rp.y = rotp;
	m_constraints.rp.z = rotp;
	m_constraints.s.x = trans;
	m_constraints.s.y = trans;
	m_constraints.s.z = trans;
	m_constraints.sp.x = transp;
	m_constraints.sp.y = transp;
	m_constraints.sp.z = transp;
	m_constraints.z = zoom;	
	m_constraints.zp = zoomp;
  
  log("... initialisation successfull!");
}

void ObjectTracker::initTrackingEntry(int i){
	log("initialising new tracking entry '%s'", m_trackinglist[i].obj->label.c_str());
	
	int id;
	m_trackinglist[i].bfc = m_bfc;
	m_trackinglist[i].textured = m_trackTexture;
	// --------------------------------
	// init tracker
	if(m_trackTexture)
		m_trackinglist[i].tracker = new TextureTracker();
	else
		m_trackinglist[i].tracker = new EdgeTracker();
	
	if(!m_trackinglist[i].tracker->init(m_ImageWidth, m_ImageHeight,	// image size in pixels
																			200,															// number of particles for each recursion (lower if tracker consumes too much power)
																			4,																// recursions of particle filter (lower if tracker consumes too much power)
																			float(45.0*PIOVER180),						// edge matching tolerance in degree
																			0.05f,														// goal tracking time in seconds (not implemented yet)
																			Particle(0.0)))										// initial pose (where to reset when pressing 'z')
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
	m_trackinglist[i].model = new Model();
	if((id = g_Resources->AddModel(m_trackinglist[i].model, m_trackinglist[i].obj->label.c_str())) == -1)
		return;
	m_trackinglist[i].model = g_Resources->GetModel(id);
	if(!convertGeometryModel(m_trackinglist[i].obj->model, m_trackinglist[i].model)){
		delete(m_trackinglist[i].model);
		log("  error can not convert VisualObject to tracker model");
		return;
	}
	m_trackinglist[i].model->computeEdges();
	m_trackinglist[i].model->computeNormals();
	m_trackinglist[i].model->UpdateDisplayLists();

	m_trackinglist[i].camera = m_camera;
	m_trackinglist[i].constraints = m_constraints;
	convertPose2Particle(m_trackinglist[i].obj->pose, m_trackinglist[i].detectpose);
	m_trackinglist[i].trackpose = m_trackinglist[i].detectpose;
	m_trackinglist[i].tracker->setZeroPose(m_trackinglist[i].detectpose);
	m_trackinglist[i].tracker->zeroParticles();

	m_trackinglist[i].valid = true;
	
	log("  initialisation successfull");
}

void ObjectTracker::runTracker(const Video::Image &image){

	if(m_trackinglist.size()<=0)
		return;

	if(!m_trackinglist[0].valid)
		initTrackingEntry(0);

	// *** Tracking Loop ***
	double dTimeStamp;
	fTimeTracker=0.0;
	int i;
	
	// * Tracking *
	m_timer.Update();
	dTimeStamp = m_timer.GetApplicationTime();

	if(m_testmode){
		m_camera->Set(	0.2, 0.2, 0.2,											// Position of camera relative to Object
										0.0, 0.0, 0.0,											// Point where camera looks at (world origin)
										0.0, 1.0, 0.0,											// Up vector (y-axis)
										45, image.width, image.height,		  // field of view angle, image width and height
										0.1, 10.0,													// camera z-clipping planes (far, near)
										GL_PERSPECTIVE);										// Type of projection (GL_ORTHO, GL_PERSPECTIVE)
	}

	m_trackinglist[0].tracker->image_processing((unsigned char*)(&image.data[0]));
	m_trackinglist[0].tracker->drawImage(NULL);

	// Track all models
	for(i=0; i<(int)m_trackinglist.size(); i++){
		if(!m_trackinglist[i].valid){
			initTrackingEntry(i);
			return;
		}else{			
			// Track model
			m_trackinglist[i].track();
			
			
			// conversion from ObjectTracker coordinates to ObjectTracker CogX.vision coordinates
			convertParticle2Pose(m_trackinglist[i].trackpose, m_trackinglist[i].obj->pose);
		
			// Send new data to working memory
			m_trackinglist[i].obj->detectionConfidence = m_trackinglist[i].trackpose.c;
			m_trackinglist[i].obj->time = convertTime(dTimeStamp);
			overwriteWorkingMemory(m_trackinglist[i].castWMA.id, m_trackinglist[i].obj);
		}
	}

	m_trackinglist[0].tracker->drawCoordinates();
	for(int id=0; id<i; id++){
			m_trackinglist[id].tracker->drawResult(&m_trackinglist[id].trackpose, m_trackinglist[id].model);
	}

	m_trackinglist[0].tracker->swap();

	fTimeTracker = m_timer.Update();
}



