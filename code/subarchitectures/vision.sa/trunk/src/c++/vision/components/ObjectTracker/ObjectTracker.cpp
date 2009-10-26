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
  bfc = true;
  trackTexture = false;
}

ObjectTracker::~ObjectTracker(){
  delete(g_Resources);
}

void ObjectTracker::initTracker(const Video::Image &image){
  
  log("Initialising ...");
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
  
  // add Tracker
  for(int i=0; i<m_maxModels; i++){
		Tracker* tracker = new EdgeTracker();
		if(!tracker->init(m_ImageWidth, m_ImageHeight,	// image size in pixels
									200,															// number of particles for each recursion (lower if tracker consumes too much power)
									4,																// recursions of particle filter (lower if tracker consumes too much power)
									float(30.0 * PIOVER180),					// standard deviation of rotational noise in degree
									0.05f,														// standard deviation of translational noise in meter
									30.0f,														// edge matching tolerance in degree
									0.05f,														// goal tracking time in seconds (not implemented yet)
									Particle(0.0)))									// initial pose (where to reset when pressing 'z')
		{														
			log("initialisation of tracker failed!");
			running = false;
		}
		m_tracker_list.push_back(tracker);
	}
 
  // initialize camera
  if((id = g_Resources->AddCamera("cam_extrinsic")) == -1)
  	running = false;
  m_camera = g_Resources->GetCamera(id);
   
  // load camera parameters from Video::Image.camPars to OpenGL camera 'm_camera'
  loadCameraParameters(m_camera, image.camPars, 0.1, 10.0);
  m_tracker_list[0]->setCamPerspective(m_camera);
  
  log("... initialisation successfull!");
}


void ObjectTracker::runTracker(const Video::Image &image){
//println("A");
	
	// *** Tracking Loop ***
	Model* model;
	VisualObjectPtr obj;
	double dTimeStamp;
	fTimeTracker=0.0;
	int i;
	
	
	// * Tracking *
	m_timer.Update();
	dTimeStamp = m_timer.GetApplicationTime();

	if(testmode){
		m_camera->Set(	0.2, 0.2, 0.2,											// Position of camera relative to Object
										0.0, 0.0, 0.0,											// Point where camera looks at (world origin)
										0.0, 1.0, 0.0,											// Up vector (y-axis)
										45, image.width, image.height,		  // field of view angle, image width and height
										0.1, 10.0,													// camera z-clipping planes (far, near)
										GL_PERSPECTIVE);										// Type of projection (GL_ORTHO, GL_PERSPECTIVE)
	}
	
	m_tracker_list[0]->image_processing((unsigned char*)(&image.data[0]));
	m_tracker_list[0]->drawImage(NULL);
	
	// Track all models
	for(i=0; i<(int)m_modelID_list.size() && i<m_maxModels; i++){
			model = g_Resources->GetModel(m_modelID_list[i].resources_ID);
			obj = getMemoryEntry<VisualObject>(m_modelID_list[i].cast_AD);
			
			// conversion from CogX.vision coordinates to ObjectTracker coordinates
			// *** not needed any more -> to change tracking pose use:
			//		 tracker->setZeroPose() and tracker->zeroPose()
			//convertPose2Particle(obj->pose, m_modelID_list[i].trackpose);
			//m_modelID_list[i].trackpose.w = obj->detectionConfidence;
						
			// Track model
			m_tracker_list[i]->track(	model,
																m_camera,
																m_modelID_list[i].trackpose);
			
			
			// conversion from ObjectTracker coordinates to ObjectTracker CogX.vision coordinates
			convertParticle2Pose(m_modelID_list[i].trackpose, obj->pose);
		
			// Send new data to working memory
			obj->detectionConfidence = m_modelID_list[i].trackpose.w;
			obj->time = convertTime(dTimeStamp);
			overwriteWorkingMemory(m_modelID_list[i].cast_AD.id, obj);
	}
	
	//m_tracker_list[0]->drawTest();
	m_tracker_list[0]->drawCoordinates();
	
	
	for(int id=0; id<i; id++){
		if(id==0) printf("\n");
		printf("  Cube %d: %.3f %.3f %.3f", id, 
			m_modelID_list[id].trackpose.tX, m_modelID_list[id].trackpose.tY, m_modelID_list[id].trackpose.tZ);
		m_tracker_list[id]->drawResult(&m_modelID_list[id].trackpose, g_Resources->GetModel(m_modelID_list[id].resources_ID));
	}
	
	m_tracker_list[0]->swap();
	
	fTimeTracker = m_timer.Update();
}


// *** Working Memory Listeners ***

void ObjectTracker::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc){
	log("receiving VisualObject");
	
	VisualObjectPtr obj = getMemoryEntry<VisualObject>(_wmc.address);
	
	// Test if number of max. trackable models reached
	if((int)m_modelID_list.size()>=m_maxModels){
		log("  number of maximum trackable models reached (%d)", m_maxModels);
		return;
	}
	
	// Test if model is valid
	if(!obj->model || obj->model->vertices.size()<=0){
		log("  no valid model received, adding nothing");
		return;
	}
	
	// Convert GeometryModel to Model for tracker
	Model* model = new Model();
	if(!convertGeometryModel(obj->model, model)){
		delete(model);
		log("  can not convert VisualObject to tracker model");
		return;
	}
	
	// Get IDs of working memory object and resources object
	IDList ids;
	ids.resources_ID = g_Resources->AddModel(model, obj->label.c_str());
	ids.cast_AD = _wmc.address;
	
	// converte pose of object to tracking pose (=particle)
	convertPose2Particle(obj->pose, ids.trackpose);

	// set detection pose to this trackpose
	ids.detectpose = ids.trackpose;
	
	// set zero pose of tracker
	m_tracker_list[m_modelID_list.size()]->setZeroPose(ids.detectpose);
	m_tracker_list[m_modelID_list.size()]->zeroParticles();
	
	// add IDs and visual object to lists
	m_modelID_list.push_back(ids);
	
	// Standard outputs
	log("received VisualObject: '%s'", obj->label.c_str());
	
	if((int)m_modelID_list.size()>m_maxModels){
		log("warning: maximum trackable models set to %d", m_maxModels);
	}
}

void ObjectTracker::receiveTrackingCommand(const cdl::WorkingMemoryChange & _wmc){
	TrackingCommandPtr track_cmd = getMemoryEntry<TrackingCommand>(_wmc.address);
	
	log("received tracking command ...");
	switch(track_cmd->cmd){
		case VisionData::START:
			if(track){
				log("start tracking: I'm allready tracking");
			}else{
				log("start tracking: ok");
				track = true;
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
  
/*
  log("**    ObjectTracker by Thomas Moerwald    **");
  log("**       moerwald@acin.tuwien.ac.at       **");
  log("**                                        **");
  log("** please lookup the README file in the   **");
  log("** component directory for usage          **");
 */
  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> camId;
  }
  
	if((it = _config.find("--testmode")) != _config.end()){
		testmode = true;
		log("test mode enabled (only for ObjectTrackerTest component)");
	}
	
	if((it = _config.find("--BFC_disabled")) != _config.end()){
		bfc = false;
		if(bfc)
			log("Backface culling enabled: suitable for objects with closed surface (i.e. a cube)");
		else
			log("Backface culling disabled: suitable for non closed surfaces (i.e. a polyflap)");
	}

	if((it = _config.find("--trackTexture")) != _config.end()){
		trackTexture = true;
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
  	g_Resources->ShowLog(false);
  	
}

void ObjectTracker::start(){
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

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
  // Release Tracker
  for(int id=0; id<m_maxModels; id++){
  	delete(m_tracker_list[id]);
  }
  delete(g_Resources);
}

void ObjectTracker::receiveImages(const std::vector<Video::Image>& images)
{
/*
  assert(images.size() > 0);
  m_image = images[0];
  runTracker(m_image);
  */
}

void ObjectTracker::runComponent(){
  
  // Initialize Tracker
  // Grab one image from VideoServer for initialisation
  videoServer->getImage(camId, m_image);
  initTracker(m_image);
  
  while(running)
  {
    if(track){
      videoServer->getImage(camId, m_image);
      runTracker(m_image);
    }
    else if(!track){
			// * Idle *
			sleepComponent(1000);
		}
		// Query keyboard input		
		running = inputsControl(m_tracker_list, fTimeTracker);
  }
  
  log("stop");
}



