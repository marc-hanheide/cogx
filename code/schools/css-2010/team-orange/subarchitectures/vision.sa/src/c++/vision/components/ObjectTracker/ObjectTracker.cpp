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
  // *** Initialisation of Tracker ***
  int id = 0;
  
  // Set pathes for resource manager
  g_Resources->SetModelPath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/model/");
  g_Resources->SetTexturePath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/texture/");
  g_Resources->SetShaderPath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/shader/");

  // Initialize SDL screen
  g_Resources->InitScreen(image.width, image.height);
 
  // Initialize tracking (parameters for edge-based tracking)
  if(trackTexture)
  	m_tracker = new TextureTracker();
  else
		m_tracker = new EdgeTracker();
  if(!m_tracker->init(	image.width, image.height,				// image size in pixels
												3000,															// maximum number of particles (=storage size of particle list)
												20.0*PIOVER180,										// standard deviation of rotational noise in degree
												0.03,															// standard deviation of translational noise in meter
												40.0,															// edge matching tolerance in degree
												0.05))														// goal tracking time in seconds
	{														
		log("initialisation of tracker failed!");
		running = false;
  }
  
  // *** Setting up camera ***
  if((id = g_Resources->AddCamera("cam_extrinsic")) == -1)
  	running = false;
  m_camera = g_Resources->GetCamera(id);
  
  // load camera parameters from Video::Image.camPars to OpenGL camera 'm_camera'
  loadCameraParameters(m_camera, image.camPars, 0.1, 10.0);
  
  // link camera with tracker
	m_tracker->setCamPerspective(m_camera);
	m_tracker->lock(false);
	m_tracker->setBFC(bfc);
	
  log("initialisation successfull!");
}


void ObjectTracker::runTracker(const Video::Image &image){
println("A");
	
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

println("B");
	fTimeImage = m_timer.Update();
println("BB w h %d %d", image.width, image.height);
	if(testmode){
		m_camera->Set(	0.2, 0.2, 0.2,											// Position of camera relative to Object
										0.0, 0.0, 0.0,											// Point where camera looks at (world origin)
										0.0, 1.0, 0.0,											// Up vector (y-axis)
										45, image.width, image.height,		  // field of view angle, image width and height
										0.1, 10.0,													// camera z-clipping planes (far, near)
										GL_PERSPECTIVE);										// Type of projection (GL_ORTHO, GL_PERSPECTIVE)
	}
	
m_camera->Print();
println("C");
	m_tracker->drawImage(NULL);
println("D");
	
	// Track all models
	for(i=0; i<m_modelID_list.size() && i<m_maxModels; i++){
		IDList* ids = &m_modelID_list[i];
		model = g_Resources->GetModel(ids->resources_ID);
		obj = getMemoryEntry<VisualObject>(ids->cast_AD);
				
		// conversion from CogX.vision coordinates to ObjectTracker coordinates
		convertPose2Particle(obj->pose, m_modelID_list[i].trackpose);
		m_modelID_list[i].trackpose.w = obj->detectionConfidence;

		// Track model
		m_tracker->track((unsigned char*)(&image.data[0]), model, m_camera, ids->trackpose, ids->trackpose);
		m_tracker->drawResult(&ids->trackpose);
		m_tracker->drawTest();
	
		// Query keyboard input		
		//running = inputsControl(m_tracker); HACK
				
		// conversion from ObjectTracker coordinates to ObjectTracker CogX.vision coordinates
		convertParticle2Pose(ids->trackpose, obj->pose);
		
		// Send new data to working memory
		obj->detectionConfidence = ids->trackpose.w;
		obj->time = convertTime(dTimeStamp);
		//log("WM_id: %s", m_modelID_list[i].cast_AD.id.c_str());
		overwriteWorkingMemory(ids->cast_AD.id, obj);
		
	}
	
println("Y");
	m_tracker->drawCoordinates();
	m_tracker->swap();
	
	
	fTimeTracker = m_timer.Update();
println("Z");
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
		log("receive VisualObject: can not convert VisualObject to tracker model");
		return;
	}
	
	// Get IDs of working memory object and resources object
	IDList ids;
	ids.resources_ID = g_Resources->AddModel(model, obj->label.c_str());
	ids.cast_AD = _wmc.address;
	
	// converte pose of object to tracking pose (=particle)
	convertPose2Particle(obj->pose, ids.trackpose);

	// set zero pose of tracker to this trackpose
	m_tracker->setZeroPose(ids.trackpose);
	
	// add IDs and visual object to lists
	m_modelID_list.push_back(ids);
	
	
	log("received VisualObject: '%s'", obj->label.c_str());

	if(m_modelID_list.size()>m_maxModels){
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
				if(g_Resources->GetNumModels()<=0)
					log("start tracking: no model to track in memory");
				else{
					log("start tracking: ok");
					//vector<int> camIds;
					//camIds.push_back(camId);
					// start receiving images pushed by the video server
					//videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);
					track = true;
				}
			}
			break;
		case VisionData::STOP:
			if(track){
				log("stop tracking: ok");
				//videoServer->stopReceiveImages(getComponentID().c_str());
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
 
  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> camId;
  }
  
	if((it = _config.find("--testmode")) != _config.end())
		testmode = true;
	
	if((it = _config.find("--BFC_disabled")) != _config.end())
		bfc = false;

	if((it = _config.find("--trackTexture")) != _config.end())
		trackTexture = true;

	if((it = _config.find("--maxModels")) != _config.end())
	{
    istringstream istr(it->second);
    istr >> m_maxModels;
  }else{
		m_maxModels = 3;
	}
		
  /*
  if((it = _config.find("--log")) != _config.end())
  	g_Resources->ShowLog(true);
  else
  	g_Resources->ShowLog(false);
  */
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
  delete(m_tracker);
}

void ObjectTracker::receiveImages(const std::vector<Video::Image>& images)
{
  assert(images.size() > 0);
  m_image = images[0];
  runTracker(m_image);
}

void ObjectTracker::runComponent(){
  
  // Initialize Tracker
  // Grab one image from VideoServer for initialisation
  videoServer->getImage(camId, m_image);
  initTracker(m_image);
  
  while(running)
  {
    if(track){
      // HACK: actually should use receiveImages(), but that is still buggy
      videoServer->getImage(camId, m_image);
      runTracker(m_image);
    }
    else if(!track){
			// * Idle *
			sleepComponent(1000);
			running = inputsControl(m_tracker);	// ask for inputs (e.g. quit command)
		}
  }
  
  log("stop");
}



