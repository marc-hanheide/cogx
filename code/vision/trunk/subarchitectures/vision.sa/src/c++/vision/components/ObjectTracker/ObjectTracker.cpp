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
  m_tracker = new EdgeTracker();
  if(!m_tracker->init(	m_image.width, m_image.height,		// image size in pixels
												3000,															// maximum number of particles (=storage size of particle list)
												20.0,															// standard deviation of rotational noise in degree
												0.05,															// standard deviation of translational noise in meter
												20.0,															// edge matching tolerance in degree
												0.05,															// goal tracking time in seconds
												true,															// kalman filtering enabled
												false))														// locked particles (press 'l' to unlock)
	{														
		log("Initialisation failed!");
		running = false;
  }
  
  // Load Camera
  if((id = g_Resources->AddCamera("cam_extrinsic")) == -1)
  	running = false;
  m_camera = g_Resources->GetCamera(id);
  
  log("setting camera parameters");
	m_tracker->lock();
	m_camera->Set(	0.0993, -0.12907, 0.3031,
									0.1, 0.12, 0.0,
									0.0, 0.0, 1.0,
									49, m_image.width, m_image.height,
									zNear=0.1, zFar=100.0,
									GL_PERSPECTIVE);
									
	// intrinsic parameters
	float fx = 2.0*m_image.camPars.fx / m_image.width;				// scale range from [0 ... 640] to [0 ... 2]
  float fy = -2.0*m_image.camPars.fy / m_image.height;			// scale range from [0 ... 480] to [0 ...-2]
  float cx = (2.0*m_image.camPars.cx / m_image.width)-1.0;	// move coordinates from left to middle of image: [0 ... 2] -> [-1 ... 1]
  float cy = 1.0-(2.0*m_image.camPars.cy / m_image.height);	// flip and move coordinates from top to middle of image: [0 ...-2] -> [-1 ... 1]
  float z1 = (zFar+zNear)/(zNear-zFar);											// entries for clipping planes
  float z2 = 2*zFar*zNear/(zNear-zFar);
  
  m_intrinsic[0]=fx;	m_intrinsic[1]=0;		m_intrinsic[2]=0;		m_intrinsic[3]=0;		// transposed matrix loading
  m_intrinsic[4]=0;		m_intrinsic[5]=fy;	m_intrinsic[6]=0;		m_intrinsic[7]=0;
  m_intrinsic[8]=cx;	m_intrinsic[9]=cy;	m_intrinsic[10]=0;	m_intrinsic[11]=1;
  m_intrinsic[12]=0;	m_intrinsic[13]=0;	m_intrinsic[14]=0;	m_intrinsic[15]=0;
	
	// extrinsic parameters
	cogx::Math::Matrix33 R = m_image.camPars.pose.rot;
	cogx::Math::Vector3 t = m_image.camPars.pose.pos * 0.001;
	m_extrinsic[0]=R.m00;	m_extrinsic[1]=R.m01;	m_extrinsic[2]=R.m02;		m_extrinsic[3]=0.0;
	m_extrinsic[4]=R.m10;	m_extrinsic[5]=R.m11;	m_extrinsic[6]=R.m12;		m_extrinsic[7]=0.0;	
	m_extrinsic[8]=R.m20;	m_extrinsic[9]=R.m21;	m_extrinsic[10]=R.m22;	m_extrinsic[11]=0.0;	
	m_extrinsic[12]=0.0;	m_extrinsic[13]=0.0;	m_extrinsic[14]=0.0;		m_extrinsic[15]=1.0;
	vec4 tp = -(m_extrinsic * vec4(t.x, t.y, t.z, 1.0));
	m_extrinsic[12]=tp.x; m_extrinsic[13]=tp.y; m_extrinsic[14]=tp.z;
	
	
	v1 = vec4(0.188, 0.151, 0.0, 1.0);
	v1 = m_extrinsic * v1;
	log("view: %f %f %f %f", v1.x, v1.y, v1.z, v1.w);
	v1 = m_intrinsic * v1;
	log("clip: %f %f %f %f", v1.x, v1.y, v1.z, v1.w);
	v1.x = v1.x/v1.w;
	v1.y = v1.y/v1.w;
	v1.z = v1.z/v1.w;
	log("ndc: %f %f %f %f", v1.x, v1.y, v1.z, v1.w);
	v1.x = m_image.width * 0.5 * v1.x + m_image.width*0.5;
	v1.y = m_image.height * 0.5 * v1.y + m_image.height*0.5;
	v1.z = (zFar-zNear)*0.5 * v1.z + (zFar+zNear)*0.5;
	log("v1: %f %f %f", v1.x, v1.y, v1.z);
	
	
	
	
	
	m_camera->SetIntrinsic(m_intrinsic);
	m_camera->SetExtrinsic(m_extrinsic);
	m_camera->Print();
	
	track = true;
	
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
	
	m_tracker->setCamPerspective(m_camera);
	m_tracker->drawImage((unsigned char*)(&m_image.data[0]));
	m_tracker->drawTest();
	m_tracker->drawPixel(v1.x, v1.y, vec3(0.0,1.0,0.0),3.0);
	
	m_tracker->swap();
	running = m_tracker->inputs();
	
/*
	fTimeImage = m_timer.Update();
	if(testmode){
		m_camera->Set(	0.2, 0.2, 0.2,
										0.0, 0.0, 0.0,
										0.0, 1.0, 0.0,
										49, m_image.width, m_image.height,
										0.1, 10.0,
										GL_PERSPECTIVE);
	}
	
	

	// Track all models
	for(i=0; i<m_modelID_list.size(); i++){
		model = g_Resources->GetModel(m_modelID_list[i].resources_ID);
		obj = getMemoryEntry<VisualObject>(m_modelID_list[i].cast_AD);

		// conversion from CogX.vision coordinates to ObjectTracker coordinates
		convertPose2Particle(obj->pose, m_trackpose);
		m_trackpose.w = obj->detectionConfidence;

		// Track model
		running = m_tracker->track((unsigned char*)(&m_image.data[0]), model, m_camera, m_trackpose, m_trackpose);
		m_tracker->drawResult(&m_trackpose);
		m_tracker->renderCoordinates();
		m_tracker->drawTest();
		//m_tracker->drawPixel(100,100,vec3(1.0,0.0,0.0),3.0);
		m_tracker->swap();
		
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
*/
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
	
	/*
	if(!makeCube(model, model)){
		log("makeCube: Warning could not make perfect cube");
	}
	*/

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
				m_tracker->unlock();
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
  
  /*
  if((it = _config.find("--log")) != _config.end())
  	g_Resources->ShowLog(true);
  else
  	g_Resources->ShowLog(false);
  */
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
  sleepComponent(1000);
  
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
  delete(m_tracker);
  log("stop");
  
}


