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
  m_track = false;
  m_running = true;
  m_testmode = false;
  m_textured = true;
  m_automatictexturing = true;
  m_bfc = true;
  
  fTimeTotal = 0.0;
}

ObjectTracker::~ObjectTracker(){
	
}

// *** Working Memory Listeners ***
void ObjectTracker::receiveTrackingCommand(const cdl::WorkingMemoryChange & _wmc){
	TrackingCommandPtr track_cmd = getMemoryEntry<TrackingCommand>(_wmc.address);
	log("Received tracking command: %d, VisualObject: %s", track_cmd->cmd, track_cmd->visualObjectID.c_str());
	m_trackingCommandList.push_back(track_cmd);
	m_trackingCommandWMID.push_back(_wmc.address.id);
}
	
// *** base functions *** (configure, start, runcomponent)
void ObjectTracker::configure(const map<string,string> & _config){
  map<string,string>::const_iterator it;
  
  log("Configure:");
  
  if((it = _config.find("--videoname")) != _config.end())
  {
    m_videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
    int camId;
    istringstream istr(it->second);
    istr >> camId;
    m_camIds.push_back(camId);
    log("  Camera ID: %d", camId);
  }
  
// 	if((it = _config.find("--testmode")) != _config.end()){
// 		m_testmode = true;
// 		log("test mode enabled (only for ObjectTrackerTest component)");
// 	}
	
	if((it = _config.find("--inifile")) != _config.end()){
		m_ini_file = it->second;
		log("  INI file: '%s'", m_ini_file.c_str());
	}else{
		throw runtime_error(exceptionMessage(__HERE__, "No INI file given!"));
	}
	
	m_textured = false;
	if((it = _config.find("--textured")) != _config.end()){
		m_textured = true;
		log("  Mode: texture tracking");
	}else{
		m_textured = false;
		log("  Mode: edge tracking");
	}
	
	if((it = _config.find("--automatictexturing")) != _config.end()){
		m_automatictexturing = true;
		log("  Texturing: automatic");
	}else{
		m_automatictexturing = false;
		log("  Texturing: manual");
	}
	
	if((it = _config.find("--BFC_disabled")) != _config.end()){
		m_bfc = false;
	}
	if(m_bfc)
		log("  Backface culling: enabled, suitable for objects with closed surface (i.e. a cube)");
	else
		log("  Backface culling: disabled, suitable for non closed surfaces (i.e. a polyflap)");

	if((it = _config.find("--maxModels")) != _config.end())
	{
    istringstream istr(it->second);
    istr >> m_maxModels;
  }else{
		m_maxModels = 3;
	}
	
	log("  max. models: %d", m_maxModels);
}

void ObjectTracker::start(){
  // get connection to the video server
  m_videoServer = getIceServer<Video::VideoInterface>(m_videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);
  
  addChangeFilter(createLocalTypeFilter<TrackingCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectTracker>(this,
        &ObjectTracker::receiveTrackingCommand));
}

void ObjectTracker::destroy(){
	TrackingEntryList::iterator it;
	for(it=m_trackinglist.begin(); it<m_trackinglist.end(); it++){
		delete((*it));
	}
}

void ObjectTracker::receiveImages(const std::vector<Video::Image>& images){
  lockComponent();
		assert(images.size() > 0);
		m_image = images[0];
		convertCameraParameter(m_image.camPars, m_trackCamPars);
	unlockComponent();
}

void ObjectTracker::runComponent(){
  
  int i=0;
  // Initialize Tracker
  // Grab one image from VideoServer for initialisation
  initTracker();
  m_videoServer->startReceiveImages(getComponentID().c_str(), m_camIds, 0, 0);
  
  while(isRunning() && m_running)
  {
    while(m_trackingCommandList.size()>0){
			applyTrackingCommand();
		}
		
		if(m_track){
    	// * Tracking *
      runTracker();
    }
    else if(!m_track){
			// * Idle *
			sleepComponent(1000);
		}
		
		// Query keyboard input
		m_running = inputsControl(m_tracker, fTimeTracker);
  }
  
	delete(m_tracker);	// delete tracker in this thread (OpenGL !!!)
  log("stop");
}

// Tracking
void ObjectTracker::initTracker(){
  
  log("Initialising Tracker");
  // *** Initialisation of Tracker ***
  int id = 0;
  
	m_videoServer->getImage(m_camIds[0], m_image);
	m_ImageWidth = m_image.width;
	m_ImageHeight = m_image.height;
	last_image_time = m_image.time;
	convertCameraParameter(m_image.camPars, m_trackCamPars);

	
	// Create edge or texture tracker
	if(m_textured){
		m_tracker = new TextureTracker();
	}else{
		m_tracker = new EdgeTracker();
	}
	
	// Initialize tracker
	if(!m_tracker->init(m_ini_file.c_str(), m_ImageWidth, m_ImageHeight)){														
		throw runtime_error(exceptionMessage(__HERE__, "INI file not found!"));
		m_running = false;
	}
	
	
	m_trackCamPars.zFar = m_tracker->getCamZFar();
	m_trackCamPars.zNear = m_tracker->getCamZNear();
	if( !m_tracker->setCameraParameters(m_trackCamPars) ){
		throw runtime_error(exceptionMessage(__HERE__, "Wrong Camera Parameter"));
		m_running = false;
	}
	 
  log("... initialisation successfull!");
}

void ObjectTracker::applyTrackingCommand(){
	lockComponent();
	TrackingCommandPtr track_cmd = m_trackingCommandList.front();
	m_trackingCommandList.erase(m_trackingCommandList.begin());
	string track_id = m_trackingCommandWMID.front();
	m_trackingCommandWMID.erase(m_trackingCommandWMID.begin());
	unlockComponent();

	
	TrackingEntryList::iterator it;
	TrackingEntry* trackingEntry=0;
	
	if(track_cmd->cmd == VisionData::START){
		log("  VisionData::START");
		if(m_track){
			log("  VisionData::START: I'm allready tracking");
		}else{
			log("  VisionData::START: ok");
			m_track = true;
		}
	}else if(track_cmd->cmd == VisionData::STOP){
		log("  VisionData::STOP");
		if(m_track){
			log("  VisionData::STOP: ok");
			m_track = false;
		}else{
			log("  VisionData::STOP: I'm not tracking");
		}
	}else if(track_cmd->cmd == VisionData::ADDMODEL){
		log("  VisionData::ADDMODEL");
		if(m_trackinglist.size()<m_maxModels){
			Tracking::Pose pose;
			Tracking::Model model;
			trackingEntry = new TrackingEntry();
			trackingEntry->visualObjectID = track_cmd->visualObjectID;
			trackingEntry->obj = getMemoryEntry<VisualObject>(trackingEntry->visualObjectID);
			convertPose2Particle(trackingEntry->obj->pose, pose);
			convertGeometryModel(trackingEntry->obj->model, model);
			trackingEntry->id = m_tracker->addModel(model, pose, trackingEntry->obj->label, true);
			log("  VisionData::ADDMODEL '%s' at (%.3f, %.3f, %.3f)", trackingEntry->obj->label.c_str(), pose.t.x, pose.t.y, pose.t.z);
			m_trackinglist.push_back(trackingEntry);
			log("  VisionData::ADDMODEL: ok");
		}else{
			log("  VisionData::ADDMODEL: number of max. trackable models reached: %d", m_maxModels);
		}
		
	}else if(track_cmd->cmd == VisionData::REMOVEMODEL){
		log("  VisionData::REMOVEMODEL");
		for(it = m_trackinglist.begin(); it != m_trackinglist.end(); it++){
			if((*it)->visualObjectID.compare(track_cmd->visualObjectID) == 0){ // if (m_trackinglist[i].id == track_cmd.visualObjectID)
				trackingEntry = (*it);
				m_tracker->removeModel(trackingEntry->id);
				delete(trackingEntry);
				m_trackinglist.erase(it);
				log("  VisionData::REMOVEMODEL: ok");
				return;
			}
		}			
	}else if(track_cmd->cmd == VisionData::LOCK){
		log("  VisionData::LOCK");
		for(it = m_trackinglist.begin(); it != m_trackinglist.end(); it++){
			if((*it)->visualObjectID.compare(track_cmd->visualObjectID) == 0){ // if (m_trackinglist[i].id == track_cmd.visualObjectID)
				m_tracker->setModelLock((*it)->id, true);
				log("  VisionData::LOCK: ok");
				return;
			}
		}
	}else if(track_cmd->cmd == VisionData::UNLOCK){
		log("  VisionData::UNLOCK");
		for(it = m_trackinglist.begin(); it != m_trackinglist.end(); it++){
			if((*it)->visualObjectID.compare(track_cmd->visualObjectID) == 0){ // if (m_trackinglist[i].id == track_cmd.visualObjectID)
				m_tracker->setModelLock((*it)->id, false);
				log("  VisionData::UNLOCK: ok");
				return;
			}
		}
	}else if(track_cmd->cmd == VisionData::GETPOINT3D){
		log("  VisionData::GETPOINT3D");
		for(it = m_trackinglist.begin(); it != m_trackinglist.end(); it++){
			if((*it)->visualObjectID.compare(track_cmd->visualObjectID) == 0){ // if (m_trackinglist[i].id == track_cmd.visualObjectID)
				bool b;
				float x, y, z;
				trackingEntry = (*it);
				
				track_cmd->pointOnModel.assign(track_cmd->points.size(), 0);
				for(unsigned i=0; i<track_cmd->points.size(); i++){
		
					b=m_tracker->getModelPoint3D(	trackingEntry->id,
																				(int)track_cmd->points[i].texCoord.x,
																				(int)track_cmd->points[i].texCoord.y,
																				x, y, z);
		
					track_cmd->points[i].pos.x = x;
					track_cmd->points[i].pos.y = y;
					track_cmd->points[i].pos.z = z;
					track_cmd->pointOnModel[i] = b;
				}
				overwriteWorkingMemory(track_id, track_cmd);
				
				log("  VisionData::GETPOINT3D: ok");
				return;
			}
		}
	}else if(track_cmd->cmd == VisionData::RELEASEMODELS){
		log("  VisionData::RELEASEMODELS");
		for(it = m_trackinglist.begin(); it != m_trackinglist.end(); it++){
			trackingEntry = (*it);
			m_tracker->removeModel(trackingEntry->id);
			delete(trackingEntry);
			m_trackinglist.erase(it);
			return;
		}
		log("  VisionData::RELEASEMODELS: ok");
	}else{
		log("  VisionData::???UNKNOWN???: doing nothing");
	}
}

void ObjectTracker::runTracker(){

	fTimeTracker=0.0;
	int i,c;
	Pose pose;
	double dTime;
	
	// Get image and update it
	if( !m_tracker->setCameraParameters(m_trackCamPars) ){
		throw runtime_error(exceptionMessage(__HERE__, "Wrong Camera Parameter"));
		m_running = false;
	}
	
	// update time
	m_timer.Update();
	
	lockComponent();
		dTime = getFrameTime(last_image_time, m_image.time);
		// image processing
		m_tracker->image_processing((unsigned char*)(&m_image.data[0]));
	unlockComponent();
	
	last_image_time = m_image.time;
	m_tracker->setFrameTime(dTime);
	m_tracker->drawImage(NULL);

	// track models
	m_tracker->track();
			
	// update pose and confidence in WorkingMemory
	for(i=0; i<m_trackinglist.size(); i++){
		m_tracker->getModelPose(m_trackinglist[i]->id, pose);
		m_tracker->getModelConfidence(m_trackinglist[i]->id, c);
		convertParticle2Pose(pose, m_trackinglist[i]->obj->pose);		
		m_trackinglist[i]->obj->time = m_image.time;
		overwriteWorkingMemory(m_trackinglist[i]->visualObjectID, m_trackinglist[i]->obj);
	}
	double time = m_image.time.s + m_image.time.us * 1e-6;
// 	printf("%e %f %f %f %f\n", time, fTimeTotal, pose.t.x, pose.t.y, pose.t.z);
	
	// draw results
	m_tracker->drawResult();
// 	m_tracker->drawCalibrationPattern();
	m_tracker->drawCoordinates();
	m_tracker->swap();
	
	if(m_automatictexturing)
		m_tracker->textureFromImage(true);

	fTimeTracker = m_timer.Update();
	fTimeTotal += fTimeTracker;
}



