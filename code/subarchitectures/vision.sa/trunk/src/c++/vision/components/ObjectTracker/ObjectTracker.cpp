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
  m_drawcoords = false;
  m_bfc = true;

  fTimeTotal = 0.0;
  m_screenID = 0;
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

void ObjectTracker::overwriteVisualObject(const cdl::WorkingMemoryChange & _wmc){

	VisualObjectPtr obj = getMemoryEntry<VisualObject>(_wmc.address);
	TrackingEntryList::iterator it;
	if(obj->componentID.compare(getComponentID())!=0){
		for(it = m_trackinglist.begin(); it != m_trackinglist.end(); it++){
			if((*it)->visualObjectID.compare(_wmc.address.id) == 0){
				log("Locking VisualObject '%s' for tracking called by '%s'", _wmc.address.id.c_str(), obj->componentID.c_str());
				(*it)->pose = obj->pose;
				(*it)->model = obj->model;
				log("VisualObject pose: %f %f %f", (*it)->pose.pos.x, (*it)->pose.pos.y, (*it)->pose.pos.z);
			}
		}
	}
}

// *** base functions *** (configure, start, runcomponent)
void ObjectTracker::configure(const map<string,string> & _config){
  map<string,string>::const_iterator it;

  log("Configure:");

  if((it = _config.find("--videoname")) != _config.end())
  {
    m_videoServerName = it->second;
  }

  if((it = _config.find("--pcserver")) != _config.end()){
    m_pcServerName = it->second;
    PointCloudClient::configureServerCommunication(_config);
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
	if((it = _config.find("--coordinateframe")) != _config.end()){
		m_drawcoords = true;
	}else{
		m_drawcoords = false;
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
  if (m_videoServerName.length() > 0) {
    m_videoServer = getIceServer<Video::VideoInterface>(m_videoServerName);
  }

  if (m_pcServerName.length() > 0) {
	  PointCloudClient::startPCCServerCommunication(*this);
  }

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

  addChangeFilter(createLocalTypeFilter<TrackingCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectTracker>(this,
        &ObjectTracker::receiveTrackingCommand));
  addChangeFilter(createLocalTypeFilter<VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<ObjectTracker>(this,
        &ObjectTracker::overwriteVisualObject));
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

  // Initialize Tracker
  // Grab one image from VideoServer for initialisation
  initTracker();

  // if this cannot be done then we only get the first image & so need to pull images in runTracker()
  if (m_videoServerName.length() > 0) {
    m_videoServer->startReceiveImages(getComponentID().c_str(), m_camIds, 0, 0);
    // images will be received via the callback method receiveImages()
  }

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

	getImage();
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
			log("  VisionData::START: I'm already tracking");
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
		log("  VisionData::ADDMODEL '%s'",track_cmd->visualObjectID.c_str());
		if(m_trackinglist.size()<m_maxModels){
			if(track_cmd->visualObjectID.empty()){
				log("  VisionData::ADDMODEL: Warning, no valid ID for VisualObject. (empty)");
				return;
			}
			Tracking::Pose pose;
			Tracking::Model model;
			trackingEntry = new TrackingEntry();
			trackingEntry->visualObjectID = track_cmd->visualObjectID;
			trackingEntry->obj = getMemoryEntry<VisualObject>(trackingEntry->visualObjectID);
			convertPose2Particle(trackingEntry->obj->pose, pose);
			convertGeometryModel(trackingEntry->obj->model, model);
			trackingEntry->id = m_tracker->addModel(model, pose, trackingEntry->obj->identLabels[0], true);
			m_trackinglist.push_back(trackingEntry);
			log("  VisionData::ADDMODEL '%s' at (%.3f, %.3f, %.3f): ok", trackingEntry->obj->identLabels[0].c_str(), pose.t.x, pose.t.y, pose.t.z);
		}else{
			log("  VisionData::ADDMODEL: number of max. trackable models reached: %d", m_maxModels);
		}

	}else if(track_cmd->cmd == VisionData::REMOVEMODEL){
		log("  VisionData::REMOVEMODEL '%s'", track_cmd->visualObjectID.c_str());
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
	}else if(track_cmd->cmd == VisionData::OVERWRITE){
		log("  VisionData::OVERWRITE '%s'", track_cmd->visualObjectID.c_str());
		for(it = m_trackinglist.begin(); it != m_trackinglist.end(); it++){
			if((*it)->visualObjectID.compare(track_cmd->visualObjectID) == 0){ // if (m_trackinglist[i].id == track_cmd.visualObjectID)
				Tracking::Pose pose;
				trackingEntry = (*it);
				convertPose2Particle(trackingEntry->pose, pose);
// 				convertGeometryModel(trackingEntry->model, model);
				m_tracker->addPoseHypothesis(trackingEntry->id, pose, trackingEntry->obj->identLabels[0], true);
				log("  VisionData::OVERWRITE: ok, id: %d", trackingEntry->id);
				return;
			}
		}

	}else if(track_cmd->cmd == VisionData::LOCK){
		log("  VisionData::LOCK '%s'",track_cmd->visualObjectID.c_str());
		for(it = m_trackinglist.begin(); it != m_trackinglist.end(); it++){
			if((*it)->visualObjectID.compare(track_cmd->visualObjectID) == 0){ // if (m_trackinglist[i].id == track_cmd.visualObjectID)
				m_tracker->setModelLock((*it)->id, true);
				(*it)->lock = true;
				log("  VisionData::LOCK: ok");
				return;
			}
		}
	}else if(track_cmd->cmd == VisionData::UNLOCK){
		log("  VisionData::UNLOCK '%s'",track_cmd->visualObjectID.c_str());
		for(it = m_trackinglist.begin(); it != m_trackinglist.end(); it++){
			if((*it)->visualObjectID.compare(track_cmd->visualObjectID) == 0){ // if (m_trackinglist[i].id == track_cmd.visualObjectID)
				m_tracker->setModelLock((*it)->id, false);
				(*it)->lock = false;
				log("  VisionData::UNLOCK: ok");
				return;
			}
		}
	}else if(track_cmd->cmd == VisionData::GETPOINT3D){
		log("  VisionData::GETPOINT3D 's'",track_cmd->visualObjectID.c_str());
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
		it = m_trackinglist.begin();
		while(it != m_trackinglist.end()){
			m_tracker->removeModel((*it)->id);
			delete(trackingEntry);
			m_trackinglist.erase(it);
		}
		log("  VisionData::RELEASEMODELS: ok %d", m_trackinglist.size());
	}else if(track_cmd->cmd == VisionData::SCREENSHOT){
		log("  VisionData::SCREENSHOT");
		char filename[16];
		sprintf(filename, "img_%d.jpg", m_screenID++);
		m_tracker->saveScreenshot(filename);
	}else{
		log("  VisionData::???UNKNOWN???: doing nothing");
	}
}

void ObjectTracker::runTracker(){

	int i;
	float c;
	Pose pose;
	double dTime;

// 	m_timer.Update();

	// Get image and update it
	if(	m_trackCamPars.width != m_ImageWidth ||
		m_trackCamPars.height != m_ImageHeight){
		throw runtime_error(exceptionMessage(__HERE__, "Resolution of image changed"));
		m_running = false;
	}
	if( !m_tracker->setCameraParameters(m_trackCamPars) ){
		throw runtime_error(exceptionMessage(__HERE__, "Wrong Camera Parameter"));
		m_running = false;
	}


	lockComponent();

	if (m_pcServerName.length()>0) {
		getRectImage(m_camIds[0], 640, m_image);
		//log("got image from Kinect: " + m_image.height);
		convertCameraParameter(m_image.camPars, m_trackCamPars);
	}
  else {
// this line was commented out before the pcServer functionality was added
//		m_videoServer->getImage(m_camIds[0], m_image);    
  }

		dTime = getFrameTime(last_image_time, m_image.time);

// 		if(dTime==0.0){
// 			unlockComponent();
// 			return;
// 		}

		// image processing
		m_tracker->image_processing((unsigned char*)(&m_image.data[0]));
		m_tracker->setFrameTime(dTime);
	unlockComponent();

	last_image_time = m_image.time;


	// track models
	m_tracker->track();

	fTimeTracker = m_timer.Update();

	// update pose and confidence in WorkingMemory
	for(i=0; i<m_trackinglist.size(); i++){
		if(!m_trackinglist[i]->lock){
			m_trackinglist[i]->obj = getMemoryEntry<VisualObject>(m_trackinglist[i]->visualObjectID);
			m_tracker->getModelPose(m_trackinglist[i]->id, pose);
			m_tracker->getModelConfidence(m_trackinglist[i]->id, c);
			convertParticle2Pose(pose, m_trackinglist[i]->obj->pose);
			m_trackinglist[i]->obj->time = m_image.time;
			m_trackinglist[i]->obj->detectionConfidence = c;
			m_trackinglist[i]->obj->componentID = getComponentID();
			overwriteWorkingMemory(m_trackinglist[i]->visualObjectID, m_trackinglist[i]->obj);
		}
	}
	double time = m_image.time.s + m_image.time.us * 1e-6;

// 	fTimeCvt = m_timer.Update();

	// draw results
	m_tracker->drawImage(NULL);
	m_tracker->drawResult();
// 	m_tracker->drawCalibrationPattern();
// 	if(m_drawcoords) m_tracker->drawCoordinates();
	m_tracker->swap();

// 	fTimeDraw = m_timer.Update();

// 	printf("%f %f %f %f %f %f\n", fTimeGrab, fTimeIP, fTimeTracker, fTimeCvt, fTimeDraw, fTimeGrab+fTimeIP+fTimeTracker+fTimeCvt+fTimeDraw);

	if(m_automatictexturing)
		m_tracker->textureFromImage(true);

	fTimeTotal += fTimeTracker;
}

void ObjectTracker::getImage()
{
	if (m_pcServerName.length()>0) {
		getRectImage(m_camIds[0], 640, m_image);
		log("got image from Kinect: " + m_image.height);
	}
	else {
		m_videoServer->getImage(m_camIds[0], m_image);
	}
}

