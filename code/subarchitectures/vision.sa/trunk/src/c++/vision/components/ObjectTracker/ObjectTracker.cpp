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
	destroy();
	log("[ObjectTracker::~ObjectTracker()]");
}

// *** Working Memory Listeners ***
void ObjectTracker::receiveTrackingCommand(const cdl::WorkingMemoryChange & _wmc){
	TrackingCommandPtr track_cmd = getMemoryEntry<TrackingCommand>(_wmc.address);
	
	log("Received tracking command ...");
	switch(track_cmd->cmd){
		case VisionData::START:
			if(m_track){
				log("  Start tracking: I'm allready tracking");
			}else{
				log("  Start tracking: ok");
				m_track = true;
			}
			break;
		case VisionData::STOP:
			if(m_track){
				log("  Stop tracking: ok");
				m_track = false;
			}else{
				log("  Stop tracking: I'm not tracking");
			}
			break;
		case VisionData::RELEASEMODELS:
			log("  Release models: releasing all models (not implemented)");
			break;
		default:
			log("  Unknown tracking command, doing nothing");
			break;
	}	
}

void ObjectTracker::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc){
	log("Receiving VisualObject");
	
	if((int)m_trackinglist.size()>=m_maxModels){
		log("  warning: maximum trackable models set to %d", m_maxModels);
		return;
	}
	
	TrackingEntry* trackingEntry = new TrackingEntry();
	trackingEntry->obj = getMemoryEntry<VisualObject>(_wmc.address);
	trackingEntry->castWMA = _wmc.address;
	trackingEntry->cmd = TrackingEntry::ADD;
	m_trackinglist.push_back(trackingEntry);
	
	log("  VisualObject received: '%s'", trackingEntry->obj->label.c_str());
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
  
  log("Configure:");
  
  if((it = _config.find("--videoname")) != _config.end())
  {
    m_videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> m_camId;
    log("  Camera ID: %d", m_camId);
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
		log("  Mode: Texture tracking");
	}else{
		m_textured = false;
		log("  Mode: Edge tracking");
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
	
	log("  Objects: %d", m_maxModels);
  	
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
	TrackingEntryList::iterator it;
	for(it=m_trackinglist.begin(); it<m_trackinglist.end(); it++){
		delete((*it));
	}
	delete(m_tracker);
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
		m_running = inputsControl(m_tracker, fTimeTracker);

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

	if(m_textured){
		m_tracker = new TextureTracker();
	}else{
		m_tracker = new EdgeTracker();
	}

	if(!m_tracker->init(m_ini_file.c_str(), m_ImageWidth, m_ImageHeight))										// initial pose (where to reset when pressing 'z')
	{														
		throw runtime_error(exceptionMessage(__HERE__, "INI file not found!"));
		m_running = false;
	}
	 
  log("... initialisation successfull!");
}

void ObjectTracker::modifyTrackingEntry(TrackingEntryList::iterator it){
	
	TrackingEntry* trackingEntry = (*it);
	
	log("Modifying tracking entry: '%s'", trackingEntry->obj->label.c_str());
	
	if(trackingEntry->cmd == TrackingEntry::ADD){
		Tracking::Pose pose;
		Tracking::Model model;
		convertPose2Particle(trackingEntry->obj->pose, pose);
		convertGeometryModel(trackingEntry->obj->model, model);
		trackingEntry->id = m_tracker->addModel(model, pose, true);
		trackingEntry->cmd = TrackingEntry::TRACK;
		log("  TrackingEntry::ADD: '%s'", trackingEntry->obj->label.c_str());
	}	
}

void ObjectTracker::runTracker(const Video::Image &image){

	// *** Tracking Loop ***
	double dTimeStamp;
	fTimeTracker=0.0;
	int i;
	Model* model;
	
	// * Tracking *
	m_timer.Update();
	dTimeStamp = m_timer.GetApplicationTime();

	// Check if new models added to tracking list
	TrackingEntryList::iterator it;
	for(it=m_trackinglist.begin(); it<m_trackinglist.end(); it++){
		if((*it)->cmd != TrackingEntry::TRACK)
			modifyTrackingEntry(it);
	}

	m_tracker->image_processing((unsigned char*)(&image.data[0]));
	m_tracker->drawImage(NULL);

	// Track model
	m_tracker->track();
		
		// conversion from ObjectTracker coordinates to ObjectTracker CogX.vision coordinates
		// TODO TODO TODO get pose of objects and convert them
// 		convertParticle2Pose(m_trackinglist[i].trackpose, m_trackinglist[i].obj->pose);
	
		// TODO TODO TODO Send new data to working memory
// 		m_trackinglist[i].obj->detectionConfidence = m_trackinglist[i].trackpose.c; 
// 		m_trackinglist[i].obj->time = convertTime(dTimeStamp);
// 		overwriteWorkingMemory(m_trackinglist[i].castWMA.id, m_trackinglist[i].obj);

	
	m_tracker->drawCoordinates();
	
	m_tracker->drawResult();
	
	
	m_tracker->swap();

	fTimeTracker = m_timer.Update();
}



