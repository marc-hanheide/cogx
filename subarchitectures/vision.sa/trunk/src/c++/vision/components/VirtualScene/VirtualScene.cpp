/**
 * @author Thomas Mörwald
 * @date April 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "VirtualScene.h"
#include <VideoUtils.h>


/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::VirtualScene();
  }
}



using namespace cast;
using namespace std;
using namespace VisionData;


VirtualScene::VirtualScene(){
	m_running = true;
	m_render = true;
}

VirtualScene::~VirtualScene(){

}

// *** Working Memory Listeners ***
void VirtualScene::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc){
	log("receiving VisualObject");
	
	VisualObjectPtr obj = getMemoryEntry<VisualObject>(_wmc.address);
	
	ModelEntry newModelEntry;
	
	if(!convertGeometryModel(obj->model, newModelEntry.model)){
		log("  error can not convert VisualObject to tracker model");
		return;
	}
	
	 // just an example material (should be random to separate objects more easily
	tgModel::Material matSilver; 
	matSilver.ambient = vec4(0.19,0.19,0.19,1.0);
	matSilver.diffuse = vec4(0.51,0.51,0.51,1.0);
	matSilver.specular = vec4(0.77,0.77,0.77,1.0);
	matSilver.shininess = 51.2;
	
	newModelEntry.model.m_material = matSilver;
	newModelEntry.model.ComputeNormals();
	newModelEntry.model.PrintInfo();
	convertPose2tgPose(obj->pose, newModelEntry.model.m_pose);
	newModelEntry.obj = obj;
	newModelEntry.castWMA = _wmc.address;
	m_modellist.push_back(newModelEntry);
	
	log("VisualObject added to Scene");
}

void VirtualScene::changeVisualObject(const cdl::WorkingMemoryChange & _wmc){
	
	VisualObjectPtr obj = getMemoryEntry<VisualObject>(_wmc.address);
	
	for(int i=0; i<m_modellist.size(); i++){
		if(m_modellist[i].castWMA == _wmc.address){
			convertPose2tgPose(obj->pose, m_modellist[i].model.m_pose);
		}			
	}
	
	//log("[VirtualScene::changeofVisualObject] WARNING: function not implemented");
}

void VirtualScene::removeVisualObject(const cdl::WorkingMemoryChange & _wmc){
	log("[VirtualScene::removeVisualObject] WARNING: function not implemented");
}

// *** base functions *** (configure, start, runcomponent)
void VirtualScene::configure(const map<string,string> & _config){
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
  
  if((it = _config.find("--maxModels")) != _config.end())
	{
    istringstream istr(it->second);
    istr >> m_maxModels;
  }else{
		m_maxModels = 3;
	}
	
}

void VirtualScene::start(){
  // get connection to the video server
  m_videoServer = getIceServer<Video::VideoInterface>(m_videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

  addChangeFilter(createLocalTypeFilter<VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<VirtualScene>(this,
        &VirtualScene::receiveVisualObject));
        
	addChangeFilter(createLocalTypeFilter<VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<VirtualScene>(this,
        &VirtualScene::changeVisualObject));
}

void VirtualScene::destroy(){

}

void VirtualScene::receiveImages(const std::vector<Video::Image>& images){
/*
  assert(images.size() > 0);
  m_image = images[0];
  runTracker(m_image);
  */
}

void VirtualScene::runComponent(){
  
  // Initialize Tracker
  // Grab one image from VideoServer for initialisation
  m_videoServer->getImage(m_camId, m_image);
  initScene(m_image);
  
  while(m_running)
  {
    if(m_render){
      //m_videoServer->getImage(m_camId, m_image);
      runScene();
    }else{
			// * Idle *
			sleepComponent(1000);
		}
  }
  
  delete(m_engine);
  log("stop");
}

// Tracking
void VirtualScene::initScene(const Video::Image &image){
  
  log("Initialising Scene");
  
  // *** Initialisation of Scene ***
  m_width = image.width;
  m_height = image.height;
  
  m_engine = new(tgEngine);
	m_engine->Init(m_width, m_height, 1.0);
	
	loadCameraParameters(&m_camera, image.camPars, 0.1, 10.0);
	m_engine->SetCamera(m_camera);

  log("... initialisation successfull!");
}

void VirtualScene::runScene(){
	
	for(int i=0; i<m_modellist.size(); i++){
		m_modellist[i].model.DrawFaces();
	}
	m_running = m_engine->Update(m_fTime);
}



