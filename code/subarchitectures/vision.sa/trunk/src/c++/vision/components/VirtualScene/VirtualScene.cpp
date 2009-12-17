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
	
 	tgModel model;
	if(!convertGeometryModel(obj->model, &model)){
		log("  error can not convert VisualObject to tracker model");
		return;
	}
	tgModel::Material matSilver;
	matSilver.ambient = vec4(0.19,0.19,0.19,1.0);
	matSilver.diffuse = vec4(0.51,0.51,0.51,1.0);
	matSilver.specular = vec4(0.77,0.77,0.77,1.0);
	matSilver.shininess = 51.2;
	model.m_material = matSilver;
	model.computeNormals();
	model.printInfo();
	m_modellist.push_back(model);
	
	log("VisualObject added to Scene");
}

void VirtualScene::changeVisualObject(const cdl::WorkingMemoryChange & _wmc){
	log("[VirtualScene::changeofVisualObject] WARNING: function not implemented");
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
  
  log("stop");
}

// Tracking
void VirtualScene::initScene(const Video::Image &image){
  
  log("Initialising Scene");
  
  // *** Initialisation of Scene ***
  m_width = image.width;
  m_height = image.height;
	m_engine.Init(m_width, m_height, 1.0);
	
  // load camera parameters from Video::Image.camPars to OpenGL camera 'm_camera'
  //loadCameraParameters(m_camera, image.camPars, 0.1, 10.0);

  log("... initialisation successfull!");
}

void VirtualScene::runScene(){

	
	
	for(int i=0; i<m_modellist.size(); i++){
		m_modellist[i].drawFaces();
	}
	m_running = m_engine.Update();
}



