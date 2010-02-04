/**
 * @author Thomas Mörwald
 * @date April 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "VirtualScene.h"
#include "VirtualSceneUtils.hpp"


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
using namespace TomGine;


VirtualScene::VirtualScene(){
	m_running = true;
	m_render = true;
}

VirtualScene::~VirtualScene(){

}

// *** Working Memory Listeners ***
void VirtualScene::addVisualObject(const cdl::WorkingMemoryChange & _wmc){
	log("receiving VisualObject");
// 	lockComponent();
	
	VisualObjectPtr obj = getMemoryEntry<VisualObject>(_wmc.address);
	
//	for(unsigned i=0; i<obj->model->vertices.size(); i++)
//		printf("    Got object vertices: %4.3f - %4.3f - %4.3f\n", obj->model->vertices[i].pos.x, obj->model->vertices[i].pos.y, obj->model->vertices[i].pos.z);

	ModelEntry newModelEntry;
	
	if(!convertGeometryModel(obj->model, newModelEntry.model)){
		log("  error can not convert VisualObject to virtual scene model");
		return;
	}
	
	// just an example material (should be random to separate objects more easily
	float r = 0.8 * float(rand())/RAND_MAX;
	float g = 0.8 * float(rand())/RAND_MAX;
	float b = 0.8 * float(rand())/RAND_MAX;
	tgRenderModel::Material matSilver; 
	matSilver.ambient = vec4(0.2+r,0.2+g,0.2+b,1.0);
	matSilver.diffuse = vec4(0.2+r,0.2+g,0.2+b,1.0);
	matSilver.specular = vec4(0.5,0.5,0.5,1.0);
	matSilver.shininess = 60.0 * float(rand())/RAND_MAX;
	
	newModelEntry.model.m_material = matSilver;
	convertPose2tgPose(obj->pose, newModelEntry.model.m_pose);
	newModelEntry.obj = obj;
	newModelEntry.castWMA = _wmc.address;
	m_modellist.push_back(newModelEntry);
	
	tgVector3 vCenter = tgVector3(obj->pose.pos.x, obj->pose.pos.y, obj->pose.pos.z);
	m_engine->SetCenterOfRotation(vCenter.x, vCenter.y, vCenter.z);
	
// 	unlockComponent();
	log("VisualObject added to Scene: %s", obj->label.c_str());
}

void VirtualScene::overwriteVisualObject(const cdl::WorkingMemoryChange & _wmc){
	
	VisualObjectPtr obj = getMemoryEntry<VisualObject>(_wmc.address);
	
	for(int i=0; i<m_modellist.size(); i++){
		if(m_modellist[i].castWMA == _wmc.address){
			convertPose2tgPose(obj->pose, m_modellist[i].model.m_pose);
		}
	}
	
	//log("[VirtualScene::changeofVisualObject] WARNING: function not implemented");
}

void VirtualScene::deleteVisualObject(const cdl::WorkingMemoryChange & _wmc){
	vector<ModelEntry>::iterator it;
	for(it=m_modellist.begin(); it<m_modellist.end(); it++){
		if((*it).castWMA == _wmc.address){
			m_modellist.erase(it);
		}
	}
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
        &VirtualScene::addVisualObject));
        
	addChangeFilter(createLocalTypeFilter<VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<VirtualScene>(this,
        &VirtualScene::overwriteVisualObject));
  
  addChangeFilter(createLocalTypeFilter<VisualObject>(cdl::DELETE),
      new MemberFunctionChangeReceiver<VirtualScene>(this,
        &VirtualScene::deleteVisualObject));
}

void VirtualScene::destroy(){
	delete(m_engine);
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
      runScene();
    }else{
			// * Idle *
			sleepComponent(1000);
		}
  }
  
  destroy();
  log("stop");
}

// Tracking
void VirtualScene::initScene(const Video::Image &image){
  
  log("Initialising Scene");
  
  // *** Initialisation of Scene ***
  m_width = image.width;
  m_height = image.height;
  
  m_engine = new(tgEngine);
	m_engine->Init(m_width, m_height, 1.0, "VirtualScene");
	
	loadCameraParameters(&m_camera, image.camPars, 0.1, 5.0);
	m_engine->SetCamera(m_camera);

  log("... initialisation successfull!");
}

void VirtualScene::runScene(){
	for(int i=0; i<m_modellist.size(); i++){
		m_modellist[i].model.DrawFaces();
// 		m_modellist[i].model.DrawNormals(0.01);
	}
	m_running = m_engine->Update(m_fTime);
}



