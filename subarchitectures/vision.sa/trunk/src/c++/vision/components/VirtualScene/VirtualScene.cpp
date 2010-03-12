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
	m_lock = false;
	m_objectType = 0;
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
	
	if(!convertGeometry2Model(obj->model, newModelEntry.model)){
		log("  error can not convert VisualObject to virtual scene model");
		return;
	}
	
	// just an example material (should be random to separate objects more easily
	newModelEntry.model.m_material = getRandomMaterial();
	convertPose2tgPose(obj->pose, newModelEntry.model.m_pose);
// 	newModelEntry.obj = obj;
	newModelEntry.castWMA = _wmc.address;
	m_VisualObjectList.push_back(newModelEntry);
	
	tgVector3 vCenter = tgVector3(obj->pose.pos.x, obj->pose.pos.y, obj->pose.pos.z);
	m_engine->SetCenterOfRotation(vCenter.x, vCenter.y, vCenter.z);
	
// 	unlockComponent();
	log("VisualObject added to Scene: %s - %s", obj->label.c_str(), _wmc.address.id.c_str());
}

void VirtualScene::overwriteVisualObject(const cdl::WorkingMemoryChange & _wmc){
	
	VisualObjectPtr obj = getMemoryEntry<VisualObject>(_wmc.address);
	
	for(int i=0; i<m_VisualObjectList.size(); i++){
		if(m_VisualObjectList[i].castWMA == _wmc.address){
			convertPose2tgPose(obj->pose, m_VisualObjectList[i].model.m_pose);
		}
	}
	
	//log("[VirtualScene::changeofVisualObject] WARNING: function not implemented");
}

void VirtualScene::deleteVisualObject(const cdl::WorkingMemoryChange & _wmc){
	vector<ModelEntry>::iterator it;
	for(it=m_VisualObjectList.begin(); it<m_VisualObjectList.end(); it++){
		if((*it).castWMA == _wmc.address){
			m_VisualObjectList.erase(it);
		}
	}
}

void VirtualScene::addConvexHull(const cdl::WorkingMemoryChange & _wmc){
	if(!m_lock){
	
	
		log("receiving ConvexHull: %s", _wmc.address.id.c_str());
		ConvexHullPtr obj = getMemoryEntry<ConvexHull>(_wmc.address);
		
		ModelEntry newModelEntry;
		
		if(!convertConvexHull2Model(obj, newModelEntry.model)){
			log("  error can not convert ConvexHull to virtual scene model");
			return;
		}
		
// 		newModelEntry.obj = obj;
		newModelEntry.model.m_material = getRandomMaterial();
		newModelEntry.castWMA = _wmc.address;
		
		m_ConvexHullList.push_back(newModelEntry);
		
		m_lock = true;
	}
}

void VirtualScene::overwriteConvexHull(const cdl::WorkingMemoryChange & _wmc){

}

void VirtualScene::deleteConvexHull(const cdl::WorkingMemoryChange & _wmc){

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

	// ******************************************************************
	// Visual Objects
  addChangeFilter(createLocalTypeFilter<VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<VirtualScene>(this,
        &VirtualScene::addVisualObject));
        
	addChangeFilter(createLocalTypeFilter<VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<VirtualScene>(this,
        &VirtualScene::overwriteVisualObject));
  
  addChangeFilter(createLocalTypeFilter<VisualObject>(cdl::DELETE),
      new MemberFunctionChangeReceiver<VirtualScene>(this,
        &VirtualScene::deleteVisualObject));
  
  // ******************************************************************
	// Convex Hulls
	addChangeFilter(createLocalTypeFilter<ConvexHull>(cdl::ADD),
      new MemberFunctionChangeReceiver<VirtualScene>(this,
        &VirtualScene::addConvexHull));

// 	addChangeFilter(createLocalTypeFilter<ConvexHull>(cdl::OVERWRITE),
//       new MemberFunctionChangeReceiver<VirtualScene>(this,
//         &VirtualScene::overwriteVisualObject));
//   
//   addChangeFilter(createLocalTypeFilter<ConvexHull>(cdl::DELETE),
//       new MemberFunctionChangeReceiver<VirtualScene>(this,
//         &VirtualScene::deleteVisualObject));
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
  
  while(m_running && isRunning())
  {
    if(m_render){
    	switch(m_objectType){
      	case 1:
      		drawVisualObjects();
      		break;
      	case 2:
      		drawConvexHulls();
      		break;
      	case 3:
      		break;
      	default:
      		drawVisualObjects();
      		drawConvexHulls();
      		break;
      }
      m_engine->Activate2D();
			m_font->Print("VirtualSzene", 16, 5, 5);
      m_running = m_engine->Update(m_fTime, m_eventlist);
      m_wireframe = m_engine->GetWireframeMode();
      sleepComponent(5);
    }else{
			// * Idle *
			sleepComponent(1000);
		}
		inputControl();
  }
  
  delete(m_font);
  delete(m_engine);
  log("stop");
}

// *** GL functions ***
void VirtualScene::initScene(const Video::Image &image){
  
  log("Initialising Scene");
  
  // *** Initialisation of Scene ***
  m_width = image.width;
  m_height = image.height;
  
  m_engine = new(tgEngine);
	m_engine->Init(m_width, m_height, 1.0, "VirtualScene");
	
	loadCameraParameters(&m_camera, image.camPars, 0.1, 5.0);
	m_engine->SetCamera(m_camera);
	
	m_font = new tgFont("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf");

  log("... initialisation successfull!");
}

void VirtualScene::drawCamera(){
// 	glMatrixMode(GL_MODELVIEW);
// 	glPushMatrix();
// 		glMultMatrix(m_camera.GetExtrinsic());
// 		
}

void VirtualScene::drawVisualObjects(){
	for(int i=0; i<m_VisualObjectList.size(); i++){
		m_VisualObjectList[i].model.DrawFaces(m_wireframe);
// 		m_VisualObjectList[i].model.DrawNormals(0.01);
	}
}

void VirtualScene::drawConvexHulls(){
	for(int i=0; i<m_ConvexHullList.size(); i++){
		glDisable(GL_LIGHTING);
		glPointSize(2);
		m_ConvexHullList[i].model.ApplyColor();
		m_ConvexHullList[i].model.DrawPoints();
		glPointSize(1);
		
		m_ConvexHullList[i].model.DrawPolygons();
// 		m_VisualObjectList[i].model.DrawNormals(0.01);
	}
}

void VirtualScene::inputControl(){
	tgEvent event;
	for(int i=0; i<m_eventlist.size(); i++){
		event = m_eventlist[i];
		switch(event.type){
		
			// *********************************************************
			case KeyPress:
				switch(event.key.keysym){
					case XK_1:
						log("[1] Showing 'VisualObject'");
						m_objectType = 1;
						break;
					case XK_2:
						log("[2] Showing 'ConvexHull'");
						m_objectType = 2;
						break;
					case XK_3:
						log("[3] Showing 'na'");
						m_objectType = 3;
						break;
					case XK_0:
						log("[0] Showing all");
						m_objectType = 0;
						break;
					default:
						break;
				}
			default:
				break;
		}
	}
	m_eventlist.clear();
}

TomGine::vec3 VirtualScene::getRandomColor(){
	TomGine::vec3 col;
	col.x = float(rand())/RAND_MAX;
	col.y = float(rand())/RAND_MAX;
	col.z = float(rand())/RAND_MAX;
	return col;
}

TomGine::tgRenderModel::Material VirtualScene::getRandomMaterial(){
	TomGine::vec3 c;
	tgRenderModel::Material material; 
	material.color = c = getRandomColor();
	material.ambient = vec4(c.x,c.y,c.z,1.0) * 0.5;
	material.diffuse = vec4(0.2,0.2,0.2,1.0) + vec4(c.x,c.y,c.z,1.0) * 0.8;
	material.specular = vec4(0.5,0.5,0.5,1.0);
	material.shininess = 50.0 * float(rand())/RAND_MAX;
	return material;
}

