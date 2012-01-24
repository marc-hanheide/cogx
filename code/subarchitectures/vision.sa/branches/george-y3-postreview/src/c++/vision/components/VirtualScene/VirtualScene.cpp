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
	m_labels = false;
	m_normals = false;
	m_drawcamera = false;
	m_objectType = 0;
	m_cor_num = 0;
}

VirtualScene::~VirtualScene(){
	vector<ModelEntry*>::iterator it;

	for(it=m_VisualObjectList.begin(); it<m_VisualObjectList.end(); it++)
		delete(*it);

}

// *** Working Memory Listeners ***
void VirtualScene::addVisualObject(const cdl::WorkingMemoryChange & _wmc){
	VirtualSceneChange vsc;
	vsc.wmc = _wmc;
	vsc.cmd = ADDVISUALOBJECT;
	m_vsc.push_back(vsc);
}

void VirtualScene::overwriteVisualObject(const cdl::WorkingMemoryChange & _wmc){
	VirtualSceneChange vsc;
	vsc.wmc = _wmc;
	vsc.cmd = OVERWRITEVISUALOBJECT;
	m_vsc.push_back(vsc);
}

void VirtualScene::deleteVisualObject(const cdl::WorkingMemoryChange & _wmc){
	VirtualSceneChange vsc;
	vsc.wmc = _wmc;
	vsc.cmd = DELETEVISUALOBJECT;
	m_vsc.push_back(vsc);
}

void VirtualScene::addConvexHull(const cdl::WorkingMemoryChange & _wmc){
	VirtualSceneChange vsc;
	vsc.wmc = _wmc;
	vsc.cmd = ADDCONVEXHULL;
	m_vsc.push_back(vsc);
}

void VirtualScene::overwriteConvexHull(const cdl::WorkingMemoryChange & _wmc){
	VirtualSceneChange vsc;
	vsc.wmc = _wmc;
	vsc.cmd = OVERWRITECONVEXHULL;
	m_vsc.push_back(vsc);
}

void VirtualScene::deleteConvexHull(const cdl::WorkingMemoryChange & _wmc){

}

void VirtualScene::addSOI(const cdl::WorkingMemoryChange & _wmc){
	VirtualSceneChange vsc;
	vsc.wmc = _wmc;
	vsc.cmd = ADDSOI;
	m_vsc.push_back(vsc);
}

void VirtualScene::overwriteSOI(const cdl::WorkingMemoryChange & _wmc){
	VirtualSceneChange vsc;
	vsc.wmc = _wmc;
	vsc.cmd = OVERWRITESOI;
	m_vsc.push_back(vsc);
}

void VirtualScene::deleteSOI(const cdl::WorkingMemoryChange & _wmc){
	VirtualSceneChange vsc;
	vsc.wmc = _wmc;
	vsc.cmd = DELETESOI;
	m_vsc.push_back(vsc);
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

  if((it = _config.find("--labels")) != _config.end())
  {
    m_labels = true;
  }

  if((it = _config.find("--drawcamera")) != _config.end())
  {
    m_drawcamera = true;
  }

  if((it = _config.find("--normals")) != _config.end())
  {
    m_normals = true;
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

  // ******************************************************************
	// SOIs
	addChangeFilter(createLocalTypeFilter<SOI>(cdl::ADD),
      new MemberFunctionChangeReceiver<VirtualScene>(this,
        &VirtualScene::addSOI));
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

    	while(m_vsc.size() > 0)
    		applyVirtualSceneCmd();

    	updateGL();

    	m_engine->Activate3D();
    	if(m_drawcamera) drawCamera();
    	switch(m_objectType){
      	case 1:
      		drawVisualObjects();
      		break;
      	case 2:
      		drawConvexHulls();
      		break;
      	case 3:
      		drawSOIs();
      		break;
      	default:
      		drawVisualObjects();
      		drawConvexHulls();
      		// Blending objects at the end
      		drawSOIs();
      		break;
      }



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
	m_engine->Init(m_width, m_height, 10.0, 0.01, "VirtualScene");

	m_engine->GetCamera0(m_camera0);
	loadCameraParameters(&m_camera0, image.camPars, 0.1, 10.0);
	m_camera = m_camera0;
	m_engine->SetCamera(m_camera);
	m_engine->UpdateCameraViews(m_camera);

	tgModelLoader loader;
	loader.LoadPly(m_camModel.model, "instantiations/ply-models/logitech.ply");
// 	m_camModel.model.m_pose = image.camPars.pose;
	cogx::Math::Pose3 pose = image.camPars.pose;
	convertPose2tgPose( pose, m_camModel.model.m_pose);
// 	m_camModel.model.m_material = getRandomMaterial();

	m_fontfilename = string("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf");
	m_font = new tgFont(m_fontfilename.c_str());

  log("... initialisation successfull!");
}

void VirtualScene::updateCameraViews(){
	TomGine::tgVector3 cor_cam;
	TomGine::tgVector3 new_cam_pos;
	tgCamera cam = m_camera0;

	cor_cam = TomGine::tgVector3(m_camera0.GetPos().x-m_coo.x, m_camera0.GetPos().y-m_coo.y, m_camera0.GetPos().z-m_coo.z);
	m_cor.x = 0.5*(m_camera0.GetPos().x+m_coo.x);
	m_cor.y = 0.5*(m_camera0.GetPos().y+m_coo.y);
	m_cor.z = 0.5*(m_camera0.GetPos().z+m_coo.z);
	m_engine->SetCenterOfRotation(m_cor.x, m_cor.y, m_cor.z);

	float l = cor_cam.length();
	new_cam_pos = m_camera0.GetPos() - m_camera0.GetF() * l;
	cam.SetPos( new_cam_pos.x, new_cam_pos.y, new_cam_pos.z);
	cam.ApplyTransform();

	m_engine->UpdateCameraViews(cam);
}

void VirtualScene::updateGL(){

	m_engine->Activate2D();

	// Print FPS
	char text[32];
	sprintf(text,"FPS: %.1f", 1.0 / m_timer.Update());
	m_font->Print(text, 16, 5, 5);

	// Query events from engine
	m_running = m_engine->Update(m_fTime, m_eventlist);

	// Get drawing mode from engine
	m_wireframe = m_engine->GetWireframeMode();

	sleepComponent(5);
}

void VirtualScene::applyVirtualSceneCmd(){
	vector<ModelEntry*>::iterator it;
	VirtualSceneChange vsc = m_vsc.front();
	m_vsc.erase(m_vsc.begin());

	// ADD VISUAL OBJECT
	if(vsc.cmd == ADDVISUALOBJECT){
		log("ADDVISUALOBJECT: %s", vsc.wmc.address.id.c_str());
		VisualObjectPtr obj = getMemoryEntry<VisualObject>(vsc.wmc.address);
		ModelEntry* newModelEntry = new ModelEntry(m_fontfilename.c_str());
		if(!convertGeometry2Model(obj->model, newModelEntry->model)){
			log("ADDVISUALOBJECT: error can not convert VisualObject to virtual scene model");
			return;
		}
		newModelEntry->model.m_material = getRandomMaterial();
		convertPose2tgPose(obj->pose, newModelEntry->model.m_pose);
		newModelEntry->castWMA = vsc.wmc.address;
		if(m_labels) newModelEntry->label.AddText(!obj->identLabels.empty() ? obj->identLabels[0].c_str() : "have-no-label", 30);
		if(m_labels) newModelEntry->label.SetPose(newModelEntry->model.m_pose);
		m_VisualObjectList.push_back(newModelEntry);
// 		addVectorToCenterOfRotation(m_coo, m_cor_num, obj->pose.pos);				/// HACK @ ARI
// 		updateCameraViews();									/// HACK @ ARI
		log("ADDVISUALOBJECT: ok %s", !obj->identLabels.empty() ? obj->identLabels[0].c_str() : "have-no-label");

	// OVERWRITE VISUAL OBJECT
	}else if(vsc.cmd == OVERWRITEVISUALOBJECT){
		VisualObjectPtr obj = getMemoryEntry<VisualObject>(vsc.wmc.address);
		for(int i=0; i<m_VisualObjectList.size(); i++){
			if(m_VisualObjectList[i]->castWMA == vsc.wmc.address){
				convertPose2tgPose(obj->pose, m_VisualObjectList[i]->model.m_pose);
				if(m_labels) m_VisualObjectList[i]->label.SetPose(m_VisualObjectList[i]->model.m_pose);
			}
		}

	// DELETE VISUAL OBJECT
	}else if(vsc.cmd == DELETEVISUALOBJECT){
		for(it=m_VisualObjectList.begin(); it<m_VisualObjectList.end(); it++){
			if((*it)->castWMA == vsc.wmc.address){
				delete(*it);
				m_VisualObjectList.erase(it);
			}
		}


	// ADD CONVEX HULL
	}else if(vsc.cmd == ADDCONVEXHULL){
		log("ADDCONVEXHULL: %s", vsc.wmc.address.id.c_str());
		ConvexHullPtr obj = getMemoryEntry<ConvexHull>(vsc.wmc.address);
		ModelEntry* newModelEntry = new ModelEntry(m_fontfilename.c_str());
		// Convert plane to geometry
		if(!convertConvexHullPlane2Model(obj, newModelEntry->model)){
			log("ADDCONVEXHULL: error can not convert ConvexHullPlane to virtual scene model");
			return;
		}
		// Convert each object to geometry
		for(int i=0; i<obj->Objects.size(); i++){

			if(!convertConvexHullObj2Model(obj->Objects[i], newModelEntry->model)){
				log("ADDCONVEXHULL: error can not convert ConvexHullObject to virtual scene model");
				return;
			}
		}
		newModelEntry->model.m_material = getRandomMaterial();
		newModelEntry->castWMA = vsc.wmc.address;
		m_ConvexHullList.push_back(newModelEntry);
		addVectorToCenterOfRotation(m_coo, m_cor_num, obj->center.pos);
		updateCameraViews();
		log("ADDCONVEXHULL: ok");

	// OVERWRITE CONVEX HULL
	}else if(vsc.cmd == OVERWRITECONVEXHULL){
		log("OVERWRITECONVEXHULL: not implemented yet!");

	// DELETE CONVEX HULL
	}else if(vsc.cmd == DELETECONVEXHULL){
		for(it=m_ConvexHullList.begin(); it<m_ConvexHullList.end(); it++){
			if((*it)->castWMA == vsc.wmc.address){
				delete(*it);
				m_ConvexHullList.erase(it);
			}
		}

	// ADD SOI
	}else if(vsc.cmd == ADDSOI){
		log("ADDSOI: %s", vsc.wmc.address.id.c_str());
		SOIPtr soi = getMemoryEntry<SOI>(vsc.wmc.address);
		cogx::Math::Pose3 pose;

		ModelEntry* newModelEntry = new ModelEntry(m_fontfilename.c_str());
		if(!convertSOI2Model(soi, newModelEntry->model, pose.pos)){
			log("ADDSOI: error can not convert ConvexHullObject to virtual scene model");
			return;
		}
		cogx::Math::setIdentity(pose.rot);
		convertPose2tgPose(pose, newModelEntry->model.m_pose);
		newModelEntry->model.m_material = getRandomMaterial(0.5);
		newModelEntry->castWMA = vsc.wmc.address;
		m_SOIList.push_back(newModelEntry);
		addVectorToCenterOfRotation(m_coo, m_cor_num, pose.pos);
		log("ADDSOI: ok");

	// OVERWRITE SOI
	}else if(vsc.cmd == OVERWRITESOI){
		log("OVERWRITESOI: not implemented yet!");

	// DELETE SOI
	}else if(vsc.cmd == DELETESOI){
		for(it=m_SOIList.begin(); it<m_SOIList.end(); it++){
			if((*it)->castWMA == vsc.wmc.address){
				delete(*it);
				m_SOIList.erase(it);
			}
		}
	}



// // void VirtualScene::overwriteSOI(const cdl::WorkingMemoryChange & _wmc){
//
//
//
// // void VirtualScene::deleteSOI(const cdl::WorkingMemoryChange & _wmc){
// 	vector<ModelEntry>::iterator it;


}

// *** Draw functions ***
void VirtualScene::drawCamera(){
	m_camModel.model.DrawFaces(!m_wireframe);
	m_camModel.model.m_pose.Activate();
	glColor3f(0.5,0.5,0.5);
	m_camera0.DrawFrustum();
	m_camModel.model.m_pose.Deactivate();

	// Draw center of rotation
	glPointSize(2);
	glColor3f(1.0,0.0,0.0);
	glBegin(GL_POINTS);
		glVertex3f(m_cor.x, m_cor.y, m_cor.z);
	glEnd();
}

void VirtualScene::drawVisualObjects(){
	for(int i=0; i<m_VisualObjectList.size(); i++){
		m_VisualObjectList[i]->model.DrawFaces(!m_wireframe);
		if(m_normals) m_VisualObjectList[i]->model.DrawNormals(0.01);
		if(m_labels) m_VisualObjectList[i]->label.Draw();
	}
}

void VirtualScene::drawConvexHulls(){
	for(int i=0; i<m_ConvexHullList.size(); i++){
		m_ConvexHullList[i]->model.DrawFaces(!m_wireframe);
		m_ConvexHullList[i]->model.DrawQuadstrips();
		if(m_normals) m_ConvexHullList[i]->model.DrawNormals(0.01);
	}
}

void VirtualScene::drawSOIs(){
	glEnable(GL_BLEND);
	glEnable(GL_CULL_FACE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	for(int i=0; i<m_SOIList.size(); i++){
		m_SOIList[i]->model.DrawFaces();
		if(m_normals) m_SOIList[i]->model.DrawNormals(0.01);
	}
	glDisable(GL_CULL_FACE);
	glDisable(GL_BLEND);
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
						log("[3] Showing 'SOI'");
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

TomGine::vec4 VirtualScene::getRandomColor(float alpha){
	TomGine::vec4 col;
	col.x = float(rand())/RAND_MAX;
	col.y = float(rand())/RAND_MAX;
	col.z = float(rand())/RAND_MAX;
	col.w = alpha;
	return col;
}

TomGine::tgRenderModel::Material VirtualScene::getRandomMaterial(float alpha){
	TomGine::vec4 c;
	tgRenderModel::Material material;
	material.color = c = getRandomColor(alpha);
	material.ambient = vec4(c.x,c.y,c.z,alpha) * 0.6;
	material.diffuse = vec4(0.2,0.2,0.2,alpha) + vec4(c.x,c.y,c.z,0.0) * 0.8;
	material.specular = vec4(0.3,0.3,0.3,alpha);
	material.shininess = 20.0 * float(rand())/RAND_MAX;
	return material;
}

