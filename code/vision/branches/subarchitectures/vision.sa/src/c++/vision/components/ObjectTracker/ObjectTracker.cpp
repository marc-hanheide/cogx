/**
 * @author Thomas Mörwald
 * @date April 2009
 */

#include "ObjectTracker.h"
#include <VideoUtils.h>
#include <ChangeFilterFactory.hpp>

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

namespace cast
{

using namespace std;
using namespace VisionData;


ObjectTracker::ObjectTracker(){
  camId = 0;
  m_model = 0;
  track = false;
}

ObjectTracker::~ObjectTracker(){
  delete(g_Resources);
}

// *** Working Memory Listeners ***

void ObjectTracker::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc){
	VisualObjectPtr obj = getMemoryEntry<VisualObject>(_wmc.address);
	log("adding VisualObject '%s'", obj->label.c_str());
	/*
	// Convert VisualObject::GeometryModel to ObjectTracker::ModelData
	int i,j;
	ModelData* md = new ModelData();
	md->num_vertices = obj->model->vertices.size();
	md->num_faces = obj->model->faces.size();
	md->m_vertexlist = (ModelData::Vertex*)malloc(sizeof(ModelData::Vertex) * md->num_vertices);
	md->m_facelist = (ModelData::Face*)malloc(sizeof(ModelData::Face) * md->num_faces);
	for(i=0; i<md->num_vertices; i++){
		md->m_vertexlist[i].x = obj->model->vertices[i].pos.x;
		md->m_vertexlist[i].y = obj->model->vertices[i].pos.y;
		md->m_vertexlist[i].z = obj->model->vertices[i].pos.z;
		md->m_vertexlist[i].nx = obj->model->vertices[i].normal.x;
		md->m_vertexlist[i].ny = obj->model->vertices[i].normal.y;
		md->m_vertexlist[i].nz = obj->model->vertices[i].normal.z;
		md->m_vertexlist[i].s = obj->model->vertices[i].texCoord.x;
		md->m_vertexlist[i].t = obj->model->vertices[i].texCoord.y;
	}
	for(i=0; i<md->num_faces; i++){
		md->m_facelist[i].nverts = obj->model->faces[i].vertices.size();
		for(j=0; j<md->m_facelist[i].nverts; j++)
			md->m_facelist[i].v[j] = obj->model->faces[i].vertices[j];		
	}
	
	// Generate new Model using ModelData
	Model* model = new Model();
	model->load(*md);
	delete(md);
	model->computeEdges();
	model->computeNormals();
	
	// Save ID of working memory and resource manager
	IDList ids;
	ids.resources_ID = g_Resources->AddModel(model, obj->label.c_str());
	istringstream istr(_wmc.address.id);
	istr >> ids.cast_ID;
	m_modellist.push_back(ids);
	*/
}

void ObjectTracker::receiveTrackingCommand(const cdl::WorkingMemoryChange & _wmc){
	TrackingCommandPtr track_cmd = getMemoryEntry<TrackingCommand>(_wmc.address);
	
	log("received tracking command ...");
	switch(track_cmd->cmd){
		case VisionData::START:
			if(track){
				log("allready started tracking");
			}else{
				log("starting tracking");
				track = true;
			}
			break;
		case VisionData::STOP:
			if(track){
				log("stopping tracking");
				track = false;
			}else{
				log("allready stopped tracking");
			}
			break;
		case VisionData::RELEASEMODELS:
			log("releasing all models");
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
  Tracker tracker;
  Video::Image image;
  IplImage* cvImage;
  Model* model;
  Particle m_result = Particle(0.0);
  
  // Set pathes of resource manager
  g_Resources->SetModelPath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/model/");
  g_Resources->SetTexturePath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/texture/");
  g_Resources->SetShaderPath("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/shader/");

  // Grab one image from video server for initialisation
  getImage(camId, image);
  cvImage = convertImageToIpl(image);
    
  // Initialize SDL screen
  g_Resources->InitScreen(cvImage->width, cvImage->height);
 
  // Initialize tracking (parameters for edge-based tracking)
  if(!tracker.init(	cvImage->width, cvImage->height,
					1000,
					49.0,
					0.25, 0.165, 0.25,
					0.0, 40.0,
					0.0, 0.1,
					17.0))
	log("Initialisation failed!");
	
  // Load model with resource manager
  int id;
  if((id = g_Resources->AddModel("box_blender.ply")) == -1)
	log("failed loading model!");
  m_model = g_Resources->GetModel(id);
  
  cvReleaseImage(&cvImage);
  
  float fTimeImage;
  float fTimeTracker;
  
  while(isRunning())
  {
  	if(track){
  	  m_timer.Update();
  	  
  	  getImage(camId, image);
  	  cvImage = convertImageToIpl(image);
  	  cvConvertImage(cvImage, cvImage, CV_CVTIMG_FLIP);
  	  fTimeImage = m_timer.Update();
	  
	  tracker.trackEdge((unsigned char*)cvImage->imageData, m_model, &m_result, &m_result);
	  glFlush();
	  fTimeTracker = m_timer.Update();
	  
	  //printf("TimeImage:   %.0f ms\n", fTimeImage*1000.0);
	  //printf("TimeTracker: %.0f ms\n\n", fTimeTracker*1000.0);
	  
	  cvReleaseImage(&cvImage);
	  sleepComponent(10);
	}else{
      sleepComponent(1000);
	}
  }
 
  // Releasing Resources here cause ~ObjectTracker() not called when [STRG+C]
  tracker.release();
  delete(g_Resources);
}

}

