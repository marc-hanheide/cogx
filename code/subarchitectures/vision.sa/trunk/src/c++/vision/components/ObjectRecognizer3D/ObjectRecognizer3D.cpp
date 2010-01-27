
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "ObjectRecognizer3D.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ObjectRecognizer3D();
  }
}

using namespace Tracking;
using namespace cast;
using namespace std;

void ObjectRecognizer3D::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
  
  if((it = _config.find("--plyfile")) != _config.end()){
		m_plyfile = it->second;
		log("ply file: '%s'", m_plyfile.c_str());
	}
  
}

void ObjectRecognizer3D::start()
{
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<ObjectRecognizer3D>(this,
        &ObjectRecognizer3D::receiveVisualObject));
}

void ObjectRecognizer3D::runComponent()
{
  // ***********************************************************
  // Load geometry and start tracker
 	log("loading ply model");
	ModelLoader modelloader;
	Model model;
	modelloader.LoadPly(model, m_plyfile.c_str());
	
	VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
  obj->model = new VisionData::GeometryModel;
	convertModel2Geometry(model, obj->model);
	obj->label = "Testobject";
	obj->detectionConfidence = 0.0;
	Tracking::Pose tPose;	
	tPose.translate(0.0,0.0,0.05);
	convertParticle2Pose(tPose, obj->pose); 
	 
  log("add model to working memory: '%s'", obj->label.c_str());
  addToWorkingMemory(newDataID(), obj);
  
  sleepProcess(1000);
  
  log("send tracking command: START");
  VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
  track_cmd->cmd = VisionData::START;
  addToWorkingMemory(newDataID(), track_cmd);
  
  sleepProcess(1000);
  
  // ***********************************************************
  // now start learning SIFT features
  
  
}

void ObjectRecognizer3D::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
//   VisionData::VisualObjectPtr obj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
}



