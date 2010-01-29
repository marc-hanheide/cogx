
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

using namespace TomGine;
using namespace Tracking;
using namespace cast;
using namespace std;

void ObjectRecognizer3D::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
 
  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream str(it->second);
    int id;
    while(str >> id)
      camIds.push_back(id);
  }
  
  if((it = _config.find("--plyfile")) != _config.end()){
		m_plyfile = it->second;
		log("ply file: '%s'", m_plyfile.c_str());
	}
  
}

void ObjectRecognizer3D::start()
{
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);
  
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<ObjectRecognizer3D>(this,
        &ObjectRecognizer3D::receiveVisualObject));
}

void ObjectRecognizer3D::receiveImages(const std::vector<Video::Image>& images)
{
  if(images.size() == 0)
    throw runtime_error(exceptionMessage(__HERE__, "image list is empty"));
}

void ObjectRecognizer3D::initSzene(const Video::Image &image){

  // *** Initialisation of Scene ***
  m_width = image.width;
  m_height = image.height;
  
  log("Initialising Scene 1");
//   m_engine = new tgEngine();
//   log("Initialising Scene 2");
	m_engine.Init(m_width, m_height, 1.0, "VirtualScene");
// 	
// 	log("Loading Camera Parameters");
	loadCameraParameters(&m_camera, image.camPars, 0.1, 5.0);
	m_engine.SetCamera(m_camera);

  log("... initialisation successfull!");
}

void ObjectRecognizer3D::runComponent()
{
  sleepProcess(1000);  // HACK
  
  videoServer->getImage(camId, m_image);
  
  initSzene(m_image);
  
   while(m_engine.Update(m_fTime)){
 
	 }
	    
  // ***********************************************************
  // Load geometry and start tracker
//  	log("loading ply model");
// 	ModelLoader modelloader;
// 	Model model;
// 	modelloader.LoadPly(model, m_plyfile.c_str());
// 	
// 	VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
//   obj->model = new VisionData::GeometryModel;
// 	convertModel2Geometry(model, obj->model);
// 	obj->label = "Testobject";
// 	obj->detectionConfidence = 0.0;
// 	Tracking::Pose tPose;	
// 	tPose.translate(0.0,0.0,0.05);
// 	convertParticle2Pose(tPose, obj->pose); 
// 	 
//   log("add model to working memory: '%s'", obj->label.c_str());
//   addToWorkingMemory(newDataID(), obj);
//   
//   sleepProcess(1000);
//   
//   log("send tracking command: START");
//   VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
//   track_cmd->cmd = VisionData::START;
//   addToWorkingMemory(newDataID(), track_cmd);
//   
//   sleepProcess(1000);
  
//   std::string compID = getComponentID();
//   videoServer->startReceiveImages(compID.c_str(), camIds, 0, 0);
//   sleepProcess(1000);
//   videoServer->stopReceiveImages(compID);
  
  // ***********************************************************
  // now start learning SIFT features
 

 
 
  
}

void ObjectRecognizer3D::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
//   VisionData::VisualObjectPtr obj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
}



