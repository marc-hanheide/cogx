
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "ObjectRecognizer3D.h"
#include <VideoUtils.h>

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
	}else{
		throw runtime_error(exceptionMessage(__HERE__, "No model file given"));
	}
	
	if((it = _config.find("--siftfile")) != _config.end()){
		m_siftfile = it->second;
		log("sift file: '%s'", m_plyfile.c_str());
	}else{
		throw runtime_error(exceptionMessage(__HERE__, "No sift file given"));
	}
  
}

void ObjectRecognizer3D::start()
{
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);
  
  addChangeFilter(createLocalTypeFilter<VisionData::TrackingCommand>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<ObjectRecognizer3D>(this,
        &ObjectRecognizer3D::receiveTrackingCommand));
}

void ObjectRecognizer3D::receiveImages(const std::vector<Video::Image>& images)
{
  if(images.size() == 0)
    throw runtime_error(exceptionMessage(__HERE__, "image list is empty"));
}

void ObjectRecognizer3D::loadModelToWM(std::string filename, std::string& modelID){
	// ***********************************************************
	// Load geometry
 	log("Loading ply model");
	ModelLoader modelloader;
	Model model;
	modelloader.LoadPly(model, filename.c_str());
	
	VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
  obj->model = new VisionData::GeometryModel;
	convertModel2Geometry(model, obj->model);
	obj->label = filename.c_str();
	obj->detectionConfidence = 0.0;
	Tracking::Pose tPose;	
	tPose.translate(0.0,0.0,0.05);
	convertParticle2Pose(tPose, obj->pose); 
	 
  log("Add model to working memory: '%s'", obj->label.c_str());
  modelID = newDataID();
  addToWorkingMemory(modelID, obj);
}

void ObjectRecognizer3D::startTracker(){
	log("Send tracking command: START");
  VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
  track_cmd->cmd = VisionData::START;
  addToWorkingMemory(newDataID(), track_cmd);
}

void ObjectRecognizer3D::addTrackerModel(std::string& modelID){
	log("Send tracking command: ADD");
	VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
  track_cmd->cmd = VisionData::ADDMODEL;
  track_cmd->visualObjectID = modelID;
  addToWorkingMemory(newDataID(), track_cmd);
}

void ObjectRecognizer3D::lockTrackerModel(std::string& modelID){
	log("Send tracking command: LOCK");
  VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
  track_cmd->cmd = VisionData::LOCK;
  track_cmd->visualObjectID = modelID;
  addToWorkingMemory(newDataID(), track_cmd);
}

void ObjectRecognizer3D::unlockTrackerModel(std::string& modelID){
	log("Send tracking command: UNLOCK");
  VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
  track_cmd->cmd = VisionData::UNLOCK;
  track_cmd->visualObjectID = modelID;
  addToWorkingMemory(newDataID(), track_cmd);
}

void ObjectRecognizer3D::get3DPointFromTrackerModel(std::string& modelID, VisionData::VertexSeq& vertexlist){
	log("get3DPointFromTrackerModel");
	VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
	track_cmd->cmd = VisionData::GETPOINT3D;
	track_cmd->visualObjectID = modelID;
	track_cmd->pointOnModel.assign(vertexlist.size(), 0);
	track_cmd->points = vertexlist;
	addToWorkingMemory(newDataID(), track_cmd);
}

void ObjectRecognizer3D::receiveTrackingCommand(const cdl::WorkingMemoryChange & _wmc){
	log("receiveTrackingCommand::OVERWRITE");
	VisionData::TrackingCommandPtr track_cmd = getMemoryEntry<VisionData::TrackingCommand>(_wmc.address);
	bool b;
	if(	track_cmd->points.size() != track_cmd->pointOnModel.size())
		throw runtime_error(exceptionMessage(__HERE__, "List of SIFT features (points) does not match list of validation (pointOnModel)"));
	
	if(	image_keys.Size() != track_cmd->points.size() )
		throw runtime_error(exceptionMessage(__HERE__, "List of image_keys does not match list of points returned by tracker"));

	temp_keys.Clear();
	for (unsigned i=0; i<image_keys.Size(); i++)
	{
		if (track_cmd->pointOnModel[i])
		{
			image_keys[i]->pos.create(3);
			image_keys[i]->pos(1) = track_cmd->points[i].pos.x;
			image_keys[i]->pos(2) = track_cmd->points[i].pos.y;
			image_keys[i]->pos(3) = track_cmd->points[i].pos.z;
			temp_keys.PushBack(image_keys[i]);
		}
// 		b = track_cmd->pointOnModel[i];
// 		printf(	"%f %f %f, %d\n", 
// 						track_cmd->points[i].pos.x,
// 						track_cmd->points[i].pos.y,
// 						track_cmd->points[i].pos.z,
// 						b);
	}
	sift_model_learner.AddToModel(temp_keys,object);
	log("Add new sifts to model: %d (%d)", temp_keys.Size(), object.codebook.Size());
}


void ObjectRecognizer3D::runComponent()
{
  
	P::DetectGPUSIFT 	sift;
  P::ODetect3D			detect;
  IplImage *iplImage;
  IplImage *iplGray;
  VisionData::Vertex vertex;
  VisionData::VertexSeq vertexlist;

  sleepProcess(1000);  // HACK
  
  loadModelToWM(m_plyfile, m_modelID);
  addTrackerModel(m_modelID);
  startTracker();
  
  cvNamedWindow ("ObjectRecognizer3D", 1 );
  
  videoServer->getImage(camId, m_image);
  iplImage = convertImageToIpl(m_image);
  iplGray = cvCreateImage ( cvGetSize ( iplImage ), 8, 1 );
  cvConvertImage( iplImage, iplGray );
   
 	int key;
 	do{
 	  
 	  sleepProcess(1000);
 	  unlockTrackerModel(m_modelID);
 	  
		do{
				key = cvWaitKey ( 10 );
		}while (((char)key)!=' ' && ((char)key!='s'));
		
		// Lock model
		lockTrackerModel(m_modelID);
		
		// Grab image from VideoServer
 		videoServer->getImage(camId, m_image);
		iplImage = convertImageToIpl(m_image);
		iplGray = cvCreateImage ( cvGetSize ( iplImage ), 8, 1 );
		cvConvertImage( iplImage, iplGray );
		
		// Calculate SIFTs from image
		sift.Operate(iplGray,image_keys);
		
		// Convert 2D image points to 3D model points
		vertexlist.clear();
		for(unsigned i=0; i<image_keys.Size(); i++ ){
		  vertex.texCoord.x = image_keys[i]->p.x;
			vertex.texCoord.y = image_keys[i]->p.y;
			vertexlist.push_back(vertex);
		}
		
		get3DPointFromTrackerModel(m_modelID, vertexlist);

		for (unsigned i=0; i<image_keys.Size(); i++){
				image_keys[i]->Draw( iplImage,*image_keys[i],CV_RGB(0,0,255) );
		}
		cvShowImage ( "ObjectRecognizer3D", iplImage );
  }while(((char)key!='s'));
  
  
  // ***********************************************************************
  // Quit Learning
  sleepProcess(1000);
  
  // Save model_keys
  log("Saving model to file");
  sift_model_learner.SaveModel(m_siftfile.c_str(),object);
    
  cvReleaseImage(&iplImage);
	cvReleaseImage(&iplGray);
	for (unsigned i=0; i<image_keys.Size(); i++)
	  delete(image_keys[i]);
	image_keys.Clear();
 
 	log("Destroying cv Window");
	cvDestroyWindow("ObjectRecognizer3D");
	    
  
//   std::string compID = getComponentID();
//   videoServer->startReceiveImages(compID.c_str(), camIds, 0, 0);
//   sleepProcess(1000);
//   videoServer->stopReceiveImages(compID);

 

  
}



