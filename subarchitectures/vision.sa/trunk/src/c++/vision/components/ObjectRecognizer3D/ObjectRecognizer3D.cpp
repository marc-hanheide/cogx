
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
using namespace VisionData;
using namespace cast;
using namespace cogx;
using namespace Math;
using namespace std;

ObjectRecognizer3D::ObjectRecognizer3D(){
	camId = 0;
	m_detect = 0;
}

ObjectRecognizer3D::~ObjectRecognizer3D(){
	if(m_detect)
		delete(m_detect);
}

void ObjectRecognizer3D::configure(const map<string,string> & _config){
  map<string,string>::const_iterator it;
  log("Configure:");
  
  istringstream plyiss;
  istringstream siftiss;
  istringstream labeliss;
  
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
  
  if((it = _config.find("--labels")) != _config.end()){
		labeliss.str(it->second);
	}else{
		throw runtime_error(exceptionMessage(__HERE__, "No labels given"));
	}
	
	if((it = _config.find("--siftfiles")) != _config.end()){
		siftiss.str(it->second);
	}
	
	if((it = _config.find("--plyfiles")) != _config.end()){
		plyiss.str(it->second);
	}

	std::string label, plystr, siftstr;
	
	while(labeliss >> label && plyiss >> plystr && siftiss >> siftstr){
		m_recEntries[label].plyfile = plystr;
		m_recEntries[label].siftfile = siftstr;
	}
}

void ObjectRecognizer3D::start(){
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);
  
  addChangeFilter(createLocalTypeFilter<VisionData::TrackingCommand>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<ObjectRecognizer3D>(this,
        &ObjectRecognizer3D::receiveTrackingCommand));
  
  addChangeFilter(createLocalTypeFilter<VisionData::Recognizer3DCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectRecognizer3D>(this,
        &ObjectRecognizer3D::receiveRecognizer3DCommand));
}

void ObjectRecognizer3D::runComponent(){
  
  P::DetectGPUSIFT 	sift;
	
  sleepProcess(1000);  // HACK
  
  // Initialisation
  init();
  
  // Running Loop
  while(isRunning()){
    
    if(m_task == RECLEARN){
  		learnSiftModel(sift);   // m_task = STOP called in receiveTrackingCommand()
  			
  	}else if(m_task == RECOGNIZE){
  		recognizeSiftModel(sift);
  		
  	}else if(m_task == RECSTOP){
  		m_starttask = false;
  		lockComponent();
  		if(m_recCommandList.empty())
  			sleepComponent(500);
  		else{
  			m_rec_cmd = m_recCommandList.front();
  			m_task = m_rec_cmd->cmd;
  			m_label = m_rec_cmd->label;
  			if(m_rec_cmd->visualObjectID.empty()){
  				log("Warning no VisualObject given");
  				loadVisualModelToWM(m_recEntries[m_label].plyfile, m_recEntries[m_label].visualObjectID, Math::Pose3());
  			}
  			m_recCommandList.erase(m_recCommandList.begin());
  			m_starttask = true;
  		}
			unlockComponent();
  	}  
  }
  
  // Clean up
  if(m_detect)
		delete(m_detect);
		
  cvDestroyWindow("ObjectRecognizer3D");
  log("stop");  
}

void ObjectRecognizer3D::destroy(){
	log("ObjectRecognizer3D::destroy()");

// 	
// 	std::map<std::string,RecEntry>::iterator it;
// 	for(it = m_recEntries.begin(); it!=m_recEntries.end(); it++){
// 		if((*it).second.object)
// 			delete((*it).second.object);
// 	}
}

// *** Working Memory Listeners ***
void ObjectRecognizer3D::receiveImages(const std::vector<Video::Image>& images){
  if(images.size() == 0)
    throw runtime_error(exceptionMessage(__HERE__, "image list is empty"));
}

void ObjectRecognizer3D::receiveRecognizer3DCommand(const cdl::WorkingMemoryChange & _wmc){
	log("Receiving Recognizer3DCommand");
	Recognizer3DCommandPtr rec_cmd = getMemoryEntry<Recognizer3DCommand>(_wmc.address);
	
	if(m_recEntries.find(rec_cmd->label)==m_recEntries.end())
		return;
	
	m_recCommandList.push_back(rec_cmd);
}

void ObjectRecognizer3D::receiveTrackingCommand(const cdl::WorkingMemoryChange & _wmc){
	log("Receive TrackingCommand::OVERWRITE");
	VisionData::TrackingCommandPtr track_cmd = getMemoryEntry<VisionData::TrackingCommand>(_wmc.address);
	bool b;
	if(	track_cmd->points.size() != track_cmd->pointOnModel.size())
		throw runtime_error(exceptionMessage(__HERE__, "List of SIFT features (points) does not match list of validation (pointOnModel)"));
	
	if(	m_image_keys.Size() != track_cmd->points.size() )
		throw runtime_error(exceptionMessage(__HERE__, "List of m_image_keys does not match list of points returned by tracker"));
		
	if( m_image_keys.Size() == 0 )
		return;
	
	m_temp_keys.Clear();
	for (unsigned i=0; i<m_image_keys.Size(); i++)
	{
		if (track_cmd->pointOnModel[i])
		{
			m_image_keys[i]->SetPos(track_cmd->points[i].pos.x,track_cmd->points[i].pos.y,track_cmd->points[i].pos.z);
			m_temp_keys.PushBack(m_image_keys[i]);
// 			printf("%f %f %f\n", m_image_keys[i]->pos->data.fl[0], m_image_keys[i]->pos->data.fl[1], m_image_keys[i]->pos->data.fl[2]);
// 			b = track_cmd->pointOnModel[i];
// 			printf(	"%f %f %f, %d\n", 
// 							track_cmd->points[i].pos.x,
// 							track_cmd->points[i].pos.y,
// 							track_cmd->points[i].pos.z,
// 							b);
		}

	}
	sift_model_learner.AddToModel(m_temp_keys, (*m_recEntries[m_label].object));
	
	for (unsigned i=0; i<m_temp_keys.Size(); i++){
			m_temp_keys[i]->Draw( m_iplImage,*m_temp_keys[i],CV_RGB(255,0,0) );
	}
	
	m_wait4data = false;
	
	cvShowImage ( "ObjectRecognizer3D", m_iplImage );
	log("  New sifts added to model: %d (%d)", m_temp_keys.Size(), m_recEntries[m_label].object->codebook.Size());
}

// *** Tracking Commands ***
void ObjectRecognizer3D::addTrackerCommand(VisionData::TrackingCommandType cmd, std::string& modelID){
	log("Send tracking command: %d", cmd);
	VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
  track_cmd->cmd = cmd;
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
	
	lockComponent();
	m_wait4data = true;
	unlockComponent();
	
	addToWorkingMemory(newDataID(), track_cmd);
}

void ObjectRecognizer3D::loadVisualModelToWM(std::string filename, std::string& modelID, cogx::Math::Pose3 pose){
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
	obj->pose = pose;
// 	Tracking::Pose tPose;	
// 	tPose.translate(0.0,0.0,0.05);
// 	convertParticle2Pose(tPose, obj->pose); 
	 
  log("Add model to working memory: '%s'", obj->label.c_str());
  modelID = newDataID();
  addToWorkingMemory(modelID, obj);
}

// *** Recognizer3D functions ***

void ObjectRecognizer3D::init(){
	
	m_task = RECSTOP;
  m_wait4data = false;
	cvNamedWindow("ObjectRecognizer3D", 1 );
	
	m_detect = new(P::ODetect3D);
	
	log("Loading Sift Model");
	std::map<std::string,RecEntry>::iterator it;
	for(it = m_recEntries.begin(); it!=m_recEntries.end(); it++){
		(*it).second.object = new(P::Object3D);
		sift_model_learner.LoadModel((*it).second.siftfile.c_str(),(*(*it).second.object));
	}
	
  videoServer->getImage(camId, m_image);
  m_iplImage = convertImageToIpl(m_image);
  m_iplGray = cvCreateImage ( cvGetSize ( m_iplImage ), 8, 1 );
  cvConvertImage( m_iplImage, m_iplGray );
  
  double fx, fy, cx, cy;
  fx = m_image.camPars.fx;
  fy = m_image.camPars.fy;
  cx = m_image.camPars.cx;
  cy = m_image.camPars.cy;
  cout << fx << ' ' << fy << ' ' <<  cx << ' ' <<  cy << endl;
  
  CvMat *C = cvCreateMat(3,3, CV_32F);
  cvmSet(C, 0, 0, fx);
  cvmSet(C, 0, 1, 0.);
  cvmSet(C, 0, 2, cx);
  cvmSet(C, 1, 0, 0.);
  cvmSet(C, 1, 1, fy);
  cvmSet(C, 1, 2, cy);
  cvmSet(C, 2, 0, 0.);
  cvmSet(C, 2, 1, 0.);
  cvmSet(C, 2, 2, 1.);
  
  m_detect->SetCameraParameter(C);
  
  cvReleaseMat(&C);
}

void ObjectRecognizer3D::learnSiftModel(P::DetectGPUSIFT &sift){
	log("Learning sift model: hit space bar");
	VisionData::Vertex vertex;
  VisionData::VertexSeq vertexlist;
  
  // Start tracking
  addTrackerCommand(VisionData::START, m_rec_cmd->visualObjectID);
  
  if(m_starttask){
  	addTrackerCommand(VisionData::ADDMODEL, m_rec_cmd->visualObjectID);
  	m_starttask = false;
  }
	
	addTrackerCommand(VisionData::UNLOCK, m_rec_cmd->visualObjectID);
	
	int key;
	do{
			key = cvWaitKey ( 10 );
	}while( isRunning() && (m_wait4data || ((char)key)!=' ' && ((char)key!='s')) );
	
	if((char)key==' '){
		// 	Lock model
		addTrackerCommand(VisionData::LOCK, m_rec_cmd->visualObjectID);
		
		// 	Grab image from VideoServer
		videoServer->getImage(camId, m_image);
		m_iplImage = convertImageToIpl(m_image);
		m_iplGray = cvCreateImage ( cvGetSize ( m_iplImage ), 8, 1 );
		cvConvertImage( m_iplImage, m_iplGray );
		
		// 	Calculate SIFTs from image
		sift.Operate(m_iplGray,m_image_keys);

		// 	Convert 2D image points to 3D model points
		vertexlist.clear();
		for(unsigned i=0; i<m_image_keys.Size(); i++ ){
			vertex.texCoord.x = m_image_keys[i]->p.x;
			vertex.texCoord.y = m_image_keys[i]->p.y;
			vertexlist.push_back(vertex);
		}
		
		get3DPointFromTrackerModel(m_rec_cmd->visualObjectID, vertexlist);
		
		while( isRunning() && m_wait4data ){
			sleepComponent(20);
		}
		
	}else if((char)key=='s'){

		//	Save sift model
		log("Saving model to file '%s'", m_recEntries[m_label].siftfile.c_str());
		sift_model_learner.SaveModel(m_recEntries[m_label].siftfile.c_str(),(*m_recEntries[m_label].object));
		
		// Clean up
		cvReleaseImage(&m_iplImage);
		cvReleaseImage(&m_iplGray);
		for (unsigned i=0; i<m_image_keys.Size(); i++)
			delete(m_image_keys[i]);
		m_image_keys.Clear();
		
		vertexlist.clear();
	
		// Quit learning
		m_task = RECSTOP;
		log("Sift Model learned");
 	}
}

void ObjectRecognizer3D::recognizeSiftModel(P::DetectGPUSIFT &sift){
	log("Recognizing model pose");

  int key;

	if(m_starttask){
		addTrackerCommand(VisionData::START, m_rec_cmd->visualObjectID);
		m_starttask = false;
	}else{
		addTrackerCommand(REMOVEMODEL, m_recEntries[m_label].visualObjectID);
	}
	
	// Grab image from VideoServer
	videoServer->getImage(camId, m_image);
	m_iplImage = convertImageToIpl(m_image);
	m_iplGray = cvCreateImage ( cvGetSize ( m_iplImage ), 8, 1 );
	cvConvertImage( m_iplImage, m_iplGray );
	
	// Calculate SIFTs from image
	sift.Operate(m_iplGray,m_image_keys);
	
	m_detect->SetDebugImage(m_iplImage);
	if(m_detect->Detect(m_image_keys, (*m_recEntries[m_label].object))){
		P::SDraw::DrawPoly(m_iplImage, m_recEntries[m_label].object->contour.v, CV_RGB(0,255,0), 2);
	}
	
	// Transform pose from Camera to world coordinates
	Pose3 P, A, B;
	P = m_image.camPars.pose;
	convertPoseCv2MathPose(m_recEntries[m_label].object->pose, A);
	Math::transform(P,A,B);
	transpose(B.rot, B.rot);
	
	// if(first time recognition)
	loadVisualModelToWM(m_recEntries[m_label].plyfile,  m_recEntries[m_label].visualObjectID, B);
	addTrackerCommand(ADDMODEL, m_recEntries[m_label].visualObjectID);

	cvShowImage ( "ObjectRecognizer3D", m_iplImage );
 
	// Clean up   
  cvReleaseImage(&m_iplImage);
	cvReleaseImage(&m_iplGray);
	for (unsigned i=0; i<m_image_keys.Size(); i++)
	  delete(m_image_keys[i]);
	m_image_keys.Clear();

	m_task = RECSTOP;
	log("Object recognized");
}






