
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
	m_showCV = true;
	m_confidence = 0.03;
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

  if((it = _config.find("--videoname")) != _config.end()){
    videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())  {
    istringstream str(it->second);
    int id;
    while(str >> id)
      camIds.push_back(id);
  }

  if((it = _config.find("--display")) != _config.end()){
    m_showCV = true;
  }else{
  	m_showCV = false;
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

  addChangeFilter(createLocalTypeFilter<VisionData::DetectionCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectRecognizer3D>(this,
        &ObjectRecognizer3D::receiveDetectionCommand));
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

  		if(m_recCommandList.empty())
  			sleepComponent(500);
  		else{
  			lockComponent();
  			m_rec_cmd = m_recCommandList.front();
  			m_recCommandList.erase(m_recCommandList.begin());
  			m_rec_cmd_id = m_recCommandID.front();
  			m_recCommandID.erase(m_recCommandID.begin());

  			m_task = m_rec_cmd->cmd;
  			m_label = m_rec_cmd->label;


				if(m_rec_cmd->cmd == RECOGNIZE && m_recEntries[m_label].learn){
					log("%s: Warning no Sift file available: starting to learn", m_label.c_str());
					m_rec_cmd->cmd = RECLEARN;
					m_task = RECLEARN;
				}
				if(m_rec_cmd->cmd == RECLEARN && m_rec_cmd->visualObjectID.empty()){
  				log("%s: Warning no VisualObject given", m_label.c_str());
  				Math::Pose3 pose;
  				setIdentity(pose);
					// NOTE: useful values for a user presenting an object to the system:
					// 1.0 meters away and rotated 90 deg around x, so the object stands upright
					// assuming that the objects local z axis points "up"
  				pose.pos.x = 0.0; pose.pos.y = 0.0; pose.pos.z = 1.0;
					fromRotVector(pose.rot, vector3(M_PI/2., 0., 0.));
  				loadVisualModelToWM(m_recEntries[m_label], pose, m_label);
  				m_rec_cmd->visualObjectID =  m_recEntries[m_label].visualObjectID;
  			}
  			// HACK
  			// (this must be one of the ugliest hacks I have ever perpetrated)
  			// if we received a detection command with several labels, we created a corresponding
  			// number of single-label recognition commands in m_recCommandList with identical
  			// IDs in m_recCommandID, which is the ID of the original *detection* command.
  			// Now we only want to delete this detection command from WM (within recognizeSiftModel()) when
  			// the last of these single-label recognition commands was executed, i.e. when the
  			// complete original detection command was executed.
  			// So we remember here whether the next command ID is different or it was the last command ID,
  			// and only then do the delete from WM (in recognizeSiftModel())
				if(m_rec_cmd->cmd == RECOGNIZE)
				{
          if(m_recCommandID.empty())
          {
            m_delete_command_from_wm = true;
          }
          else
          {
            std::string next_cmd_id = m_recCommandID.front();
            if(next_cmd_id != m_rec_cmd_id)
              m_delete_command_from_wm = true;
            else
              m_delete_command_from_wm = false;
          }
				}
  			m_starttask = true;
  			unlockComponent();
  		}

  	}
  }

  // Clean up
  if(m_detect)
		delete(m_detect);

	if(m_showCV)
  	cvDestroyWindow("ObjectRecognizer3D");

  log("stop");
}

void ObjectRecognizer3D::destroy(){

}

// *** Working Memory Listeners ***
void ObjectRecognizer3D::receiveImages(const std::vector<Video::Image>& images){
  if(images.size() == 0)
    throw runtime_error(exceptionMessage(__HERE__, "image list is empty"));
}

/**
 * A slight hack for now: the execution layer sende DetectObject commands, which contain
 * a list of labels. Translate from onw DetectObject command to several
 * Recognizer3DCommands with one label each.
 */
void ObjectRecognizer3D::receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc){
  log("Receiving receiveDetectionCommand");
  DetectionCommandPtr det_cmd = getMemoryEntry<DetectionCommand>(_wmc.address);
  
  for(size_t i = 0; i < det_cmd->labels.size(); i++)
    {
      if(m_recEntries.find(det_cmd->labels[i]) != m_recEntries.end())
	{
	  Recognizer3DCommandPtr rec_cmd = new Recognizer3DCommand();
	  rec_cmd->cmd = RECOGNIZE;
	  rec_cmd->label = det_cmd->labels[i];
	  m_recCommandList.push_back(rec_cmd);
	  m_recCommandID.push_back(_wmc.address.id);
	}
    }
}

void ObjectRecognizer3D::receiveRecognizer3DCommand(const cdl::WorkingMemoryChange & _wmc){
	log("Receiving Recognizer3DCommand");
	Recognizer3DCommandPtr rec_cmd = getMemoryEntry<Recognizer3DCommand>(_wmc.address);

	if(m_recEntries.find(rec_cmd->label) == m_recEntries.end())
		return;

	m_recCommandList.push_back(rec_cmd);
	m_recCommandID.push_back(_wmc.address.id);
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

	if(m_showCV){
		for (unsigned i=0; i<m_temp_keys.Size(); i++){
				m_temp_keys[i]->Draw( m_iplImage,*m_temp_keys[i],CV_RGB(255,0,0) );
		}
		cvShowImage ( "ObjectRecognizer3D", m_iplImage );
	}

	m_wait4data = false;
	log("  New sifts added to model: %d (%d)", m_temp_keys.Size(), m_recEntries[m_label].object->codebook.Size());
}

// *** Tracking Commands ***
void ObjectRecognizer3D::addTrackerCommand(VisionData::TrackingCommandType cmd, std::string& modelID){
	log("Send tracking command: %d '%s'", cmd, modelID.c_str());
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

void ObjectRecognizer3D::loadVisualModelToWM(RecEntry &rec_entry, cogx::Math::Pose3 pose, std::string label){

  bool newModel = rec_entry.visualObjectID.empty();
  VisionData::VisualObjectPtr obj;

  if(newModel){
		// Load geometry
		log("Loading ply model");
		ModelLoader modelloader;
		Model model;
		modelloader.LoadPly(model, rec_entry.plyfile.c_str());

		obj = new VisionData::VisualObject;
		obj->model = new VisionData::GeometryModel;
		convertModel2Geometry(model, obj->model);
		rec_entry.visualObjectID = newDataID();
  }else{
		obj = getMemoryEntry<VisualObject>(rec_entry.visualObjectID);
  }

  // create a very simple distribution: label and unknown
  obj->identLabels.push_back(label);
  obj->identLabels.push_back("unknown");
  // note: distribution must of course sum to 1
  obj->identDistrib.push_back(rec_entry.object->conf);
  obj->identDistrib.push_back(1. - rec_entry.object->conf);
  // the information gain if we know the label, just set to 1, cause we don't
  // have any alternative thing to do
  obj->identGain = 1.;
  // ambiguity in the distribution: we use the distribution's entropy
  obj->identAmbiguity = 0.;
  for(size_t i = 0; i < obj->identDistrib.size(); i++)
    if(fpclassify(obj->identDistrib[i]) != FP_ZERO)
      obj->identAmbiguity -= obj->identDistrib[i]*::log(obj->identDistrib[i]);
  obj->pose = pose;
  obj->componentID = getComponentID();

  if(newModel){
		addToWorkingMemory(rec_entry.visualObjectID, obj);
		addTrackerCommand(ADDMODEL, rec_entry.visualObjectID);
		log("Add model to working memory: '%s' id: %s", obj->identLabels[0].c_str(), rec_entry.visualObjectID.c_str());
	}else{
		overwriteWorkingMemory(rec_entry.visualObjectID, obj);
		addTrackerCommand(OVERWRITE, rec_entry.visualObjectID);
		log("Overwriting VisualObject '%s'", getComponentID().c_str());
	}
}

// *** Recognizer3D functions ***

void ObjectRecognizer3D::init(){

	m_task = RECSTOP;
  m_wait4data = false;
  m_delete_command_from_wm = false;

  if(m_showCV){
		cvNamedWindow("ObjectRecognizer3D", 1 );
		cvWaitKey(10);
	}

	m_detect = new(P::ODetect3D);


	std::map<std::string,RecEntry>::iterator it;
	for(it = m_recEntries.begin(); it!=m_recEntries.end(); it++){
		log("Loading Sift Model '%s'", (*it).second.siftfile.c_str());
		(*it).second.object = new(P::Object3D);
		(*it).second.learn = !sift_model_learner.LoadModel((*it).second.siftfile.c_str(),(*(*it).second.object));
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

  string empty;
  addTrackerCommand(VisionData::START, empty);

  cvReleaseMat(&C);
}

void ObjectRecognizer3D::learnSiftModel(P::DetectGPUSIFT &sift){
	log("Learning sift model '%s': hit space bar", m_rec_cmd->label.c_str());
	VisionData::Vertex vertex;
  VisionData::VertexSeq vertexlist;

  // Start tracking
  addTrackerCommand(VisionData::START, m_rec_cmd->visualObjectID);

  if(m_starttask){
  	addTrackerCommand(VisionData::LOCK, m_rec_cmd->visualObjectID);
  	m_starttask = false;
  }else{
		addTrackerCommand(VisionData::UNLOCK, m_rec_cmd->visualObjectID);
	}

	int key;
	do{
			key = cvWaitKey ( 10 );
	}while( isRunning() && (m_wait4data || ((char)key)!=' ' && ((char)key!='s') && ((char)key!='q')) );

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
			sleepComponent(10);
		}

	}else if((char)key=='s'){

		//	Save sift model
		log("Saving model to file '%s'", m_recEntries[m_label].siftfile.c_str());
		sift_model_learner.SaveModel(m_recEntries[m_label].siftfile.c_str(),(*m_recEntries[m_label].object));

		// Clean up
		addTrackerCommand(VisionData::REMOVEMODEL, m_rec_cmd->visualObjectID);
		cvReleaseImage(&m_iplImage);
		cvReleaseImage(&m_iplGray);
		for (unsigned i=0; i<m_image_keys.Size(); i++)
			delete(m_image_keys[i]);
		m_image_keys.Clear();

		vertexlist.clear();

		// Quit learning
		m_task = RECSTOP;
		log("Sift Model learned");
 	}else if((char)key=='q'){

 		// Clean up
 		addTrackerCommand(VisionData::REMOVEMODEL, m_rec_cmd->visualObjectID);
		cvReleaseImage(&m_iplImage);
		cvReleaseImage(&m_iplGray);
		for (unsigned i=0; i<m_image_keys.Size(); i++)
			delete(m_image_keys[i]);
		m_image_keys.Clear();

		vertexlist.clear();

		// Quit learning
		m_task = RECSTOP;
		log("Learning Sift Model canceled");
 	}
}

void ObjectRecognizer3D::recognizeSiftModel(P::DetectGPUSIFT &sift){
	log("%s: Recognizing model pose", m_label.c_str());

  int key;
  m_rec_cmd->confidence = 0.0;

// 	if(m_starttask){
// 		m_starttask = false;
// 	}else{
// 		addTrackerCommand(REMOVEMODEL, m_recEntries[m_label].visualObjectID);
// 	}

// 	addTrackerCommand(VisionData::LOCK, m_recEntries[m_label].visualObjectID);

	// Grab image from VideoServer
	videoServer->getImage(camId, m_image);
	m_iplImage = convertImageToIpl(m_image);
	m_iplGray = cvCreateImage ( cvGetSize ( m_iplImage ), 8, 1 );
	cvConvertImage( m_iplImage, m_iplGray );

	// Calculate SIFTs from image
	sift.Operate(m_iplGray,m_image_keys);

	//if(m_image_keys.Size() < 10){
	//		log("%s: Too less keypoints detected, no pose estimation possible",m_label.c_str());
	//}else{

	//nick has been hacking here to get VisualObjects to WM even with low confidence

		m_detect->SetDebugImage(m_iplImage);

		if(!m_detect->Detect(m_image_keys, (*m_recEntries[m_label].object))){
			log("%s: No object detected", m_label.c_str());

			//not sure if this is actually necessary, but let's do it anyway
			m_recEntries[m_label].object->conf = 0;
		}
		//else{
			m_rec_cmd->confidence = m_recEntries[m_label].object->conf;
			
			if(m_recEntries[m_label].object->conf < m_confidence){
				log("%s: Confidence of detected object to low: %f<%f", m_label.c_str(), m_recEntries[m_label].object->conf,m_confidence);
				P::SDraw::DrawPoly(m_iplImage, m_recEntries[m_label].object->contour.v, CV_RGB(255,0,0), 2);
				m_detect->DrawInlier(m_iplImage, CV_RGB(255,0,0));
			}
			//nah: sure, it's too low, but we want the objects on WM anyway
			//else{
				P::SDraw::DrawPoly(m_iplImage, m_recEntries[m_label].object->contour.v, CV_RGB(0,255,0), 2);
				m_detect->DrawInlier(m_iplImage, CV_RGB(255,0,0));
				// Transform pose from Camera to world coordinates
				Pose3 P, A, B;
				P = m_image.camPars.pose;
				convertPoseCv2MathPose(m_recEntries[m_label].object->pose, A);

				//printf("Pose: %f %f %f\n", A.pos.x, A.pos.y, A.pos.z);

				Math::transform(P,A,B);
// 				transpose(B.rot, B.rot);

				// if(first time recognition)
				log("%s: Found object at: (%.3f %.3f %.3f), Confidence: %f", m_label.c_str(), B.pos.x, B.pos.y, B.pos.z, m_recEntries[m_label].object->conf);
				loadVisualModelToWM(m_recEntries[m_label], B, m_label);
				m_rec_cmd->visualObjectID = m_recEntries[m_label].visualObjectID;
				//}
			//}
		//}

	if(m_showCV){
		cvShowImage ( "ObjectRecognizer3D", m_iplImage );
		cvWaitKey(50);
	}

// 	addTrackerCommand(VisionData::UNLOCK, m_recEntries[m_label].visualObjectID);

	// Send result to WM
	//overwriteWorkingMemory(m_rec_cmd_id, m_rec_cmd);
	// note: execution layer expects the comand to be deleted as a signal of completion

	try {
	  if(m_delete_command_from_wm) {
      deleteFromWorkingMemory(m_rec_cmd_id);
      m_delete_command_from_wm = false;
	  }
	}
	catch(CASTException &e) {
	  println("exception while deleting command: " + e.message);
	}


	// Clean up
  cvReleaseImage(&m_iplImage);
	cvReleaseImage(&m_iplGray);
	for (unsigned i=0; i<m_image_keys.Size(); i++)
	  delete(m_image_keys[i]);
	m_image_keys.Clear();

	m_task = RECSTOP;
}






