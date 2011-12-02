
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "ObjectRecognizer3D.h"
#include <VideoUtils.h>
#include <VisionUtils.h>
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
	m_confidence = 0.08;
	m_simulationOnly=false;

	// initial pose for tracking when learning a new object:
  // useful values for a user presenting an object to the system:
  // 1.0 meters away and rotated 90 deg around x, so the object stands upright
  // assuming that the objects local z axis points "up"
  // can be overwritten in configure
  setIdentity(initPose);
  initPose.pos.x = 0.0;
  initPose.pos.y = 0.0;
  initPose.pos.z = 1.0;
  fromRotVector(initPose.rot, vector3(M_PI/2., 0., 0.));
	m_noLearning=false;
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

  if((it = _config.find("--pcserver")) != _config.end()){
    pcServerName = it->second;
    PointCloudClient::configureServerCommunication(_config);
  }

  if((it = _config.find("--simulation-only")) != _config.end()){
    m_simulationOnly=true;
  }

  if((it = _config.find("--no-learning")) != _config.end()){
    m_noLearning=true;
  }

  if((it = _config.find("--camid")) != _config.end())  {
    istringstream str(it->second);
    str >> camId;
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

	if((it = _config.find("--initpose")) != _config.end()){
    istringstream str(it->second);
    str >> initPose;
	}
  if((it = _config.find("--initpose_xml")) != _config.end())
  {
    string filename = it->second;
    readXML(filename, initPose);
  }

	std::string label, plystr, siftstr;

	while(labeliss >> label && plyiss >> plystr && siftiss >> siftstr){
		m_recEntries[label].plyfile = plystr;
		m_recEntries[label].siftfile = siftstr;
	}

#ifdef FEAT_VISUALIZATION	
  m_display.configureDisplayClient(_config);
#endif
}

void ObjectRecognizer3D::start(){
  
  if (!m_simulationOnly) {
    // get connection to the video server
	if (videoServerName.length() >0 )
		videoServer = getIceServer<Video::VideoInterface>(videoServerName);

	if (pcServerName.length() > 0)
		PointCloudClient::startPCCServerCommunication(*this);
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
  addChangeFilter(createGlobalTypeFilter<VisionData::Post3DObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectRecognizer3D>(this,
	&ObjectRecognizer3D::PostFake3DObject));


  // init recognizer, phase 1
  initInStart();

#ifdef FEAT_VISUALIZATION        
  if (!m_simulationOnly) {
    m_display.connectIceClient(*this);
    m_display.setClientData(this);
    m_display.installEventReceiver();
    for(map<string,RecEntry>::iterator it = m_recEntries.begin(); it != m_recEntries.end(); it++)
    {
      ostringstream id, label;
      id << "button." << it->first;
      label << "Recognize " << it->first;
      m_display.addButton(getComponentID(), id.str(), label.str());
    }
  }
#endif
}

#ifdef FEAT_VISUALIZATION
void ObjectRecognizer3D::CDisplayClient::handleEvent(const Visualization::TEvent &event)
{
  if(event.type == Visualization::evButtonClick)
  {
    for(map<string,RecEntry>::iterator it = pRec->m_recEntries.begin(); it != pRec->m_recEntries.end(); it++)
    {
      ostringstream id;
      id << "button." << it->first;
      if(event.sourceId == id.str())
      {
        Recognizer3DCommandPtr rec_cmd = new Recognizer3DCommand;
        rec_cmd->cmd = RECOGNIZE;
        rec_cmd->label = it->first;
        rec_cmd->visualObjectID = "";
        pRec->addToWorkingMemory(pRec->newDataID(), rec_cmd);
      }
    }
  }
}
#endif

void ObjectRecognizer3D::PostFake3DObject(const cdl::WorkingMemoryChange & _wmc) {
	log("Fake 3D Object Received");
	Post3DObjectPtr f = getMemoryEntry<Post3DObject> (_wmc.address);

	if (f->positiveDetection) {
		m_recEntries[f->label].object->conf=0.99;
		loadVisualModelToWM(m_recEntries[f->label], f->pose, f->label, false);
	} else {
		Pose3 nullPose;
		m_recEntries[f->label].object->conf=0.0;
		setIdentity(nullPose);
		loadVisualModelToWM(m_recEntries[f->label], nullPose, f->label, false);
	}
}


void ObjectRecognizer3D::runComponent(){
  if (m_simulationOnly)
    return;
  P::DetectGPUSIFT 	sift;

  sleepProcess(1000);  // HACK

  // init recognizer, phase 2
  initInRun();

  // Running Loop
  while(isRunning()){

    if(m_task == RECLEARN){
  		learnSiftModel(sift);   // m_task = STOP called in receiveTrackingCommand()

  	}else if(m_task == RECOGNIZE){
  		recognizeSiftModel(sift);

  	}else if(m_task == RECSTOP){
  		m_starttask = false;

  		if (m_recCommandList.empty())
				sleepComponent(500);
			else {
				lockComponent();
				m_rec_cmd = m_recCommandList.front();
				m_recCommandList.erase(m_recCommandList.begin());
				m_rec_cmd_id = m_recCommandID.front();
				m_recCommandID.erase(m_recCommandID.begin());

				m_task = m_rec_cmd->cmd;
				m_label = m_rec_cmd->label;

				if (m_rec_cmd->cmd == RECOGNIZE) {

					if (m_recEntries.find(m_label) != m_recEntries.end()
							&& m_recEntries[m_label].learn) {
						if (m_noLearning) {
							println("%s: couldn't find a SIFT file and learning is disabled, so we ignore this one", m_label.c_str());
							m_rec_cmd->cmd=RECSTOP;
							m_task=RECSTOP;
						} else {
							log(
									"%s: Warning no Sift file available: starting to learn",
									m_label.c_str());
							m_rec_cmd->cmd = RECLEARN;
							m_task = RECLEARN;
						}
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
					if (m_recCommandID.empty()) {
						m_delete_command_from_wm = true;
					} else {
						std::string next_cmd_id = m_recCommandID.front();
						if (next_cmd_id != m_rec_cmd_id)
							m_delete_command_from_wm = true;
						else
							m_delete_command_from_wm = false;
					}
				} else if (m_rec_cmd->cmd == RECLEARN) {
					if (m_rec_cmd->visualObjectID.empty()) {
						log("%s: Warning no VisualObject given",
								m_label.c_str());
						Math::Pose3 pose;
						setIdentity(pose);
						// NOTE: useful values for a user presenting an object to the system:
						// 1.0 meters away and rotated 90 deg around x, so the object stands upright
						// assuming that the objects local z axis points "up"
						pose.pos.x = 0.0;
						pose.pos.y = 0.0;
						pose.pos.z = 1.0;
						fromRotVector(pose.rot, vector3(M_PI / 2., 0., 0.));
						loadVisualModelToWM(m_recEntries[m_label], pose,
								m_label);
						m_rec_cmd->visualObjectID
								= m_recEntries[m_label].visualObjectID;
					}
				}
				m_starttask = true;
				unlockComponent();
			}

  	}
    sleepComponent(500);  
  }

  // Clean up
  if(m_detect)
		delete(m_detect);

	if(m_showCV)
  	cvDestroyWindow(getComponentID().c_str());

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
  log("Receiving DetectionCommand");
  try {
  DetectionCommandPtr det_cmd = getMemoryEntry<DetectionCommand>(_wmc.address);
  
	// CASTComponent::Lock lock(this);
  for(size_t i = 0; i < det_cmd->labels.size(); i++)
  {
    Recognizer3DCommandPtr rec_cmd = new Recognizer3DCommand();
    rec_cmd->cmd = RECOGNIZE;
    rec_cmd->label = det_cmd->labels[i];
    m_recCommandList.push_back(rec_cmd);
    // note: here we add (multiple times) the WM Id of the detection command
    m_recCommandID.push_back(_wmc.address.id);
  }
  } catch (const DoesNotExistOnWMException& e) {
	  getLogger()->warn("detection command has already disappeared. not detecting.");
  }
}

// @author: mmarko
void ObjectRecognizer3D::receiveRecognitionCommand(const cdl::WorkingMemoryChange & _wmc){
	log("Receiving RecognitionCommand");
	try {
		RecognitionCommandPtr det_cmd = getMemoryEntry<RecognitionCommand>(_wmc.address);

		// When there are no labels in the list, try to recognize all known labels.
		vector<string> labels;
		if (det_cmd->labels.size() > 0)
			labels = det_cmd->labels;
		else {
			map<string,RecEntry>::iterator it;
			for(it = m_recEntries.begin(); it != m_recEntries.end(); it++) {
				labels.push_back(it->first);
			}
		}

		//CASTComponent::Lock lock(this);
		for(size_t i = 0; i < labels.size(); i++)
		{
			Recognizer3DCommandPtr rec_cmd = new Recognizer3DCommand();
			rec_cmd->cmd = RECOGNIZE;
			rec_cmd->label = labels[i];
			rec_cmd->visualObjectID = det_cmd->visualObject->address.id;
			m_recCommandList.push_back(rec_cmd);
			// note: here we add (multiple times) the WM Id of the detection command
			m_recCommandID.push_back(_wmc.address.id);
		}
	} catch (const DoesNotExistOnWMException& e) {
		getLogger()->warn("recognition command has already disappeared. not detecting.");
	}
}

void ObjectRecognizer3D::receiveRecognizer3DCommand(const cdl::WorkingMemoryChange & _wmc){
	log("Receiving Recognizer3DCommand");
	Recognizer3DCommandPtr rec_cmd = getMemoryEntry<Recognizer3DCommand>(_wmc.address);

	log("ID is %s", rec_cmd->visualObjectID.c_str());
	// CASTComponent::Lock lock(this);
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

#ifdef FEAT_VISUALIZATION
  m_display.setImage(getComponentID(), m_iplImage);
#endif

	if(m_showCV){
		for (unsigned i=0; i<m_temp_keys.Size(); i++){
				m_temp_keys[i]->Draw( m_iplImage,*m_temp_keys[i],CV_RGB(255,0,0) );
		}
		cvShowImage ( getComponentID().c_str(), m_iplImage );
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

/**
 * load a recogizsed object to WM.
 * @param rec_entry recognition entry with confidence etc.
 * @param pose detected object pose
 * @param label the objects label
 * @param forceNewObject if true, we will always add a new object to WM.
 *        otherwise we only add a new model if rec_entry does not yet
 *        have an object id
 */
void ObjectRecognizer3D::loadVisualModelToWM(RecEntry &rec_entry,
  cogx::Math::Pose3 &pose, std::string &label, bool forceNewObject){

  bool newModel = forceNewObject || rec_entry.visualObjectID.empty();
  VisionData::VisualObjectPtr obj;

  if(newModel){
    // Load geometry
    ModelLoader modelloader;
    Model model;
    modelloader.LoadPly(model, rec_entry.plyfile.c_str());

    obj = cogx::createVisualObject();
    obj->model = new VisionData::GeometryModel;
    convertModel2Geometry(model, obj->model);
    rec_entry.visualObjectID = newDataID();
  }else{
    obj = getMemoryEntry<VisualObject>(rec_entry.visualObjectID);
    // these arrays are modified and need to be cleared first
    obj->identLabels.clear();
    obj->identDistrib.clear();
  }

  // create a very simple distribution: label and unknown
  obj->identLabels.push_back(label);
  obj->identLabels.push_back("unknown");
  // note: distribution must of course sum to 1
  if(forceNewObject)
    rec_entry.object->conf = 0.9;
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
  obj->affordance="";
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

/**
 * Load an empty (i.e. actually not detected) visual model to WM. It has a label,
 * identity pose and confidence 0.
 * @return the WM ID of the newly created object
 */
std::string ObjectRecognizer3D::loadEmptyVisualModelToWM(std::string &label){

  log("creating empty object %s", label.c_str());
  VisionData::VisualObjectPtr obj = cogx::createVisualObject();

  // create a very simple distribution: label and unknown
  obj->identLabels.push_back(label);
  obj->identLabels.push_back("unknown");
  // note: distribution must of course sum to 1
  obj->identDistrib.push_back(0.);
  obj->identDistrib.push_back(1.);
  // the information gain if we know the label, just set to 1, cause we don't
  // have any alternative thing to do
  obj->identGain = 1.;
  // ambiguity in the distribution: we use the distribution's entropy
  obj->identAmbiguity = 0.;
  for(size_t i = 0; i < obj->identDistrib.size(); i++)
    if(fpclassify(obj->identDistrib[i]) != FP_ZERO)
      obj->identAmbiguity -= obj->identDistrib[i]*::log(obj->identDistrib[i]);
  setIdentity(obj->pose);
  obj->componentID = getComponentID();

  std::string newObjID = newDataID();
  addToWorkingMemory(newObjID, obj);
  // do not add a command to track it (as would be the case for a properly detected
  // visual object)
  log("Add model to working memory: '%s' id: %s", obj->identLabels[0].c_str(), newObjID.c_str());
  return newObjID;
}


/**
 * Allocate the recignizer and load models, set some state variables.
 * To be called from start()
 */
void ObjectRecognizer3D::initInStart(){

  m_task = RECSTOP;
  m_wait4data = false;
  m_delete_command_from_wm = false;

  if(m_showCV){
    cvNamedWindow(getComponentID().c_str(), 1 );
    cvWaitKey(10);
  }

  log("loading models ...");
  std::map<std::string,RecEntry>::iterator it;
  for(it = m_recEntries.begin(); it!=m_recEntries.end(); it++){
    log("Loading Sift Model '%s'", (*it).second.siftfile.c_str());
    (*it).second.object = new(P::Object3D);
    (*it).second.learn = !sift_model_learner.LoadModel((*it).second.siftfile.c_str(),(*(*it).second.object));
  }
  log("... done loading models");
}

/**
 * Set camera parameters for the recognizer. This needs an alive video server
 * hence must be done in runComponent()
 */
void ObjectRecognizer3D::initInRun(){

  m_detect = new(P::ODetect3D);

  getImage();
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

  // string empty;
  // addTrackerCommand(VisionData::START, empty);

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
    key = cvWaitKey ( 100 );
  }while( isRunning() && (m_wait4data || ((char)key)!=' ' && ((char)key!='s') && ((char)key!='q')) );

  if((char)key==' '){

    // 	Lock model
    addTrackerCommand(VisionData::LOCK, m_rec_cmd->visualObjectID);

    // 	Grab image from VideoServer
    getImage();
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
      sleepComponent(100);
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

void ObjectRecognizer3D::getImage()
{
	if (pcServerName.length()>0) {
		getRectImage(camId, 640, m_image);
		log("got image from Kinect: " + m_image.height);
	}
	else
		videoServer->getImage(camId, m_image);
}

void ObjectRecognizer3D::recognizeSiftModel(P::DetectGPUSIFT &sift){

  // if we don't know that label we want to produce a non-detected
  // VisualObject
  if(m_recEntries.find(m_label) == m_recEntries.end())
  {
    log("%s: Unknown model", m_label.c_str());
    std::string newID = loadEmptyVisualModelToWM(m_label);
    m_rec_cmd->confidence = 0.;
    m_rec_cmd->visualObjectID = newID;
  }
  else
  {
    log("%s: Detecting object", m_label.c_str());
    // 	if(m_starttask){
    // 		m_starttask = false;
    // 	}else{
    // 		addTrackerCommand(REMOVEMODEL, m_recEntries[m_label].visualObjectID);
    // 	}

    // 	addTrackerCommand(VisionData::LOCK, m_recEntries[m_label].visualObjectID);

    // Grab image from VideoServer
      getImage();
      m_iplImage = convertImageToIpl(m_image);
      m_iplGray = cvCreateImage(cvGetSize(m_iplImage), 8, 1);
      cvConvertImage(m_iplImage, m_iplGray);
      // Calculate SIFTs from image
      sift.Operate(m_iplGray, m_image_keys);
      m_detect->SetDebugImage(m_iplImage);
      if(!m_detect->Detect(m_image_keys, (*m_recEntries[m_label].object)))
    {
      log("Failed to find object %s", m_label.c_str());
      Math::Pose3 nonPose;
      setIdentity(nonPose);
      loadVisualModelToWM(m_recEntries[m_label], nonPose, m_label);
      m_rec_cmd->confidence = m_recEntries[m_label].object->conf;
      m_rec_cmd->visualObjectID = m_recEntries[m_label].visualObjectID;
    }
    else
    {
      // Transform pose from Camera to world coordinates
      Pose3 P, A, B;
      P = m_image.camPars.pose;
      convertPoseCv2MathPose(m_recEntries[m_label].object->pose, A);
      Math::transform(P, A, B);

      if(m_recEntries[m_label].object->conf < m_confidence)
      {
        log("Found object %s with below threshold conf %f at:\n%s\n",
            m_label.c_str(), m_recEntries[m_label].object->conf, toString(B).c_str());
        // set confidence to 0 to indicate that we consider the object not detected
        // NOTE: this threshold is stupid. let the caller decide, what to do with the result.
        // m_recEntries[m_label].object->conf = 0.;
        P::SDraw::DrawPoly(m_iplImage, m_recEntries[m_label].object->contour.v, CV_RGB(255,0,0), 2);
        m_detect->DrawInlier(m_iplImage, CV_RGB(255,0,0));
      }
      else
      {
        log("Found object %s with conf %f at:\n%s\n",
            m_label.c_str(), m_recEntries[m_label].object->conf, toString(B).c_str());
        P::SDraw::DrawPoly(m_iplImage, m_recEntries[m_label].object->contour.v, CV_RGB(0,255,0), 2);
        m_detect->DrawInlier(m_iplImage, CV_RGB(255,0,0));
      }

      // if(first time recognition)
      loadVisualModelToWM(m_recEntries[m_label], B, m_label);
      m_rec_cmd->confidence = m_recEntries[m_label].object->conf;
      m_rec_cmd->visualObjectID = m_recEntries[m_label].visualObjectID;
    }

    if(m_showCV){
      cvShowImage(getComponentID().c_str(), m_iplImage);
      cvWaitKey(50);
    }

    cvReleaseImage(&m_iplImage);
    cvReleaseImage(&m_iplGray);
    for (unsigned i=0; i<m_image_keys.Size(); i++)
      delete(m_image_keys[i]);
    m_image_keys.Clear();
  }

  // 	addTrackerCommand(VisionData::UNLOCK, m_recEntries[m_label].visualObjectID);

  // Send result to WM
  overwriteWorkingMemory(m_rec_cmd_id, m_rec_cmd);

  m_task = RECSTOP;
}






