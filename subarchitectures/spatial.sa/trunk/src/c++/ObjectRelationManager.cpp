//
// = FILENAME
//    ObjectRelationManager.cpp
//
// = FUNCTION
//    Evaluate spatial relations between objects and between objects and places,
//    and write appropriate Property structs to Spatial WM
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#include "ObjectRelationManager.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <VisionData.hpp>
#include <AddressBank/ConfigFileReader.hh>
#include <SensorData/SensorPose.hh>
#include <VideoUtils.h>
#include <SpatialData.hpp>
#include <Navigation/LocalGridMap.hh>
#include "CureMapConversion.hpp"
#include "DensitySampling.hpp"
#include "SpatialGridMap.hh"
#include "PBVisualization.hh"
#include "GridMapData.hh"
#include <sstream>

#define USE_KDE

#define ASSERT_TYPE(x) {if (x->type != OBJECT_PLANE && x->type != OBJECT_BOX && x->type != OBJECT_HOLLOW_BOX) {cerr << "Type assert on " << __LINE__ << endl;}}

using namespace cast;
#include <Pose3.h>

using namespace std;
using namespace cast;
using namespace boost;
using namespace spatial;
using namespace cogx::Math;
/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new ObjectRelationManager();
  }
}


ObjectRelationManager::ObjectRelationManager() : m_sampler(&m_evaluator)
{
  m_bRecognitionIssuedThisStop = false;
  m_maxObjectCounter = 0;
  m_standingStillThreshold = 0.2;
  m_recognitionTimeThreshold = DBL_MAX;
  m_trackerTimeThreshold = 1.0;
  m_timeSinceLastMoved = 0.0;
}

ObjectRelationManager::~ObjectRelationManager() 
{ 
  for (map<string, spatial::Object*>::iterator it = m_objectModels.begin();
      it != m_objectModels.end(); it++) {
    delete it->second;
  }
}



void ObjectRelationManager::configure(const map<string,string>& _config) 
{
  map<string,string>::const_iterator it = 
    _config.find("--display-planes-in-pb");
  m_bDisplayPlaneObjectsInPB = false;
  if (it != _config.end()) {
    m_bDisplayPlaneObjectsInPB = true;
  }

  it = _config.find("--display-objects-in-pb");
  m_bDisplayVisualObjectsInPB = false;
  if (it != _config.end()) {
    m_bDisplayVisualObjectsInPB = true;
  }

  m_bNoPTZ = false;
  it = _config.find("--no-ptz");
  if (it != _config.end()) {
    m_bNoPTZ = true;
  }

  m_bNoVision = false;
  it = _config.find("--no-vision");
  if (it != _config.end()) {
    m_bNoVision = true;
  }

  m_planeModelFilename = "";
  it = _config.find("--plane-model-file");
  if (it != _config.end()) {
    m_planeModelFilename = it->second;
  }

  it = _config.find("--orientations");
  if (it != _config.end()) {
    m_sampler.setOrientationQuantization(atoi(it->second.c_str()));
  }

  it = _config.find("--samples");
  if (it != _config.end()) {
    m_sampler.setSampleNumberTarget(atoi(it->second.c_str()));
  }

  it = _config.find("--kernel-width-factor");
  if (it != _config.end()) {
    m_sampler.setKernelWidthFactor(atoi(it->second.c_str()));
  }

  istringstream labeliss;
  if((it = _config.find("--look-for-objects")) != _config.end()){
		labeliss.str(it->second);
	std::string label, plystr, siftstr;
	
	while(labeliss >> label){
	  m_lookForObjects.insert(label);
	}
  }

  m_RetryDelay = 10;

  Cure::ConfigFileReader *cfg = 0;
  it = _config.find("-c");

  if (it != _config.end()) {
    cfg = new Cure::ConfigFileReader;
    log("About to try to open the config file");
    if (cfg->init(it->second) != 0) {
      delete cfg;
      cfg = 0;
      log("Could not init Cure::ConfigFileReader with -c argument");
    } else {
      log("Managed to open the Cure config file");

      if (!m_bNoPTZ) {
	Cure::SensorPose CameraPoseR;
	if (cfg->getSensorPose(2, CameraPoseR)) {
	  println("configure(...) Failed to get sensor pose for camera. (Run with --no-planes to skip)");
	  m_bNoPTZ = true;
	  std::abort();
	} 
	double xytheta[3];
	CameraPoseR.getXYTheta(xytheta);
	fromRotZ(m_CameraPoseR.rot, xytheta[2]);
	m_CameraPoseR.pos.x = xytheta[0];
	m_CameraPoseR.pos.y = xytheta[1];
      }
    }
  }

  m_PbPort = 5050;
  m_PbHost = "localhost";

  if (cfg) {
    std::string usedCfgFile, tmp;
    if (cfg && cfg->getString("PEEKABOT_HOST", true, tmp, usedCfgFile) == 0) {
      m_PbHost = tmp;
    }
  }
} 

void ObjectRelationManager::start() 
{
  addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject>(cdl::ADD), 
		  new MemberFunctionChangeReceiver<ObjectRelationManager>(this,
								  &ObjectRelationManager::newObject));  


  addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE), 
		  new MemberFunctionChangeReceiver<ObjectRelationManager>(this,
								  &ObjectRelationManager::newObject));  

  addChangeFilter(createLocalTypeFilter<FrontierInterface::ObservedPlaneObject>
      (cdl::ADD), 
      new MemberFunctionChangeReceiver<ObjectRelationManager>(this, 
	&ObjectRelationManager::newPlaneObject));  

  addChangeFilter(createLocalTypeFilter<FrontierInterface::ObservedPlaneObject>
      (cdl::OVERWRITE), 
      new MemberFunctionChangeReceiver<ObjectRelationManager>(this, 
	&ObjectRelationManager::newPlaneObject));  

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
		  new MemberFunctionChangeReceiver<ObjectRelationManager>(this,
								  &ObjectRelationManager::newRobotPose));

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<ObjectRelationManager>(this,
								  &ObjectRelationManager::newRobotPose));  

  addChangeFilter(createGlobalTypeFilter<FrontierInterface::ObjectPriorRequest>(cdl::ADD), 
		  new MemberFunctionChangeReceiver<ObjectRelationManager>(this,
								  &ObjectRelationManager::newPriorRequest));  
  addChangeFilter(createGlobalTypeFilter<FrontierInterface::ObjectTiltAngleRequest>(cdl::ADD), 
		  new MemberFunctionChangeReceiver<ObjectRelationManager>(this,
								  &ObjectRelationManager::newTiltAngleRequest));  

//  m_placeInterface = getIceServer<FrontierInterface::PlaceInterface>("place.manager");
  if (!m_bNoPTZ) {
    log("connecting to PTU");
    Ice::CommunicatorPtr ic = getCommunicator();

    Ice::Identity id;
    id.name = "PTZServer";
    id.category = "PTZServer";

    std::ostringstream str;
    str << ic->identityToString(id) 
      << ":default"
      << " -h localhost"
      << " -p " << cast::cdl::CPPSERVERPORT;

    Ice::ObjectPrx base = ic->stringToProxy(str.str());    
    m_ptzInterface = ptz::PTZInterfacePrx::uncheckedCast(base);
  }

  if (m_bDisplayPlaneObjectsInPB || m_bDisplayVisualObjectsInPB) {
    while(!m_PeekabotClient.is_connected() && (m_RetryDelay > -1)){
      connectPeekabot();
      sleep(m_RetryDelay);
    }

    if (m_bDisplayPlaneObjectsInPB) {
      m_planeProxies.add(m_PeekabotClient, "plane_objects", peekabot::REPLACE_ON_CONFLICT);
    }
    if (m_bDisplayVisualObjectsInPB) {
      m_objectProxies.add(m_PeekabotClient, "visual_objects", peekabot::REPLACE_ON_CONFLICT);
    }
    println("Connected to peekabot, ready to go");
  }

  if (m_planeModelFilename != "") {
    log("Loading plane models from file '%s'", m_planeModelFilename.c_str());
    readPlaneModelsFromFile();
  }


  log("ObjectRelationManager started");
}

void 
ObjectRelationManager::newRobotPose(const cdl::WorkingMemoryChange &objID) 
{
  double oldX = 0.0;
  double oldY = 0.0;
  double oldTheta = 0.0;
  double oldTime;
  if (lastRobotPose) {
    oldX = lastRobotPose->x;
    oldY = lastRobotPose->y;
    oldTheta = lastRobotPose->theta;
    oldTime = lastRobotPose->time.s + 0.000001*lastRobotPose->time.us;
  }

  try {
    lastRobotPose =
      getMemoryEntry<NavData::RobotPose2d>(objID.address);
  }
  catch (DoesNotExistOnWMException e) {
    log("Error! robotPose missing on WM!");
    return;
  }

  double distMoved = sqrt((oldX - lastRobotPose->x)*(oldX - lastRobotPose->x) +
      (oldY - lastRobotPose->y)*(oldY - lastRobotPose->y));
  double angleShift = lastRobotPose->theta - oldTheta;
  if (angleShift > M_PI) angleShift -= 2*M_PI;
  if (angleShift < -M_PI) angleShift += 2*M_PI;
  distMoved += abs(angleShift)*1.0;
  double deltaT = lastRobotPose->time.s +
    lastRobotPose->time.us*0.000001 - oldTime;
  double momVel = deltaT > 0 ? distMoved/deltaT : 0.0;
  //  log("momVel = %f, deltaT = %f", momVel, deltaT);

  if (momVel > m_standingStillThreshold) {
    m_timeSinceLastMoved = 0.0;
    if (m_bRecognitionIssuedThisStop) {
      // Signal tracking stop
      for (std::set<string>::iterator it = m_lookForObjects.begin(); it != m_lookForObjects.end(); it++) {
	addTrackerCommand(VisionData::REMOVEMODEL, it->c_str());
      }
    }
    m_bRecognitionIssuedThisStop = false;
  }
  else if (deltaT > 0.0) {
    double diff = lastRobotPose->time.s + lastRobotPose->time.us*0.000001 - oldTime;
    m_timeSinceLastMoved += diff;
    //    log("time %f %f %f", diff, oldTime, m_timeSinceLastMoved);
  }
}

void ObjectRelationManager::readRelationsFromFile(const string &filename)
{
  ifstream infile(filename.c_str());
  if (infile.bad()) {
    log("Error! Couldn't open relation file!");
    return;
  }

  int lineNo = 0;
  while (!infile.eof()) {
    char line[1024];
    lineNo++;
    infile.getline(line, 1023);
    istringstream is(line);

    string token;
    is >> token;

    if (token == "nRooms") {
      is >> nRooms;
    }
    else if (token == "nRoomCategories") {
      is >> nRoomCategories;
    }
    else if (token == "nObjects") {
      is >> nObjects;
    }
    else if (token == "Objects") {
      lineNo++;
      infile.getline(line, 1023);
      while (strlen(line)>0 && !infile.eof()) {
	hierarchyObjects.push_back(line);
	lineNo++;
	infile.getline(line, 1023);
      }
    }
    else if (token == "roomCategoryDefault") {
      roomCategoryDefault.resize(nRooms * nRoomCategories, 0.0);
      lineNo++;
      infile.getline(line, 1023);
      while (strlen(line)>0 && !infile.eof()) {
	istringstream is2(line);
	int a, b;
	double c;
	string tmp;
	is2 >> a;
	is2 >> tmp;
	is2 >> b;
	if (a < 0 || a >= nRooms || b < 0 || b >= nRoomCategories || tmp != "is") {
	  log("Error in file around line %i", lineNo);
	  return;
	}
	is2 >> c;
	roomCategoryDefault[nRoomCategories * a + b] = c;

	lineNo++;
	infile.getline(line, 1023);
      }
    }
    else if (token == "objectInRoomDefault") {
      objectInRoomDefault.resize(nObjects * nRoomCategories, 0.0);
      lineNo++;
      infile.getline(line, 1023);
      while (strlen(line)>0 && !infile.eof()) {
	istringstream is2(line);
	int a, b;
	double c;
	string tmp;
	is2 >> a;
	is2 >> tmp;
	if (tmp != "in") {
	  log("Error in file around line %i", lineNo);
	  return;
	}
	is2 >> b;
	if (a < 0 || a >= nObjects || b < 0 || b >= nRooms || tmp != "in") {
	  log("Error in file around line %i", lineNo);
	  return;
	}
	is2 >> c;
	objectInRoomDefault[nRoomCategories * a + b] = c;

	lineNo++;
	infile.getline(line, 1023);
      }
    }
    else if (token == "objectInObjectDefault") {
      objectInObjectDefault.resize(nObjects * nObjects, 0.0);
      lineNo++;
      infile.getline(line, 1023);
      while (strlen(line)>0 && !infile.eof()) {
	istringstream is2(line);
	int a, b;
	double c;
	string tmp;
	is2 >> a;
	is2 >> tmp;
	is2 >> b;
	if (a < 0 || a >= nObjects || b < 0 || b >= a || tmp != "in") {
	  log("Error in file around line %i", lineNo);
	  return;
	}
	is2 >> c;
	objectInObjectDefault[nObjects * a + b] = c;

	lineNo++;
	infile.getline(line, 1023);
      }
    }
    else if (token == "objectOnObjectDefault") {
      objectOnObjectDefault.resize(nObjects * nObjects, 0.0);
      lineNo++;
      infile.getline(line, 1023);
      while (strlen(line)>0 && !infile.eof()) {
	istringstream is2(line);
	int a, b;
	double c;
	string tmp;
	is2 >> a;
	is2 >> tmp;
	is2 >> b;
	if (a < 0 || a >= nObjects || b < 0 || b >= a || tmp != "on") {
	  log("Error in file around line %i", lineNo);
	  return;
	}
	is2 >> c;
	objectOnObjectDefault[nObjects * a + b] = c;

	lineNo++;
	infile.getline(line, 1023);
      }
    }
    else if (token == "") {
    }
    else {
      log("Error in file around line %i", lineNo);
      return;
    }
  }
}

void ObjectRelationManager::runComponent() 
{
  log("I am running!");

  sleepComponent(2000);

  SpatialGridMap::GridMapData def;
  def.occupancy = SpatialGridMap::UNKNOWN;
  def.pdf = 0.0;
  SpatialGridMap::GridMap<SpatialGridMap::GridMapData> pdfMap(100, 100, 0.05, 0.05, 0, 2.0, 0, 0, 0, def);

  sleepComponent(2000);

  while (isRunning()) {
    // Dispatch recognition commands if the robot has been standing still
    // long enough

    if (!m_bNoVision) {
      lockComponent();

      if (!m_bRecognitionIssuedThisStop &&
	  m_timeSinceLastMoved > m_recognitionTimeThreshold) {
	log("Issuing recognition commands");
	for (std::set<string>::iterator it = m_lookForObjects.begin(); it != m_lookForObjects.end(); it++) {
	  addRecognizer3DCommand(VisionData::RECOGNIZE, it->c_str(), "");
	}
	m_bRecognitionIssuedThisStop = true;
      }

      unlockComponent();
    }

    sleepComponent(500);
  }
}

void
ObjectRelationManager::newObject(const cast::cdl::WorkingMemoryChange &wmc)
{
  log("newObject called");
  try {
    VisionData::VisualObjectPtr observedObject =
      getMemoryEntry<VisionData::VisualObject>(wmc.address);
      string obsLabel = observedObject->identLabels[0];

   // if (m_timeSinceLastMoved > m_trackerTimeThreshold) {
      log("Got VisualObject: %s (%f,%f,%f)", obsLabel.c_str(),
	  observedObject->pose.pos.x,
	  observedObject->pose.pos.y,
	  observedObject->pose.pos.z);

      if (observedObject->pose.pos.x == 0.0 && observedObject->pose.pos.y == 0.0 && observedObject->pose.pos.z == 0.0)
      {
	log("Warning: object has invalid pose, returning without doing anything");
	return;
      }
      Pose3 pose = observedObject->pose;
      //Get robot pose
      Pose3 robotTransform;
      if (lastRobotPose != 0) {
	fromRotZ(robotTransform.rot, lastRobotPose->theta);

	robotTransform.pos.x = lastRobotPose->x;
	robotTransform.pos.y = lastRobotPose->y;
      }
      else {
	fromRotZ(robotTransform.rot, 0);
	robotTransform.pos.x = 0.0;
	robotTransform.pos.y = 0.0;
      }
      transform(robotTransform, pose, pose);

      //FIXME: if tag objects don't do this
      if(obsLabel == "metalbox" || obsLabel == "table1" || obsLabel == "table2" 
	  || obsLabel == "shelves" || obsLabel == "table"){
	pose = observedObject->pose;
      }


      // For now, assume each label represents a unique object
      map<std::string, SpatialObjectPtr>::iterator it = m_objects.find(obsLabel);
      bool bNewObject = false;
      if (it != m_objects.end()) {
	// update object
	//	  log("Updating object %i(%s)", objectID, observedObject->label.c_str());
	log("Updating %s on %s", obsLabel.c_str(), wmc.address.id.c_str());
	m_visualObjectIDs[obsLabel] = wmc.address.id;
      }
      else {
	// New object
	bNewObject = true;
	log("New SpatialObject: %s", obsLabel.c_str());
	m_objects[obsLabel] = new SpatialData::SpatialObject;
	m_objects[obsLabel]->label = obsLabel;
	m_objects[obsLabel]->pose = pose;
//	//FIXME: This is a pure, unadulterated hack
//	if (obsLabel == "bookcase_lg") {
//	  Matrix33 flip;
//	  fromRotZ(flip, M_PI);
//	  m_objects[obsLabel]->pose.rot = flip*pose.rot;
//	}
      }

      if (m_objectModels.find(obsLabel) == m_objectModels.end()) {
	m_objectModels[obsLabel] = generateNewObjectModel(obsLabel);
      }
      ASSERT_TYPE(m_objectModels[obsLabel]);

      spatial::Object *obsObject = m_objectModels[obsLabel];

          log("2");

      double diff = length(m_objects[obsLabel]->pose.pos - pose.pos);
      diff += length(getRow(m_objects[obsLabel]->pose.rot - pose.rot, 1));
      diff += length(getRow(m_objects[obsLabel]->pose.rot - pose.rot, 2));
      if (diff > 0.01 || bNewObject) {
	      log("3");
	if (obsObject->type == OBJECT_PLANE ||
	    //FIXME
	    obsLabel == "table" || 
	    obsLabel == "table1" ||
	      obsLabel == "table2" ||
	      obsLabel == "bookcase_sm" ||
	      obsLabel == "bookcase_lg" ||
	      obsLabel == "shelves" ||
	      obsLabel == "desk"){
	  // Flatten pose for plane objects
	  if (pose.rot.m00 == 0.0 && pose.rot.m10 == 0.0) {
	    setIdentity(pose.rot);
	  }
	  else {
	    fromRotZ(pose.rot, atan2(pose.rot.m01, pose.rot.m00));
	  }
	}


	m_objects[obsLabel]->pose = pose;
	m_lastObjectPoseTimes[obsLabel] = observedObject->time;

	if (m_objectWMIDs.find(obsLabel) == m_objectWMIDs.end()) {
	  string newID = newDataID();

	  addToWorkingMemory<SpatialData::SpatialObject>(newID, m_objects[obsLabel]);
	  m_objectWMIDs[obsLabel]=newID;
	}
	else {

	  try {
	    overwriteWorkingMemory<SpatialData::SpatialObject>(m_objectWMIDs[obsLabel],
		m_objects[obsLabel]);
	  }
	  catch (DoesNotExistOnWMException) {
	    log("Error! SpatialObject disappeared from memory!");
	    return;
	  }
	}
	    log("4");

	if (m_bDisplayVisualObjectsInPB && m_objectProxies.is_assigned()) {
	  log("5");
	  if (obsObject->type == OBJECT_BOX) {
	    BoxObject *box = (BoxObject*)obsObject;
	    peekabot::CubeProxy theobjectproxy;
	    theobjectproxy.add(m_objectProxies, m_objects[obsLabel]->label, peekabot::REPLACE_ON_CONFLICT);
	    theobjectproxy.translate(pose.pos.x, pose.pos.y, pose.pos.z);
	    double angle;
	    Vector3 axis;
	    toAngleAxis(pose.rot, angle, axis);
	    log("x = %f y = %f z = %f   angle = %f axis=(%f,%f,%f)",
		pose.pos.x, pose.pos.y, pose.pos.z, angle, 
		axis.x, axis.y, axis.z);
	    theobjectproxy.rotate(angle, axis.x, axis.y, axis.z);
	    theobjectproxy.set_scale(box->radius1*2, box->radius2*2,
		box->radius3*2);
	  }
	  else if (obsObject->type == OBJECT_HOLLOW_BOX){
	    HollowBoxObject *box = (HollowBoxObject*)obsObject;
	    peekabot::CubeProxy theobjectproxy;
	    theobjectproxy.add(m_objectProxies, m_objects[obsLabel]->label, peekabot::REPLACE_ON_CONFLICT);
	    theobjectproxy.translate(pose.pos.x, pose.pos.y, pose.pos.z);
	    double angle;
	    Vector3 axis;
	    toAngleAxis(pose.rot, angle, axis);
	    log("x = %f y = %f z = %f   angle = %f axis=(%f,%f,%f)",
		pose.pos.x, pose.pos.y, pose.pos.z, angle, 
		axis.x, axis.y, axis.z);
	    theobjectproxy.rotate(angle, axis.x, axis.y, axis.z);
	    theobjectproxy.set_scale(box->radius1*2, box->radius2*2,
		box->radius3*2);
	  }
	  else {
	  }
	}
	//    log("5");

	// Check degree of onness
/* HACK HACK Marc (15/01/2011) disabled for IJCAI eval and Dora yr2 stable
	recomputeOnnessForObject(obsLabel);
	recomputeInnessForObject(obsLabel);

*/
	if (m_placeInterface != 0) {
	  // Check degree of containment in Places
	  FrontierInterface::PlaceMembership membership = 
	    m_placeInterface->getPlaceMembership(pose.pos.x, pose.pos.y);

	  setContainmentProperty(obsLabel, membership.placeID, 1.0);
	}

	//  Needs estimate of extent of Places, in sensory frame
	//   Add interface to PlaceManager?
	// Post one or more Place containment structs on WM
	// Find and remove/update extant property structs

	// Check degree of support between Objects
	//  Need a list of tracked Object
	//  Check contact points and estimated centers of gravity
	//   Need access to representation of object models
	//   
      }
    //}
  }
  catch (DoesNotExistOnWMException) {
    log("Error! new Object missing on WM!");
  }
}

void 
ObjectRelationManager::objectChanged(const cast::cdl::WorkingMemoryChange &wmc)
{
  try {
    VisionData::VisualObjectPtr observedObject =
      getMemoryEntry<VisionData::VisualObject>(wmc.address);
  }
  catch (DoesNotExistOnWMException) {
    log("Error! overwritten Object missing on WM!");
  }
}

void 
ObjectRelationManager::newPlaneObject(const cast::cdl::WorkingMemoryChange &wmc)
{
  try {
    FrontierInterface::ObservedPlaneObjectPtr observedObject =
      getMemoryEntry<FrontierInterface::ObservedPlaneObject>
      (wmc.address);

    log("Read object %s at %f, %f, %f", observedObject->label.c_str(),
	observedObject->pos.x, observedObject->pos.y,
	observedObject->pos.z);

    string obsPlaneLabel = observedObject->label;

    bool isNew = false;
    if (m_planeObjects.find(obsPlaneLabel) == m_planeObjects.end()) {
      // New plane object
      m_planeObjects[obsPlaneLabel] = observedObject;
      isNew = true;
    }

    double diff = length(m_planeObjects[obsPlaneLabel]->pos - observedObject->pos);
    double angDiff = m_planeObjects[obsPlaneLabel]->angle - observedObject->angle;
    if (angDiff > M_PI) angDiff -= 2*M_PI;
    if (angDiff < -M_PI) angDiff += 2*M_PI;
    diff += abs(angDiff*1.0);

    if (diff > 0.01 || isNew) {
      m_planeObjects[obsPlaneLabel]->pos = observedObject->pos;
      m_planeObjects[obsPlaneLabel]->angle = observedObject->angle;

      recomputeOnnessForPlane(obsPlaneLabel);

      if (m_bDisplayPlaneObjectsInPB) {
	if (m_PeekabotClient.is_connected()) {
	  if (m_planeObjectModels.find(obsPlaneLabel) ==
	      m_planeObjectModels.end()) {
	    log("Error! Plane model for %s was missing!", obsPlaneLabel.c_str());
	  }
	  PlaneObject &planeObject = 
	    m_planeObjectModels[obsPlaneLabel];
	  char identifier[100];
	  sprintf(identifier, "label%d", 0);
	  peekabot::PolygonProxy pp;
	  peekabot::VertexSet polyVerts;
	  pp.add(m_planeProxies, identifier, peekabot::REPLACE_ON_CONFLICT);
	  polyVerts.add(planeObject.radius1, planeObject.radius2, 0);
	  polyVerts.add(-planeObject.radius1, planeObject.radius2, 0);
	  polyVerts.add(-planeObject.radius1, -planeObject.radius2, 0);
	  polyVerts.add(planeObject.radius1, -planeObject.radius2, 0);
	  pp.add_vertices(polyVerts);

	  pp.translate(observedObject->pos.x, observedObject->pos.y, 
	      observedObject->pos.z);
	  pp.rotate(observedObject->angle, 0.0, 0.0, 1.0);
	}
      }
    }
  }
  catch (DoesNotExistOnWMException) {
    log("Error! overwritten Object missing on WM!");
  }
}

void
ObjectRelationManager::setContainmentProperty(const string &label, int placeID, double confidence)
{
  map<string, PlaceContainmentObjectPropertyPtr>::iterator it =
    m_containmentProperties.find(label);
  SpatialProperties::PlaceContainmentObjectPropertyPtr containmentProp;
  if (it == m_containmentProperties.end()) {
    //New containment property for this Object

    containmentProp =
      new SpatialProperties::PlaceContainmentObjectProperty;
    m_containmentProperties[label] = containmentProp;
  }
  else {
    containmentProp = it->second;
  }
  SpatialProperties::IntegerValuePtr placeValue1 = 
    new SpatialProperties::IntegerValue;
  SpatialProperties::IntegerValuePtr placeValue2 = 
    new SpatialProperties::IntegerValue;
  placeValue1->value = placeID;
  placeValue2->value = -1;

  SpatialProperties::ValueProbabilityPair pair1 =
  { placeValue1, confidence };
  SpatialProperties::ValueProbabilityPair pair2 =
  { placeValue2, 1.0-confidence };

  SpatialProperties::ValueProbabilityPairs pairs;
  pairs.push_back(pair1);
  pairs.push_back(pair2);

  SpatialProperties::DiscreteProbabilityDistributionPtr discDistr =
    new SpatialProperties::DiscreteProbabilityDistribution;
  discDistr->data = pairs;
  containmentProp->placeId = placeID;
//  containmentProp->objectId = objectID;
  containmentProp->distribution = discDistr;
  containmentProp->mapValue = confidence > 0.5 ? placeValue1 : placeValue2;
  containmentProp->mapValueReliable = confidence > 0.8 || confidence < 0.2;

  //string newID = newDataID();
  //addToWorkingMemory<SpatialProperties::PlaceContainmentObjectProperty>(newID, containmentProp);
}

void 
ObjectRelationManager::connectPeekabot()
{
  try {
    log("Trying to connect to Peekabot (again) on host %s and port %d",
        m_PbHost.c_str(), m_PbPort);

    m_PeekabotClient.connect(m_PbHost, m_PbPort);

  } catch(std::exception &e) {
    log("Caught exception when connecting to peekabot (%s)",
        e.what());
    return;
  }
}

Pose3 ObjectRelationManager::getCameraToWorldTransform()
{
  //Get camera ptz from PTZServer
  Pose3 cameraRotation;
  if (m_ptzInterface != 0) {
    ptz::PTZReading reading = m_ptzInterface->getPose();

    Matrix33 pan;
    fromRotZ(pan, reading.pose.pan);

    Matrix33 tilt;
    fromRotX(tilt, -reading.pose.tilt);

    cameraRotation.rot = pan*tilt;
  }

  //Additional transform of ptz base frame
  Matrix33 tmp;
  fromRotZ(tmp, -M_PI/2);
  cameraRotation.rot = cameraRotation.rot * tmp;

  fromRotY(tmp, -M_PI/2);
  cameraRotation.rot = cameraRotation.rot * tmp;

  //Get robot pose
  Pose3 robotTransform;
  if (lastRobotPose != 0) {
    fromRotZ(robotTransform.rot, lastRobotPose->theta);

    robotTransform.pos.x = lastRobotPose->x;
    robotTransform.pos.y = lastRobotPose->y;
  }

  Pose3 cameraOnRobot;
  transform(m_CameraPoseR, cameraRotation, cameraOnRobot);

  Pose3 cameraInWorld;
  transform(robotTransform, cameraOnRobot, cameraInWorld);
  return cameraInWorld;
} 

void 
ObjectRelationManager::recomputeOnnessForObject(const string &label)
{
  if (m_objectModels.find(label) == m_objectModels.end()) {
    log("Error! Object model was missing!");
    return;
  }
  if (m_objects.find(label) == m_objects.end()) {
    log("Error! Object was missing!");
    return;
  }

//  log("1");
      ASSERT_TYPE(m_objectModels[label]);
  spatial::Object *obj = m_objectModels[label];

  obj->pose = m_objects[label]->pose;

//  log("2");

  for (map<string, FrontierInterface::ObservedPlaneObjectPtr>::iterator it = m_planeObjects.begin(); 
      it != m_planeObjects.end(); it++) {
    PlaneObject po;
    po.type = OBJECT_PLANE;
    po.shape = PLANE_OBJECT_RECTANGLE;
    po.pose.pos = it->second->pos;
    fromAngleAxis(po.pose.rot, it->second->angle, vector3(0.0, 0.0, 1.0));
    if (m_planeObjectModels.find(it->second->label) != m_planeObjectModels.end()) {
      po.radius1 = m_planeObjectModels[it->second->label].radius1;
      po.radius2 = m_planeObjectModels[it->second->label].radius2;
    }
    else {
      log("Error! Unknown plane object!");
      return;
    }
//    log("3");
    double onness = m_evaluator.evaluateOnness(&po, obj);
//    log("4");
    log("Object %s on object %s is %f", label.c_str(), 
	it->second->label.c_str(), onness);
  }

  for (map<string, SpatialObjectPtr>::iterator it = m_objects.begin();
      it != m_objects.end(); it++) {
    string supportObjectLabel = it->first;
    if (supportObjectLabel != label) {

      if (m_objectModels.find(supportObjectLabel) == m_objectModels.end()) {
	log("Error! Support object model was missing!");
	return;
      }
      ASSERT_TYPE(m_objectModels[supportObjectLabel]);
      m_objectModels[supportObjectLabel]->pose = 
	m_objects[supportObjectLabel]->pose;

      double onness = m_evaluator.evaluateOnness(m_objectModels[supportObjectLabel],
	  obj);

      log("Object %s on object %s is %f", label.c_str(), 
	  it->second->label.c_str(), onness);
    }
  }
}

void
ObjectRelationManager::recomputeInnessForObject(const string &label)
{
  if (m_objectModels.find(label) == m_objectModels.end()) {
    log("Error! Object model was missing!");
    return;
  }
  if (m_objects.find(label) == m_objects.end()) {
    log("Error! Object was missing!");
    return;
  }

      ASSERT_TYPE(m_objectModels[label]);
  spatial::Object *obj = m_objectModels[label];

  obj->pose = m_objects[label]->pose;

  for (map<string, SpatialObjectPtr>::iterator it = m_objects.begin();
      it != m_objects.end(); it++) {
    string containerObjectLabel = it->first;
    if (containerObjectLabel != label) {

      if (m_objectModels.find(containerObjectLabel) == m_objectModels.end()) {
	log("Error! Container object model was missing!");
	return;
      }
      ASSERT_TYPE(m_objectModels[containerObjectLabel]);
      m_objectModels[containerObjectLabel]->pose =
	m_objects[containerObjectLabel]->pose;

      double inness = m_evaluator.evaluateInness(m_objectModels[containerObjectLabel],
	  obj);

      log("Object %s in object %s is %f", label.c_str(),
	  it->second->label.c_str(), inness);
    }
  }
}

void
ObjectRelationManager::recomputeOnnessForPlane(const string &planeLabel)
{
  if (m_planeObjects.find(planeLabel) == m_planeObjects.end()) {
    log("Error! Plane object missing!");
    return;
  }

  PlaneObject po;
  po.type = OBJECT_PLANE;
  po.shape = PLANE_OBJECT_RECTANGLE;
  po.pose.pos = m_planeObjects[planeLabel]->pos;
  fromAngleAxis(po.pose.rot, m_planeObjects[planeLabel]->angle, 
      vector3(0.0, 0.0, 1.0));
    if (m_planeObjectModels.find(planeLabel) != m_planeObjectModels.end()) {
    po.radius1 = m_planeObjectModels[planeLabel].radius1;
    po.radius2 = m_planeObjectModels[planeLabel].radius2;
  }
  else {
    log("Error! Unknown plane object!");
    return;
  }

  for (map<string, SpatialData::SpatialObjectPtr>::iterator it = m_objects.begin(); 
      it != m_objects.end(); it++) {
    string objectLabel = it->first;

    if (m_objectModels.find(objectLabel) == m_objectModels.end()) {
      log("Error! Object model was missing!");
    }
    ASSERT_TYPE(m_objectModels[objectLabel]);
    m_objectModels[objectLabel]->pose = m_objects[objectLabel]->pose;
    log("Evaluating object %s on object %s",objectLabel.c_str(), 
	planeLabel.c_str());
    double onness = m_evaluator.evaluateOnness(&po, m_objectModels[objectLabel]);
    log("Object %s on object %s is %f", objectLabel.c_str(), 
	planeLabel.c_str(), onness);
  }
}

//void
//ObjectRelationManager::sampleOnnessForObject(const string &supLabel, 
//    const string &onLabel) 
//{
//  if (m_objectModels.find(supLabel) == m_objectModels.end() ||
//      m_objects.find(supLabel) == m_objects.end()) {
//    log("Error! Support object model missing!");
//    return;
//  }
//  ASSERT_TYPE(m_objectModels[supLabel]);
//    m_objects[supLabel]->pose;
//
//  spatial::Object *objectS = m_objectModels[supLabel];
//
//  if (m_objectModels.find(onLabel) == m_objectModels.end()) {
//    log("Error! Supported object model missing!");
//    return;
//  }
//
//  ASSERT_TYPE(m_objectModels[onLabel]);
//  spatial::Object *objectO = m_objectModels[onLabel];
//
//  Pose3 oldPose = objectO->pose;
//  vector<Vector3> points;
//
//  BoxObject *supportBox = (BoxObject *)objectS;
//
//  double frameRadius = supportBox->radius1 > supportBox->radius2 ?
//    supportBox->radius1 : supportBox->radius2;
//  frameRadius = frameRadius > supportBox->radius3 ? 
//    frameRadius : supportBox->radius3;
//  m_evaluator.sampleOnnessDistribution(objectS, objectO, points, 
//      objectS->pose.pos.x - frameRadius, objectS->pose.pos.x + frameRadius,
//      objectS->pose.pos.y - frameRadius, objectS->pose.pos.y + frameRadius,
//      objectS->pose.pos.z - frameRadius, objectS->pose.pos.z + frameRadius*4,
//      0.04, 0.02);
//  objectO->pose = oldPose;
//
//  peekabot::PointCloudProxy pcloud;
//  peekabot::VertexSet cloudPoints;
//  pcloud.add(m_relationTester,"onpoints", peekabot::REPLACE_ON_CONFLICT);
//
//  for (vector<Vector3>::iterator it = points.begin(); it != points.end();
//      it++) {
//    cloudPoints.add(it->x,it->y,it->z);
//  }
//  pcloud.add_vertices(cloudPoints);
//}
//
//void
//ObjectRelationManager::sampleOnnessForPlane(const string &planeLabel, const string &objectLabel) 
//{
//
//
//  if (m_planeObjects.find(planeLabel) == m_planeObjects.end()) {
//    log("Error! Plane object missing!");
//    return;
//  }
//
//  PlaneObject po;
//  po.type = OBJECT_PLANE;
//  po.pose.pos = m_planeObjects[planeLabel]->pos;
//  fromAngleAxis(po.pose.rot, m_planeObjects[planeLabel]->angle, 
//      vector3(0.0, 0.0, 1.0));
//
//  if (m_planeObjectModels.find(planeLabel) != m_planeObjectModels.end()) {
//    po.radius1 = m_planeObjectModels[planeLabel].radius1;
//    po.radius2 = m_planeObjectModels[planeLabel].radius2;
//  }
//  else {
//    log("Error! Unknown plane object!");
//    return;
//  }
//
//  if (m_objectModels.find(objectLabel) == m_objectModels.end()) {
//    log("Error! Object model missing!");
//    return;
//  }
//
//  ASSERT_TYPE(m_objectModels[objectLabel]);
//  spatial::Object *objectO = m_objectModels[objectLabel];
//
//  Pose3 oldPose = objectO->pose;
//  vector<Vector3> points;
//  double frameRadius = 1.5 * 
//    (po.radius1 > po.radius2 ? po.radius1 : po.radius2);
//  m_evaluator.sampleOnnessDistribution(&po, objectO, points, 
//      po.pose.pos.x - frameRadius, po.pose.pos.x + frameRadius,
//      po.pose.pos.y - frameRadius, po.pose.pos.y + frameRadius,
//      0, 1.5, 
//      0.1, 0.025);
//  objectO->pose = oldPose;
//
//  peekabot::PointCloudProxy pcloud;
//  peekabot::VertexSet cloudPoints;
//  pcloud.add(m_relationTester,"onpoints", peekabot::REPLACE_ON_CONFLICT);
//
//  for (vector<Vector3>::iterator it = points.begin(); it != points.end();
//      it++) {
//    cloudPoints.add(it->x,it->y,it->z);
//  }
//  pcloud.add_vertices(cloudPoints);
//}

void 
ObjectRelationManager::addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, std::string label, std::string visualObjectID){
	VisionData::Recognizer3DCommandPtr rec_cmd = new VisionData::Recognizer3DCommand;
  rec_cmd->cmd = cmd;
  rec_cmd->label = label;
  rec_cmd->visualObjectID = visualObjectID;
  addToWorkingMemory(newDataID(), "vision.sa", rec_cmd);
}

void 
ObjectRelationManager::addTrackerCommand(VisionData::TrackingCommandType cmd, std::string label){
  VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
  log("addTrackerCommand");
  track_cmd->cmd = cmd;
  if (m_objects.find(label) == m_objects.end()) {
    log("Error! Can't issue tracking command; object unknown!");
    return;
  }

  if (m_visualObjectIDs.find(label) == m_visualObjectIDs.end()) {
    log ("Error! Visual object WM ID was unknown!");
    return;
  }
  track_cmd->visualObjectID = m_visualObjectIDs[label];;

  addToWorkingMemory(newDataID(), "vision.sa", track_cmd);
}

void
ObjectRelationManager::readPlaneModelsFromFile()
{
  ifstream infile(m_planeModelFilename.c_str());
  if (!infile.good()) {
    log("Error opening file");
    return;
  }

  char buf[256];
  infile.getline(buf, 255);
  while (!infile.eof() && strlen(buf) > 0) {
    if (buf[0] == '#') {
      //Comment
      infile.getline(buf, 255);
      continue;
    }

    istringstream ss(buf);
    string label;
    double width, depth;
    ss >> label >> width >> depth;
    if (label == "" || width == 0.0 || depth == 0.0) {
      infile.getline(buf, 255);
      continue;
    }
    PlaneObject obj;
    obj.type = OBJECT_PLANE;
    obj.shape = PLANE_OBJECT_RECTANGLE;
    obj.radius1 = width/2;
    obj.radius2 = depth/2;
    setIdentity(obj.pose.rot);
    obj.pose.pos.x = -FLT_MAX;
    obj.pose.pos.y = -FLT_MAX;
    obj.pose.pos.z = -FLT_MAX;

    m_planeObjectModels[label] = obj;
    infile.getline(buf, 255);
  }
}

void
ObjectRelationManager::newPriorRequest(const cdl::WorkingMemoryChange &wmc) {
  try {
    FrontierInterface::ObjectPriorRequestPtr request =
      getMemoryEntry<FrontierInterface::ObjectPriorRequest>(wmc.address);

    if (request->objects.size() < 2) {
      log("Error! Can't compute onness for less than 2 objects!");
      return;
    }

    vector<spatial::Object *> objectChain;
    vector<spatial::SpatialRelationType> relations;

    //Fill it
    string supportObjectLabel = request->objects.back();
    log("SupportObject: %s", supportObjectLabel.c_str());

    spatial::Object *supportObject;

    //Look for the base object in the requested hierarchy.
    //If it doesn't exist, create a model for it and generate a 
    //sample cloud based on random orientation, position = origin

    request->outCloud->isBaseObjectKnown = true;

    if (m_planeObjectModels.find(supportObjectLabel) != m_planeObjectModels.end()) {
      supportObject = &m_planeObjectModels[supportObjectLabel];
      for (map<string, FrontierInterface::ObservedPlaneObjectPtr>::iterator it = m_planeObjects.begin(); it != m_planeObjects.end(); it++) {
	if (it->second->label == supportObjectLabel) {
	  // update object
	  supportObject->pose.pos = it->second->pos;
	  fromAngleAxis(supportObject->pose.rot, it->second->angle, vector3(0.0, 0.0, 1.0));
	}
      }
    }
    else {
      // For now, assume each label represents a unique object

      // Check if the object model is known (with a known pose).
      if (m_objectModels.find(supportObjectLabel) == m_objectModels.end()) {
	m_objectModels[supportObjectLabel] = generateNewObjectModel(supportObjectLabel);
      }
      // Pose is known
      supportObject = m_objectModels[supportObjectLabel];
      ASSERT_TYPE(supportObject);

      // Otherwise, generate a model and have its poses randomly sampled
      if (m_objects.find(supportObjectLabel) == m_objects.end()) {
	// The pose of this object is not known.
	log("Support object was unknown; computing distribution with object in origin");

	request->outCloud->isBaseObjectKnown = false;
      }
      else {
	supportObject->pose = m_objects[supportObjectLabel]->pose;
      }
    }
//    objectChain.push_back(supportObject);

    FrontierInterface::RelationSeq::iterator rit =
      request->relationTypes.begin();
    for (vector<string>::iterator it = request->objects.begin();
	it != request->objects.end(); it++, rit++) {
      if (m_planeObjectModels.find(*it) != m_planeObjectModels.end()) {
	log("Can't compute ON for table on sth else!");
	return;
      }
      else {
	// For now, assume each label represents a unique object
	if (m_objectModels.find(*it) == m_objectModels.end()) {
	  // New object model
	  m_objectModels[*it] = generateNewObjectModel(*it);
	}
	ASSERT_TYPE(m_objectModels[*it]);
	objectChain.push_back(m_objectModels[*it]);
	if (rit != request->relationTypes.end()) {
	  switch (*rit) {
	    case FrontierInterface::ON: 
	      relations.push_back(RELATION_ON);
	      break;
	    case FrontierInterface::IN:
	      relations.push_back(RELATION_IN);
	      break;
	  }
	}
      }
    }

    Pose3 tmpPose = supportObject->pose;

    if (request->baseObjectPose.size() > 0) {
      // The base object pose is given. Set the model's pose to this value
      // temporarily
      supportObject->pose = request->baseObjectPose[0];
    }

    SampleCloud cloud;
    m_sampler.
      sampleBinaryRelationSystematically(relations, objectChain,
	  request->objects, request->cellSize, cloud);

    log("got cloud.");
    supportObject->pose = tmpPose;

    cloud.compact();

    cloud.makePointCloud(request->outCloud->center, 
	request->outCloud->interval,
	request->outCloud->xExtent,
	request->outCloud->yExtent,
	request->outCloud->zExtent,
	request->outCloud->values);
    log("made point cloud.");

    if (request->outCloud->isBaseObjectKnown) {
      request->outCloud->center += m_objects[supportObjectLabel]->pose.pos;
    }
    // Hard-coded z-coordinates of some floor-bound objects
    else if (supportObjectLabel == "table1") {
      request->outCloud->center.z += 0.45-0.225;
    }
    else if (supportObjectLabel == "table2") {
      request->outCloud->center.z += 0.72-0.36;
    }
    else if (supportObjectLabel == "desk") {
      request->outCloud->center.z += 0.75-0.05;
    }
    else if (supportObjectLabel == "bookcase_sm") {
      request->outCloud->center.z += 0.75+0.08;
    }
    else if (supportObjectLabel == "bookcase_lg") {
      request->outCloud->center.z += 0.965+0.08;
    }
    else if (supportObjectLabel == "shelves") {
      request->outCloud->center.z += 1.075+0.00;
    }
    
    //HACK for review; removeme

    if (request->objects.size() == 2 && request->objects[1] == "table") {
      string filename = "table+with+object.txt";
      ofstream tableCloudFile(filename.c_str(), ios::out);
      tableCloudFile << request->outCloud->center.x;
     tableCloudFile << " " ; 
      tableCloudFile << request->outCloud->center.y;
     tableCloudFile << " " ; 
      tableCloudFile << request->outCloud->center.z;
     tableCloudFile << " " ; 
      tableCloudFile << request->outCloud->interval;
     tableCloudFile << " " ; 
      tableCloudFile << request->outCloud->xExtent;
     tableCloudFile << " " ; 
      tableCloudFile << request->outCloud->yExtent;
     tableCloudFile << " " ; 
      tableCloudFile << request->outCloud->zExtent;
     tableCloudFile << " " ; 
      tableCloudFile << request->outCloud->values.size();
      tableCloudFile << " ";

      for (unsigned long i = 0; i < request->outCloud->values.size(); i++) {
	tableCloudFile << request->outCloud->values[i];
     tableCloudFile << " " ; 
      }
      tableCloudFile << request->outCloud->isBaseObjectKnown;
     tableCloudFile << " " ; 

      tableCloudFile << request->totalMass;
    }

    overwriteWorkingMemory<FrontierInterface::ObjectPriorRequest>(wmc.address, request);
    log("overwrote point cloud.");
  }

  catch (DoesNotExistOnWMException) {
    log("Error! Prior request disappeared from WM!");
  }

}

  void 
ObjectRelationManager::newTiltAngleRequest(const cast::cdl::WorkingMemoryChange &wmc)
{
  const unsigned int pointCount = 25;
  try {
    FrontierInterface::ObjectTiltAngleRequestPtr request =
      getMemoryEntry<FrontierInterface::ObjectTiltAngleRequest>(wmc.address);

    if (request->objects.size() < 2) {
      log("Error! Can't compute onness for less than 2 objects!");
      return;
    }

    vector<Vector3> outPoints;
    outPoints.reserve(pointCount);

    string supportObjectLabel = request->objects.back();
    spatial::Object *supportObject;
    if (m_planeObjectModels.find(supportObjectLabel) != m_planeObjectModels.end()) {
      supportObject = &m_planeObjectModels[supportObjectLabel];
      for (map<string, FrontierInterface::ObservedPlaneObjectPtr>::iterator it = m_planeObjects.begin(); it != m_planeObjects.end(); it++) {
	if (it->second->label == supportObjectLabel) {
	  // update object
	  supportObject->pose.pos = it->second->pos;
	  fromAngleAxis(supportObject->pose.rot, it->second->angle, vector3(0.0, 0.0, 1.0));
	}
      }
    }
    else {
      // For now, assume each label represents a unique object
      if (m_objects.find(supportObjectLabel) == m_objects.end()) {
	// This object's pose is not known. Cannot compute
	// onness for this hierarchy.
	supportObject = 0;
      }
      else {
	if (m_objectModels.find(supportObjectLabel) == m_objectModels.end()) {
	  m_objectModels[supportObjectLabel] =
	    generateNewObjectModel(supportObjectLabel);
	}
	  ASSERT_TYPE(m_objectModels[supportObjectLabel]);

	supportObject = m_objectModels[supportObjectLabel];
      }
    }

    if (supportObject != 0) {

      vector<Vector3> triangle;
      triangle.push_back(vector3(request->triangle[0].x,request->triangle[0].y, 0));
      triangle.push_back(vector3(request->triangle[1].x,request->triangle[1].y, 0));
      triangle.push_back(vector3(request->triangle[2].x,request->triangle[2].y, 0));

      int iterations = 0;
      while (iterations < 1000 && outPoints.size() < pointCount) {
	iterations++;
//	sampleOnnessRecursively(request->objects, request->objects.size()-2, nSamplesPerStep, 
//	    pointCount, outPoints, supportObject);
	cogx::Math::Vector3 tiltAngle;
	for (vector<Vector3>::iterator it = outPoints.begin(); it != outPoints.end(); it++) {
	  tiltAngle.x = it->x;
	  tiltAngle.y = it->y;
	  tiltAngle.z = it->z;
	  request->tiltAngles.push_back(tiltAngle);
	}
      }
    }
    else {
      log("Error! Support object was unknown; cannot compute PDF for this hierarchy!");
    }

    overwriteWorkingMemory<FrontierInterface::ObjectTiltAngleRequest>(wmc.address, request);
  }

  catch (DoesNotExistOnWMException) {
    log("Error! Height request disappeared from WM!");
  }
}

struct Precursor 
{
  int object; // -1 means there isn't one
  int state; // 0 means on, 1 means in
};

bool
updateHierarchy(Precursor precursors[], int current)
{
  int &currentPrecursor = precursors[current].object;
  if (currentPrecursor == -1) {
    return true;
  }

  bool precursorDone = updateHierarchy(precursors, currentPrecursor);
  if (precursorDone) {
    precursors[current].state++;
    if (precursors[current].state == 2) {
      currentPrecursor--;
      precursors[current].state = 0;
    }

    precursors[currentPrecursor].object = currentPrecursor - 1;
    precursors[currentPrecursor].state = 0;
  }
  return false;
}

// Enumerate all non-forbidden relations
// Store them in an array each
// each contains a small struct describing the involved objects
// and 

vector<string>
ObjectRelationManager::computeMarginalDistribution(string objName)
{
  unsigned int object;
  for (object = 0; object < hierarchyObjects.size(); object++) {
    if (hierarchyObjects[object] == objName)
      break;
  }
  if (object == hierarchyObjects.size()) {
    log("Error! Asked to compute distributions for unknown object!");
  }

  vector<string> ret;
  vector<double> probs;
  // Get all relational chains pertaining to this object,

  int roomTypeVal[nRooms];
  int objectInRoomVal[nObjects];
  int objectInObjectVal[nObjects*nObjects];
  int objectOnObjectVal[nObjects*nObjects];
  int objectDirectlyOnObjectVal[nObjects];
  if (nRooms*nRoomCategories != (int)roomCategoryDefault.size()) {
    log ("Error! roomCategoryDefault inconsistency!");
  }
  if (nObjects*nRoomCategories != (int)objectInRoomDefault.size()) {
    log ("Error! objectInRoomDefault inconsistency!");
  }
  if (nObjects*nObjects != (int)objectInObjectDefault.size()) {
    log ("Error! objectInObjectDefault inconsistency!");
  }
  if (nObjects*nObjects != (int)objectOnObjectDefault.size()) {
    log ("Error! objectOnObjectDefault inconsistency!");
  }

  for (int i = 0; i < nRooms; i++) {
    roomTypeVal[i] = 0;
  }
  for (int j = 0; j < nObjects; j++) {
    objectInRoomVal[j] = 0;
  }

  for (int i = 0; i < nObjects; i++) {
    for (int j = 0; j < nObjects; j++) {
      objectInObjectVal[i * nObjects + j] = 0;
      objectOnObjectVal[i * nObjects + j] = 0;
    }
    objectDirectlyOnObjectVal[i] = -1;
  }

  double totalOverallMass = 0.0;
  
  // Loop over rooms
  // Loop over all objects higher in the ordering than the query object.
  // Each object can appear in the chain as "ON_d" or "IN", or not at all.

  for (int fixedRoom = 0; fixedRoom < nRooms; fixedRoom++) {


    // Fix all object relations involving query object as trajector, AND
    // all relations involving those landmarks as trajectors, recursively.

    Precursor precursors[nObjects];
    for (int id = object; id >= 0; id--) {
      precursors[id].object = id-1;
      precursors[id].state = 0;
    }

    // Loop over possible immediate precursors to query object (including "no precursors")
    while (true) {
      ostringstream s("");
      s << hierarchyObjects[object];
      int i = object;
      while (precursors[i].object != -1) {
	s << (precursors[i].state == 0 ? " on " : " in ") << 
	  hierarchyObjects[precursors[i].object];
	i = precursors[i].object;
      }
      s << " in room " << fixedRoom << ": ";
      log("%s", s.str().c_str());
      string tmp = s.str();

      int combinations = 0;
      double totalProbMass = 0.0;

      bool fixedRoomTypeVals[nRooms]; // Not currently used
      bool fixedObjectInRoomVals[nObjects];
      bool fixedObjectInObjectVals[nObjects*nObjects];
      bool fixedObjectOnObjectVals[nObjects*nObjects];
      bool fixedObjectDirectlyOnObjectVals[nObjects];
      for (int j = 0; j < nObjects; j++) {
	fixedObjectInRoomVals[j] = false;
	objectInRoomVal[j] = 0;
	fixedObjectDirectlyOnObjectVals[j] = false;
	objectDirectlyOnObjectVal[j] = -1;
	for (int k = 0; k < nObjects; k++) {
	  fixedObjectInObjectVals[j * nObjects + k] = false;
	  fixedObjectOnObjectVals[j * nObjects + k] = false;
	  objectOnObjectVal[j * nObjects + k] = 0;
	  objectInObjectVal[j * nObjects + k] = 0;
	}
      }
      for (int j = 0; j < nRooms; j++) {
	fixedRoomTypeVals[j] = false;
      }

      bool skipThis = false;

      // Fix query object to be in fixedRoom
      fixedObjectInRoomVals[object] = true;
      objectInRoomVal[object] = fixedRoom;
      if (objectInRoomDefault[object * nRoomCategories + roomTypeVal[fixedRoom]] == 0.0)
	skipThis = true;

      i = object;
      while (precursors[i].object != -1) {
	fixedObjectInObjectVals[i * nObjects + precursors[i].object] = true;
	fixedObjectDirectlyOnObjectVals[i] = true;
	if (precursors[i].state == 0) {
	  fixedObjectOnObjectVals[i * nObjects + precursors[i].object] = true;
	  objectDirectlyOnObjectVal[i] = precursors[i].object;
	  objectOnObjectVal[i * nObjects + precursors[i].object] = 1;
	  objectInObjectVal[i * nObjects + precursors[i].object] = 0;
	  if (objectOnObjectDefault[i * nObjects + precursors[i].object] == 0.0 ||
objectInObjectDefault[i * nObjects + precursors[i].object] == 1.0) {
	    skipThis = true;
	  }
	}
	else {
	  objectDirectlyOnObjectVal[i] = -1;
	  objectInObjectVal[i * nObjects + precursors[i].object] = 1;
	  if (objectInObjectDefault[i * nObjects + precursors[i].object] == 0.0) {
	    skipThis = true;
	  }
	}
	i = precursors[i].object;
      }

      if (!skipThis) {
	//      // Fixes all relations involving the unrelated trajector to 0
	//      for (int j = i-1; j >= 0; j--) {
	//	fixedObjectInObjectVals[i * nObjects + j] = true;
	//	fixedObjectOnObjectVals[i * nObjects + j] = true;
	//      }


	while (true) {
	  bool skipThis = false;

	  double prob = probabilityOfConfig(roomTypeVal, objectInRoomVal,
	      objectInObjectVal, objectOnObjectVal, objectDirectlyOnObjectVal);
	  if (prob > 0.0)
	    combinations++;

	  totalProbMass += prob;

	  int index = 0;

	  while (index < nRooms) {
	    if (fixedRoomTypeVals[index]) {
	      index++;
	    }
	    else {
	      roomTypeVal[index]++;
	      if (roomTypeVal[index] == nRoomCategories) {
		roomTypeVal[index] = 0;
		index++;
	      }
	      else {
		break;
	      }
	    }
	  }

	  if (index < nRooms) 
	    continue;

	  index = 0;
	  int index2 = 1;
	  while (index < nObjects) {
	    if (index2 >= nObjects) {
	      index++;
	      index2 = index+1;
	    }
	    else {
	      if (objectInRoomVal[index] != objectInRoomVal[index2] ||
		  fixedObjectInObjectVals[index2 * nObjects + index]) {
		index2++;
	      }
	      else {
		objectInObjectVal[index2 * nObjects + index]++;

		if (objectInObjectVal[index2 * nObjects + index] == 2) {
		  objectInObjectVal[index2 * nObjects + index] = 0;
		  index2++;
		}
		else {
		  break;
		}
	      }
	    }
	  }


	  if (index < nObjects) 
	    continue;

//	  	for (int i = 0; i < nRooms; i++) 
//	  	  cout <<roomTypeVal[i];
//	  	cout << " ";
//	  	for (int i = 0; i < nObjects; i++) 
//	  	  cout <<objectInRoomVal[i];
//	  	cout << " ";
//	  	for (int i = 0; i < nObjects*nObjects; i++) 
//	  	  cout <<objectInObjectVal[i];
//	  	cout << " ";
//	  	for (int i = 0; i < nObjects*nObjects; i++) 
//	  	  cout <<objectOnObjectVal[i];
//	  	cout << " ";
//	  	for (int i = 0; i < nObjects; i++) 
//	  	  cout <<objectDirectlyOnObjectVal[i];
//	  	cout << "\n";

	  index = 0;
	  index2 = 1;
	  while (index < nObjects) {
	    if (index2 >= nObjects) {
	      index++;
	      index2 = index+1;
	    }
	    else {
	      if (objectInRoomVal[index] != objectInRoomVal[index2] ||
		  fixedObjectOnObjectVals[index2 * nObjects + index]) {
		index2++;
	      }
	      else {
		objectOnObjectVal[index2 * nObjects + index]++;

		if (objectOnObjectVal[index2 * nObjects + index] == 2) {
		  objectOnObjectVal[index2 * nObjects + index] = 0;
		  index2++;
		}
		else {
		  break;
		}
	      }
	    }
	  }

	  if (index < nObjects) 
	    continue;

	  index = 0;
	  while (index < nObjects) {
	    if (fixedObjectInRoomVals[index]) {
	      index++;
	    }
	    else {
	      objectInRoomVal[index]++;

	      if (objectInRoomVal[index] == nRooms) {
		objectInRoomVal[index] = 0;
		index++;
	      }
	      else {
		break;
	      }
	    }
	  }

	  if (index < nObjects) 
	    continue;

	  index = 0;
	  while (index < nObjects) {
	    if (fixedObjectDirectlyOnObjectVals[index]) {
	      index++;
	    }
	    else {
	      // Skip past values of objectDirectlyOnObjectVal
	      // that violate objectOnObjectVal
	      do {
		objectDirectlyOnObjectVal[index]++;
		if (objectDirectlyOnObjectVal[index] >= index)
		  break;
	      } while (objectOnObjectVal[index * nObjects + 
		  objectDirectlyOnObjectVal[index]] == 0 ||
		  objectOnObjectDefault[index * nObjects +
		  objectDirectlyOnObjectVal[index]] == 0);

	      //	      if (objectDirectlyOnObjectVal[index] == nObjects) 
	      if (objectDirectlyOnObjectVal[index] >= index) 
	      { 
		// We don't need to go farther; no object is allowed
		//to be on an object with a larger id than itself
		objectDirectlyOnObjectVal[index] = -1;
		index++;
	      }
	      else {
		break;
	      }
	    }
	  }

	  if (index == nObjects) 
	    break;
	};

	ret.push_back(s.str());
	probs.push_back(totalProbMass);
	totalOverallMass += totalProbMass;

	s << totalProbMass << ", " << combinations << " combinations\n";
	//      log("%s", s.str().c_str());
      }
      else {
	log("Skipped");
      }

      // Recursively find the precursor that should be incremented
      bool done = updateHierarchy(precursors, object);
      if (done)  {
	break;
      }
    }
  }

  for (unsigned int i = 0; i < ret.size(); i++) {
    probs[i] /= totalOverallMass;
    ostringstream s;
    s << ret[i] << probs[i];
    ret[i] = s.str();
  }

  return ret;
}

double 
ObjectRelationManager::probabilityOfConfig(int *roomTypeVal, int *objectInRoomVal, 
    int *objectInObjectVal, int *objectOnObjectVal, int *objectDirectlyOnObjectVal)
{
  double prob = 1.0;
  //First, check functions that represent 1/0 values.
  for (int o1 = nObjects-1; o1 >= 0; o1--) {
    //Object-in-room exclusivity

    int inRoom = objectInRoomVal[o1];
    if (inRoom < 0) {
      // Must be in exactly 1 room
      return 0;
    }

    int category = roomTypeVal[inRoom];

    prob *= objectInRoomDefault[o1 * nRoomCategories + category];

    for (int o2 = o1-1; o2 >= 0; o2--) {
      //Object onness basic check
      if (objectOnObjectVal[o1 * nObjects + o2] == 0 &&
	  objectDirectlyOnObjectVal[o1] == o2) {
	return 0;
      }

      bool o1Ino2 = (objectInObjectVal[o1 * nObjects + o2] != 0);
      bool o1Ono2 = (objectOnObjectVal[o1 * nObjects + o2] != 0);

      if ( o1Ino2 != 0 ||
	   o1Ono2 != 0) {

	//Object inness transitivity/exchangeability
	for (int o3 = o2-1; o3 >= 0; o3--) {
	  // If A in/on B, then A in C iff B in C
	  if (objectInObjectVal[o1 * nObjects + o3] !=
	      objectInObjectVal[o2 * nObjects + o3]) {
	    return 0;
	  }

	  // If A in/on B, then A on C iff B on C
	  if (objectOnObjectVal[o1 * nObjects + o3] !=
	      objectOnObjectVal[o2 * nObjects + o3]) {
	    return 0;
	  }
	}

	// If A in/on B, then A in R iff B in R
	if (objectInRoomVal[o1] != objectInRoomVal[o2]) {
	    return 0;
	}
      }

      if (o1Ino2) {
	prob *= objectInObjectDefault[o1 * nObjects + o2];
      }
      else {
	prob *= (1-objectInObjectDefault[o1 * nObjects + o2]);
      }
      if (o1Ono2) {
	prob *= objectOnObjectDefault[o1 * nObjects + o2];
      }
      else {
	prob *= (1-objectOnObjectDefault[o1 * nObjects + o2]);
      }

      // Direct support requirement check
      if (objectOnObjectVal[o1 * nObjects + o2] != 0) {
	bool found = false;
	int supports = 0;

	// Check that there is *at least one* object that o1
	// object is on or in, and that that object is in turn
	// on o2
	for (int o3 = o1-1; o3 > o2; o3--) {
	  if ((objectDirectlyOnObjectVal[o1] == o3 ||
	      objectInObjectVal[o1 * nObjects + o3] != 0) &&
	    objectOnObjectVal[o3 * nObjects + o2]) {
	    found = true;
	  }
	}
	// Unique direct support check
	for (int o3 = o1-1; o3 >= 0; o3--) {
	  if (objectDirectlyOnObjectVal[o1] == o3) {
	    supports++;
	    found = true;
	  }
	}
	if (!found) {
	  return 0;
	}
	if (supports > 1) {
	  return 0;
	}
      }
    }
  }

  for (int r1 = 0; r1 < nRooms; r1++) {
    prob *= roomCategoryDefault[r1 * nRoomCategories + roomTypeVal[r1]];
  }

  return prob;
}

FrontierInterface::StringSeq
ObjectRelationManager::RelationServer::getObjectRelationProbabilities(const string &object)
{
  m_pOwner->lockComponent();
  vector<string> ret = m_pOwner->computeMarginalDistribution(object);
  m_pOwner->unlockComponent();
  return ret;
}
