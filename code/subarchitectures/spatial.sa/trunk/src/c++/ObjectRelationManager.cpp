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

namespace spatial {
double squareDistanceWeight			= 1.0;
// Square distance at which onness drops by half
double squareDistanceFalloff			= 0.03; 

double supportCOMContainmentOffset		= 0.5;
double supportCOMContainmentWeight		= 1.0;
double supportCOMContainmentSteepness		= 5.0;

double bottomCOMContainmentOffset		= 0.0;
double bottomCOMContainmentWeight		= 1.0;
double bottomCOMContainmentSteepness		= 1.0;

double planeInclinationWeight			= 1.0;

double overlapWeight				= 1.0;
};

ObjectRelationManager::ObjectRelationManager()
{
  m_maxObjectCounter = 0;
}

ObjectRelationManager::~ObjectRelationManager() 
{ 
  for (map<int, spatial::Object*>::iterator it = m_objectModels.begin();
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

  m_bTestOnness = false;
  it = _config.find("--test-onness");
  if (it != _config.end()) {
    m_bTestOnness = true;
  }

  m_bNoPTZ = false;
  it = _config.find("--no-ptz");
  if (it != _config.end()) {
    m_bNoPTZ = true;
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

  log("ObjectRelationManager started");
}

void 
ObjectRelationManager::newRobotPose(const cdl::WorkingMemoryChange &objID) 
{
  try {
    lastRobotPose =
      getMemoryEntry<NavData::RobotPose2d>(objID.address);
  }
  catch (DoesNotExistOnWMException e) {
    log("Error! robotPose missing on WM!");
    return;
  }
}

void ObjectRelationManager::runComponent() 
{
  log("I am running!");

  peekabot::GroupProxy root;
  if (m_bDisplayPlaneObjectsInPB || m_bDisplayVisualObjectsInPB || m_bTestOnness) {
    while(!m_PeekabotClient.is_connected() && (m_RetryDelay > -1)){
      sleep(m_RetryDelay);
      connectPeekabot();
    }

    root.assign(m_PeekabotClient, "root");
    if (m_bDisplayPlaneObjectsInPB) {
      m_planeProxies.add(root, "plane_objects", peekabot::REPLACE_ON_CONFLICT);
    }
    if (m_bDisplayPlaneObjectsInPB || m_bTestOnness) {
      m_objectProxies.add(root, "visual_objects", peekabot::REPLACE_ON_CONFLICT);
    }
    if (m_bTestOnness) {
      m_onnessTester.add(root, "on-ness_tester", peekabot::REPLACE_ON_CONFLICT);
    }
    println("Connected to peekabot, ready to go");
  }


  if (m_bTestOnness) {

    PlaneObject table1;

    table1.type = OBJECT_PLANE;

    Matrix33 rotation;
    double rotAngle = 0.0;
    fromAngleAxis(rotation, rotAngle, vector3(0.0, 0.0, 1.0));
    table1.pose = pose3(vector3(0.0, 0.0, 1.0), rotation);

    table1.shape = PLANE_OBJECT_RECTANGLE;
    table1.radius1 = 0.5;
    table1.radius2 = 0.5;

    peekabot::GroupProxy sliders;
    sliders.add(m_onnessTester, "weights", peekabot::REPLACE_ON_CONFLICT);

    peekabot::SphereProxy sqdp;
    sqdp.add(sliders, "squareDistance", peekabot::REPLACE_ON_CONFLICT);
    sqdp.translate(-1.0, 6.0, squareDistanceWeight);
    sqdp.set_scale(0.1);
    peekabot::SphereProxy scwp;
    scwp.add(sliders, "supportEdge", peekabot::REPLACE_ON_CONFLICT);
    scwp.translate(0.0, 6.0, supportCOMContainmentWeight);
    scwp.set_scale(0.1);
    peekabot::SphereProxy bcwp;
    bcwp.add(sliders, "bottomEdge", peekabot::REPLACE_ON_CONFLICT);
    bcwp.translate(1.0, 6.0, bottomCOMContainmentWeight);
    bcwp.set_scale(0.1);
    peekabot::SphereProxy pip;
    pip.add(sliders, "planeInclination", peekabot::REPLACE_ON_CONFLICT);
    pip.translate(2.0, 6.0, planeInclinationWeight);
    pip.set_scale(0.1);
    peekabot::SphereProxy op;
    op.add(sliders, "overlap", peekabot::REPLACE_ON_CONFLICT);
    op.translate(3.0, 6.0, overlapWeight);
    op.set_scale(0.1);

    peekabot::CubeProxy dfp;
    dfp.add(sliders, "distanceFalloff", peekabot::REPLACE_ON_CONFLICT);
    dfp.translate(-1.0, 6.0, squareDistanceFalloff);
    dfp.set_scale(0.1);
    peekabot::CubeProxy csp;
    csp.add(sliders, "containmentSteepness", peekabot::REPLACE_ON_CONFLICT);
    csp.translate(0.0, 6.0, supportCOMContainmentSteepness);
    csp.set_scale(0.1);
    peekabot::CubeProxy cop;
    cop.add(sliders, "containmentOffset", peekabot::REPLACE_ON_CONFLICT);
    cop.translate(0.0, 6.0, supportCOMContainmentOffset);
    cop.set_scale(0.1);

    peekabot::PolygonProxy pp;
    pp.add(m_onnessTester, "table", peekabot::REPLACE_ON_CONFLICT);
    pp.add_vertex(table1.radius1, table1.radius2, 0);
    pp.add_vertex(-table1.radius1, table1.radius2, 0);
    pp.add_vertex(-table1.radius1, -table1.radius2, 0);
    pp.add_vertex(table1.radius1, -table1.radius2, 0);

    pp.translate(table1.pose.pos.x, table1.pose.pos.y, table1.pose.pos.z);
    pp.rotate(rotAngle, 0.0, 0.0, 1.0);


    peekabot::CubeProxy bp;
    bp.add(m_onnessTester, "box", peekabot::REPLACE_ON_CONFLICT);
    bp.translate(table1.pose.pos.x, table1.pose.pos.y, table1.pose.pos.z + 0.5);
    bp.set_scale(0.4, 0.4, 0.4);

    while (isRunning()) {
      peekabot::Result<peekabot::Vector3f> vr;
      vr = sqdp.get_position();
      if (vr.succeeded()) squareDistanceWeight= vr.get_result()(2);
      vr = scwp.get_position();
      if (vr.succeeded()) supportCOMContainmentWeight = vr.get_result()(2);
      vr = bcwp.get_position();
      if (vr.succeeded()) bottomCOMContainmentWeight = vr.get_result()(2);
      vr = pip.get_position();
      if (vr.succeeded()) planeInclinationWeight = vr.get_result()(2);
      vr = op.get_position();
      if (vr.succeeded()) overlapWeight = vr.get_result()(2);

      vr = dfp.get_position();
      if (vr.succeeded()) squareDistanceFalloff = vr.get_result()(2);
      vr = csp.get_position();
      if (vr.succeeded()) {
	bottomCOMContainmentSteepness = 
	  supportCOMContainmentSteepness = vr.get_result()(2);
      }
      vr = cop.get_position();
      if (vr.succeeded()) {
	bottomCOMContainmentOffset = 
	  supportCOMContainmentOffset = vr.get_result()(2);
      }
	

      peekabot::Result<peekabot::Matrix4f> r;

      r = bp.get_transformation(peekabot::WORLD_COORDINATES);
      if (r.succeeded()) {
	Pose3 boxPose;
	double m[16];
	m[0] = r.get_result()(0,0);
	m[1] = r.get_result()(0,1);
	m[2] = r.get_result()(0,2);
	m[3] = r.get_result()(0,3);
	m[4] = r.get_result()(1,0);
	m[5] = r.get_result()(1,1);
	m[6] = r.get_result()(1,2);
	m[7] = r.get_result()(1,3);
	m[8] = r.get_result()(2,0);
	m[9] = r.get_result()(2,1);
	m[10] = r.get_result()(2,2);
	m[11] = r.get_result()(2,3);
	m[12] = r.get_result()(3,0);
	m[13] = r.get_result()(3,1);
	m[14] = r.get_result()(3,2);
	m[15] = r.get_result()(3,3);

	setRow44(boxPose, m);

	BoxObject box1;

	box1.type = OBJECT_BOX;
	box1.pose = boxPose;
	box1.radius1 = 0.2;
	box1.radius2 = 0.2;
	box1.radius3 = 0.2;

	//	  peekabot::PointCloudProxy pcloud;
	//	  pcloud.add(root,"onpoints", peekabot::REPLACE_ON_CONFLICT);
	//	  for (double x = -1.5; x <= 1.5; x += 0.1) {
	//	    for (double y = -1.5; y <= 1.5; y += 0.1) {
	//	      for (double z = 0; z <= 2.5; z += 0.1) {
	//		box1.pose.pos = vector3(x, y, z);
	//		double onness = evaluateOnness(&table1, &box1);
	//		cout << "(" << x << " " << y << " " << z << "):" << onness << "   ";
	//		if (onness > 0.8) {
	//		  pcloud.add_vertex(x,y,z);
	//		}
	//	      }
	//	      cout << endl;
	//	    }
	//	  }


	//	  cout << "Onness: " << evaluateOnness(&table1, &box1);

	peekabot::SphereProxy sp;
	sp.add(m_onnessTester, "Onness", peekabot::REPLACE_ON_CONFLICT);
	sp.translate(0.0, 3.0, 1.0);
	sp.set_scale(evaluateOnness(&table1, &box1));
	peekabot::SphereProxy spm;
	spm.add(m_onnessTester, "Onness-max", peekabot::REPLACE_ON_CONFLICT);
	spm.translate(0.0, 3.0, 1.0);
	spm.set_opacity(0.3);



	//  vector<Vector3> polygon1;
	//  polygon1.push_back(vector3(0.0, 0.0, 1.0));
	//  polygon1.push_back(vector3(1.0, 0.0, 1.0));
	//  polygon1.push_back(vector3(1.0, 1.0, 1.0));
	//  polygon1.push_back(vector3(0.0, 1.0, 1.0));

	//  vector<Vector3> polygon2;
	//  polygon2.push_back(vector3(0.5, 0.5, 1.0));
	//  polygon2.push_back(vector3(1.0, 0.5, 1.0));
	//  polygon2.push_back(vector3(1.0, 1.5, 1.0));
	//  polygon2.push_back(vector3(0.5, 1.5, 1.0));
	//  polygon2.push_back(vector3(0.5, 0.5, 1.0));
	//  polygon2.push_back(vector3(1.5, 0.5, 1.0));
	//  polygon2.push_back(vector3(1.5, 1.5, 1.0));
	//  polygon2.push_back(vector3(0.5, 1.5, 1.0));

	//  vector<Vector3> polygon3;
	//  polygon3.push_back(vector3(0.2, 0.2, 1.0));
	//  polygon3.push_back(vector3(0.8, 0.2, 1.0));
	//  polygon3.push_back(vector3(0.8, 0.8, 1.0));
	//  polygon3.push_back(vector3(0.2, 0.8, 1.0));

	//  cout << getPolygonArea(findPolygonIntersection(polygon1, polygon2)) << endl;
	//  cout << getPolygonArea(
	//      findPolygonIntersection(polygon1, findPolygonIntersection(polygon1, polygon2))) << endl;
	//  cout << getPolygonArea(findPolygonIntersection(polygon1, polygon3)) << endl;
	//  cout << getPolygonArea(findPolygonIntersection(polygon2, polygon3)) << endl;
	//  cout << getPolygonArea(findPolygonIntersection(polygon2, polygon1)) << endl;
	//  cout << getPolygonArea(findPolygonIntersection(polygon2, polygon3)) << endl;
	//  cout << getPolygonArea(findPolygonIntersection(polygon3, polygon1)) << endl;
	//  cout << getPolygonArea(findPolygonIntersection(polygon3, polygon2)) << endl;

	//  cout << findOverlappingArea(polygon1, vector3(0.0, 0.0, 1.0), 0.5) << endl;
	//  cout << findOverlappingArea(polygon1, vector3(0.0, 0.0, 1.0), 1.0) << endl;
	//  cout << findOverlappingArea(polygon1, vector3(0.5, 0.5, 1.0), 0.2) << endl;
	//  cout << findOverlappingArea(polygon1, vector3(0.5, 0.5, 1.0), 4.0) << endl;
	//  cout << findOverlappingArea(polygon1, vector3(0.5, 0.5, 1.0), 0.5) << endl;
	//  cout << findOverlappingArea(polygon1, vector3(0.5, 0.5, 1.0), sqrt(0.5)) << endl;
	//  cout << findOverlappingArea(polygon1, vector3(0.5, 0.5, 1.0), 0.5001) << endl;
	//  while(isRunning()){
	//    usleep(250000);
	//  }
      }
      sleepComponent(500);
    }
  }
}

void
ObjectRelationManager::newObject(const cast::cdl::WorkingMemoryChange &wmc)
{
  try {
    VisionData::VisualObjectPtr observedObject =
      getMemoryEntry<VisionData::VisualObject>(wmc.address);

    log("Got VisualObject: %s", observedObject->label.c_str());

    Pose3 pose = observedObject->pose;
    //Get robot pose
    Pose3 robotTransform;
    if (lastRobotPose != 0) {
      fromRotZ(robotTransform.rot, lastRobotPose->theta);

      robotTransform.pos.x = lastRobotPose->x;
      robotTransform.pos.y = lastRobotPose->y;
    }
    transform(robotTransform, pose, pose);

    // For now, assume each label represents a unique object
    int objectID = -1;
    for (map<int, SpatialObjectPtr>::iterator it = m_objects.begin(); it != m_objects.end(); it++) {
      if (it->second->label == observedObject->label) {
	// update object
	objectID = it->first;
	log("Updating object %i(%s)", objectID, observedObject->label.c_str());
	break;
      }
    }
    bool bNewObject = false;
    if (objectID == -1) {
      // New object
      bNewObject = true;
      log("New SpatialObject: %s", observedObject->label.c_str());
      objectID = m_maxObjectCounter++;
      m_objects[objectID] = new SpatialData::SpatialObject;
      m_objects[objectID]->id = objectID;
      m_objects[objectID]->label = observedObject->label;
      m_objects[objectID]->pose = observedObject->pose;

      BoxObject *newBoxObject = new BoxObject;
      m_objectModels[objectID] = newBoxObject;
      newBoxObject->type = OBJECT_BOX;
      if (observedObject->label == "krispies") {
	newBoxObject->radius1 = 0.095;
	newBoxObject->radius2 = 0.045;
	newBoxObject->radius3 = 0.145;
      }
      else if (observedObject->label == "joystick") {
	newBoxObject->radius1 = 0.115;
	newBoxObject->radius2 = 0.105;
	newBoxObject->radius3 = 0.13;
      }
      else  {
	newBoxObject->radius1 = 0.1;
	newBoxObject->radius2 = 0.1;
	newBoxObject->radius3 = 0.1;
      }
    }
//    log("2");

    double diff = length(m_objects[objectID]->pose.pos - pose.pos);
    diff += length(getRow(m_objects[objectID]->pose.rot - pose.rot, 1));
    diff += length(getRow(m_objects[objectID]->pose.rot - pose.rot, 2));
    if (diff > 0.01 || bNewObject) {
      //      log("3");
      m_objects[objectID]->pose = pose;
      m_lastObjectPoseTimes[objectID] = observedObject->time;

      if (m_objectWMIDs.find(objectID) == m_objectWMIDs.end()) {
	string newID = newDataID();

	addToWorkingMemory<SpatialData::SpatialObject>(newID, m_objects[objectID]);
	m_objectWMIDs[objectID]=newID;
      }
      else {

	try {
	  overwriteWorkingMemory<SpatialData::SpatialObject>(m_objectWMIDs[objectID],
	      m_objects[objectID]);
	}
	catch (DoesNotExistOnWMException) {
	  log("Error! SpatialObject disappeared from memory!");
	  return;
	}
      }
      //    log("4");

      if (m_bDisplayVisualObjectsInPB && m_objectProxies.is_assigned()) {
	if (m_objectModels[objectID]->type == OBJECT_BOX) {
	  BoxObject *box = (BoxObject*)m_objectModels[objectID];
	  peekabot::CubeProxy theobjectproxy;
	  peekabot::GroupProxy root;
	  root.assign(m_PeekabotClient, "root");
	  theobjectproxy.add(m_objectProxies, m_objects[objectID]->label, peekabot::REPLACE_ON_CONFLICT);
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
      recomputeOnnessForObject(objectID);


      if (m_placeInterface != 0) {
	// Check degree of containment in Places
	FrontierInterface::PlaceMembership membership = 
	  m_placeInterface->getPlaceMembership(pose.pos.x, pose.pos.y);

	setContainmentProperty(objectID, membership.placeID, 1.0);
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

    log("Read object at %f, %f, %f", observedObject->pos.x, observedObject->pos.y,
	observedObject->pos.z);

    int objectID = -1;
    for (map<int, FrontierInterface::ObservedPlaneObjectPtr>::iterator it = m_planeObjects.begin(); it != m_planeObjects.end(); it++) {
      if (it->second->id == observedObject->id) {
	// update object
	objectID = it->first;
	break;
      }
    }

    if (objectID == -1)  {
      // New plane object
      objectID = observedObject->id;
      m_planeObjects[objectID] = observedObject;
    }

    double diff = length(m_planeObjects[objectID]->pos - observedObject->pos);
    double angDiff = m_planeObjects[objectID]->angle - observedObject->angle;
    if (angDiff > M_PI) angDiff -= 2*M_PI;
    if (angDiff < -M_PI) angDiff += 2*M_PI;
    diff += abs(angDiff*1.0);

    if (diff > 0.01) {
      m_planeObjects[objectID]->pos = observedObject->pos;
      m_planeObjects[objectID]->angle = observedObject->angle;

      recomputeOnnessForPlane(objectID);

      if (m_bDisplayPlaneObjectsInPB) {
	if (m_PeekabotClient.is_connected()) {
	  char identifier[100];
	  sprintf(identifier, "label%d", 0);
	  peekabot::PolygonProxy pp;
	  pp.add(m_planeProxies, identifier, peekabot::REPLACE_ON_CONFLICT);
	  pp.add_vertex(0.55, 0.45, 0);
	  pp.add_vertex(-0.55, 0.45, 0);
	  pp.add_vertex(-0.55, -0.45, 0);
	  pp.add_vertex(0.55, -0.45, 0);

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
ObjectRelationManager::setContainmentProperty(int objectID, int placeID, double confidence)
{
  map<int, PlaceContainmentObjectPropertyPtr>::iterator it =
    m_containmentProperties.find(objectID);
  SpatialProperties::PlaceContainmentObjectPropertyPtr containmentProp;
  if (it == m_containmentProperties.end()) {
    //New containment property for this Object

    containmentProp =
      new SpatialProperties::PlaceContainmentObjectProperty;
    m_containmentProperties[objectID] = containmentProp;
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
  containmentProp->objectId = objectID;
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

    m_PeekabotClient.connect(m_PbHost, m_PbPort, true);

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
ObjectRelationManager::recomputeOnnessForObject(int objectID)
{
  if (m_objectModels.find(objectID) == m_objectModels.end()) {
    log("Error! Object model was missing!");
    return;
  }
  if (m_objects.find(objectID) == m_objects.end()) {
    log("Error! Object was missing!");
    return;
  }

//  log("1");

  m_objectModels[objectID]->pose = m_objects[objectID]->pose;

//  log("2");

  for (map<int, FrontierInterface::ObservedPlaneObjectPtr>::iterator it = m_planeObjects.begin(); 
      it != m_planeObjects.end(); it++) {
    PlaneObject po;
    po.pose.pos = it->second->pos;
    fromAngleAxis(po.pose.rot, it->second->angle, vector3(0.0, 0.0, 1.0));
    if (it->second->label == "planeObject0") {
      po.radius1 = 0.45;
      po.radius2 = 0.55;
    }
    else {
      log("Error! Unknown plane object!");
      return;
    }
//    log("3");
    double onness = evaluateOnness(&po, m_objectModels[objectID]);
//    log("4");
    log("Object %s on object %s is %f", m_objects[objectID]->label.c_str(), 
	it->second->label.c_str(), onness);
  }
}

void
ObjectRelationManager::recomputeOnnessForPlane(int planeObjectID)
{
  if (m_planeObjects.find(planeObjectID) == m_planeObjects.end()) {
    log("Error! Plane object missing!");
    return;
  }

  PlaneObject po;
  po.pose.pos = m_planeObjects[planeObjectID]->pos;
  fromAngleAxis(po.pose.rot, m_planeObjects[planeObjectID]->angle, 
      vector3(0.0, 0.0, 1.0));
  if (m_planeObjects[planeObjectID]->label == "planeObject0") {
    po.radius1 = 0.45;
    po.radius2 = 0.55;
  }
  else {
    log("Error! Unknown plane object!");
    return;
  }

  for (map<int, SpatialData::SpatialObjectPtr>::iterator it = m_objects.begin(); 
      it != m_objects.end(); it++) {
    int objectID = it->second->id;

    if (m_objectModels.find(objectID) == m_objectModels.end()) {
      log("Error! Object model was missing!");
    }

    m_objectModels[objectID]->pose = m_objects[objectID]->pose;
    double onness = evaluateOnness(&po, m_objectModels[objectID]);
    log("Object %s on object %s is %f", m_objects[objectID]->label.c_str(), 
	m_planeObjects[planeObjectID]->label.c_str(), onness);
  }
}
