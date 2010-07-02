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

#define USE_KDE

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


ObjectRelationManager::ObjectRelationManager()
{
  m_bRecognitionIssuedThisStop = false;
  m_maxObjectCounter = 0;
  m_standingStillThreshold = 0.2;
  m_recognitionTimeThreshold = 5.0;
  m_trackerTimeThreshold = 3.0;
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

  m_bTestOnness = false;
  m_bSampleOnness = false;
  it = _config.find("--test-onness");
  if (it != _config.end()) {
    m_bTestOnness = true;
    it = _config.find("--sample-onness");
    if (it != _config.end()) {
      m_bSampleOnness = true;
    }
  }

  m_bTestInness = false;
  m_bSampleInness = false;
  it = _config.find("--test-inness");
  if (it != _config.end()) {
    m_bTestInness = true;
    it = _config.find("--sample-inness");
    if (it != _config.end()) {
      m_bSampleInness = true;
    }
  }

  m_bDemoSampling = false;
  it = _config.find("--demo-sampling");
  if (it != _config.end()) {
    m_bDemoSampling = true;
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

  if (m_bDisplayPlaneObjectsInPB || m_bDisplayVisualObjectsInPB || m_bTestOnness
      || m_bTestInness) {
    while(!m_PeekabotClient.is_connected() && (m_RetryDelay > -1)){
      sleep(m_RetryDelay);
      connectPeekabot();
    }

    peekabot::GroupProxy root;
    root.assign(m_PeekabotClient, "root");
    if (m_bDisplayPlaneObjectsInPB) {
      m_planeProxies.add(root, "plane_objects", peekabot::REPLACE_ON_CONFLICT);
    }
    if (m_bDisplayPlaneObjectsInPB || m_bTestOnness || m_bTestInness) {
      m_objectProxies.add(root, "visual_objects", peekabot::REPLACE_ON_CONFLICT);
    }
    if (m_bTestOnness) {
      m_relationTester.add(root, "on-ness_tester", peekabot::REPLACE_ON_CONFLICT);
    }
    else if (m_bTestInness) {
      m_relationTester.add(root, "in-ness_tester", peekabot::REPLACE_ON_CONFLICT);
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
      addTrackerCommand(VisionData::REMOVEMODEL, "joystick");
      addTrackerCommand(VisionData::REMOVEMODEL, "krispies");
      addTrackerCommand(VisionData::REMOVEMODEL, "rice");
      addTrackerCommand(VisionData::REMOVEMODEL, "printer");
    }
    m_bRecognitionIssuedThisStop = false;
  }
  else if (deltaT > 0.0) {
    double diff = lastRobotPose->time.s + lastRobotPose->time.us*0.000001 - oldTime;
    m_timeSinceLastMoved += diff;
    //    log("time %f %f %f", diff, oldTime, m_timeSinceLastMoved);
  }
}

void ObjectRelationManager::runComponent() 
{
  log("I am running!");

  peekabot::GroupProxy root;
  if (m_bDisplayPlaneObjectsInPB || m_bDisplayVisualObjectsInPB || m_bTestOnness 
      || m_bTestInness) {
    root.assign(m_PeekabotClient, "root");
  }

  peekabot::SphereProxy sqdp;
  peekabot::SphereProxy scwp;
  peekabot::SphereProxy bcwp;
  peekabot::SphereProxy op;
  peekabot::CubeProxy csp;
  peekabot::CubeProxy cop;
  peekabot::PolygonProxy pp;
  peekabot::CubeProxy bp;
  peekabot::CubeProxy bp2;
  PlaneObject table1;

  peekabot::SphereProxy sp;
  peekabot::SphereProxy spm;
  peekabot::SphereProxy sp2;
  peekabot::SphereProxy spm2;

  if (m_bTestOnness || m_bTestInness) {
    table1.type = OBJECT_PLANE;

    Matrix33 rotation;
    double rotAngle = 0.0;
    fromAngleAxis(rotation, rotAngle, vector3(0.0, 0.0, 1.0));
    table1.pose = pose3(vector3(0.0, 0.0, 1.0), rotation);

    table1.shape = PLANE_OBJECT_RECTANGLE;
    table1.radius1 = 0.5;
    table1.radius2 = 0.5;

    pp.add(m_relationTester, "table", peekabot::REPLACE_ON_CONFLICT);
    pp.add_vertex(table1.radius1, table1.radius2, 0);
    pp.add_vertex(-table1.radius1, table1.radius2, 0);
    pp.add_vertex(-table1.radius1, -table1.radius2, 0);
    pp.add_vertex(table1.radius1, -table1.radius2, 0);

    pp.translate(table1.pose.pos.x, table1.pose.pos.y, table1.pose.pos.z);
    pp.rotate(rotAngle, 0.0, 0.0, 1.0);


    bp.add(m_relationTester, "krispies", peekabot::REPLACE_ON_CONFLICT);
    bp.translate(table1.pose.pos.x, table1.pose.pos.y, table1.pose.pos.z + 0.26+0.145);
    bp.set_scale(0.19, 0.09, 0.29);
    bp.set_opacity(0.5);

    bp2.add(m_relationTester, "joystick", peekabot::REPLACE_ON_CONFLICT);
    bp2.translate(table1.pose.pos.x, table1.pose.pos.y, table1.pose.pos.z + 0.13);
    bp2.set_scale(0.23, 0.21, 0.26);
    bp2.set_opacity(0.5);

    if (m_bTestOnness) {
      peekabot::GroupProxy sliders;
      sliders.add(m_relationTester, "weights", peekabot::REPLACE_ON_CONFLICT);

      sqdp.add(sliders, "squareDistanceOutside", peekabot::REPLACE_ON_CONFLICT);
      sqdp.translate(-1.0, 6.0, 10*distanceFalloffOutside);
      sqdp.set_scale(0.1);
      scwp.add(sliders, "squareDistanceInside", peekabot::REPLACE_ON_CONFLICT);
      scwp.translate(0.0, 6.0, 10*distanceFalloffInside);
      scwp.set_scale(0.1);
      bcwp.add(sliders, "patchThreshold", peekabot::REPLACE_ON_CONFLICT);
      bcwp.translate(1.0, 6.0, 10*patchThreshold);
      bcwp.set_scale(0.1);
      csp.add(sliders, "containmentSteepness", peekabot::REPLACE_ON_CONFLICT);
      csp.translate(0.0, 6.0, 10*supportCOMContainmentSteepness);
      csp.set_scale(0.1);
      cop.add(sliders, "containmentOffset", peekabot::REPLACE_ON_CONFLICT);
      cop.translate(0.0, 6.0, supportCOMContainmentOffset);
      cop.set_scale(0.1);

      sp.add(m_relationTester, "Onness", peekabot::REPLACE_ON_CONFLICT);
      sp.translate(0.0, 3.0, 1.0);
      spm.add(m_relationTester, "Onness-max", peekabot::REPLACE_ON_CONFLICT);
      spm.translate(0.0, 3.0, 1.0);
      spm.set_opacity(0.3);
      sp2.add(m_relationTester, "Onness2", peekabot::REPLACE_ON_CONFLICT);
      sp2.translate(2.0, 3.0, 1.0);
      spm2.add(m_relationTester, "Onness-max2", peekabot::REPLACE_ON_CONFLICT);
      spm2.translate(2.0, 3.0, 1.0);
      spm2.set_opacity(0.3);
    }
    if (m_bTestInness) {
      sp.add(m_relationTester, "Inness", peekabot::REPLACE_ON_CONFLICT);
      sp.translate(0.0, 3.0, 1.0);
      spm.add(m_relationTester, "Inness-max", peekabot::REPLACE_ON_CONFLICT);
      spm.translate(0.0, 3.0, 1.0);
      spm.set_opacity(0.3);
      sp2.add(m_relationTester, "Inness2", peekabot::REPLACE_ON_CONFLICT);
      sp2.translate(2.0, 3.0, 1.0);
      spm2.add(m_relationTester, "Inness-max2", peekabot::REPLACE_ON_CONFLICT);
      spm2.translate(2.0, 3.0, 1.0);
      spm2.set_opacity(0.3);
    }
  }

  sleepComponent(2000);

  while (isRunning()) {
    // Dispatch recognition commands if the robot has been standing still
    // long enough

    if (!m_bNoVision) {
      lockComponent();

      if (!m_bRecognitionIssuedThisStop &&
	  m_timeSinceLastMoved > m_recognitionTimeThreshold) {
	log("Issuing recognition commands");
	addRecognizer3DCommand(VisionData::RECOGNIZE, "joystick", "");
	addRecognizer3DCommand(VisionData::RECOGNIZE, "krispies", "");
	addRecognizer3DCommand(VisionData::RECOGNIZE, "rice", "");
	addRecognizer3DCommand(VisionData::RECOGNIZE, "printer", "");
	m_bRecognitionIssuedThisStop = true;
      }

      unlockComponent();
    }

    if (m_bTestOnness || m_bTestInness) {
    
      
      peekabot::Result<peekabot::Transformation> r;

      r = bp.get_transformation(peekabot::WORLD_COORDINATES);
      if (r.succeeded()) {
	Pose3 boxPose;
	setIdentity(boxPose);
	double m[16];

//	m[0] = r.get_result().x.x; 
//	m[1] = r.get_result().y.x;
//	m[2] = r.get_result().z.x;
//	m[3] = r.get_result().pos.x;
//	m[4] = r.get_result().y.x;
//	m[5] = r.get_result().y.y;
//	m[6] = r.get_result().z.z;
//	m[7] = r.get_result().pos.y;
//	m[8] = r.get_result().z.x;
//	m[9] = r.get_result().z.y; 
//	m[10] = r.get_result().z.z; 
//	m[11] = r.get_result().pos.z; 
//	m[12] = 0;
//	m[13] = 0;
//	m[14] = 0;
//	m[15] = 0;
//
//	setRow44(boxPose, m);

	BoxObject box1;

	box1.type = OBJECT_BOX;
	box1.pose = boxPose;
	box1.radius1 = 0.095;
	box1.radius2 = 0.045;
	box1.radius3 = 0.145;






	r = bp2.get_transformation(peekabot::WORLD_COORDINATES);
	if (r.succeeded()) {
	  Pose3 boxPose;
	  setIdentity(boxPose);
//	  double m[16];
//
//	  m[0] = r.get_result().x.x;
//	  m[1] = r.get_result().y.x;
//	  m[2] = r.get_result().z.x;
//	  m[3] = r.get_result().pos.x;
//	  m[4] = r.get_result().y.x;
//	  m[5] = r.get_result().y.y;
//	  m[6] = r.get_result().z.z;
//	  m[7] = r.get_result().pos.y;
//	  m[8] = r.get_result().z.x;
//	  m[9] = r.get_result().z.y;
//	  m[10] = r.get_result().z.z;
//	  m[11] = r.get_result().pos.z;
//	  m[12] = 0;
//	  m[13] = 0;
//	  m[14] = 0;
//	  m[15] = 0;
//
	  
	 /* m[0] = r.get_result()(0,0);
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
	  m[15] = r.get_result()(3,3);*/

//	  setRow44(boxPose, m);

	  HollowBoxObject box2;

	  box2.type = OBJECT_HOLLOW_BOX;
	  box2.pose = boxPose;
	  box2.radius1 = 0.115;
	  box2.radius2 = 0.105;
	  box2.radius3 = 0.13;
	  box2.thickness = 0.02;


	  if (m_bTestOnness) {
	    peekabot::Result<peekabot::Transformation> vr;
	    vr = sqdp.get_transformation();
//	    if (vr.succeeded()) distanceFalloffOutside= 0.1*vr.get_result().pos.z;
	    vr = scwp.get_transformation();
//	    if (vr.succeeded()) distanceFalloffInside = 0.1*vr.get_result().pos.z;
	    vr = bcwp.get_transformation();
//	    if (vr.succeeded()) patchThreshold = 0.1*vr.get_result().pos.z;
	    vr = csp.get_transformation();
	    if (vr.succeeded()) {
	      //	bottomCOMContainmentSteepness = 
//	      supportCOMContainmentSteepness = 0.1*vr.get_result().pos.z;
	    }
	    vr = cop.get_transformation();
	    if (vr.succeeded()) {
	      //	bottomCOMContainmentOffset = 
//	      supportCOMContainmentOffset = vr.get_result().pos.z;
	    }

	    sp.set_scale(evaluateOnness(&table1, &box1));
	    sp2.set_scale(evaluateOnness(&box2, &box1));

	    vector<Vector3> patch;
	    Witness witness = findContactPatch(box2, box1, &patch);
	    if (patch.size() > 2) {
	      peekabot::PolygonProxy patchp;
	      patchp.add(m_relationTester, "Patch", peekabot::REPLACE_ON_CONFLICT);
	      patchp.set_color(1,0,0);
	      for (vector<Vector3>::iterator it = patch.begin(); it != patch.end();it++){
		patchp.add_vertex(it->x, it->y, it->z);
	      }
	    }
	    //	  peekabot::CylinderProxy normp;
	    //	  normp.add(m_onnessTester, "Normal", peekabot::REPLACE_ON_CONFLICT);
	    //	  normp.set_color(0,0,1);
	    //	  normp.set_scale(0.005, 0.005, 0.1);
	    //	  normp.translate(0.05,0,0);
	    //	  normp.set_orientation(witness.normal.x, witness.normal.y, witness.normal.z);
	    //	  normp.rotate(M_PI/2, 0, 1, 0);
	    //	  normp.translate(witness.point1.x, witness.point1.y, witness.point1.z,
	    //	      peekabot::PARENT_COORDINATES);


	    peekabot::SphereProxy witp1;
	    witp1.add(m_relationTester, "Witness 1", peekabot::REPLACE_ON_CONFLICT);
	    witp1.translate(witness.point1.x, witness.point1.y, witness.point1.z);
	    witp1.set_scale(0.01);
	    peekabot::SphereProxy witp2;
	    witp2.add(m_relationTester, "Witness 2", peekabot::REPLACE_ON_CONFLICT);
	    witp2.translate(witness.point2.x, witness.point2.y, witness.point2.z);
	    witp2.set_scale(0.01);

	    if (m_bSampleOnness) {
	      static bool sampleTable = false;

	      Cure::LocalGridMap<double> pdf(50, 0.05, 0.0, 
		  Cure::LocalGridMap<double>::MAP1, 0, 0);
	      vector<spatial::Object *>objects;
	      vector<string> objectLabels;
	      vector<spatial::SpatialRelationType> relations;

	      if (sampleTable) {
		objects.push_back(&box2);
		objectLabels.push_back("box2");
		objects.push_back(&box1);
		objectLabels.push_back("box1");
		relations.push_back(RELATION_ON);
		objects.push_back(&table1);
		objectLabels.push_back("table1");
		relations.push_back(RELATION_ON);
	      }
	      else {
		objects.push_back(&box1);
		objectLabels.push_back("box1");
		objects.push_back(&box2);
		objectLabels.push_back("box2");
		relations.push_back(RELATION_ON);
		objects.push_back(&table1);
		objectLabels.push_back("table1");
		relations.push_back(RELATION_ON);
	      }

	      //	    objects.push_back(&box2);
	      //	    objectLabels.push_back("box2");
	      //	    relations.push_back(RELATION_ON);

	      double total = 0.0;
	      vector<cogx::Math::Matrix33> supportObjectOrientations;
	      supportObjectOrientations.push_back(objects.back()->pose.rot); 
	      m_sampler.
		sampleBinaryRelationSystematically(relations, objects, 
		    supportObjectOrientations, 
		    objectLabels, pdf,
		  total, 1.0);

	      sampleTable = !sampleTable;

	      peekabot::LineCloudProxy linecloudp;

	      linecloudp.add(m_PeekabotClient, "root.distribution",
		  peekabot::REPLACE_ON_CONFLICT);
	      linecloudp.clear_vertices();
	      linecloudp.set_color(0.5, 0, 0.5);

	      double maxPDFValue = 0.0;
	      for (int x = -pdf.getSize(); x <= pdf.getSize(); x++) {
		for (int y = -pdf.getSize(); y <= pdf.getSize(); y++) {
		  if (pdf(x,y) > maxPDFValue) {
		    maxPDFValue = pdf(x,y);
		  }
		}
	      }

	      for (int x = -pdf.getSize(); x < pdf.getSize(); x++) {
		for (int y = -pdf.getSize(); y <= pdf.getSize(); y++) {
		  if (pdf(x, y) == 0)
		    continue;
		  double xW2, yW2;
		  double xW3, yW3;
		  pdf.index2WorldCoords(x, y, xW2, yW2);
		  pdf.index2WorldCoords(x+1, y, xW3, yW3);
		  linecloudp.add_line(xW2, yW2, pdf(x, y)/maxPDFValue,
		      xW3, yW3, pdf(x+1, y)/maxPDFValue);
		}
	      }
	      for (int x = -pdf.getSize(); x <= pdf.getSize(); x++) {
		for (int y = -pdf.getSize(); y < pdf.getSize(); y++) {
		  if (pdf(x, y) == 0)
		    continue;
		  double xW2, yW2;
		  double xW3, yW3;
		  pdf.index2WorldCoords(x, y, xW2, yW2);
		  pdf.index2WorldCoords(x, y+1, xW3, yW3);
		  linecloudp.add_line(xW2, yW2, pdf(x, y)/maxPDFValue,
		      xW3, yW3, pdf(x, y+1)/maxPDFValue);
		}
	      }

	    }

	  } // if (m_bTestOnness)

	  if (m_bTestInness) {
	    sp.set_scale(evaluateInness(&table1, &box1));
	    sp2.set_scale(evaluateInness(&box2, &box1));

	    if (m_bSampleInness) {
	      Cure::LocalGridMap<double> pdf(25, 0.05, 0.0, 
		  Cure::LocalGridMap<double>::MAP1, 0, 0);
	      vector<spatial::Object *>objects;
	      vector<string>objectLabels;
	      objects.push_back(&box1);
	      objectLabels.push_back("box1");
	      objects.push_back(&box2);
	      objectLabels.push_back("box2");
	      //	    objects.push_back(&box2);
	      vector<spatial::SpatialRelationType> relations;
	      relations.push_back(RELATION_IN);
	      //	    relations.push_back(RELATION_IN);

	      double total;
	      m_sampler.
		sampleBinaryRelationRecursively(relations, objects, objects.size()-2, pdf,
		  total);
	      peekabot::LineCloudProxy linecloudp;

	      linecloudp.add(m_PeekabotClient, "root.distribution",
		  peekabot::REPLACE_ON_CONFLICT);
	      linecloudp.clear_vertices();
	      linecloudp.set_color(0.5, 0, 0.5);

	      double maxPDFValue = 0.0;
	      for (int x = -pdf.getSize(); x <= pdf.getSize(); x++) {
		for (int y = -pdf.getSize(); y <= pdf.getSize(); y++) {
		  if (pdf(x,y) > maxPDFValue) {
		    maxPDFValue = pdf(x,y);
		  }
		}
	      }

	      for (int x = -pdf.getSize(); x < pdf.getSize(); x++) {
		for (int y = -pdf.getSize(); y <= pdf.getSize(); y++) {
		  if (pdf(x, y) == 0)
		    continue;
		  double xW2, yW2;
		  double xW3, yW3;
		  pdf.index2WorldCoords(x, y, xW2, yW2);
		  pdf.index2WorldCoords(x+1, y, xW3, yW3);
		  linecloudp.add_line(xW2, yW2, pdf(x, y)/maxPDFValue,
		      xW3, yW3, pdf(x+1, y)/maxPDFValue);
		}
	      }
	      for (int x = -pdf.getSize(); x <= pdf.getSize(); x++) {
		for (int y = -pdf.getSize(); y < pdf.getSize(); y++) {
		  if (pdf(x, y) == 0)
		    continue;
		  double xW2, yW2;
		  double xW3, yW3;
		  pdf.index2WorldCoords(x, y, xW2, yW2);
		  pdf.index2WorldCoords(x, y+1, xW3, yW3);
		  linecloudp.add_line(xW2, yW2, pdf(x, y)/maxPDFValue,
		      xW3, yW3, pdf(x, y+1)/maxPDFValue);
		}
	      }
	    }
	  } // if (m_bTestInness)
	}
      }
    }

    sleepComponent(500);
  }
}

void
ObjectRelationManager::newObject(const cast::cdl::WorkingMemoryChange &wmc)
{
  try {
    VisionData::VisualObjectPtr observedObject =
      getMemoryEntry<VisionData::VisualObject>(wmc.address);

    if (m_timeSinceLastMoved > m_trackerTimeThreshold) {
//      log("Got VisualObject: %s (%f,%f,%f)", observedObject->label.c_str(),
//	  observedObject->pose.pos.x,
//	  observedObject->pose.pos.y,
//	  observedObject->pose.pos.z);

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

      string obsLabel = observedObject->label;

      // For now, assume each label represents a unique object
      map<std::string, SpatialObjectPtr>::iterator it = m_objects.find(obsLabel);
      bool bNewObject = false;
      if (it != m_objects.end()) {
	// update object
	//	  log("Updating object %i(%s)", objectID, observedObject->label.c_str());
	m_visualObjectIDs[obsLabel] = wmc.address.id;
      }
      else {
	// New object
	bNewObject = true;
	log("New SpatialObject: %s", obsLabel.c_str());
	m_objects[obsLabel] = new SpatialData::SpatialObject;
	m_objects[obsLabel]->label = obsLabel;
	m_objects[obsLabel]->pose = pose;

	generateNewObjectModel(obsLabel);
      }
      //    log("2");
/*      if (m_bDemoSampling) {
	string supObjectLabel = "squaretable";
	if (observedObject->label == "krispies" ||
	    observedObject->label == "joystick" ||
	    observedObject->label == "rice") {
	  // Evaluate onness for joystick object
      map<std::string, SpatialObjectPtr>::iterator it = m_objects.find(observedObject->label);
      if (it != m_objects.end()) {
	    generateNewObjectModel(observedObject->label);
	  }
	  sampleOnnessForObject(objectID, onObjectID);
	}
	  for (map<string, SpatialObjectPtr>::iterator it = m_objects.begin(); it != m_objects.end(); it++) {
	    if (it->second->label == "krispies") {
	      onObjectID = it->first;
	    }
	  }
	  if (onObjectID == -1) {
	    onObjectID = m_maxObjectCounter++;
	    generateNewObjectModel(onObjectID, "krispies");
	  }
	  sampleOnnessForObject(objectID, onObjectID);
	}*/

      double diff = length(m_objects[obsLabel]->pose.pos - pose.pos);
      diff += length(getRow(m_objects[obsLabel]->pose.rot - pose.rot, 1));
      diff += length(getRow(m_objects[obsLabel]->pose.rot - pose.rot, 2));
      if (diff > 0.01 || bNewObject) {
	//      log("3");
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
	//    log("4");

	if (m_bDisplayVisualObjectsInPB && m_objectProxies.is_assigned()) {
	  if (m_objectModels[obsLabel]->type == OBJECT_BOX) {
	    BoxObject *box = (BoxObject*)m_objectModels[obsLabel];
	    peekabot::CubeProxy theobjectproxy;
	    peekabot::GroupProxy root;
	    root.assign(m_PeekabotClient, "root");
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
	recomputeOnnessForObject(obsLabel);
	recomputeInnessForObject(obsLabel);


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
	  pp.add(m_planeProxies, identifier, peekabot::REPLACE_ON_CONFLICT);
	  pp.add_vertex(planeObject.radius1, planeObject.radius2, 0);
	  pp.add_vertex(-planeObject.radius1, planeObject.radius2, 0);
	  pp.add_vertex(-planeObject.radius1, -planeObject.radius2, 0);
	  pp.add_vertex(planeObject.radius1, -planeObject.radius2, 0);

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

  m_objectModels[label]->pose = m_objects[label]->pose;

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
    double onness = evaluateOnness(&po, m_objectModels[label]);
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
      m_objectModels[supportObjectLabel]->pose = 
	m_objects[supportObjectLabel]->pose;

      double onness = evaluateOnness(m_objectModels[supportObjectLabel],
	  m_objectModels[label]);

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

  m_objectModels[label]->pose = m_objects[label]->pose;

  for (map<string, SpatialObjectPtr>::iterator it = m_objects.begin();
      it != m_objects.end(); it++) {
    string containerObjectLabel = it->first;
    if (containerObjectLabel != label) {

      if (m_objectModels.find(containerObjectLabel) == m_objectModels.end()) {
	log("Error! Container object model was missing!");
	return;
      }
      m_objectModels[containerObjectLabel]->pose =
	m_objects[containerObjectLabel]->pose;

      double inness = evaluateInness(m_objectModels[containerObjectLabel],
	  m_objectModels[label]);

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

    m_objectModels[objectLabel]->pose = m_objects[objectLabel]->pose;
    log("Evaluating object %s on object %s",objectLabel.c_str(), 
	planeLabel.c_str());
    double onness = evaluateOnness(&po, m_objectModels[objectLabel]);
    log("Object %s on object %s is %f", objectLabel.c_str(), 
	planeLabel.c_str(), onness);
  }
}

void
ObjectRelationManager::sampleOnnessForObject(const string &supLabel, 
    const string &onLabel) 
{
  if (m_objectModels.find(supLabel) == m_objectModels.end() ||
      m_objects.find(supLabel) == m_objects.end()) {
    log("Error! Support object model missing!");
    return;
  }
  m_objectModels[supLabel]->pose = 
    m_objects[supLabel]->pose;

  spatial::Object *objectS = m_objectModels[supLabel];

  if (m_objectModels.find(onLabel) == m_objectModels.end()) {
    log("Error! Supported object model missing!");
    return;
  }

  spatial::Object *objectO = m_objectModels[onLabel];

  Pose3 oldPose = objectO->pose;
  vector<Vector3> points;

  BoxObject *supportBox = (BoxObject *)objectS;

  double frameRadius = supportBox->radius1 > supportBox->radius2 ?
    supportBox->radius1 : supportBox->radius2;
  frameRadius = frameRadius > supportBox->radius3 ? 
    frameRadius : supportBox->radius3;
  sampleOnnessDistribution(objectS, objectO, points, 
      objectS->pose.pos.x - frameRadius, objectS->pose.pos.x + frameRadius,
      objectS->pose.pos.y - frameRadius, objectS->pose.pos.y + frameRadius,
      objectS->pose.pos.z - frameRadius, objectS->pose.pos.z + frameRadius*4,
      0.04, 0.02);
  objectO->pose = oldPose;

  peekabot::GroupProxy root;
  root.assign(m_PeekabotClient, "root");

  peekabot::PointCloudProxy pcloud;
  pcloud.add(root,"onpoints", peekabot::REPLACE_ON_CONFLICT);

  for (vector<Vector3>::iterator it = points.begin(); it != points.end();
      it++) {
    pcloud.add_vertex(it->x,it->y,it->z);
  }
}

void
ObjectRelationManager::sampleOnnessForPlane(const string &planeLabel, const string &objectLabel) 
{


  if (m_planeObjects.find(planeLabel) == m_planeObjects.end()) {
    log("Error! Plane object missing!");
    return;
  }

  PlaneObject po;
  po.type = OBJECT_PLANE;
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

  if (m_objectModels.find(objectLabel) == m_objectModels.end()) {
    log("Error! Object model missing!");
    return;
  }

  spatial::Object *objectO = m_objectModels[objectLabel];

  Pose3 oldPose = objectO->pose;
  vector<Vector3> points;
  double frameRadius = 1.5 * 
    (po.radius1 > po.radius2 ? po.radius1 : po.radius2);
  sampleOnnessDistribution(&po, objectO, points, 
      po.pose.pos.x - frameRadius, po.pose.pos.x + frameRadius,
      po.pose.pos.y - frameRadius, po.pose.pos.y + frameRadius,
      0, 1.5, 
      0.1, 0.025);
  objectO->pose = oldPose;

  peekabot::GroupProxy root;
  root.assign(m_PeekabotClient, "root");

  peekabot::PointCloudProxy pcloud;
  pcloud.add(root,"onpoints", peekabot::REPLACE_ON_CONFLICT);

  for (vector<Vector3>::iterator it = points.begin(); it != points.end();
      it++) {
    pcloud.add_vertex(it->x,it->y,it->z);
  }
}

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
    log("Error! Can't issue tracking command; object uknown!");
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
ObjectRelationManager::generateNewObjectModel(const std::string &label) {
  log("generateNewObjectModel %s", label.c_str());
  BoxObject *newBoxObject = new BoxObject;
  m_objectModels[label] = newBoxObject;
  newBoxObject->type = OBJECT_BOX;
  if (label == "krispies") {
    newBoxObject->radius1 = 0.095;
    newBoxObject->radius2 = 0.045;
    newBoxObject->radius3 = 0.145;
  }
  else if (label == "joystick") {
    newBoxObject->radius1 = 0.115;
    newBoxObject->radius2 = 0.105;
    newBoxObject->radius3 = 0.13;
  }
  else if (label == "rice") {
    newBoxObject->radius1 = 0.075;
    newBoxObject->radius2 = 0.023;
    newBoxObject->radius3 = 0.095;
  }
  else if (label == "printer") {
    newBoxObject->radius1 = 0.300;
    newBoxObject->radius2 = 0.160;
    newBoxObject->radius3 = 0.260;
  }
  else  {
    newBoxObject->radius1 = 0.1;
    newBoxObject->radius2 = 0.1;
    newBoxObject->radius3 = 0.1;
  }
  newBoxObject->pose.pos.x = -FLT_MAX;
  newBoxObject->pose.pos.y = -FLT_MAX;
  newBoxObject->pose.pos.z = -FLT_MAX;
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

    FrontierInterface::GridMapDoublePtr inMap = request->outMap;
    Cure::LocalGridMap<double> outMap(inMap->size, inMap->cellSize, 0.0, 
	Cure::LocalGridMap<double>::MAP1,
	inMap->x, inMap->y);

    vector<spatial::Object *> objectChain;
    vector<spatial::SpatialRelationType> relations;

    //Fill it
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
	// The pose of this object is not known. Cannot compute onness
	// for this hierarchy.
	log("Error! Support object was unknown; can't compute PDF for hierarchy!");
	overwriteWorkingMemory<FrontierInterface::ObjectPriorRequest>(wmc.address, request);
	return;
      }
      supportObject = m_objectModels[supportObjectLabel];
    }
    objectChain.push_back(supportObject);

    for (vector<string>::iterator it = request->objects.begin();
	it != request->objects.end(); it++) {
      if (m_planeObjectModels.find(*it) != m_planeObjectModels.end()) {
	log("Can't compute ON for table on sth else!");
	return;
      }
      else {
	// For now, assume each label represents a unique object
	if (m_objects.find(*it) == m_objects.end()) {
	  // New object
	  log("New SpatialObject: %s", it->c_str());
	  m_objects[*it] = new SpatialData::SpatialObject;
	  m_objects[*it]->label = *it;

	  generateNewObjectModel(*it);
	}
	objectChain.push_back(m_objectModels[*it]);
	relations.push_back(RELATION_ON);
      }
    }

    double total = 0.0;

    vector<Vector3> dummyTriangle;
    m_sampler.
      sampleBinaryRelationRecursively(relations, objectChain, 
	request->objects.size()-2, outMap, total, 
	dummyTriangle, 1.0);

    request->outMap = convertFromCureMap(outMap);

    for (unsigned long i = 0; i < request->outMap->contents.size(); i++) {
      request->outMap->contents[i] *= (request->probSum/total);
    }

    overwriteWorkingMemory<FrontierInterface::ObjectPriorRequest>(wmc.address, request);
  }

  catch (DoesNotExistOnWMException) {
    log("Error! Prior request disappeared from WM!");
  }
}

void
ObjectRelationManager::new3DPriorRequest(const cdl::WorkingMemoryChange &wmc) {
  try {
    FrontierInterface::ObjectPriorRequestPtr request =
      getMemoryEntry<FrontierInterface::ObjectPriorRequest>(wmc.address);

    if (request->objects.size() < 2) {
      log("Error! Can't compute onness for less than 2 objects!");
      return;
    }

    FrontierInterface::GridMapDoublePtr inMap = request->outMap;
    Cure::LocalGridMap<double> outMap(inMap->size, inMap->cellSize, 0.0,
        Cure::LocalGridMap<double>::MAP1,
        inMap->x, inMap->y);

    vector<spatial::Object *> objectChain;
    vector<spatial::SpatialRelationType> relations;

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
        // The pose of this object is not known. Cannot compute onness
        // for this hierarchy.
        log("Error! Support object was unknown; can't compute PDF for hierarchy!");
        overwriteWorkingMemory<FrontierInterface::ObjectPriorRequest>(wmc.address, request);
        return;
      }
      supportObject = m_objectModels[supportObjectLabel];
    }
    objectChain.push_back(supportObject);

    for (vector<string>::iterator it = request->objects.begin();
	it != request->objects.end(); it++) {
      if (m_planeObjectModels.find(*it) != m_planeObjectModels.end()) {
	log("Can't compute ON for table on sth else!");
	return;
      }
      else {
	// For now, assume each label represents a unique object
	if (m_objects.find(*it) == m_objects.end()) {
	  // New object
	  log("New SpatialObject: %s", it->c_str());
	  m_objects[*it] = new SpatialData::SpatialObject;
	  m_objects[*it]->label = *it;

	  generateNewObjectModel(*it);
	}
	objectChain.push_back(m_objectModels[*it]);
	relations.push_back(RELATION_ON);
      }
    }

    double total = 0.0;

    vector<Vector3> dummyTriangle;
    m_sampler.
      sampleBinaryRelationRecursively(relations, objectChain, request->objects.size()-2, outMap,
	total, dummyTriangle, 1.0);

    request->outMap = convertFromCureMap(outMap);
    
    for (unsigned long i = 0; i < request->outMap->contents.size(); i++) {
      request->outMap->contents[i] *= (request->probSum/total);
    }

    overwriteWorkingMemory<FrontierInterface::ObjectPriorRequest>(wmc.address, request);
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
	supportObject = m_objectModels[supportObjectLabel];
      }
    }

    if (supportObject != 0) {
      int nSamplesPerStep = request->objects.size() == 2 ? pointCount : pointCount>>2;

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

