/**
 * @author Michael Zillich
 * @date April 2011
*/

#include <iostream>
#include "Wm5.h"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "BlueFSM.h"
#include <cast/architecture/ManagedComponent.hpp>
#include <cast/architecture.hpp>
#include "FrontierInterface.hpp"
#include "CureMapConversion.hpp"
#include <float.h>
#include "Vector3.h"
#include "Matrix33.h"
#include "Pose3.h"
#include <SpatialData.hpp>
#include <AddressBank/ConfigFileReader.hh>
#include <PTZ.hpp>

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cogx::BlueFSM();
  }
}

namespace cogx
{

  bool validPose(const Math::Pose3& p)
  {
    double sum = 0;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        sum += std::abs(Math::get(p.rot, i, j));
    return sum > 1e-6;
  }
  
  Math::Pose3 convertPose(const m::Pose& in)
  {
    Math::Pose3 p;
    for (int i = 0; i < 3; ++i)
    {
      Math::set(p.pos, i, in.first[i]);
      for (int j = 0; j < 3; ++j)
        Math::set(p.rot, i, j, m::matrixCopy(in.second)(i,j));
    }
    return p;
  }
  m::Pose convertPose(const Math::Pose3& in)
  {
    m::Pose p;
    p.first = m::Vector3(in.pos.x,in.pos.y,in.pos.z);
    m::Matrix3 m;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        m(i,j) = Math::get(in.rot, i, j);
    
    m.Orthonormalize();
    p.second = m::quaternionCopy(m);
    return p;
  }
  
  using namespace std;
  using namespace cast;
  
  BlueFSM::BlueFSM() : m_state(INIT)
  {
  }

  BlueFSM::~BlueFSM()
  {
    delete m_Glrt;
    delete m_lgm;
  }
  
  void BlueFSM::configure(const map<string, string> &_config)
  {
    map<string,string>::const_iterator it = _config.find("-c");
    if (it== _config.end()) {
      println("configure(...) Need config file (use -c option)\n");
      std::abort();
    }
    std::string configfile = it->second;

    Cure::ConfigFileReader cfg;
    if (cfg.init(configfile)) {
      println("configure(...) Failed to open with \"%s\"\n",
	  configfile.c_str());
      std::abort();
    }  

    if (cfg.getSensorPose(1, m_LaserPoseR)) {
      println("configure(...) Failed to get sensor pose for laser");
      std::abort();
    } 

    m_useDropoff = false;
    if (_config.count("--use-dropoff")) {
      m_useDropoff = true;
    }

    m_handoverPose.pos = Math::vector3(0.4, 0.4, 0.6);
    fromAngleAxis(m_handoverPose.rot, M_PI/4, Math::vector3(0,0,1));

    double CellSize = 0.1;
    int MapSize = 70;
    m_lgm = new CharMap(MapSize, CellSize, '2', CharMap::MAP1);
    m_Glrt  = new CharGridLineRayTracer(*m_lgm);

  if (_config.find("--no-x-window") == _config.end()) {
    m_Displaylgm = new Cure::XDisplayLocalGridMap<unsigned char>(*m_lgm);
    println("Will use X window to show the exploration map");
  } else {
    m_Displaylgm = 0;
    println("Will NOT use X window to show the exploration map");
  }

    m_lookForObjects.push_back("cereals-weetabix");
    m_lookForObjects.push_back("cereals-chocos");
  }

  void BlueFSM::start()
  {
    addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
	new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::objectPoseCallback));
    addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject>(cdl::ADD),
	new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::objectPoseCallback));
    addChangeFilter(createGlobalTypeFilter<NavData::RobotPose2d>(cdl::WILDCARD),
	new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::newRobotPose));

    if (false) {
      m_placeInterface = getIceServer<FrontierInterface::PlaceInterface>("place.manager");
      m_mapInterface = getIceServer<FrontierInterface::LocalMapInterface>("map.manager");
    }
  }
  
  void BlueFSM::destroy()
  {
  }
  
  void
  BlueFSM::simpleCallback(const cdl::WorkingMemoryChange &wmc) {
    manipulation::slice::ManipulationCommandPtr cmd =
      getMemoryEntry<manipulation::slice::ManipulationCommand>(wmc.address.id);

    if (cmd->comp == manipulation::slice::SUCCEEDED ||
	cmd->comp == manipulation::slice::FAILED) {
	    log ("simpleCallback: m_waiting was %i, now 0", m_waiting);
      m_waiting = false;
    }
  }

  void
  BlueFSM::navCallback(const cdl::WorkingMemoryChange &wmc) {
    SpatialData::NavCommandPtr cmd =
      getMemoryEntry<SpatialData::NavCommand>(wmc.address.id, wmc.address.subarchitecture);

    if (cmd->comp == SpatialData::COMMANDSUCCEEDED ||
	cmd->comp == SpatialData::COMMANDFAILED) {
      m_waiting = false;
    }
  }

  void
  BlueFSM::ptzCallback(const cdl::WorkingMemoryChange &wmc) {
    ptz::SetPTZPoseCommandPtr cmd =
      getMemoryEntry<ptz::SetPTZPoseCommand>(wmc.address.id);

    if (cmd->comp == ptz::SUCCEEDED ||
	cmd->comp == ptz::FAILED) {
      m_waiting = false;
    }
  }

void BlueFSM::newRobotPose(const cdl::WorkingMemoryChange &objID) 
{
  try {
    NavData::RobotPose2dPtr lastRobotPose =
      getMemoryEntry<NavData::RobotPose2d>(objID.address);
    m_SlamRobotPose.setX(lastRobotPose->x);
    m_SlamRobotPose.setY(lastRobotPose->y);
    m_SlamRobotPose.setTheta(lastRobotPose->theta);

    Cure::Pose3D cp = m_SlamRobotPose;
    m_TOPP.defineTransform(cp);

    m_CurrPose.pos = Math::vector3(lastRobotPose->x, lastRobotPose->y, 0);
    Math::fromAngleAxis(m_CurrPose.rot, lastRobotPose->theta, Math::vector3(0,0,1));
  }
  catch (DoesNotExistOnWMException e) {
    log("Error! robotPose missing on WM!");
    return;
  }

}

  void BlueFSM::receiveOdometry(const Robotbase::Odometry &castOdom)
  {
    lockComponent(); //Don't allow any interface calls while processing a callback
    Cure::Pose3D cureOdom;
    CureHWUtils::convOdomToCure(castOdom, cureOdom);

    debug("Got odometry x=%.2f y=%.2f a=%.4f t=%.6f",
	cureOdom.getX(), cureOdom.getY(), cureOdom.getTheta(),
	cureOdom.getTime().getDouble());

    m_TOPP.addOdometry(cureOdom);

    Cure::Pose3D currentPose = m_TOPP.getPose();
    double x[3];
    currentPose.getXYTheta(x);

//    m_CurrPose.pos = Math::vector3(x[0], x[1], 0);
//    fromAngleAxis(m_CurrPose.rot, x[2], Math::vector3(0,0,1));
    unlockComponent();
  }

void 
BlueFSM::receiveScan2d(const Laser::Scan2d &castScan)
{
  lockComponent(); //Don't allow any interface calls while processing a callback
  debug("Got scan with n=%d and t=%ld.%06ld",
        castScan.ranges.size(), 
        (long)castScan.time.s, (long)castScan.time.us);

  Cure::LaserScan2d cureScan;
  CureHWUtils::convScan2dToCure(castScan, cureScan);

  if (m_TOPP.isTransformDefined()) {
    
    Cure::Pose3D scanPose;
    if (m_TOPP.getPoseAtTime(cureScan.getTime(), scanPose) == 0) {
      Cure::Pose3D lpW;
      lpW.add(scanPose, m_LaserPoseR);
//      m_Mutex.lock();
      m_Glrt->addScan(cureScan, lpW, 15);      
//      m_firstScanReceived = true;
//      m_Mutex.unlock();
    }
  }
  unlockComponent();
}

  void 
  BlueFSM::addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, std::string label, std::string visualObjectID){
    VisionData::Recognizer3DCommandPtr rec_cmd = new VisionData::Recognizer3DCommand;
    rec_cmd->cmd = cmd;
    rec_cmd->label = label;
    rec_cmd->visualObjectID = visualObjectID;
    addToWorkingMemory(newDataID(), "vision.sa", rec_cmd);
  }

  void outputToFile(const std::string& s, const m::Pose& p)
  {
    std::ofstream ofs(s.c_str());
    ofs << p.first.X() << " " << p.first.Y() << " " << p.first.Z() << " " <<
    p.second.W() << " " << p.second.X() << " " << p.second.Y() << " " << p.second.Z() << std::endl;
  }

  void addToOutputFile(const std::string& s, const m::Pose& p)
  {
    std::ofstream ofs(s.c_str(), std::ios::app);
    ofs << p.first.X() << " " << p.first.Y() << " " << p.first.Z() << " " <<
    p.second.W() << " " << p.second.X() << " " << p.second.Y() << " " << p.second.Z() << std::endl;
  }

  
  void outputToFile(const std::string& s, const Math::Pose3& p)
  {
    std::ofstream ofs(s.c_str());
    ofs << p << std::endl;
    writeTextMatrix(ofs, p.rot);
  }

  void BlueFSM::nameless(const Math::Pose3& inRobotPose,
                                                   const Math::Pose3& inObjectPose,
                                                   const std::string& inObjectLabel,
                                                   Math::Pose3& outPregraspPose,
                                                   Math::Pose3& outEnvelopingPose,
                                                   double& outQuality) const
  {
    
    std::map<std::string, std::vector<m::Pose> > grasps;
    
    {
      std::map<std::string, std::pair<double,double> > sizes;
      
      sizes[std::string("cereals-bircher")] = std::make_pair(0.0655,0.1005);
      sizes[std::string("cereals-chocos")] = std::make_pair(0.0965,0.126);
      sizes[std::string("cereals-fruchtemusli")] = std::make_pair(0.0685,0.095);
      sizes[std::string("cereals-toppas")] = std::make_pair(0.073,0.1155);
      sizes[std::string("cereals-weetabix")] = std::make_pair(0.095,0.125);
      sizes[std::string("cereals-schokomusli")] = std::make_pair(0.0685,0.095);
      
      std::vector<m::Pose> v;
      // These are wrong - from when we thought that the forward gripper axis was X.
//      v.push_back(std::make_pair(m::Vector3(.100,0,0), m::Quaternion(0,0,0,1)));
//      v.push_back(std::make_pair(m::Vector3(-.100,0,0), m::Quaternion(1,0,0,0)));
//      v.push_back(std::make_pair(m::Vector3(0,0,.140), m::Quaternion(0.707107,0,0.707107,0)));
//      v.push_back(std::make_pair(m::Vector3(0,0,-.140), m::Quaternion(0.707107,0,-0.707107,0)));
//      grasps[std::string("cereals-weetabix")] = v;

      // Hand-written grasps for the weetabix
//      v.push_back(std::make_pair(m::Vector3(.100,0,0), m::Quaternion(0.707107,0,0,0.707107)));
//      v.push_back(std::make_pair(m::Vector3(-.100,0,0), m::Quaternion(-0.707107,0,0,0.707107)));
//      v.push_back(std::make_pair(m::Vector3(0,0,-.130), m::Quaternion(0.5,0.5,0.5,0.5)));
//      v.push_back(std::make_pair(m::Vector3(0,0,.130), m::Quaternion(0.5,-0.5,-0.5,0.5)));
//      grasps[std::string("cereals-weetabix")] = v;

      for (std::map<std::string, std::pair<double,double> >::const_iterator i = sizes.begin();
           i != sizes.end(); ++i)
      {
        v.push_back(std::make_pair(m::Vector3(i->second.first,0,0), m::Quaternion(0.707107,0,0,0.707107)));
        v.push_back(std::make_pair(m::Vector3(-i->second.first,0,0), m::Quaternion(-0.707107,0,0,0.707107)));
        v.push_back(std::make_pair(m::Vector3(0,0,-i->second.second), m::Quaternion(0.5,0.5,0.5,0.5)));
        v.push_back(std::make_pair(m::Vector3(0,0,i->second.second), m::Quaternion(0.5,-0.5,-0.5,0.5)));
        
        grasps[i->first] = v;
      }

    
    }

    if (!validPose(inObjectPose) || !validPose(inRobotPose))
    {
      std::cerr << "nameless: invalid input poses (check in /tmp)" << std::endl;
      log("nameless: invalid input poses (check in /tmp)");
      std::terminate();
    }
    
    //log("nameless: converting data to m::");
    
    outputToFile("/tmp/grasp-inObjectPose", inObjectPose);
    outputToFile("/tmp/grasp-inRobotPose", inRobotPose);
  
    
    
    m::Pose objectPose = convertPose(inObjectPose);
    m::Pose robotPose = convertPose(inRobotPose);
    m::Pose robotRelObjectPose;
    m::project(robotRelObjectPose.first, robotRelObjectPose.second,
               robotPose.first, robotPose.second,
               objectPose.first, objectPose.second);
    
    
    outputToFile("/tmp/grasp-objectPose", objectPose);
    outputToFile("/tmp/grasp-robotPose", robotPose);
    outputToFile("/tmp/grasp-robotRelObjectPose", robotRelObjectPose);
    
    // Get the predefined grasps for that object
    
    std::vector<m::Pose> objectRelGrasps = grasps[inObjectLabel];

    //log("nameless: checking if object lies flat");

    
    bool objectLyingFlat = false;
    if (std::abs((m::Vector3::UNIT_Z).Dot(m::normalized(m::matrixCopy(objectPose.second).GetColumn(1)))) > .707)
    {
      log("nameless: Object %s is lying flat like a cow on the table", inObjectLabel.c_str());
      objectLyingFlat = true;
    }
    
    // Find the easiest grasps in there
    
    //log("nameless: Finding the easiest grasps in there");

    double maxDotProduct = -1000;
    m::Pose bestGrasp = std::make_pair(m::Vector3::ZERO, m::Quaternion::IDENTITY);
    m::Pose bestBacktrackedGrasp = bestGrasp;
    if (objectRelGrasps.size() != 4)
    {
      log("Waaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaarning: wrong number of grasps in grasp vector");
    }
    for (std::vector<m::Pose>::iterator i = objectRelGrasps.begin(); i != objectRelGrasps.end(); ++i)
    {
      if (objectLyingFlat)
      {
        if (std::distance(objectRelGrasps.begin(), i) <= 1) continue;
      }
      m::Pose robotGrasp;
      m::transform(robotGrasp.first, robotGrasp.second, robotRelObjectPose.first, robotRelObjectPose.second, i->first, i->second);
      m::Matrix3 robotGraspOri(m::matrixCopy(robotGrasp.second));
      m::Vector3 robotToGraspVector = robotGrasp.first;
            
      robotToGraspVector.Z() = 0;
      robotToGraspVector.Normalize();

      double dot = robotToGraspVector.Dot(m::Vector3(robotGraspOri.GetColumn(1)));
      if (dot > maxDotProduct)
      {
        maxDotProduct = dot;
        bestGrasp = robotGrasp;
        bestBacktrackedGrasp = bestGrasp;
        bestBacktrackedGrasp.first -= .1*m::Vector3(robotGraspOri.GetColumn(1));
      }
    }
    
    //log("nameless: outputing things");
    
    outputToFile("/tmp/grasp-bestGrasp", bestGrasp);
    outputToFile("/tmp/grasp-bestBacktrackedGrasp", bestBacktrackedGrasp);

    addToOutputFile("/tmp/grasp-all-robotRelObjectPose", robotRelObjectPose);
    addToOutputFile("/tmp/grasp-all-bestGrasp", bestGrasp);
    {
      std::ofstream ofs("/tmp/grasp-all-quality", std::ios::app);
      ofs << maxDotProduct << std::endl;
    }
    
    outPregraspPose = convertPose(bestBacktrackedGrasp);
    std::cerr << "pregrasp: " << outPregraspPose << std::endl;
    outEnvelopingPose = convertPose(bestGrasp);
    std::cerr << "enveloping grasp: " << outEnvelopingPose << std::endl;
    outQuality = maxDotProduct;
  }
  
  void BlueFSM::runComponent()
  {
	  setupPushScan2d(*this, 0.1);
	  setupPushOdometry(*this);

    while (true)
    {

    if (m_Displaylgm) {
      Cure::Pose3D currentPose = m_TOPP.getPose();
      m_Displaylgm->updateDisplay(&currentPose);
//                                  &m_NavGraph, 
//                                  &m_Explorer->m_Fronts);
    }

      switch(m_state) {
	case INIT:
	  {
	    // INIT state: Start by homing the arm, then begin searching for
	    // objects/tables
	    log("INIT");
	    m_state = LOOK_CANONICAL;
//break;
	      bool success = movePTZ(0, -M_PI/6);
	      if (true) {
		m_state = SPINNING;
	      }
	      else {
		log("Error! Unable to move PTZ to shallow position!");
		m_state = TERMINATED;
	      }
	  }
	  break;
	
	case SPINNING:
	  {
	    log("SPINNING");

	    m_turnStep = 0;

	    while (m_turnStep < 3) {
	      lockComponent();
	      // Issue recognition commands
	      for (std::vector<string>::iterator it = m_lookForObjects.begin(); it != m_lookForObjects.end(); it++) {
		addRecognizer3DCommand(VisionData::RECOGNIZE, it->c_str(), "");
	      }
	      unlockComponent();

	      sleep(5);
	      if(m_turnStep < 2)
	      {
		turn45Degrees();
	      }
	      m_turnStep++;
	    }

	    if ((!m_useDropoff || m_dropTableDetected) && !m_globalPoses.empty()) {
	      m_state = DECIDE_POSITION;
	    }
	    else {
	      m_state = MOVE_TO_NEW_POS;
	    }
	  }
	  break;

	case DECIDE_POSITION:
	  {
	    log("DECIDE_POSITION");

log("%i", __LINE__);
	    string bestObject;
	    double bestSqDist = DBL_MAX;
	    for (map<string, Math::Pose3>::iterator it = m_globalPoses.begin();
		it != m_globalPoses.end(); it++) {
	      double x = it->second.pos.x;
	      double y = it->second.pos.y;
	      double distsq = (m_CurrPose.pos.x-x)*(m_CurrPose.pos.x-x)+
		(m_CurrPose.pos.y-y)*(m_CurrPose.pos.y-y);
	      if (distsq < bestSqDist) {
		bestSqDist = distsq;
		bestObject = it->first;
	      }
	    }
log("%i", __LINE__);

	    if (bestObject == "") {
	      log("Error: No objects to move to!");
	      m_state = MOVE_TO_NEW_POS;
	    }
	    else {
	      double bestX, bestY, bestTheta;
log("%i", __LINE__);
	      bool success = findBestGraspPose(bestObject, bestX, bestY, bestTheta);
log("%i", __LINE__);
	      if (!success) {
		log("Error: Unable to find grasp pose for %s", bestObject.c_str());
		m_state = MOVE_TO_NEW_POS;
	      }
	      else {
		success = moveTo(bestX, bestY, bestTheta);
		if (!success) {
		  log("Error: Unable to move to (%f, %f)", bestX, bestY);
		  m_state = MOVE_TO_NEW_POS;
		}
		else {
		  m_state = LOOK_CANONICAL;
		}
	      }
	    }
	  }
	  break;
	
	case MOVE_TO_NEW_POS:
	  {
log("MOVE_TO_NEW_POS");
	    double x, y;
	    findRandomPosition(x, y);

	    log("Moving to %f, %f", x, y);
	    bool success = moveTo(x, y);
	    if (!success) {
	      log("Error! Failed to move to %f, %f; picking new position");
	      m_state = MOVE_TO_NEW_POS;
	    }
	    else {
	      m_state = SPINNING;
	    }
	  }
	  break;
	
	case LOOK_AROUND:
	  log("LOOK_AROUND");
	  m_state = LOOK_CANONICAL;
	  break;
          
	case LOOK_CANONICAL:
	  log("LOOK_CANONICAL");
	  {
	    bool success = movePTZ(0, -M_PI/3);

	    if (false) { //!success) {
	      log("Error! Couldn't move the PTZ to canonical!") ;
	      m_state = TERMINATED;
	    }
	    else {
	      lockComponent();
	      // Issue recognition commands
	      for (std::vector<string>::iterator it = m_lookForObjects.begin(); it != m_lookForObjects.end(); it++) {
		addRecognizer3DCommand(VisionData::RECOGNIZE, it->c_str(), "");
	      }
	      unlockComponent();

	      m_state = DETECTING;
	    }
	  }
	  break;

	case DETECTING:
	  log("DETECTING");
	  sleep(10);

	  m_state = DECIDE_GRASP;
	  break;
          
	case DECIDE_GRASP:
	  log("DECIDE_GRASP");
          {
            // Find the label of the nearest object

            // shortest distance between the robot and an object.
            /*
            double minDistance = 100000000; // very nice
            std::string minLabel = "";
            for (std::map<std::string, cogx::Math::Pose3>::iterator i = m_poses.begin();
                 i != m_poses.end(); ++i)
              {
                if (Math::norm(i->second.pos) < minDistance)
                  {
                    minDistance = Math::norm(i->second.pos);
                    minLabel = i->first;
                  }
              }
            std::cout << minLabel << std::endl;
            */
            
            if (m_poses.begin() == m_poses.end())
            {
              log("Error: m_poses map is empty. We will now move to a new position.");
              m_state = MOVE_TO_NEW_POS;
              break;
            }
            
            std::map<std::string, cogx::Math::Pose3>::iterator bestPoseToGoTo = m_poses.begin();
            double bestPoseQuality = 0;
            for (std::map<std::string, cogx::Math::Pose3>::iterator i = m_poses.begin();
                 i != m_poses.end(); ++i)
            {
              Math::Pose3 trash;
              double quality = -1000;
              nameless(convertPose(std::make_pair(m::Vector3::ZERO, m::Quaternion::IDENTITY)),
                       m_poses[i->first],
                       i->first,
                       trash,
                       trash,
                       quality);
              if (quality > bestPoseQuality)
              {
                bestPoseQuality = quality;
                bestPoseToGoTo = i;
              }
            }

            if (bestPoseQuality < .707)
            {
              log("We're going to grasp a horrible grasp (quality = %f)", bestPoseQuality);
              std::cerr << "We're going to grasp a horrible grasp (quality = " << bestPoseQuality << ")" << std::endl;
            }
            
            std::string minLabel = bestPoseToGoTo->first;
            
            log("We're off to grasp %s", minLabel.c_str());
            
            std::cerr << m_poses[minLabel] << std::endl;

            double trash;
                        
            nameless(convertPose(std::make_pair(m::Vector3::ZERO, m::Quaternion::IDENTITY)),
                     m_poses[minLabel],
                     minLabel,
                     m_pregraspPose,
                     m_envelopingPose,
                     trash);

            m_state = GO_TO_PREGRASP;
          }
	  break;

        case GO_TO_PREGRASP:
	  log("GO_TO_PREGRASP");
	  {
	    bool success = movePregrasp(m_pregraspPose);
	    if (success) {
	      m_state = VERIFY_PREGRASP;
	    }
	    else {
	      log("Error! move pregrasp failed");
	      success = moveHome();
	      if (success) {
		m_state = INIT;
	      }
	      else {
		log("Error! Unable to return to home pose!");
		m_state = TERMINATED;
	      }
	    }
	  }
	  break;

	case VERIFY_PREGRASP:
	  log("VERIFY_PREGRASP");
	  {
	    m_state = ENVELOP;
	  }
	  break;

	case ENVELOP:
	  log("ENVELOP");
	  {
	    bool success = envelop();
	    if (success) {
	      m_state = VERIFY_ENVELOP;
	    }
	    else {
	      log("Error! envelop failed");
	      success = retract();
	      if (success) {
		m_state = LOOK_CANONICAL;
	      }
	      else {
		log("Error! Unable to retract!");
		success = moveHome();
		if (success) {
		  m_state = INIT;
		}
		else {
		  log("Error! Unable to return to home pose!");
		  m_state = TERMINATED;
		}
	      }
	    }
	  }
	  break;

	case VERIFY_ENVELOP:
	  log("VERIFY_ENVELOP");
	  {
	    m_state = GRASP;
	    break;
	  }

	case GRASP:
	  log("GRASP");
	  {
	    bool success = grasp();
	    if (success) {
	      m_state = VERIFY_GRASP;
	    }
	    else {
	      log("Error! grasp failed");
	      success = release();
	      if (success) {
		m_state = RETRACT;
	      }
	      else {
		log("Error! Unable to release!");
		m_state = TERMINATED;
	      }
	    }
	  }
	  break;

	case VERIFY_GRASP:
	  log("VERIFY_GRASP");
	  {
	    m_state = LIFT;
	  }
	  break;

	case RETRACT:
	  log("RETRACT");
	  {
	    bool success = retract();
	    if (success) {
	      m_state = LOOK_CANONICAL;
	    }
	    else {
	      log("Error! retract failed");
	      m_state = TERMINATED;
	    }
	  }
	  break;

	case LIFT:
	  log("LIFT");
	  {
	    bool success = lift();
	    if (success) {
	      m_state = DELIVER_TO_HOME_POSITION;
	    }
	    else {
	      log("Error! lift failed");
	      success = release();
	      if (success) {
		m_state = LOOK_CANONICAL;
	      }
	      else {
		log("Error! Unable to release!");
		  m_state = TERMINATED;
	      }
	    }
	  }
	  break;

	case RELEASE:
	  log("RELEASE");
	  {
	    bool success = release();
	    if (success) {
	      m_state = GO_HOME;
	    }
	    else {
	      log("Error! release failed");
	      m_state = TERMINATED;
	    }
	  }
	  break;

	case HANDOVER:
	  log("HANDOVER");
	  {
	    bool success = moveToHandover();
	    if (success) {
	      m_state = VERIFY_HANDOVER;
	    }
	    else {
	      log("Error! handover move failed");
	      m_state = RELEASE;
	    }
	  }
	  break;

	case VERIFY_HANDOVER:
	  log("VERIFY_HANDOVER");
	  {
	    m_state = RELEASE;
	  }
	  break;
	  
	case GO_HOME:
	  log("GO_HOME");
	  {
	    bool success = moveHome();
	    if (success) {
	      m_state = TERMINATED;
	    }
	    else {
	      log("Error! move home failed");
	      m_state = TERMINATED;
	    }
	  }
	  break;

        case DELIVER_TO_HOME_POSITION:
        {
          moveToSafePose();
          moveTo(0, 0, 0);
          moveToHandover();
        }
	case TERMINATED:
	  log("TERMINATED");
	  log ("Terminated");
	  break;
        default:
          log("Error: unknown state hence terminated");
          m_state = TERMINATED;
      }

      if (m_state == TERMINATED) {
	break;
      }
    }
    
    /*
    ::manipulation::slice::MoveArmToPosePtr moveArmToPose(new ::manipulation::slice::MoveArmToPose());
    moveArmToPose->comp = ::manipulation::slice::COMPINIT;
    moveArmToPose->status = ::manipulation::slice::NEW;
    
    {
      boost::unique_lock<boost::mutex> lock(mutex_);
      //moveArmToPose->targetPose = pose_;
    }
    
    try {
      addToWorkingMemory(newDataID(), moveArmToPose);
    } catch (std::exception &e1) {
      log(e1.what());
    }
    
    sleep(10); // Wait for changes to happen and pray that it worked.
    */
    
    /*
    wmcr = new WorkingMemoryChangeReceiver() {
      public void workingMemoryChanged(WorkingMemoryChange _wmc) {
        commandChanged(_wmc, wmcr);
      }
    };
    
    ((CogXRunner) manipulator.getRunner()).addChangeFilter(
                                                           ChangeFilterFactory.createIDFilter(id), wmcr);

    */
  }
  
  void BlueFSM::objectPoseCallback(const cdl::WorkingMemoryChange &_wmc)
  {
    //warn("received objectPoseCallback");

    VisionData::VisualObjectPtr vo = getMemoryEntry<VisionData::VisualObject>(_wmc.address);

    for (unsigned i = 0; i < vo->identDistrib.size(); i++)
    {
      if (vo->identLabels.at(i) == "unknown")
        continue;

      log("objectPoseCallback: Studying %s", vo->identLabels.at(i).c_str());
      
      if (vo->identDistrib.at(i) < .03)
        continue;

      log("objectPoseCallback: %s has a high conf!", vo->identLabels.at(i).c_str());

      //boost::unique_lock<boost::mutex> lock(mutex_, boost::try_to_lock_t());
      std::cerr << "Waiting for mutex_" << std::endl;
      boost::unique_lock<boost::mutex> lock(mutex_);
      std::cerr << "Have mutex_" << std::endl;
      
      //Transform from vo->pose (local) into global and store
      transform(m_CurrPose, vo->pose, m_globalPoses[vo->identLabels.at(i)]);
log("local: %f, %f, %f -- global: %f, %f, %f", vo->pose.pos.x, vo->pose.pos.y, vo->pose.pos.z,
m_globalPoses[vo->identLabels.at(i)].pos.x,
m_globalPoses[vo->identLabels.at(i)].pos.y,
m_globalPoses[vo->identLabels.at(i)].pos.z);

      m_poses[vo->identLabels.at(i)] = vo->pose;
      m_pose_confs[vo->identLabels.at(i)] = vo->identDistrib.at(i);
    }
      //warn("finished objectPoseCallback");
  }

  void IcetoCureLGM(FrontierInterface::LocalGridMap icemap, CureObstMap* lgm  ){
    int lp = 0;
    for(int x = -icemap.size ; x <= icemap.size; x++){
      for(int y = -icemap.size ; y <= icemap.size; y++){ 
	(*lgm)(x,y) = (icemap.data[lp]);
	lp++;
      }
    }
  }

bool 
BlueFSM::isCircleFree(const CureObstMap &cmap, double xW, double yW, double rad, bool unknownIsObstacle){
  int xiC,yiC;
  if (cmap.worldCoords2Index(xW,yW,xiC,yiC)!= 0)
    return false;

  double w = rad / cmap.getCellSize();
  int wi = int(w + 0.5);

  for (int x = xiC-wi; x <= xiC+wi; x++) {
    for (int y = yiC-wi; y <= yiC+wi; y++) {
      if (x >= -cmap.getSize() && x <= cmap.getSize() && y >= -cmap.getSize() && y <= cmap.getSize()) {
	if (hypot(x-xiC,y-yiC) < w) {
		if (unknownIsObstacle) {
			if (cmap(x,y) == '1' or cmap(x,y) == '2') return false;
		}
		else {
			if (cmap(x,y) == '1' ) return false;
		}
	}
      }
    }
  }
  return true;
}

bool BlueFSM::findBestGraspPose(const string &obj, double &bestX, double &bestY,
  double &bestTheta)
{
  log ("findBestGraspPose");
  //    int currentPlaceID = m_placeInterface->getCurrentPlace()->id;
  //    SpatialData::PlaceIDSeq vec;
  //    vec.push_back(currentPlaceID);

  //    FrontierInterface::LocalGridMap combined_lgm;
  //    log("getting combined lgm");
  //    combined_lgm = m_mapInterface->getCombinedGridMap(vec);
  //    log("have combined lgm");


  //    CureObstMap clgm(combined_lgm.size, 0.05, '2', CureObstMap::MAP1, combined_lgm.xCenter, combined_lgm.yCenter);

  //    IcetoCureLGM(combined_lgm,&clgm);
log("%i", __LINE__);
  if (m_globalPoses.count(obj) == 0) {
    log("Error! Trying to find grasp pose for unknown object %s", obj.c_str());
    return false;
  }

  Math::Pose3 objPose = m_globalPoses[obj];
  double objX = objPose.pos.x;
  double objY = objPose.pos.y;

  double bestAlignment = -FLT_MAX;

  vector<pair<double, double> > coordinates;
  for (int attempt = 0; attempt < 10000; attempt++) {
    const double r = 0.2 + 0.5*((double)rand())/RAND_MAX;
    const double phi = 2*M_PI*((double)rand())/RAND_MAX;
    double dx = r*cos(phi);
    double dy = r*sin(phi);
    double x = objX + dx;
    double y = objY + dy;

    if (isCircleFree(*m_lgm, x, y, 0.3, true))  {
      double alignment;
      Math::Pose3 outPregraspPose;
      Math::Pose3 outEnvelopingPose;
      Math::Pose3 inputPose;
      setIdentity(inputPose);
      inputPose.pos = Math::vector3(x,y,0);
      nameless(inputPose,
	  objPose,
	  obj,
	  outPregraspPose,
	  outEnvelopingPose,
	  alignment);
      if (alignment > bestAlignment) {
	bestAlignment = alignment;
	bestX = x;
	bestY = y;
	bestTheta = atan2(-dy,-dx);
      }
    }
else {
} 
  }
log("bestAlignment %f", bestAlignment);

  if (bestAlignment > 0.707) {
    return true;
  }
  return false;
}

  bool BlueFSM::moveToSafePose()
  {
    return movePregrasp(convertPose(std::make_pair(m::Vector3(.2,0,1), m::Quaternion(0.5, 0.5, 0.5, 0.5))));
  }

bool BlueFSM::movePregrasp(cogx::Math::Pose3 pregraspPose) 
{
  manipulation::slice::MoveArmToPosePtr cmd = new
    manipulation::slice::MoveArmToPose;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;
  cmd->targetPose = pregraspPose;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::simpleCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<manipulation::slice::MoveArmToPose>(id, cmd);

  while (m_waiting) {
    usleep(50000);
  }

//  removeChangeFilter(receiver);
//  delete receiver;

  cmd = getMemoryEntry<manipulation::slice::MoveArmToPose>(id);

  m_currentArmPose = cmd->reachedPose;

  return cmd->comp == manipulation::slice::SUCCEEDED;
}

bool BlueFSM::moveHome() 
{
	log ("moveHome");
  manipulation::slice::MoveArmToHomePositionCommandPtr cmd = new
    manipulation::slice::MoveArmToHomePositionCommand;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::simpleCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<manipulation::slice::MoveArmToHomePositionCommand>(id, cmd);

  while (m_waiting) {
    usleep(50000);
  }
  //  removeChangeFilter(receiver);
  //  delete receiver;

  cmd = getMemoryEntry<manipulation::slice::MoveArmToHomePositionCommand>(id);

  return cmd->comp == manipulation::slice::SUCCEEDED;
}

bool BlueFSM::envelop() 
{
  
  manipulation::slice::MoveArmToPosePtr cmd = new
    manipulation::slice::MoveArmToPose;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;
  cmd->targetPose = m_envelopingPose;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::simpleCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<manipulation::slice::MoveArmToPose>(id, cmd);

  while (m_waiting) {
    usleep(50000);
  }
  //  removeChangeFilter(receiver);
  //  delete receiver;

  cmd = getMemoryEntry<manipulation::slice::MoveArmToPose>(id);

  m_currentArmPose = cmd->reachedPose;

  return cmd->comp == manipulation::slice::SUCCEEDED;
}

bool BlueFSM::lift() 
{
  cogx::Math::Pose3 targetPose;

  targetPose.pos = m_currentArmPose.pos + Math::vector3(-.5,0,1) * 0.15;
  targetPose.rot = m_currentArmPose.rot;
  
  manipulation::slice::MoveArmToPosePtr cmd = new
    manipulation::slice::MoveArmToPose;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;
  cmd->targetPose = targetPose;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::simpleCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<manipulation::slice::MoveArmToPose>(id, cmd);

  while (m_waiting) {
    usleep(50000);
  }
  //  removeChangeFilter(receiver);
  //  delete receiver;

  cmd = getMemoryEntry<manipulation::slice::MoveArmToPose>(id);

  m_currentArmPose = cmd->reachedPose;

  return cmd->comp == manipulation::slice::SUCCEEDED;
}

bool BlueFSM::retract() 
{
  cogx::Math::Pose3 targetPose;
  cogx::Math::Vector3 direction = -getColumn(m_currentArmPose.rot, 0); //Get x axis of current pose

  targetPose.pos = m_currentArmPose.pos + direction * 0.05;

  manipulation::slice::MoveArmToPosePtr cmd = new
    manipulation::slice::MoveArmToPose;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;
  cmd->targetPose = targetPose;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::simpleCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<manipulation::slice::MoveArmToPose>(id, cmd);

  while (m_waiting) {
    usleep(50000);
  }
  //  removeChangeFilter(receiver);
  //  delete receiver;

  cmd = getMemoryEntry<manipulation::slice::MoveArmToPose>(id);

  m_currentArmPose = cmd->reachedPose;

  return cmd->comp == manipulation::slice::SUCCEEDED;
}

bool BlueFSM::grasp() 
{
  manipulation::slice::CloseGripperCommandPtr cmd = new
    manipulation::slice::CloseGripperCommand;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;
  cmd->graspStatus = manipulation::slice::GRASPINGSTATUSINIT;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::simpleCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<manipulation::slice::CloseGripperCommand>(id, cmd);

  while (m_waiting) {
    usleep(50000);
  }
  //  removeChangeFilter(receiver);
  //  delete receiver;

  cmd = getMemoryEntry<manipulation::slice::CloseGripperCommand>(id);

  return cmd->comp == manipulation::slice::SUCCEEDED &&
    cmd->graspStatus == manipulation::slice::GRASPING;
}

bool BlueFSM::release() 
{
  manipulation::slice::OpenGripperCommandPtr cmd = new
    manipulation::slice::OpenGripperCommand;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::simpleCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<manipulation::slice::OpenGripperCommand>(id, cmd);

  while (m_waiting) {
    usleep(50000);
  }
  //  removeChangeFilter(receiver);
  //  delete receiver;

  cmd = getMemoryEntry<manipulation::slice::OpenGripperCommand>(id);

  return cmd->comp == manipulation::slice::SUCCEEDED;
}

bool BlueFSM::moveToHandover() 
{
  manipulation::slice::MoveArmToPosePtr cmd = new
    manipulation::slice::MoveArmToPose;

  cmd->status = manipulation::slice::NEW;
  cmd->comp = manipulation::slice::COMPINIT;
  cmd->targetPose = m_handoverPose;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::simpleCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<manipulation::slice::MoveArmToPose>(id, cmd);

  while (m_waiting) {
    usleep(50000);
  }
  //  removeChangeFilter(receiver);
  //  delete receiver;

  cmd = getMemoryEntry<manipulation::slice::MoveArmToPose>(id);

  m_currentArmPose = cmd->reachedPose;

  return cmd->comp == manipulation::slice::SUCCEEDED;
}

bool
BlueFSM::turn45Degrees()
{
log("turn45Degrees");
  m_poses.clear();
  SpatialData::NavCommandPtr cmd = new SpatialData::NavCommand();
  cmd->status = SpatialData::NONE;
  cmd->comp = SpatialData::COMMANDPENDING;
  cmd->prio = SpatialData::URGENT;
  cmd->cmd = SpatialData::TURN;
  cmd->angle.resize(1);
  cmd->angle[0] = M_PI/4;

log("turn45Degrees, %i",__LINE__);
  m_waiting = true;
  string id = newDataID();
log("turn45Degrees, %i",__LINE__);
  MemberFunctionChangeReceiver<BlueFSM> *receiver = 
    new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::navCallback);
  addChangeFilter(createAddressFilter(id, "spatial.sa", cdl::OVERWRITE), receiver);
log("turn45Degrees, %i",__LINE__);
  addToWorkingMemory<SpatialData::NavCommand>(id,"spatial.sa",cmd);  

log("turn45Degrees, %i",__LINE__);
  while (m_waiting) {
    usleep(50000);
  }
log("turn45Degrees, %i",__LINE__);
  //  removeChangeFilter(receiver);
  //  delete receiver;
  cmd = getMemoryEntry<SpatialData::NavCommand>(id, "spatial.sa");
log("turn45Degrees, %i",__LINE__);

  return cmd->comp == SpatialData::COMMANDSUCCEEDED;
}

void
BlueFSM::findRandomPosition(double &x, double &y)
{
  bool finished = false;
  double minX = -1, maxX = 3;
  double minY = -1, maxY = 3;
  while (!finished) {
    x = minX+(maxX-minX)/RAND_MAX*rand();
    y = minY+(maxY-minY)/RAND_MAX*rand();

    if (isCircleFree(*m_lgm, x, y, 0.3, false)) {
      finished = true;
    }
  }
}

bool
BlueFSM::moveTo(double x, double y, double theta)
{
log("moveTo(%f,%f,%f)", x, y, theta);
  m_poses.clear();
  SpatialData::NavCommandPtr cmd = new SpatialData::NavCommand();
  cmd->prio = SpatialData::URGENT;
  cmd->cmd = SpatialData::GOTOPOSITION;

  if (theta > M_PI || theta < -M_PI) {
    cmd->pose.resize(2);
    cmd->pose[0] = x;
    cmd->pose[1] = y;
  }
  else {
    cmd->pose.resize(3);
    cmd->pose[0] = x;
    cmd->pose[1] = y;
    cmd->pose[2] = theta;
  }
  cmd->tolerance.resize(1);
  cmd->tolerance[0] = 0.1;
  cmd->status = SpatialData::NONE;
  cmd->comp = SpatialData::COMMANDPENDING;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = 
    new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::navCallback);
  addChangeFilter(createAddressFilter(id, "spatial.sa", cdl::OVERWRITE), receiver);
  addToWorkingMemory<SpatialData::NavCommand>(id, "spatial.sa", cmd);  

  while (m_waiting) {
    usleep(50000);
  }
  //  removeChangeFilter(receiver);
  //  delete receiver;
  cmd = getMemoryEntry<SpatialData::NavCommand>(id, "spatial.sa");

  return cmd->comp == SpatialData::COMMANDSUCCEEDED;
}

bool
BlueFSM::turnTo(double theta)
{
  m_poses.clear();
  SpatialData::NavCommandPtr cmd = new SpatialData::NavCommand();
  cmd->status = SpatialData::NONE;
  cmd->comp = SpatialData::COMMANDPENDING;
  cmd->prio = SpatialData::URGENT;
  cmd->cmd = SpatialData::TURNTO;
  cmd->angle.resize(1);
  cmd->angle[0] = theta;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = 
    new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::navCallback);
  addChangeFilter(createAddressFilter(id, "spatial.sa", cdl::OVERWRITE), receiver);
  addToWorkingMemory<SpatialData::NavCommand>(id, "spatial.sa", cmd);  

  while (m_waiting) {
    usleep(50000);
  }
  //  removeChangeFilter(receiver);
  //  delete receiver;
  cmd = getMemoryEntry<SpatialData::NavCommand>(id, "spatial.sa");

  return cmd->comp == SpatialData::COMMANDSUCCEEDED;
}

bool
BlueFSM::movePTZ(double pan, double tilt)
{
log("movePTZ %f, %f", pan, tilt);
  ptz::SetPTZPoseCommandPtr cmd = new ptz::SetPTZPoseCommand();
  cmd->comp = ptz::COMPINIT;
  cmd->pose.pan = pan;
  cmd->pose.tilt = tilt;
  cmd->pose.zoom = M_PI/2;

  m_waiting = true;
  string id = newDataID();
  MemberFunctionChangeReceiver<BlueFSM> *receiver = 
    new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::ptzCallback);
  addChangeFilter(createIDFilter(id, cdl::OVERWRITE), receiver);
  addToWorkingMemory<ptz::SetPTZPoseCommand>(id, cmd);  

  while (m_waiting) {
    usleep(50000);
  }
  //  removeChangeFilter(receiver);
  //  delete receiver;
  cmd = getMemoryEntry<ptz::SetPTZPoseCommand>(id);

  return cmd->comp == ptz::SUCCEEDED;
}
  
}
