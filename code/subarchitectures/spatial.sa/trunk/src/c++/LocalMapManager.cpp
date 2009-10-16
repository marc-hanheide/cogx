//
// = FILENAME
//    LocalMapManager.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#include "LocalMapManager.hpp"
#include <CureHWUtils.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

#include <AddressBank/ConfigFileReader.hh>
#include <RobotbaseClientUtils.hpp>
#include <float.h>

using namespace cast;
using namespace std;
using namespace boost;
using namespace spatial;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new LocalMapManager();
  }
}

LocalMapManager::LocalMapManager()
{
  cure_debug_level = -10;

  m_RobotServerHost = "localhost";
}

LocalMapManager::~LocalMapManager() 
{ 
  delete m_Displaylgm1;
  delete m_Displaylgm2;
  delete m_Glrt1;
  delete m_Glrt2;
  for (map<int, Cure::LocalGridMap<unsigned char> *>::iterator it =
      m_nodeGridMaps.begin(); it != m_nodeGridMaps.end(); it++) {
    delete it->second;
  }
  delete m_lgm2;
}

void LocalMapManager::configure(const map<string,string>& _config) 
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
    println("configure(...) Failed to get sensor pose");
    std::abort();
  } 

  m_MaxLaserRange = 5.0;
  it = _config.find("--laser-range");
  if (it != _config.end()) {
    m_MaxLaserRange = (atof(it->second.c_str()));
  }

  it = _config.find("--robot-server-host");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_RobotServerHost;
  }

  m_lgm1 = new Cure::LocalGridMap<unsigned char>(70, 0.1, '2', Cure::LocalGridMap<unsigned char>::MAP1);
  m_nodeGridMaps[0] = m_lgm1;
  m_Glrt1  = new Cure::GridLineRayTracer<unsigned char>(*m_lgm1);
  m_lgm2 = new Cure::LocalGridMap<unsigned char>(70, 0.1, '2', Cure::LocalGridMap<unsigned char>::MAP1);
  m_Glrt2  = new Cure::GridLineRayTracer<unsigned char>(*m_lgm2);

  if (_config.find("--no-tentative-window") == _config.end()) {
    m_Displaylgm2 = new Cure::XDisplayLocalGridMap<unsigned char>(*m_lgm2);
    println("Will use X window to show the tentative local map");
  } else {
    m_Displaylgm2 = 0;
    println("Will NOT use X window to show the tentative local map");
  }

  if (_config.find("--no-local-map-window") == _config.end()) {
    m_Displaylgm1 = new Cure::XDisplayLocalGridMap<unsigned char>(*m_lgm1);
    println("Will use X window to show the current local map");
  } else {
    m_Displaylgm1 = 0;
    println("Will NOT use X window to show the current local map");
  }
  
  m_RobotServer = RobotbaseClientUtils::getServerPrx(*this,
                                                     m_RobotServerHost);
  FrontierInterface::HypothesisEvaluatorPtr servant = new EvaluationServer(this);
  registerIceServer<FrontierInterface::HypothesisEvaluator, FrontierInterface::HypothesisEvaluator>(servant);
} 

void LocalMapManager::start() 
{

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
		  new MemberFunctionChangeReceiver<LocalMapManager>(this,
								  &LocalMapManager::newRobotPose));

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<LocalMapManager>(this,
								  &LocalMapManager::newRobotPose));  
  
  log("LocalMapManager started");
  
}

void LocalMapManager::runComponent() 
{
  setupPushScan2d(*this, 0.1);
  setupPushOdometry(*this);

  log("I am running!");

  NavData::FNodePtr curNode = getCurrentNavNode();
  int prevNode = -1;
  while(isRunning()){
    lockComponent(); //Don't allow any interface calls while processing a callback
    curNode = getCurrentNavNode();
    if (curNode != 0) {
      if (curNode->nodeId != prevNode) {
	m_Mutex.lock();
	// Node has changed! See if the node already has a lgm associated
	// with it; if so, swap the current for it. Otherwise,
	// assign the temporary to it.
	if (m_nodeGridMaps.find(curNode->nodeId) == m_nodeGridMaps.end()) {
	  // There's no grid map for the current Node. Assign the 
	  // temporary lgm to it.
	  m_nodeGridMaps[curNode->nodeId] = m_lgm2;
	  log("Movin'");
	  m_lgm2->moveCenterTo(curNode->x, curNode->y);
	  //log("Moved");
	  m_lgm1 = m_lgm2;
	  log("Allocatin'");
	  m_lgm2 = new Cure::LocalGridMap<unsigned char>(70, 0.1, '2', Cure::LocalGridMap<unsigned char>::MAP1, curNode->x, curNode->y);
	  //log("Allocated");
	}
	else {
	  // Clear the temporary lgm
	  m_lgm1 = m_nodeGridMaps[curNode->nodeId];
	  log("Clearin'");
	  m_lgm2->clearMap();
	  //log("Clear'd");
	  log("Movin' 2");
	  m_lgm2->moveCenterTo(curNode->x, curNode->y, false);
	  //log("Moved");
	}
	delete m_Glrt1;
	delete m_Glrt2;
	m_Glrt1  = new Cure::GridLineRayTracer<unsigned char>(*m_lgm1);
	m_Glrt2  = new Cure::GridLineRayTracer<unsigned char>(*m_lgm2);
	//log("Settin' maps");
	if (m_Displaylgm1) {
	  m_Displaylgm1->setMap(m_lgm1);
	}
	if (m_Displaylgm2) {
	  m_Displaylgm2->setMap(m_lgm2);
	}
	//log("Maps set");

	m_Mutex.unlock();
      }
      prevNode = curNode->nodeId;

      //log("Updatin'");
      if (m_Displaylgm1) {
	Cure::Pose3D currentPose = m_TOPP.getPose();
	m_Displaylgm1->updateDisplay(&currentPose);
      }
      if (m_Displaylgm2) {
	Cure::Pose3D currentPose = m_TOPP.getPose();
	m_Displaylgm2->updateDisplay(&currentPose);
      }
      //log("Updated");

    }
    unlockComponent();

    usleep(250000);
  }
}

void LocalMapManager::newRobotPose(const cdl::WorkingMemoryChange &objID) 
{
  lockComponent(); //Don't allow any interface calls while processing a callback

  shared_ptr<CASTData<NavData::RobotPose2d> > oobj =
    getWorkingMemoryEntry<NavData::RobotPose2d>(objID.address);
  
  //FIXME
//   m_SlamRobotPose.setTime(Cure::Timestamp(oobj->getData()->time.s,
//                                           oobj->getData()->time.us));
  m_SlamRobotPose.setX(oobj->getData()->x);
  m_SlamRobotPose.setY(oobj->getData()->y);
  m_SlamRobotPose.setTheta(oobj->getData()->theta);
  
  Cure::Pose3D cp = m_SlamRobotPose;
  m_TOPP.defineTransform(cp);
  
  unlockComponent();
}

void LocalMapManager::receiveOdometry(const Robotbase::Odometry &castOdom)
{
  lockComponent(); //Don't allow any interface calls while processing a callback
  Cure::Pose3D cureOdom;
  CureHWUtils::convOdomToCure(castOdom, cureOdom);

  debug("Got odometry x=%.2f y=%.2f a=%.4f t=%.6f",
        cureOdom.getX(), cureOdom.getY(), cureOdom.getTheta(),
        cureOdom.getTime().getDouble());
  
  m_TOPP.addOdometry(cureOdom);
  
  m_CurrPose = m_TOPP.getPose();
  unlockComponent();
}

void LocalMapManager::receiveScan2d(const Laser::Scan2d &castScan)
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
//      m_lgm->setValueInsideCircle(scanPose.getX(), scanPose.getY(),
//                                  0.5*Cure::NavController::getRobotWidth(), 
//                                  '0');
      lpW.add(scanPose, m_LaserPoseR);
      m_Mutex.lock();
      m_Glrt1->addScan(cureScan, lpW, m_MaxLaserRange);      
      m_Glrt2->addScan(cureScan, lpW, m_MaxLaserRange);      
      m_Mutex.unlock();
    }
  }
  unlockComponent();
}

NavData::FNodePtr
LocalMapManager::getCurrentNavNode()
{
  vector<NavData::FNodePtr> nodes;
  getMemoryEntries<NavData::FNode>(nodes, 0);

  vector<NavData::RobotPose2dPtr> robotPoses;
  getMemoryEntries<NavData::RobotPose2d>(robotPoses, 0);

  if (robotPoses.size() == 0) {
    log("Could not find RobotPose!");
    return 0;
  }
  
  //Find the node closest to the robotPose
  double robotX = robotPoses[0]->x;
  double robotY = robotPoses[0]->y;
  double minDistance = FLT_MAX;
  NavData::FNodePtr ret = 0;

  for (vector<NavData::FNodePtr>::iterator it = nodes.begin();
      it != nodes.end(); it++) {
    double x = (*it)->x;
    double y = (*it)->y;

    double distance = (x - robotX)*(x-robotX) + (y-robotY)*(y-robotY);
    if (distance < minDistance) {
      ret = *it;
      minDistance = distance;
    }
  }
  return ret;
}

FrontierInterface::HypothesisEvaluation
LocalMapManager::getHypothesisEvaluation(int hypID)
{
  // Find all hypotheses that have curPlaceID as source place
  // For each cell free, find which hypothesis or source node is closest
  // If the requested hypothesis is the one, also check if there are any extant
  // nodes closer than a threshold. If not, increment the free space value.
  // If the cell is also on the boundary to unexplored space, increment
  // the border value.
  // Then check if the hypothesis is situated in what looks like door-like 
  // environs.

  FrontierInterface::HypothesisEvaluation ret;
  ret.freeSpaceValue = 0;
  ret.unexploredBorderValue = 0;
  ret.gatewayValue = 0;

  vector<FrontierInterface::NodeHypothesisPtr> hyps;
  getMemoryEntries<FrontierInterface::NodeHypothesis>(hyps);

  int originPlaceID = -1;
  double relevantX;
  double relevantY;
  for (vector<FrontierInterface::NodeHypothesisPtr>::iterator it = hyps.begin();
      it != hyps.end(); it++) {
    try {
      if ((*it)->hypID == hypID) {
	originPlaceID = (*it)-> originPlaceID;
	relevantX = (*it)->x;
	relevantY = (*it)->y;
	break;
      }
    }
    catch (IceUtil::NullHandleException) {
      log("Hypothesis unexpectedly missing from WM!");
    }
  }

  vector<FrontierInterface::NodeHypothesisPtr> relevantHypotheses;

  if (originPlaceID != -1) {
    for (vector<FrontierInterface::NodeHypothesisPtr>::iterator it = hyps.begin();
	it != hyps.end(); it++) {
      try {
	if ((*it)->originPlaceID == originPlaceID) {
	  relevantHypotheses.push_back(*it);
	}
      }
      catch (IceUtil::NullHandleException) {
	log("Hypothesis unexpectedly missing from WM!");
      }
    }
    log("Relevant hypotheses: %i", relevantHypotheses.size());

    vector<NavData::FNodePtr> nodes;
    getMemoryEntries<NavData::FNode>(nodes);

    int totalFreeCells = 0;
    //Loop over local grid map
    double x;
    double y;
    m_Mutex.lock();
    log("Checkin'");
    for (int yi = -m_lgm1->getSize(); yi < m_lgm1->getSize()+1; yi++) {
      y = m_lgm1->getCentYW() + yi * m_lgm1->getCellSize();
      for (int xi = -m_lgm1->getSize(); xi < m_lgm1->getSize()+1; xi++) {
	x = m_lgm1->getCentXW() + xi * m_lgm1->getCellSize();
	if ((*m_lgm1)(xi,yi) == '0') {
	  totalFreeCells++;
	  //Cell is free
	  double relevantDistSq = (x - relevantX)*(x - relevantX) +
	    (y - relevantY)*(y - relevantY);
	  // Check if some other hypothesis from this Place is closer to this
	  // grid cell

	  bool process = true;
	  for(vector<FrontierInterface::NodeHypothesisPtr>::iterator it = 
	      relevantHypotheses.begin(); process && it != relevantHypotheses.end(); it++) {
	    if ((*it)->hypID != hypID) {
	      double distSq = (x - (*it)->x)*(x - (*it)->x) +
		(y - (*it)->y)*(y - (*it)->y);
	      if (distSq < relevantDistSq)
		process = false;
	    }
	  }

	  if (process) {
	    // Check if this cell is within the node creation radius of some
	    // other node
	    for (vector<NavData::FNodePtr>::iterator it2 = nodes.begin();
		it2 != nodes.end(); it2++) {
	      try {
		double distSq = (x - (*it2)->x)*(x - (*it2)->x) +
		  (y - (*it2)->y)*(y - (*it2)->y);
		if (distSq < 2.0*2.0)
		  process = false;
	      }
	      catch (IceUtil::NullHandleException e) {
		log("Error! FNode unexpectedly disappeared!");
	      }
	    }
	    if (process) {
	      //This cell is known to be free, closest to the hypothesis
	      //in question, and not too close to another node. Potential
	      //new node material.
	      ret.freeSpaceValue += 1.0;

	      //Check if it borders on unknown space
	      for (int yi2 = yi - 1; yi2 <= yi + 1; yi2++) {
		for (int xi2 = xi - 1; xi2 <= xi + 1; xi2++) {
		  if ((*m_lgm1)(xi2, yi2) == '2') {
		    ret.unexploredBorderValue += 1.0;
		    yi2 = yi + 2;
		    xi2 = xi + 2;
		  }
		}
	      }

	    }

	  }
	}
      }
    }
    log("Total free cells: %i", totalFreeCells);
    m_Mutex.unlock();
  }
  return ret;
}

FrontierInterface::HypothesisEvaluation
LocalMapManager::EvaluationServer::getHypothesisEvaluation(int hypID, 
    const Ice::Current &_context)
{
  m_pOwner->lockComponent();
  FrontierInterface::HypothesisEvaluation ret =
    m_pOwner->getHypothesisEvaluation(hypID);
  m_pOwner->unlockComponent();
  return ret;
}

