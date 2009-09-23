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
{ }



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

  m_lgm = new Cure::LocalGridMap<unsigned char>(200, 0.1, '2', Cure::LocalGridMap<unsigned char>::MAP1);
  m_Glrt  = new Cure::GridLineRayTracer<unsigned char>(*m_lgm);
  //m_Explorer->setExplorationConfinedByGateways(true);


  if (_config.find("--no-x-window") == _config.end()) {
    m_Displaylgm = new Cure::XDisplayLocalGridMap<unsigned char>(*m_lgm);
    println("Will use X window to show the map");
  } else {
    m_Displaylgm = 0;
    println("Will NOT use X window to show the map");
  }

  m_RobotServer = RobotbaseClientUtils::getServerPrx(*this,
                                                     m_RobotServerHost);
} 

void LocalMapManager::start() 
{
  FrontierInterface::HypothesisEvaluatorPtr servant = new EvaluationServer(this);
  registerIceServer<FrontierInterface::HypothesisEvaluator, FrontierInterface::HypothesisEvaluator>(servant);

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
  
  while(isRunning()){
    if (m_Displaylgm) {
      Cure::Pose3D currentPose = m_TOPP.getPose();
      m_Displaylgm->updateDisplay(&currentPose);
    }

    usleep(250000);
  }
}

void LocalMapManager::newRobotPose(const cdl::WorkingMemoryChange &objID) 
{

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
  
}

void LocalMapManager::receiveOdometry(const Robotbase::Odometry &castOdom)
{
  Cure::Pose3D cureOdom;
  CureHWUtils::convOdomToCure(castOdom, cureOdom);

  debug("Got odometry x=%.2f y=%.2f a=%.4f t=%.6f",
        cureOdom.getX(), cureOdom.getY(), cureOdom.getTheta(),
        cureOdom.getTime().getDouble());
  
  m_TOPP.addOdometry(cureOdom);
  
  m_CurrPose = m_TOPP.getPose();
}

void LocalMapManager::receiveScan2d(const Laser::Scan2d &castScan)
{
  debug("Got scan with n=%d and t=%ld.%06ld",
        castScan.ranges.size(), 
        (long)castScan.time.s, (long)castScan.time.us);

  Cure::LaserScan2d cureScan;
  CureHWUtils::convScan2dToCure(castScan, cureScan);

  if (m_TOPP.isTransformDefined()) {
    
    Cure::Pose3D scanPose;
    if (m_TOPP.getPoseAtTime(cureScan.getTime(), scanPose) == 0) {
      m_Mutex.lock();
      m_LMap.addScan(cureScan, m_LaserPoseR, scanPose);
      m_Mutex.unlock();

      Cure::Pose3D lpW;
//      m_lgm->setValueInsideCircle(scanPose.getX(), scanPose.getY(),
//                                  0.5*Cure::NavController::getRobotWidth(), 
//                                  '0');
      lpW.add(scanPose, m_LaserPoseR);
      m_Glrt->addScan(cureScan, lpW, m_MaxLaserRange);      
    }
  }
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
    log("Relevant hypotheses: %i", hyps.size());

    vector<NavData::FNodePtr> nodes;
    getMemoryEntries<NavData::FNode>(nodes);

    int totalFreeCells = 0;
    //Loop over local grid map
    double x;
    double y;
    for (int yi = -m_lgm->getSize(); yi < m_lgm->getSize()+1; yi++) {
      y = m_lgm->getCentYW() + yi * m_lgm->getCellSize();
      for (int xi = -m_lgm->getSize(); xi < m_lgm->getSize()+1; xi++) {
	x = m_lgm->getCentXW() + xi * m_lgm->getCellSize();
	if ((*m_lgm)(xi,yi) == '0') {
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
		  if ((*m_lgm)(xi2, yi2) == '2') {
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
  }
  return ret;
}

FrontierInterface::HypothesisEvaluation
LocalMapManager::EvaluationServer::getHypothesisEvaluation(int hypID, 
    const Ice::Current &_context)
{
  return m_pOwner->getHypothesisEvaluation(hypID);
}

