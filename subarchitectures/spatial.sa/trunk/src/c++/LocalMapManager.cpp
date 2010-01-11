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
#include "GridObjectFinder.hpp"

#include <AddressBank/ConfigFileReader.hh>
#include <RobotbaseClientUtils.hpp>
#include <float.h>
#include <NavX/XDisplayLocalGridMap.hh>
#include <VisionData.hpp>

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
  m_RobotServerHost = "localhost";
}

LocalMapManager::~LocalMapManager() 
{ 
  delete m_Displaylgm1;
  delete m_Displaylgm2;
  delete m_DisplayPlaneMap;
  delete m_Glrt1;
  delete m_Glrt2;
  delete m_planeObstacleMap;

  for (map<int, CharMap *>::iterator it =
      m_nodeGridMaps.begin(); it != m_nodeGridMaps.end(); it++) {
    delete it->second;
  }
  delete m_lgm2;

  if (m_isUsingSeparateGridMaps) {
    delete m_Glrt1_alt;
    delete m_Glrt2_alt;
    for (map<int, CharMap *>::iterator it =
	m_nodeGridMapsAlt.begin(); it != m_nodeGridMapsAlt.end(); it++) {
      delete it->second;
    }
    delete m_lgm2_alt;
  }
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
    println("configure(...) Failed to get sensor pose for laser");
    std::abort();
  } 

  m_bNoPlanes = true;
  
  if (_config.find("--no-planes") == _config.end()) {
    log("Trying to detect planes...");
    m_bNoPlanes = false;
    if (cfg.getSensorPose(2, m_CameraPoseR)) {
      println("configure(...) Failed to get sensor pose for camera. (Run with --no-planes to skip)");
      std::abort();
    } 
  }
  double coords[6];
  m_CameraPoseR.getCoordinates(coords);
  log("m_CameraPoseR = (%f, %f, %f)", coords[0], coords[1], coords[2]);

  m_MaxLaserRangeForPlaceholders = 5.0;
  m_MaxLaserRangeForCombinedMaps = 5.0;
  it = _config.find("--laser-range");
  if (it != _config.end()) {
    m_MaxLaserRangeForPlaceholders = (atof(it->second.c_str()));
    m_MaxLaserRangeForCombinedMaps = m_MaxLaserRangeForPlaceholders;
  }
  else {
    it = _config.find("--laser-range-for-placeholders");
    if (it != _config.end()) {
      m_MaxLaserRangeForPlaceholders = (atof(it->second.c_str()));
    }
    it = _config.find("--laser-range-for-combined-maps");
    if (it != _config.end()) {
      m_MaxLaserRangeForCombinedMaps = (atof(it->second.c_str()));
    }
  }
  m_isUsingSeparateGridMaps = 
    m_MaxLaserRangeForPlaceholders != m_MaxLaserRangeForCombinedMaps;

  it = _config.find("--robot-server-host");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_RobotServerHost;
  }

  PlaneData mpty;
  m_planeMap = new PlaneMap(70, 0.1, mpty, PlaneMap::MAP1);

  m_lgm1 = new CharMap(70, 0.1, '2', CharMap::MAP1);
  m_nodeGridMaps[0] = m_lgm1;
  m_Glrt1  = new CharGridLineRayTracer(*m_lgm1);
  m_lgm2 = new CharMap(70, 0.1, '2', CharMap::MAP1);
  m_Glrt2  = new CharGridLineRayTracer(*m_lgm2);

  m_planeObstacleMap = new CharMap(70, 0.1, '2', CharMap::MAP1);
  m_DisplayPlaneMap = new Cure::XDisplayLocalGridMap<unsigned char>(*m_planeObstacleMap);

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

  if (m_isUsingSeparateGridMaps) {
    m_lgm1_alt = new CharMap(70, 0.1, '2', CharMap::MAP1);
    m_nodeGridMapsAlt[0] = m_lgm1_alt;
    m_Glrt1_alt  = new CharGridLineRayTracer(*m_lgm1_alt);
    m_lgm2_alt = new CharMap(70, 0.1, '2', CharMap::MAP1);
    m_Glrt2_alt  = new CharGridLineRayTracer(*m_lgm2_alt);
  }

  m_RobotServer = RobotbaseClientUtils::getServerPrx(*this,
      m_RobotServerHost);
  FrontierInterface::HypothesisEvaluatorPtr servant = new EvaluationServer(this);
  registerIceServer<FrontierInterface::HypothesisEvaluator, FrontierInterface::HypothesisEvaluator>(servant);

  FrontierInterface::LocalMapInterfacePtr mapservant = new LocalMapServer(this);
  registerIceServer<FrontierInterface::LocalMapInterface, FrontierInterface::LocalMapInterface>(mapservant);
} 

void LocalMapManager::start() 
{

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
		  new MemberFunctionChangeReceiver<LocalMapManager>(this,
								  &LocalMapManager::newRobotPose));

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<LocalMapManager>(this,
								  &LocalMapManager::newRobotPose));  
  
  addChangeFilter(createGlobalTypeFilter<VisionData::ConvexHull>(cdl::ADD),
      new MemberFunctionChangeReceiver<LocalMapManager>(this,
	&LocalMapManager::newConvexHull));

  addChangeFilter(createGlobalTypeFilter<VisionData::ConvexHull>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<LocalMapManager>(this,
	&LocalMapManager::newConvexHull));

  m_placeInterface = getIceServer<FrontierInterface::PlaceInterface>("place.manager");
  log("LocalMapManager started");

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

void LocalMapManager::runComponent() 
{
  setupPushScan2d(*this, 0.1);
  setupPushOdometry(*this);

  log("I am running!");

  NavData::FNodePtr curNode = getCurrentNavNode();
  int prevNode = -1;
  while(isRunning()){
    debug("lock in isRunning");
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
	  m_lgm1 = m_lgm2;
	  log("Allocatin'");
	  m_lgm2 = new CharMap(70, 0.1, '2', CharMap::MAP1, curNode->x, curNode->y);

	  if (m_isUsingSeparateGridMaps) {
	    m_nodeGridMapsAlt[curNode->nodeId] = m_lgm2_alt;
	    m_lgm2_alt->moveCenterTo(curNode->x, curNode->y);
	    m_lgm1_alt = m_lgm2_alt;
	    m_lgm2_alt = new CharMap(70, 0.1, '2', CharMap::MAP1, curNode->x, curNode->y);
	  }
	}
	else {
	  // Clear the temporary lgm
	  m_lgm1 = m_nodeGridMaps[curNode->nodeId];
	  log("Clearin'");
	  m_lgm2->clearMap();
	  log("Movin' 2");
	  m_lgm2->moveCenterTo(curNode->x, curNode->y, false);

	  if (m_isUsingSeparateGridMaps){
	    m_lgm1_alt = m_nodeGridMapsAlt[curNode->nodeId];
	    m_lgm2_alt->clearMap();
	    m_lgm2_alt->moveCenterTo(curNode->x, curNode->y, false);
	  }
	}
	delete m_Glrt1;
	delete m_Glrt2;
	m_Glrt1  = new CharGridLineRayTracer(*m_lgm1);
	m_Glrt2  = new CharGridLineRayTracer(*m_lgm2);

	if (m_isUsingSeparateGridMaps) {
	  delete m_Glrt1_alt;
	  delete m_Glrt2_alt;
	  m_Glrt1_alt  = new CharGridLineRayTracer(*m_lgm1_alt);
	  m_Glrt2_alt  = new CharGridLineRayTracer(*m_lgm2_alt);
	}

	if (m_Displaylgm1) {
	  //m_Displaylgm1 = new Cure::XDisplayLocalGridMap<unsigned char>(*m_lgm1);
	  m_Displaylgm1->setMap(m_lgm1);
	}
	if (m_Displaylgm2) {
	  m_Displaylgm2->setMap(m_lgm2);
	}
	if (m_DisplayPlaneMap) {
	  m_DisplayPlaneMap->setMap(m_planeObstacleMap);
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
    if (m_DisplayPlaneMap) {
      Cure::Pose3D currentPose = m_TOPP.getPose();
      m_DisplayPlaneMap->updateDisplay(&currentPose);
    }
    unlockComponent();
    debug("unlock in isRunning");

    usleep(250000);
  }
}

void LocalMapManager::newRobotPose(const cdl::WorkingMemoryChange &objID) 
{
  try {
    lastRobotPose =
      getMemoryEntry<NavData::RobotPose2d>(objID.address);
  }
  catch (DoesNotExistOnWMException e) {
    log("Error! robotPose missing on WM!");
  }

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
      lpW.add(scanPose, m_LaserPoseR);
      m_Mutex.lock();
      m_Glrt1->addScan(cureScan, lpW, m_MaxLaserRangeForCombinedMaps);      
      m_Glrt2->addScan(cureScan, lpW, m_MaxLaserRangeForCombinedMaps);      
      if (m_isUsingSeparateGridMaps) {
	m_Glrt1_alt->addScan(cureScan, lpW, m_MaxLaserRangeForPlaceholders);      
	m_Glrt2_alt->addScan(cureScan, lpW, m_MaxLaserRangeForPlaceholders);      
      }
      m_firstScanReceived = true;
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

  while (!m_firstScanReceived) {
    log("Sleeping, waiting for initial scan");
    unlockComponent();
    usleep(500000);
    lockComponent();
  }

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
    int totalProcessed = 0;
    //Loop over local grid map
    double x;
    double y;
    m_Mutex.lock();
    log("Checkin'");

    CharMap *propertyLGM =
      m_isUsingSeparateGridMaps ? m_lgm1_alt : m_lgm1;

    for (int yi = -propertyLGM->getSize(); yi < propertyLGM->getSize()+1; yi++) {
      y = propertyLGM->getCentYW() + yi * propertyLGM->getCellSize();
      for (int xi = -propertyLGM->getSize(); xi < propertyLGM->getSize()+1; xi++) {
	x = propertyLGM->getCentXW() + xi * propertyLGM->getCellSize();
	if ((*propertyLGM)(xi,yi) == '0') {
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
	    totalProcessed++;
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
		  if ((*propertyLGM)(xi2, yi2) == '2') {
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
    log("Total free cells: %i; total processed %i", totalFreeCells, totalProcessed);
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

FrontierInterface::LocalGridMap
LocalMapManager::LocalMapServer::getCombinedGridMap(const SpatialData::PlaceIDSeq &places,
    const Ice::Current &_context)
{
  m_pOwner->lockComponent();
  FrontierInterface::LocalGridMap ret; 
  m_pOwner->log("3");
  m_pOwner->getCombinedGridMap(ret, places);
  m_pOwner->log("4");
  m_pOwner->unlockComponent();
  return ret;
}

void 
LocalMapManager::getCombinedGridMap(FrontierInterface::LocalGridMap &map, 
    const SpatialData::PlaceIDSeq &places)
{
  vector<const CharMap *>maps;
  for (SpatialData::PlaceIDSeq::const_iterator it = places.begin(); it != places.end(); it++) {
    NavData::FNodePtr node = m_placeInterface->getNodeFromPlaceID(*it);
    if (node != 0) {
      if (m_nodeGridMaps.find(node->nodeId) != m_nodeGridMaps.end()) {
	maps.push_back(m_nodeGridMaps[node->nodeId]);
      }
      else {
	log("Couldn't find grid map for node %d", node->nodeId);
      }
    }
    else {
      log ("No grid map for Place %d; not a node!", *it);
    }
  }

  if (maps.empty()) {
    map.size = 0;
    map.data.clear();
    return;
  }

  double minx = FLT_MAX;
  double maxx = -FLT_MAX;
  double miny = FLT_MAX;
  double maxy = -FLT_MAX;

  //Find bounds of the merged grid map
  for (vector<const CharMap *>::iterator it = maps.begin();
      it != maps.end(); it++) {
    double mapminx = (*it)->getCentXW() - (*it)->getSize() * (*it)->getCellSize();
    double mapmaxx = (*it)->getCentXW() + (*it)->getSize() * (*it)->getCellSize();
    double mapminy = (*it)->getCentYW() - (*it)->getSize() * (*it)->getCellSize();
    double mapmaxy = (*it)->getCentYW() + (*it)->getSize() * (*it)->getCellSize();
    minx = minx < mapminx ? minx : mapminx;
    maxx = maxx > mapmaxx ? maxx : mapmaxx;
    miny = miny < mapminy ? miny : mapminy;
    maxy = maxy > mapmaxy ? maxy : mapmaxy;
  }

  double dsize = maxx-minx;
  double cellSize = maps[0]->getCellSize();
  double cx = (minx+maxx)/2;
  double cy = (miny+maxy)/2;

  if (maxy-miny > dsize) 
    dsize = maxy-miny;
  int newSize = (int)((dsize/2 + cellSize/2) / cellSize);
  CharMap newMap(newSize, cellSize, '2',
      CharMap::MAP1, cx, cy);

  map.xCenter = cx;
  map.yCenter = cy;
  map.cellSize = cellSize;
  map.size = newSize;
  map.data.clear();
  map.data.reserve(4*newSize*newSize + 4*newSize + 1);

  //Sample each of the maps into the new map
  log("Sample each of the maps into the new map");
  for (vector<const CharMap *>::iterator it = maps.begin();
      it != maps.end(); it++) {
    log("looping over map");
    for (int y = -(*it)->getSize(); y <= (*it)->getSize(); y++) {
      for (int x = -(*it)->getSize(); x <= (*it)->getSize(); x++) {

	char val = (**it)(x,y);

	if (val != '2') {
	  double dx, dy;
	  (*it)->index2WorldCoords(x, y, dx, dy);
	  int nx, ny;
	  if (newMap.worldCoords2Index(dx, dy, nx, ny)) {
	    log("Error! bounds incorrect in getCombinedGridMaps!");
	  }
	  else {
	    if (newMap(nx, ny) == '2' || newMap(nx, ny) == '0') {
	      newMap(nx, ny) = val;
	    }
	  }
	}
      }
    }
  }

  for (int x = -newSize ; x <= newSize; x++){
    for (int y = -newSize ; y <= newSize; y++){
      map.data.push_back(newMap(x,y));
    }
  }

  CharMap newMap2(newSize, cellSize, '2',
      CharMap::MAP1, cx, cy);

  int lp = 0;
  for(int x = -map.size ; x <= map.size; x++){
    for(int y = -map.size ; y <= map.size; y++){ 
      (newMap2)(x,y) = map.data[lp];
      lp++;
    }
  }
  Cure::XDisplayLocalGridMap<unsigned char>* m_Displaykrsjlgm;
  m_Displaykrsjlgm = new Cure::XDisplayLocalGridMap<unsigned char>(newMap2);
  m_Displaykrsjlgm->updateDisplay();

}

void LocalMapManager::newConvexHull(const cdl::WorkingMemoryChange
				   &objID){
  if (!m_bNoPlanes) {

    debug("newConvexHull called");
    VisionData::ConvexHullPtr oobj =
      getMemoryEntry<VisionData::ConvexHull>(objID.address);

    log("Convex hull center at %f, %f, %f", oobj->center.x,oobj->center.y,oobj->center.z);
    Cure::Transformation3D cam2WorldTrans =
      getCameraToWorldTransform();
    double tmp[6];
    cam2WorldTrans.getCoordinates(tmp);
    log("total transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
	tmp[4], tmp[5]);

    //Convert hull to world coords
    for (unsigned int i = 0; i < oobj->PointsSeq.size(); i++) {
      Cure::Vector3D from(oobj->PointsSeq[i].x, oobj->PointsSeq[i].y, oobj->PointsSeq[i].z);
      Cure::Vector3D to;
      cam2WorldTrans.invTransform(from, to);
      //    log("vertex at %f, %f, %f", oobj->PointsSeq[i].x, oobj->PointsSeq[i].y, oobj->PointsSeq[i].z);
      oobj->PointsSeq[i].x = to.X[0];
      oobj->PointsSeq[i].y = to.X[1];
      oobj->PointsSeq[i].z = to.X[2];
      //    log("Transformed vertex at %f, %f, %f", to.X[0], to.X[1], to.X[2]);
    }
    Cure::Vector3D from(oobj->center.x, oobj->center.y, oobj->center.z);
    Cure::Vector3D to;
    cam2WorldTrans.invTransform(from, to);
    oobj->center.x = to.X[0];
    oobj->center.y = to.X[1];
    oobj->center.z = to.X[2];
    cam2WorldTrans.getCoordinates(tmp);


    // Filter polygons that are not horizontal planes


    // Paint the polygon in the grid map

    log("Painting polygon");
    PaintPolygon(oobj->PointsSeq);

    for (int x = -m_planeObstacleMap->getSize(); x <= m_planeObstacleMap->getSize(); x++) {
      for (int y = -m_planeObstacleMap->getSize(); y <= m_planeObstacleMap->getSize(); y++) {
	if ((*m_planeMap)(x,y).planes.size() > 0) {
	  (*m_planeObstacleMap)(x,y) = '1';
	}
      }
    }

    //m_planeMap->print(std::cout);
  }
}

Cure::Transformation3D LocalMapManager::getCameraToWorldTransform()
{
  //Get camera ptz from PTZServer
  Cure::Transformation3D cameraRotation;
  if (m_ptzInterface != 0) {
    ptz::PTZReading reading = m_ptzInterface->getPose();

    double angles[] = {reading.pose.pan, -reading.pose.tilt, 0.0};
    cameraRotation.setAngles(angles);
  }
//
//  //Get camera position on robot from Cure
//  m_CameraPoseR;

  //Additional transform of ptz base frame
  Cure::Transformation3D ptzBaseTransform;
//  double nR[] = {0.0, 0.0, 1.0, 
//    1.0, 0.0, 0.0,
//    0.0, 1.0, 0.0};
  double camAngles[] = {-M_PI/2, 0.0, -M_PI/2};
  ptzBaseTransform.setAngles(camAngles);

  //Get robot pose
  Cure::Transformation2D robotTransform;
  if (lastRobotPose != 0) {
    robotTransform.setXYTheta(lastRobotPose->x, lastRobotPose->y,
      lastRobotPose->theta);
  }
  Cure::Transformation3D robotTransform3 = robotTransform;
  double tmp[6];
  robotTransform3.getCoordinates(tmp);
  log("robot transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
      tmp[4], tmp[5]);
  cameraRotation.getCoordinates(tmp);
  log("ptz transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
      tmp[4], tmp[5]);
  m_CameraPoseR.getCoordinates(tmp);
  log("cam transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
      tmp[4], tmp[5]);

  Cure::Transformation3D cameraOnRobot = m_CameraPoseR + cameraRotation + ptzBaseTransform ;
  return robotTransform3 + cameraOnRobot;
} 

struct ScanLine {
	int small_x, large_x;
};

//This function adapted from code by Michael Y. Polyakov:
///////////////////////////////////////////////////////////////////////////////////////
//
//	FileName:	ConvexPolRas.cpp
//	Author	:	Michael Y. Polyakov
//	email	:	myp@andrew.cmu.edu	or  mikepolyakov@hotmail.com
//	Website	:	www.angelfire.com/linux/myp
//	Date	:	7/29/2002
//
//	Rasterizes a convex polygon. x/y - arrays of vertices of size num.
//	drawPoint - function which draws a point at (x,y). Should be provided before hand.
//	Shceck out the tutorial on my website.
///////////////////////////////////////////////////////////////////////////////////////

void LocalMapManager::PaintPolygon(const VisionData::Vector3Seq &points)
{
  const int cellfactor = 5;
  int num = (int)points.size();
  int *y = new int[num];
  int *x = new int[num];
  double cellSize = m_planeMap->getCellSize();
  double miniCellSize = cellSize / cellfactor; //Supersampling
  double XCentW = m_planeMap->getCentXW();
  double YCentW = m_planeMap->getCentYW();
  int size = m_planeMap->getSize();
  double height = points[0].z;
  log("Height: %f", height);

  for (int i = 0; i < num; i++) {
    x[i] = (int((points[i].x - XCentW)/(miniCellSize/2.0)) + (points[i].x >= XCentW ? 1: -1)) / 2;
    y[i] = (int((points[i].y - YCentW)/(miniCellSize/2.0)) + (points[i].y >= YCentW ? 1: -1)) / 2;
  }

  int small_y = y[0], large_y = y[0];	//small and large y's
  int xc, yc;							//current x/y points
  ScanLine *sl;						//array of structs - contain small/large x for each y that is drawn
  int delta_y;						//large_y - small_y + 1 (size of the above array)
  int i, j, ind;
  //line information (see LineRas.cpp for details)
  int dx, dy, shortD, longD;
  int incXH, incXL, incYH, incYL;
  int d, incDL, incDH;

  /* Step 1: find small and large y's of all the vertices */
  for(i=1; i < num; i++) {
    if(y[i] < small_y) small_y = y[i];
    else if(y[i] > large_y) large_y = y[i];
  }

  /* Step 2: array that contains small_x and large_x values for each y. */
  delta_y = large_y - small_y + 1;	
  sl = new ScanLine[delta_y];			//allocate enough memory to save all y values, including large/small

  for(i=0; i < delta_y; i++) {		//initialize the ScanLine array
    sl[i].small_x = INT_MAX;		//INT_MAX because all initial values are less
    sl[i].large_x = INT_MIN;		//INT_MIN because all initial values are greater
  }


  /* Step 3: go through all the lines in this polygon and build min/max x array. */
  for(i=0; i < num; i++) {
    ind = (i+1)%num;				//last line will link last vertex with the first (index num-1 to 0)
    if(y[ind]-y[i]) 
    {
      //initializing current line data (see tutorial on line rasterization for details)
      dx = x[ind] - x[i]; dy = y[ind] - y[i];
      if(dx >= 0) incXH = incXL = 1; else { dx = -dx; incXH = incXL = -1; }
      if(dy >= 0) incYH = incYL = 1; else { dy = -dy; incYH = incYL = -1; }
      if(dx >= dy) { longD = dx;  shortD = dy;  incYL = 0; }
      else		 { longD = dy;  shortD = dx;  incXL = 0; }
      d = 2*shortD - longD;
      incDL = 2*shortD;
      incDH = 2*shortD - 2*longD;

      xc = x[i]; yc = y[i];		//initial current x/y values
      for(j=0; j <= longD; j++) {	//step through the current line and remember min/max values at each y
	ind = yc - small_y;
	if(xc < sl[ind].small_x) sl[ind].small_x = xc;	//initial contains INT_MAX so any value is less
	if(xc > sl[ind].large_x) sl[ind].large_x = xc;	//initial contains INT_MIN so any value is greater
	//finding next point on the line ...
	if(d >= 0)	{ xc += incXH; yc += incYH; d += incDH; }	//H-type
	else		{ xc += incXL; yc += incYL; d += incDL; }	//L-type
      }
    } //end if
  } //end i for loop

  /* Step 4: drawing horizontal line for each y from small_x to large_x including. */
  for(i=0; i < delta_y; i++)
  {
    for(j=sl[i].small_x; j <= sl[i].large_x; j++)
    {
      int xindex = j / cellfactor;
      int yindex = i / cellfactor;

      if (xindex >= -size && xindex <= size && yindex >= -size && yindex <= size) {
	PlaneData &pd = (*m_planeMap)(xindex, yindex);
	PlaneList::iterator it = pd.planes.begin();
	for (; it != pd.planes.end(); it++) {
	  if (abs(it->first - height) < 0.05) {
	    //Reinforce this plane
	    double newHeight = (it->first * it->second + height * 1.0)/(it->second + 1.0);
	    double newConfidence = it->second + 1.0;
	    it->first = newHeight;
	    it->second = newConfidence;
	    break;
	  }
	}
	if (it == pd.planes.end()) {
	  //Plane was new, add it
	  pd.planes.push_back(std::pair<double, double>(height, 1.0));
	}
      }
    }
  }
  delete x;
  delete y;

  delete [] sl;	//previously allocated space for ScanLine array
}
