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

#include <peekabot.hh>
#include <peekabot/Types.hh>

#include <AddressBank/ConfigFileReader.hh>
#include <RobotbaseClientUtils.hpp>
#include <float.h>
#include <NavX/XDisplayLocalGridMap.hh>
#include <VisionData.hpp>
#include <stdlib.h>
#include "scanmap/HSS/HSSutils.hh"
#include "scanmap/HSS/HSSDisplayFunctions.hh"


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

LocalMapManager::LocalMapManager():m_planeProcessingCooldown(true)
{
  m_placeInterface = 0;
  m_currentNumberOfClusters = 0;
  m_standingStillThreshold = 0.2;
  m_maxClusterDeviation = 0.1;
  m_RobotServerName = "robot.server";
  m_maxNumberOfClusters = 3;
  m_doorwayWidth = 4.0;
}

LocalMapManager::~LocalMapManager() 
{
//  delete m_Displaylgm1;
//  delete m_Displaylgm2;
//  delete m_DisplayPlaneMap;
  delete m_Glrt1;
  delete m_Glrt2;
  for (std::vector<CharMap *>::iterator it = m_planeObstacleMaps.begin();
      it != m_planeObstacleMaps.end(); it++) {
    delete *it;
  }

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

  if (!m_bNoPlanes) {
    for (vector<GridObjectFinder*>::iterator it = m_planeObjectFinders.begin();
	it != m_planeObjectFinders.end(); it++) {
      delete *it;
    }
  }
}

void LocalMapManager::configure(const map<string,string>& _config) 
{ 
  log("configure...");
  map<string,string>::const_iterator it = _config.find("-c");
  if (it== _config.end()) {
    println("configure(...) Need config file (use -c option)\n");
    std::abort();
  }
  std::string configfile = it->second;

  m_PbPort = 5050;
  m_PbHost = "localhost";

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

  m_loadNodeLgms = false;
  m_saveNodeLgms = false;
  if (_config.find("--save-nodemap") != _config.end()) {
    m_saveNodeLgms = true;
  }

  if (_config.find("--load-nodemap") != _config.end()) {
    m_loadNodeLgms = true;
    m_saveNodeLgms = false;
  }

  m_bNoPlaces = false;
  if (_config.find("--no-places") != _config.end()) {
    m_bNoPlaces = true;
  }

  m_bDetectDoors = false;
  if (_config.find("--detect-doors") != _config.end()) {
    m_bDetectDoors = true;
  }

  m_bShowDoorsInPB = false;
  if (_config.find("--display-doors") != _config.end()) {
    m_bShowDoorsInPB = true;
  }

  m_bNoPlanes = true;
  m_bNoPTZ = true;

  if (_config.find("--no-planes") == _config.end()) {
    log("Trying to detect planes...");
    m_bNoPlanes = false;
    if (cfg.getSensorPose(2, m_CameraPoseR)) {
      println("configure(...) Failed to get sensor pose for camera. (Run with --no-planes to skip)");
      std::abort();
    } 

    if (_config.find("--no-ptz") == _config.end()) {
      m_bNoPTZ = false;
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

  it = _config.find("--robot-server");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_RobotServerName;
  }

  it = _config.find("--max-clusters");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_maxNumberOfClusters;
  }

  it = _config.find("--max-cluster-deviation");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_maxClusterDeviation;
  }

  it = _config.find("--standing-still-threshold");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_standingStillThreshold;
  }

  it = _config.find("--doorway-width");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_doorwayWidth;
  }

  m_planeObjectFilename = "";
  it = _config.find("--plane-object-file");
  if (it != _config.end()) {
    m_planeObjectFilename = it->second;
  }

  m_planeModelFilename = "";
  it = _config.find("--plane-model-file");
  if (it != _config.end()) {
    m_planeModelFilename = it->second;
  }

  int MapSize = 70;
  double CellSize = 0.1;
  it = _config.find("--planemap-size");
  if (it != _config.end()) {
    MapSize= (atoi(it->second.c_str()));
  }
  it = _config.find("--planemap-cellsize");
  if (it != _config.end()) {
    CellSize = (atof(it->second.c_str()));
  }
  m_mapsize = MapSize;
  m_cellsize = CellSize;
  PlaneData mpty;
  m_planeMap = new PlaneMap(MapSize, CellSize, mpty, PlaneMap::MAP1);

  m_lgm1 = new CharMap(MapSize, CellSize, '2', CharMap::MAP1);
  m_nodeGridMaps[0] = m_lgm1;
  m_Glrt1  = new CharGridLineRayTracer(*m_lgm1);
  m_lgm2 = new CharMap(MapSize, CellSize, '2', CharMap::MAP1);
  m_Glrt2  = new CharGridLineRayTracer(*m_lgm2);

  if (_config.find("--tentative-window") != _config.end()) {
    m_Displaylgm2 = new Cure::XDisplayLocalGridMap<unsigned char>(*m_lgm2);
    println("Will use X window to show the tentative local map");
  } else {
    m_Displaylgm2 = 0;
    println("Will NOT use X window to show the tentative local map");
  }

  if (_config.find("--local-map-window") != _config.end()) {
    m_Displaylgm1 = new Cure::XDisplayLocalGridMap<unsigned char>(*m_lgm1);
    println("Will use X window to show the current local map");
  } else {
    m_Displaylgm1 = 0;
    println("Will NOT use X window to show the current local map");
  }

  if (m_isUsingSeparateGridMaps) {
    m_lgm1_alt = new CharMap(MapSize, CellSize, '2', CharMap::MAP1);
    m_nodeGridMapsAlt[0] = m_lgm1_alt;
    m_Glrt1_alt  = new CharGridLineRayTracer(*m_lgm1_alt);
    m_lgm2_alt = new CharMap(MapSize, CellSize, '2', CharMap::MAP1);
    m_Glrt2_alt  = new CharGridLineRayTracer(*m_lgm2_alt);
  }

  if (!m_bNoPlanes) {
    if (m_planeObjectFilename == "") {
      // Create plane objects
      //m_planeObjectFinders.push_back(createTableFinder(1.1, 0.9, CellSize));
    }
    for (unsigned int i = 0; i < m_maxNumberOfClusters; i++) {
      m_planeObstacleMaps.push_back(new CharMap(MapSize, CellSize, 0, CharMap::MAP1));
    }
    m_planeHeights.reserve(m_maxNumberOfClusters);
  }


  if(m_loadNodeLgms)
  {
    m_nodeGridMaps.clear();
    LoadNodeGridMaps("NodeGridMaps.txt");
  }

//  m_RobotServer = RobotbaseClientUtils::getServerPrx(*this,
//      m_RobotServerHost);

  FrontierInterface::HypothesisEvaluatorPtr servant = new EvaluationServer(this);
  registerIceServer<FrontierInterface::HypothesisEvaluator, FrontierInterface::HypothesisEvaluator>(servant);

  FrontierInterface::LocalMapInterfacePtr mapservant = new LocalMapServer(this);
  registerIceServer<FrontierInterface::LocalMapInterface, FrontierInterface::LocalMapInterface>(mapservant);

  if (m_bShowDoorsInPB) {
    m_peekabotClient.connect(m_PbHost, m_PbPort);
    m_HSSGroupProxy.add(m_peekabotClient, "HSS", peekabot::REPLACE_ON_CONFLICT);
  }
} 


void LocalMapManager::start() 
{  
  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
		  new MemberFunctionChangeReceiver<LocalMapManager>(this,
								  &LocalMapManager::newRobotPose));

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<LocalMapManager>(this,
								  &LocalMapManager::newRobotPose));  

  if (!m_bNoPlaces) {
    m_placeInterface = getIceServer<FrontierInterface::PlaceInterface>("place.manager");
    log("LocalMapManager started");
  }

m_RobotServer = getIceServer<Robotbase::RobotbaseServer>(m_RobotServerName);

  if (!m_bNoPTZ) {
    log("connecting to PTU");
	
    m_ptzInterface = getIceServer<ptz::PTZInterface>("ptz.server");
  }
}

void LocalMapManager::runComponent() 
{
  setupPushScan2d(*this, 0.1);
  setupPushOdometry(*this);

  log("the number of cells from the center to the edge of the LGM is: %u",	m_lgm2->getSize());

  log("I am running!");

  NavData::FNodePtr curNode = getCurrentNavNode();
  int prevNode = -1;
  int count = 0;
  while(isRunning()){
  
		if(m_saveNodeLgms && count  == 12){
		  SaveNodeGridMaps();
		  count = 0;
		}
		
	  count++;
	  
    debug("lock in isRunning");    
    lockComponent(); //Don't allow any interface calls while processing a callback
    curNode = getCurrentNavNode(); //PlaceManager::getCurrentNavNode()
    
    if (curNode != 0) {
      if (curNode->nodeId != prevNode) { /* The robot is in some new node! */
				m_Mutex.lock();
				// Node has changed! See if the node already has a lgm associated
				// with it; if so, swap the current for it. Otherwise,
				// assign the temporary to it.
				/*   std::map<int, CharMap *> m_nodeGridMaps; */
				if (m_nodeGridMaps.find(curNode->nodeId) == m_nodeGridMaps.end()) { /* search on nodeId */
				  // There's no grid map for the current Node. Assign the 
				  // temporary lgm to it.
					/* m_lgm2: This grid map represents a potential new place and is reset each time the robot changes Place. */
				  m_nodeGridMaps[curNode->nodeId] = m_lgm2;
				  log("Movin'");
				  /* center of the local grid map (m_lgm2) is set to the current node location */
				  m_lgm2->moveCenterTo(curNode->x, curNode->y);
			    /* m_lgm1: This grid map represents the current Place */
				  m_lgm1 = m_lgm2;
				  log("Allocatin'");
				  /* allocating a new local grid map object filled with the proper parameters */
				  m_lgm2 = new CharMap(m_mapsize, m_cellsize, '2', CharMap::MAP1, curNode->x, curNode->y);

					if (m_isUsingSeparateGridMaps) {
						m_nodeGridMapsAlt[curNode->nodeId] = m_lgm2_alt;
						m_lgm2_alt->moveCenterTo(curNode->x, curNode->y);
						m_lgm1_alt = m_lgm2_alt;
						m_lgm2_alt = new CharMap(m_mapsize, m_cellsize, '2', CharMap::MAP1, curNode->x, curNode->y);
					}
				}
				else { /* nodeId was not found in nodeGridMaps */
					// Clear the temporary local grid map, m_lgm2
					m_lgm1 = m_nodeGridMaps[curNode->nodeId];
					log("Clearin'");
					m_lgm2->clearMap(); /* remove all the obstacles from the map */
					log("Movin' 2");
					m_lgm2->moveCenterTo(curNode->x, curNode->y, false); /* false: don't update the map */

					if (m_isUsingSeparateGridMaps){
						m_lgm1_alt = m_nodeGridMapsAlt[curNode->nodeId];
						m_lgm2_alt->clearMap();
						m_lgm2_alt->moveCenterTo(curNode->x, curNode->y, false);
					}
				}
				delete m_Glrt1;
				delete m_Glrt2;
				m_Glrt1  = new CharGridLineRayTracer(*m_lgm1); /* typedef Cure::GridLineRayTracer<unsigned char> CharGridLineRayTracer; */
				m_Glrt2  = new CharGridLineRayTracer(*m_lgm2);

				if (m_isUsingSeparateGridMaps) {
					delete m_Glrt1_alt;
					delete m_Glrt2_alt;
					m_Glrt1_alt  = new CharGridLineRayTracer(*m_lgm1_alt);
					m_Glrt2_alt  = new CharGridLineRayTracer(*m_lgm2_alt);
				}

				if (m_Displaylgm1) {
					m_Displaylgm1 = new Cure::XDisplayLocalGridMap<unsigned char>(*m_lgm1);
					m_Displaylgm1->setMap(m_lgm1);
				}
				
				if (m_Displaylgm2) {
					m_Displaylgm2->setMap(m_lgm2);
				}
				//log("Maps set");

				m_Mutex.unlock();
	    } // if (curNode->nodeId != prevNode) ends
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

  	} // if (curNode != 0) ends
    
    unlockComponent();
    
    debug("unlock in isRunning");

    sleepComponent(250);
  } // While ends
}

// Save into a file
void LocalMapManager::SaveNodeGridMaps(){
  log("Saving node gridmaps");
  ofstream fout("NodeGridMaps.txt");
 for (map<int, CharMap *>::iterator it =
      m_nodeGridMaps.begin(); it != m_nodeGridMaps.end(); it++) {
  log("saving nodeid %d", (*it).first);
   // first write node id
    fout << (*it).first;
    fout << endl;
  // then map center
    fout<<(*it).second->getCentXW();
    fout << " " ;
    fout << (*it).second->getCentYW();
    fout << endl;
    fout << endl;
    //after an empty line go for the map data
    for (int x = -(*it).second->getSize(); x <= (*it).second->getSize(); x++) {
      for (int y = -(*it).second->getSize(); y <= (*it).second->getSize(); y++) {
	fout <<(* ((*it).second))(x, y);
      }
      //fout << endl;
    }
    fout << endl;
  }
    fout.close();
}

// load from the file
void LocalMapManager::LoadNodeGridMaps(std::string filename){
  ifstream file(filename.c_str());
  if (!file.good()){
    log("Could not read node grid map file, exiting.");
    exit(0);
  }
  string line,tmp;
  int nodeid;
  double cx,cy;
  while(getline(file,line)){
   log("loading node lgms from file");
    nodeid = atoi(line.c_str());
   log("nodeid; %d",nodeid);
    getline(file,line); 
    istringstream istr(line); 
    istr >> tmp;
    cx = atof(tmp.c_str());
    istr >> tmp;
    cy = atof(tmp.c_str());
log("node cx, cy: %3.2f, %3.2f",cx,cy);
    getline(file,line); 
    getline(file,line);
    int count = 0;
    CharMap* newMap = new CharMap(m_mapsize, m_cellsize, '2',
      CharMap::MAP1, cx, cy);
 for (int x = -newMap->getSize(); x <= newMap->getSize(); x++) {
      for (int y = -newMap->getSize(); y <= newMap->getSize(); y++) {
	char c = line[count];
	(*newMap)(x,y) = c;
	count++;
      }
 }
    m_nodeGridMaps[nodeid] = newMap; 
  }
log("loaded %d maps",m_nodeGridMaps.size());
}

/* newRobotPose */
void LocalMapManager::newRobotPose(const cdl::WorkingMemoryChange &objID) 
{
  double oldX = 0.0;
  double oldY = 0.0;
  double oldTheta = 0.0;
  cast::cdl::CASTTime oldTime=getCASTTime();
  if (lastRobotPose) {
    oldX = lastRobotPose->x;
    oldY = lastRobotPose->y;
    oldTheta = lastRobotPose->theta;
    oldTime = lastRobotPose->time;
  }

  try {
    lastRobotPose =
      getMemoryEntry<NavData::RobotPose2d>(objID.address);
      /*
			poseData.x = lastRobotPose->x;
			poseData.y = lastRobotPose->y;
			poseData.theta = lastRobotPose->theta;
			*/
      
  }
  catch (DoesNotExistOnWMException e) {
    log("Error! robotPose missing on WM!");
    return;
  }

  if (m_ptzInterface != 0) {
    ptz::PTZReading reading = m_ptzInterface->getPose();
		//sprintf(buff,"Pan-Tilt(%f,%f)",reading.pose.pan,reading.pose.tilt);
		//log(buff);
  }

  double distMoved = sqrt((oldX - lastRobotPose->x)*(oldX - lastRobotPose->x) +
    (oldY - lastRobotPose->y)*(oldY - lastRobotPose->y));
  double angleShift = lastRobotPose->theta - oldTheta;
  if (angleShift > M_PI) angleShift -= 2*M_PI;
  if (angleShift < -M_PI) angleShift += 2*M_PI;
  distMoved += abs(angleShift)*1.0;
  double deltaT = lastRobotPose->time.s - oldTime.s +
    (lastRobotPose->time.us - oldTime.us)*0.000001;
  double momVel = deltaT > 0 ? distMoved/deltaT : 0.0;

  if (momVel > m_standingStillThreshold || deltaT < 0) {
    m_lastTimeMoved = lastRobotPose->time;
  }

  //FIXME
   m_SlamRobotPose.setTime(Cure::Timestamp(lastRobotPose->time.s,
                                           lastRobotPose->time.us));
  m_SlamRobotPose.setX(lastRobotPose->x);
  m_SlamRobotPose.setY(lastRobotPose->y);
  m_SlamRobotPose.setTheta(lastRobotPose->theta);
  
  Cure::Pose3D cp = m_SlamRobotPose;
  m_TOPP.defineTransform(cp);

}

/* receiveOdometry */
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

int g_NextScanId = 0;

void laala(HSS::Scan2D &sink, const Cure::SICKScan &src)
{
  sink.m_Id = g_NextScanId++;

  sink.range.resize(src.getNPts());
  sink.valid.resize(src.getNPts());
  sink.theta.resize(src.getNPts());
  
  double da = src.getAngleStep();
  //double da = 1.0*M_PI/180;
  
  sink.min_theta = src.getStartAngle();
  sink.max_theta = src.getStartAngle() + (src.getNPts()-1)*da;

  sink.min_range = 0.1;
  sink.max_range = 5.55;

  sink.range_sigma = 0.05;

  for (int i = 0; i < src.getNPts(); i++) {
    sink.theta[i] = src.getStartAngle() + da * i;
    sink.valid[i] = 1;
    sink.range[i] = src.getRange(i);
  }
  sink.tv.tv_sec = src.getTime().Seconds;
  sink.tv.tv_usec = src.getTime().Microsec;
}

/* receiveScan2d is called by setupPushScan2d(*this, 0.1); */
void LocalMapManager::receiveScan2d(const Laser::Scan2d &castScan) // <--- Laser::Scan2d @ Laser.ice
{
  lockComponent(); //Don't allow any interface calls while processing a callback
  debug("Got scan with n=%d and t=%ld.%06ld",
        castScan.ranges.size(), 
        (long)castScan.time.s, (long)castScan.time.us);
        
	
  Cure::LaserScan2d cureScan; // <---- LaserScan2d.hh = SICKScan.hh
  CureHWUtils::convScan2dToCure(castScan, cureScan);

  if (m_TOPP.isTransformDefined()) {    
    Cure::Pose3D scanPose;
    if (m_TOPP.getPoseAtTime(cureScan.getTime(), scanPose) == 0) {
      Cure::Pose3D lpW;
      lpW.add(scanPose, m_LaserPoseR);
      m_Mutex.lock();
      m_Glrt1->addScan(cureScan, lpW, m_MaxLaserRangeForCombinedMaps);      
      m_Glrt2->addScan(cureScan, lpW, m_MaxLaserRangeForCombinedMaps);      
      m_lgm1->setValueInsideCircle(m_SlamRobotPose.getX(), m_SlamRobotPose.getY(),
      0.55*0.40, '0');                                  
  m_lgm2->setValueInsideCircle(m_SlamRobotPose.getX(), m_SlamRobotPose.getY(),
      0.55*0.45, '0');                                  
    if (m_isUsingSeparateGridMaps) {
        m_Glrt1_alt->addScan(cureScan, lpW, m_MaxLaserRangeForPlaceholders);      
        m_Glrt2_alt->addScan(cureScan, lpW, m_MaxLaserRangeForPlaceholders);      
      }
      m_firstScanReceived = true;

      if (m_bDetectDoors) {
	HSS::Scan2D scanNew;
	Cure::SICKScan &tmp = cureScan;
	laala(scanNew, tmp);
//	scanNew << tmp;
	m_doorExtractor.extract(scanNew);

	list<HSS::RedundantLine2DRep> detDoors =
	  m_doorExtractor.doors();

	Eigen::Vector3d xsR;
	xsR[0] = m_LaserPoseR.getX();
	xsR[1] = m_LaserPoseR.getY();
	xsR[2] = m_LaserPoseR.getTheta();

	Eigen::Vector3d odomNew;
	odomNew[0] = lastRobotPose->x;
	odomNew[1] = lastRobotPose->y;
	odomNew[2] = lastRobotPose->theta;

	if (m_bShowDoorsInPB) {
	  HSS::displayDoorMeas(m_HSSGroupProxy, odomNew, xsR, m_doorExtractor);
	}

	for (list<HSS::RedundantLine2DRep>::iterator it = detDoors.begin();
	    it != detDoors.end(); it++) {
	  bool found = false;
	  double doorX = odomNew[0] + cos(odomNew[2])*it->xC() - sin(odomNew[2])*it->yC();
	  double doorY = odomNew[1] + sin(odomNew[2])*it->xC() + cos(odomNew[2])*it->yC();

	  debug("Door detected at (%f,%f)", doorX, doorY);
	  for (map<string, FrontierInterface::DoorHypothesisPtr>::iterator it2 =
	      m_detectedDoors.begin(); it2 != m_detectedDoors.end(); it2++) {
	    double dx = it2->second->x - doorX;
	    double dy = it2->second->y - doorY;
	    double dist = dx*dx+dy*dy;
	    if (dist < 1.0) {
	      found = true;
	      break;
	    }
	  }
	  if (!found) {
	    double doorTheta = odomNew[2] + it->theta();
	    if (doorTheta < -M_PI) doorTheta += 2*M_PI;
	    if (doorTheta > M_PI) doorTheta -= 2*M_PI;
	    double doorWidth = it->length();

	    FrontierInterface::DoorHypothesisPtr newDoor =
	      new FrontierInterface::DoorHypothesis;
	    newDoor->x = doorX;
	    newDoor->y = doorY;
	    newDoor->theta = doorTheta;
	    newDoor->width = doorWidth;

	    string newID = newDataID();
	    addToWorkingMemory<FrontierInterface::DoorHypothesis>(newID,
		newDoor);
	    m_detectedDoors[newID] = newDoor;

	 /*   if (m_bShowDoorsInPB) {
	      peekabot::SphereProxy sph;
	      sph.add(m_HSSGroupProxy, "GW", peekabot::AUTO_ENUMERATE_ON_CONFLICT);
	      sph.set_scale(0.1, 0.1, 0.1);
	      sph.translate(doorX, doorY, 2.0);
	    }*/

//	    updatePlaceholderGatewayProperties(doorX, doorY);
	  }
	}
      }

      m_Mutex.unlock();
    }
  }
  unlockComponent();
}

//void
//LocalMapManager::updatePlaceholderGatewayProperties(double doorX, double doorY)
//{
//  if (m_placeInterface) {
//    //Check all gateway placeholder properties on WM; see if they should be upgraded
//    //based on the new doorway
//
//    vector<SpatialProperties::GatewayPlaceholderPropertyPtr> gwProps;
//
//    getMemoryEntries<SpatialProperties::GatewayPlaceholderProperty>(gwProps);
//
//    vector< boost::shared_ptr< cast::CASTData<SpatialProperties::GatewayPlaceholderProperty> > > gwProps;
//    getWorkingMemoryEntries<SpatialProperties::GatewayPlaceholderProperty> ("coma", 0, comaRoomBeliefs);
//
//    for (unsigned int i = 0; i < gwProps.size(); i++) {
//      GatewayPlaceholderPropertyPtr gw = gwProps[i]->getData();
//
//      if (gw == 0) {
//	log("Unexpected null pointer at LocalMapManager::%i", __LINE__);
//	continue;
//      }
//
//      try {
//	double currentProb = ((DiscreteProbabilityDistributionPtr)gw->distribution)->data[0]->probability;
//
//	FrontierInterface::NodeHypothesisPtr hyp =
//	  m_placeInterface->getHypFromPlaceID(it->placeID);
//
//	bool update = false;
//	for (vector<Eigen::Vector3d>::iterator it2 = m_detectedDoors.begin(); it2 != 
//	    m_detectedDoors.end(); it2++) {
//	  double dx = (*it2)[0] - hyp->x;
//	  double dy = (*it2)[1] - hyp->y;
//
//	  double distsq = dx*dx+dy*dy;
//	  double value = exp(-distsq/m_doorwayWidth);
//	  if (value < currentProb) {
//	    currentProb = value;
//	    update = true;
//	  }
//	}
//	if (update) {
//	  cast::cdl::WorkingMemoryAddress owtAddr;
//	  owtAddr.subarchitecture = "spatial.sa";
//	  owtAddr.id = gwProps[i]->getID();
//
//	  ((DiscreteProbabilityDistributionPtr)gw->distribution)->data[0]->probability =
//	    currentProb;
//	  ((DiscreteProbabilityDistributionPtr)gw->distribution)->data[1]->probability =
//	    1-currentProb;
//	  overwriteWorkingMemory(owtAddr, gw);
//	}
//      }
//      catch (DoesNotExistOnWMException) {
//	log("Error at LocalMapManager::%i - DoesNotExistOnWMException", __LINE__);
//      }
//    }
//  }
//}

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

/* getHypothesisEvaluation */
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
		  if (xi2 < -propertyLGM->getSize() ||
		      xi2 > propertyLGM->getSize() ||
		      yi2 < -propertyLGM->getSize() ||
		      yi2 > propertyLGM->getSize())
		    continue;
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

//  if (m_bDetectDoors) {
//    double minDistSq = DBL_MAX;
//    for (vector<Eigen::Vector3d>::iterator it = m_detectedDoors.begin();
//	it != m_detectedDoors.end(); it++) {
//    	log("Doorway at %f %f", (*it)[0], (*it)[1]);
//      double dx = relevantX - (*it)[0];
//      double dy = relevantY - (*it)[1];
//      double distSq = dx*dx+dy*dy;
//      if (distSq < minDistSq) {
//	minDistSq = distSq;
//      }
//    }
//    ret.gatewayValue = exp(-minDistSq/m_doorwayWidth);
//  }

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

SpatialData::LocalGridMap
LocalMapManager::LocalMapServer::getCombinedGridMap(const SpatialData::PlaceIDSeq &places,
    const Ice::Current &_context)
{
  SpatialData::LocalGridMap ret; 

  // Before locking this component, get all the node IDs from (possibly blocking)
  // interface
  vector<NavData::FNodePtr> nodes;
  if (m_pOwner->m_placeInterface) {
    for (SpatialData::PlaceIDSeq::const_iterator it = places.begin(); it != places.end(); it++) {

      //NOTE: May block
      NavData::FNodePtr node = m_pOwner->m_placeInterface->getNodeFromPlaceID(*it);
      if (node != 0) {
				nodes.push_back(node);
      }
    }
  }

  // Lock so that no one messes with the map set while we're grabbing the data
  m_pOwner->lockComponent();
  m_pOwner->getCombinedGridMap(ret, nodes);
  m_pOwner->unlockComponent();
  return ret;
}

void 
LocalMapManager::getCombinedGridMap(SpatialData::LocalGridMap &map, 
    const vector<NavData::FNodePtr> &nodes)
{
  vector<const CharMap *>maps;

  for (vector<NavData::FNodePtr>::const_iterator it = nodes.begin(); it != nodes.end(); it++) {
    if (m_nodeGridMaps.find((*it)->nodeId) != m_nodeGridMaps.end()) {
      maps.push_back(m_nodeGridMaps[(*it)->nodeId]);
    }
    else {
      log("Couldn't find grid map for node %i", (*it)->nodeId);
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



  //This is all new stuff!
  // Get obstacles we may have missed from kinect data
  log("Adding kinect data to combined local gridmap");
  SpatialData::MapInterfacePrx mapPrx(getIceServer<SpatialData::MapInterface>("spatial.control"));
  SpatialData::LocalGridMap obstacleMap = mapPrx->getBoundedMap(minx, maxx, miny, maxy);

  Cure::LocalGridMap<unsigned char> cureObstacleMap(obstacleMap.size, obstacleMap.cellSize, '2', Cure::LocalGridMap<unsigned char>::MAP1, obstacleMap.xCenter, obstacleMap.yCenter);

  // Convert from SpatialData::LocalGridMap to Cure::LocalGridMap
  int lp = 0;
  for(int x = -obstacleMap.size ; x <= obstacleMap.size; x++){
    for(int y = -obstacleMap.size ; y <= obstacleMap.size; y++){ 
      cureObstacleMap(x,y) = (obstacleMap.data[lp]);
      lp++;
    }
  }

  for (int x = -newSize; x <= newSize; x++) {
    for (int y = -newSize; y <= newSize; y++) {
      if(newMap(x,y) == '0') { // We only add obstacles on current free space
        double xw, yw; // World coordinates
        int xi, yi; // obstaclemap coordinates
        if (newMap.index2WorldCoords(x,y,xw,yw) != 0)
          continue;

        if (cureObstacleMap.worldCoords2Index(xw,yw,xi,yi) != 0)
          continue;

        if (cureObstacleMap(xi,yi) == '1') {
          newMap(x,y) = '1';
        }
      }
    }
  }

  // Add the data to the map we'll return
  for (int x = -newSize ; x <= newSize; x++){
    for (int y = -newSize ; y <= newSize; y++){
      map.data.push_back(newMap(x,y));
    }
  }

  CharMap newMap2(newSize, cellSize, '2',
      CharMap::MAP1, cx, cy);

  lp = 0;
  for(int x = -map.size ; x <= map.size; x++){
    for(int y = -map.size ; y <= map.size; y++){ 
      (newMap2)(x,y) = map.data[lp];
      lp++;
    }
  }
  
  /*Cure::XDisplayLocalGridMap<unsigned char>* m_Displaykrsjlgm;
  m_Displaykrsjlgm = new Cure::XDisplayLocalGridMap<unsigned char>(newMap2);
  m_Displaykrsjlgm->updateDisplay();*/

}


Cure::Transformation3D LocalMapManager::getCameraToWorldTransform()
{
  // Assuming useGlobalPoints is on in PlanePopOut, only need to
  // add the robot transform
  //Get robot pose
  Cure::Transformation2D robotTransform;
  if (lastRobotPose != 0) {
    robotTransform.setXYTheta(lastRobotPose->x, lastRobotPose->y,
      lastRobotPose->theta);
  }
  Cure::Transformation3D robotTransform3 = robotTransform;
  return robotTransform3;






  //Get camera ptz from PTZServer
  Cure::Transformation3D cameraRotation;
  if (m_ptzInterface != 0) {

    //NOTE: May block
    ptz::PTZReading reading = m_ptzInterface->getPose();

    double angles[] = {reading.pose.pan, -reading.pose.tilt, 0.0};
//    double angles[] = {0.0, M_PI/4, 0.0};
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
  //Cure::Transformation2D robotTransform;
  if (lastRobotPose != 0) {
    robotTransform.setXYTheta(lastRobotPose->x, lastRobotPose->y,
      lastRobotPose->theta);
  }
  //Cure::Transformation3D robotTransform3 = robotTransform;
//  double tmp[6];
//  robotTransform3.getCoordinates(tmp);
//  log("robot transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
//      tmp[4], tmp[5]);
//  cameraRotation.getCoordinates(tmp);
//  log("ptz transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
//      tmp[4], tmp[5]);
//  m_CameraPoseR.getCoordinates(tmp);
//  log("cam transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
//      tmp[4], tmp[5]);

  Cure::Transformation3D cameraOnRobot = m_CameraPoseR + cameraRotation + ptzBaseTransform ;
  return robotTransform3 + cameraOnRobot;
} 

