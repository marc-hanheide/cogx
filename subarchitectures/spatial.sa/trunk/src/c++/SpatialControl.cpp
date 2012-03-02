//
// = FILENAME
//    SpatialControl.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//    Chandana Paul
//    Dorian Galvez Lopez
//    Kristoffer Sjöö
//
// = COPYRIGHT
//                  2007 Dorian Galvez Lopez
//                  2009 Patric Jensfelt
//                  2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#include "SpatialControl.hpp"
#include <CureHWUtils.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

#include <AddressBank/ConfigFileReader.hh>
#include <RobotbaseClientUtils.hpp>
#include <FrontierInterface.hpp>
#include <Utils/CureDebug.hh>
#include "PTZ.hpp"
#include <Rendezvous.h>
#include "CameraParameters.h"
#include "TimeLogger.hpp"

#include <utility>
#include <queue>

using namespace cast;
using namespace std;
using namespace boost;
using namespace spatial;

#define SCOPED_TIME_LOG TimeLogger logger(this, __FILE__, __LINE__);

#define CAM_ID_DEFAULT 0

// Amount of time after pan-tilt movement terminates, until point clouds
// are admitted to the obstacle map (seconds)
#define KINECT_PANTILT_DELAY 0.3

//NOTE: This file "adapted" (nicked) from nav.sa version with NavCommand structs.
//SpatialData doesn't have everything that NavData does; the extraneous
//portions of this code have been commented out.
 
/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new SpatialControl();
  }
}

int SpatialControl::MapServer::findClosestNode(double x, double y, const Ice::Current &_context) {
  return m_pOwner->findClosestNode(x,y);
}

int SpatialControl::MapServer::findClosestPlace(double x, double y, const SpatialData::NodeIDSeq& nodeids, const Ice::Current &_context) {
  return m_pOwner->findClosestPlace(x,y, nodeids);
}

SpatialControl::SpatialControl()
  :NewNavController(m_NavGraph, m_LMap, this),
   NewNavControllerEventListener("SpatialControl"),
   FrontierExplorerEventListener("SpatialControl")
{
  m_CurrPerson = -1;
  m_CurrPersonWMid = "";
  m_firstScanAdded = false;

	m_Npts = 1440;
	m_StartAngle = -3.141592654;//-2.086214;
	m_AngleStep = 0.004363323;//0.006136;
	
  m_CurrentCmdFinalStatus = NavData::UNKNOWN;
  
  cure_debug_level = 10;

  m_RobotServerName = "robot.server";

  m_waitingForPTZCommandID = "";

  m_NumInhibitors = 0;
  m_SentInhibitStop = false;

  m_Glrt = NULL;
  m_catGlrt = NULL;
  m_Displaylgm = NULL;
  m_displayBinaryMap = NULL;
  m_displayObstacleMap = NULL;
  m_FrontierFinder = NULL;
  m_lgm = NULL;
  m_lgmLM = NULL;
  m_lgmKH = NULL;
  m_binaryMap = NULL;
  m_obstacleMap = NULL;
}

void SpatialControl::connectPeekabot()
{
  try {
    log("Trying to connect to Peekabot (again?) on host %s and port %d",
        m_PbHost.c_str(), m_PbPort);
    
    m_PeekabotClient.connect(m_PbHost, m_PbPort);

    if (m_usePeekabot){
      log("Showing grid map in Peekabot");
      m_ProxyMap.add(m_PeekabotClient, "frontiers",peekabot::REPLACE_ON_CONFLICT);

      m_OdomPoseProxy.add(m_PeekabotClient, "Odom_pose_SC",peekabot::REPLACE_ON_CONFLICT);
      m_OdomPoseProxy.set_scale(0.1,0.04,0.04);
      m_OdomPoseProxy.set_color(1,0,0);

      m_ProxyScan.add(m_PeekabotClient, "scan", peekabot::REPLACE_ON_CONFLICT);
      m_ProxyScan.set_color(0,0,1);
    }

    log("Connection to Peekabot established");

    
  } catch(std::exception &e) {
    log("Caught exception when connecting to peekabot (%s)",
        e.what());
    return;
  }
}

void SpatialControl::CreateGridMap() {
    double cellSize=m_lgm->getCellSize();
    m_ProxyGridMap.add(m_PeekabotClient, "grid_map", cellSize,
	peekabot::REPLACE_ON_CONFLICT);
    m_ProxyGridMap.set_occupied_color(0.1,0.1,0.1);
    m_ProxyGridMap.set_unoccupied_color(0.8,0.9,1);
    peekabot::OccupancySet2D cells;
    m_ProxyGridMap.set_cells(cells);
    m_ProxyMap.set_position(0,0,-0.005);
    m_ProxyGridMap.set_position(0,0,-0.01);

/*    m_ProxyGridMapKinect.add(m_PeekabotClient, "grid_map_kinect", cellSize, 
    1,0.9,0.8,
    0.1,0.1,0.1, peekabot::REPLACE_ON_CONFLICT);
    peekabot::OccupancySet2D cells1;
    m_ProxyGridMapKinect.set_cells(cells1);
    m_ProxyGridMapKinect.set_position(0,0,-0.009);
*/
}

void SpatialControl::UpdateGridMap() {
    peekabot::OccupancySet2D cells;
    double cellSize=m_lgm->getCellSize();
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
        for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
            if ((* (m_lgm))(x, y)=='0')
                cells.add(x*cellSize,y*cellSize,0);
            else if ((* (m_lgm))(x, y)=='1')
                cells.add(x*cellSize,y*cellSize,1);
        }
    }
    m_ProxyGridMap.set_cells(cells);
/*
    peekabot::OccupancySet2D cells1;
    for (int x = -m_kinlgm->getSize(); x <= m_kinlgm->getSize(); x++) {
        for (int y = -m_kinlgm->getSize(); y <= m_kinlgm->getSize(); y++) {
            if ((* (m_kinlgm))(x, y)=='0')
                cells1.add(x*cellSize,y*cellSize,0);
            else if ((* (m_kinlgm))(x, y)=='1')
                cells1.add(x*cellSize,y*cellSize,1);
        }
    }
    m_ProxyGridMapKinect.set_cells(cells1);
*/
//    if (m_MapsMutex.tryLock()){
      std::list<Cure::FrontierPt> *fPts = &m_Frontiers;
      m_ProxyMap.clear();
      if (fPts) {
//        int r = int(0.8 / m_lgm->getCellSize() + 0.5);
        for (std::list<Cure::FrontierPt>::const_iterator fi = fPts->begin();
             fi != fPts->end(); fi++) {
          int i, j;
          if (m_lgm->worldCoords2Index(fi->getX(), fi->getY(), i, j) == 0) {        
            i = i + m_lgm->getSize();
            j = m_lgm->getSize() - j;

//            double halfLen = 0.5 * fi->m_Width / m_lgm->getCellSize();
            double color[3];
            color[0] = 0.1;
            color[1] = 0.1;
            color[2] = 0.9;
//            GC *gc = &gcBlue;
            if (fi->m_State == Cure::FrontierPt::FRONTIER_STATUS_CURRENT) {
//              gc = &gcYellow;
                color[0] = 0.9;
                color[1] = 0.9;
                color[2] = 0.1;

            } else if (fi->m_State == Cure::FrontierPt::FRONTIER_STATUS_UNREACHABLE) {
//              gc = &gcRed;
                color[0] = 0.9;
                color[1] = 0.1;
                color[2] = 0.1;
            } else if (fi->m_State == Cure::FrontierPt::FRONTIER_STATUS_PATHBLOCKED) {
//              gc = &gcMagenta;
                color[0] = 0.9;
                color[1] = 0.1;
                color[2] = 0.9;
            } else if (fi->m_State == Cure::FrontierPt::FRONTIER_STATUS_GATEWAYBLOCKED) {
//              gc = &gcGreen;
                color[0] = 0.1;
                color[1] = 0.9;
                color[2] = 0.1;
            }

            fi->m_Width;

            peekabot::PolygonProxy p;
            p.add(m_ProxyMap,"frontier",peekabot::AUTO_ENUMERATE_ON_CONFLICT);
            int num_points = 10;
	    peekabot::VertexSet vSet;
            for(int k=0;k<num_points;k++)
                vSet.add(0.5*cos(k * 2 * M_PI_2 / num_points ),0.3*fi->m_Width*0.5*sin(k * 2 * M_PI_2 / num_points),0.);
	    p.add_vertices(vSet);
            p.set_color(color[0],color[1],color[2]);
            p.set_opacity(1);
            p.set_position(fi->getX(), fi->getY(),0);
            p.set_rotation(fi->getTheta() - M_PI_2,0,0);
          }
        }
//      }
//    m_MapsMutex.unlock();
  }
}








void
SpatialControl::startMovePanTilt(double pan, double tilt, double tolerance) 
{
  ptz::SetPTZPoseCommandPtr newPTZPoseCommand = new ptz::SetPTZPoseCommand;
  newPTZPoseCommand->pose.pan = pan;
  newPTZPoseCommand->pose.tilt = tilt;
  newPTZPoseCommand->comp = ptz::COMPINIT;

  string cmdId = newDataID();
  addToWorkingMemory(cmdId, newPTZPoseCommand);

  m_waitingForPTZCommandID = cmdId;
  error("alex startMovePanTilt %s",m_waitingForPTZCommandID.c_str());
}

void SpatialControl::newPanTiltCommand(const cdl::WorkingMemoryChange &objID) {
  m_ptzInNavigationPose = false; // Wait until the command is done before checking the pose
  m_ptzInMappingPose = false; 
}

void
SpatialControl::overwrittenPanTiltCommand(const cdl::WorkingMemoryChange &objID) 
{
  try {
    ptz::SetPTZPoseCommandPtr overwritten =
      getMemoryEntry<ptz::SetPTZPoseCommand>(objID.address);

    if (objID.address.id == m_waitingForPTZCommandID) {
      if (overwritten->comp != ptz::SUCCEEDED) {
        log ("Warning! Failed to move PTZ before moving!");
      }
      deleteFromWorkingMemory(objID.address);
    }

    checkPTZPose(overwritten->pose);

    m_lastPtzNavPoseCompletion = getCASTTime();
  }
  catch (DoesNotExistOnWMException e)
  {
    log ("Error: SetPTZPoseCommand went missing! "); 
  }

  if (objID.address.id == m_waitingForPTZCommandID)
      m_waitingForPTZCommandID = "";
}

void
SpatialControl::checkPTZPose(const ptz::PTZPose &pose)
{
  if(pose.tilt < -M_PI/4 + 0.05 && pose.tilt > -M_PI/4 -0.05) {
    m_ptzInMappingPose = true;
  }
  else {
    m_ptzInMappingPose = false;
  } 

  if(pose.tilt < -M_PI/4 + 0.05 && pose.tilt > -M_PI/4 -0.05 &&
      pose.pan < 0.05 && pose.pan > -0.05)
  {
    m_ptzInNavigationPose = true;
  }
  else {
    m_ptzInNavigationPose = false;
  }

  log("PTZ in navigation pose: %d", m_ptzInNavigationPose);
  log("PTZ in mapping pose: %d", m_ptzInMappingPose);
  m_currentPTZPose = pose;
}

SpatialControl::~SpatialControl() 
{ 
  delete m_Glrt;
  delete m_catGlrt;
  delete m_Displaylgm;
  delete m_displayBinaryMap;
  delete m_displayObstacleMap;
  delete m_FrontierFinder;
  delete m_lgm;
  delete m_lgmLM;
  delete m_lgmKH;
  delete m_binaryMap;
  delete m_obstacleMap;
}

//saves m_lgm
void SpatialControl::SaveGridMap(){
  log("Saving node gridmaps");
  ofstream fout("GridMap.txt");
  //write size
  fout << m_lgm->getSize() << endl;
  // then map center
  fout<<m_lgm->getCentXW();
  fout << " " ;
  fout << m_lgm->getCentYW();
  fout << endl;
  fout << endl;
  //after an empty line go for the map data
  for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
    for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
      fout <<(* (m_lgm))(x, y);
    }
    //fout << endl;
  }
  fout << endl;
  fout.close();
}

// load from the file
void SpatialControl::LoadGridMap(std::string filename){
  ifstream file(filename.c_str());
  if (!file.good()){
    log("Could not read grid map file, exiting.");
    return;
  }
  string line,tmp;
  double cx,cy;
  int sz;
    getline(file,line);
    istringstream istr(line); 
    istr >> tmp;
    sz = atoi(tmp.c_str());
    log("GridMap size: %d",sz);
    getline(file,line);
    istringstream istr1(line); 
    istr1 >> tmp;
    cx = atof(tmp.c_str());
    istr1 >> tmp;
    cy = atof(tmp.c_str());
    log("GridMap cx, cy: %3.2f, %3.2f",cx,cy);
    getline(file,line); 
    getline(file,line);
    int count = 0;

    m_lgm = new Cure::LocalGridMap<unsigned char>(sz, 0.05, '2', Cure::LocalGridMap<unsigned char>::MAP1);

    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        char c = line[count];
        (*m_lgm)(x,y) = c;
        count++;
      }
    }
   
    log("loaded gridmap");
}

void SpatialControl::configure(const map<string,string>& _config) 
{

  if (_config.find("--save-map") != _config.end()) {
    m_saveLgm = true;
  }

  if (_config.find("--load-map") != _config.end()) {
    m_loadLgm = true;
    m_saveLgm = false;
  }

  m_usePeekabot = false;
    if (_config.find("--usepeekabot") != _config.end())
      m_usePeekabot= true;


  m_UsePointCloud = false;
  if (_config.find("--pcserver") != _config.end()) {
    configureServerCommunication(_config);
    m_UsePointCloud = true;

    m_obstacleMinHeight = 0.07; 
    map<string,string>::const_iterator it = _config.find("--min-obstacle-height");
    if (it != _config.end()) {
      m_obstacleMinHeight = atof(it->second.c_str());
    }
    m_obstacleMaxHeight = 1.35; 
    it = _config.find("--max-obstacle-height");
    if (it != _config.end()) {
      m_obstacleMaxHeight = atof(it->second.c_str());
    }

  }

  m_simulateKinect = false;
  if (_config.find("--simulate-kinect") != _config.end()) {
    if (m_UsePointCloud) {
      log("--simulate-kinect ignored: --pcserver provided");
    }
    else {
      m_simulateKinect = true;
    }
  }

  camId = CAM_ID_DEFAULT;
  map<string,string>::const_iterator it = _config.find("--camid");
  if(it != _config.end())
  {
    istringstream str(it->second);
    str >> camId;
  }

  m_sendPTZCommands = false;
  if (_config.find("--ctrl-ptu") != _config.end()) {
    m_sendPTZCommands = true;
    log("will use ptu");
  }

  it = _config.find("-c");
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

  m_PbPort = 5050;
  cfg.getRoboLookHost(m_PbHost);
  std::string usedCfgFile, tmp;
  if (cfg.getString("PEEKABOT_HOST", true, tmp, usedCfgFile) == 0) {
    m_PbHost = tmp;
  }


  m_bNoNavGraph = false;
  if(_config.find("--no-graph") != _config.end()){
    m_bNoNavGraph = true;
  }

  if (Cure::NewNavController::config(configfile)) {
    println("configure(...) Failed to config with \"%s\", use -c option\n",
	configfile.c_str());
    std::abort();
  } 


  if (cfg.getSensorPose(1, m_LaserPoseR)) {
    println("configure(...) Failed to get sensor pose");
    std::abort();
  } 

  m_MaxExplorationRange = 1.5;
  it = _config.find("--explore-range");
  if (it != _config.end()) {
    m_MaxExplorationRange = (atof(it->second.c_str()));
  }

  it = _config.find("--robot-server");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_RobotServerName;
  }

  m_lgm = new Cure::LocalGridMap<unsigned char>(300, 0.05, '2', Cure::LocalGridMap<unsigned char>::MAP1);
  m_lgmKH = new Cure::LocalGridMap<double>(300, 0.05, FLT_MAX, Cure::LocalGridMap<double>::MAP1);

  if(m_loadLgm)
  {
    LoadGridMap("GridMap.txt");
  } else {
    m_lgm->setValueInsideCircle(0,   0, 0.5, '0'); 
    m_lgm->setValueInsideCircle(0.1, 0, 0.5, '0'); 
    m_lgm->setValueInsideCircle(0.2, 0, 0.5, '0'); 
    m_lgm->setValueInsideCircle(0.3, 0, 0.5, '0'); 

    m_lgmKH->setValueInsideCircle(0,   0, 0.5, 0.01); 
    m_lgmKH->setValueInsideCircle(0.1, 0, 0.5, 0.01); 
    m_lgmKH->setValueInsideCircle(0.2, 0, 0.5, 0.01); 
    m_lgmKH->setValueInsideCircle(0.3, 0, 0.5, 0.01); 


  }


  m_binaryMap = new Cure::LocalGridMap<unsigned char>(300, 0.05, '2', Cure::LocalGridMap<unsigned char>::MAP1);

  m_DisplayCureObstacleMap = false;
  if (_config.find("--display-cure-obstacle-map") != _config.end()) {
    m_DisplayCureObstacleMap = true;
    m_obstacleMap = new Cure::LocalGridMap<unsigned char>(300, 0.05, 1, Cure::LocalGridMap<unsigned char>::MAP1);
    m_displayObstacleMap = new Cure::XDisplayLocalGridMap<unsigned char>(*m_obstacleMap);
  }

  m_Glrt  = new Cure::GridLineRayTracer<unsigned char>(*m_lgm);

  m_FrontierFinder = new Cure::FrontierFinder<unsigned char>(*m_lgm);
  //  m_FrontierFinderKinect = new Cure::FrontierFinder<unsigned char>(*m_kinlgm);

  if ((_config.find("--x-window") != _config.end())) {
    m_Displaylgm = new Cure::XDisplayLocalGridMap<unsigned char>(*m_lgm);
    println("Will use X window to show the exploration map");
  } else {
    m_Displaylgm = 0;
    println("Will NOT use X window to show the exploration map");
  }

  if(_config.find("--show-binary-map") != _config.end()) {
    m_displayBinaryMap = new Cure::XDisplayLocalGridMap<unsigned char>(*m_binaryMap);
  } else {
    m_displayBinaryMap = 0;
  }

  double maxGotoV = 0.5;
  double maxGotoW = 0.5;

  if ((it = _config.find("--max-goto-v")) != _config.end()) {
    std::istringstream str(it->second);
    str >> maxGotoV;
  }
  if ((it = _config.find("--max-goto-w")) != _config.end()) {
    std::istringstream str(it->second);
    str >> maxGotoW;
  }

  Cure::NewNavController::addEventListener(this);
  Cure::NewNavController::setTurnAngleIntoSpeed(true, 0.5);
  Cure::NewNavController::setMinNonzeroSpeeds(0.04, 
      Cure::HelpFunctions::deg2rad(10));
  Cure::NewNavController::setApproachTolerances(0.5, 
      Cure::HelpFunctions::deg2rad(10));
  Cure::NewNavController::setUsePathTrimming(false);
  Cure::NewNavController::setMaxPathTrimDist(3);
  Cure::NewNavController::setProgressTimeout(20);
  Cure::NewNavController::setGotoMaxSpeeds(maxGotoV, maxGotoW);
  Cure::NewNavController::setGatewayMaxSpeeds(0.3, 0.3);

  Cure::NewNavController::setFollowDistances(0.8, 0.4);
  Cure::NewNavController::setFollowTolerances(0.1, 
      Cure::HelpFunctions::deg2rad(10));

  Cure::NewNavController::setPoseProvider(m_TOPP);

  /*
     it = _config.find("--max-target-graph-dist");
     double maxDist = 5;
     if (it != _config.end()) {
     std::istringstream str(it->second);
     str >> maxDist;
     }  
     m_NavGraph.setMaxDistTargetFromNode(maxDist);
   */

  m_taskId = 1;
  m_taskStatus = NothingToDo;
  m_ready = false;
  m_DefTolPos = 0.25;
  m_DefTolRot = Cure::HelpFunctions::deg2rad(5);

  //  m_RobotServer = RobotbaseClientUtils::getServerPrx(*this,
  //                                                     m_RobotServerHost);

  FrontierInterface::FrontierReaderPtr servant = new FrontierServer(this);
  registerIceServer<FrontierInterface::FrontierReader, FrontierInterface::FrontierReader>(servant);

  registerIceServer<SpatialData::MapInterface, SpatialData::MapInterface>(new MapServer(this));
} 

void SpatialControl::start() 
{
  if (m_UsePointCloud) {
    startPCCServerCommunication(*this);
  }
  //registerIceServer<cast::CASTComponent,FrontierReaderAsComponent>
    //(getComponentPointer());
 
  addChangeFilter(createLocalTypeFilter<NavData::InternalNavCommand>(cdl::ADD), 
		  new MemberFunctionChangeReceiver<SpatialControl>(this,
								  &SpatialControl::newNavCtrlCommand));  

  addChangeFilter(createLocalTypeFilter<NavData::InhibitNavControl>(cdl::ADD),
		  new MemberFunctionChangeReceiver<SpatialControl>(this,
								  &SpatialControl::newInhibitor));

  addChangeFilter(createLocalTypeFilter<NavData::VisualExplorationCommand>(cdl::ADD),
		  new MemberFunctionChangeReceiver<SpatialControl>(this,
								  &SpatialControl::newVisualExplorationCommand));

  addChangeFilter(createLocalTypeFilter<NavData::InhibitNavControl>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<SpatialControl>(this,
								  &SpatialControl::deleteInhibitor));
	        
  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
		  new MemberFunctionChangeReceiver<SpatialControl>(this,
								  &SpatialControl::newRobotPose));

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<SpatialControl>(this,
								  &SpatialControl::newRobotPose));  
  
  addChangeFilter(createLocalTypeFilter<NavData::NavGraph>(cdl::ADD),
		  new MemberFunctionChangeReceiver<SpatialControl>(this,
								  &SpatialControl::newNavGraph));
  
  addChangeFilter(createLocalTypeFilter<NavData::NavGraph>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<SpatialControl>(this,
								  &SpatialControl::newNavGraph));    
  
  addChangeFilter(createLocalTypeFilter<NavData::Person>(cdl::ADD),
		  new MemberFunctionChangeReceiver<SpatialControl>(this,
                                                               &SpatialControl::newPersonData));
  
  addChangeFilter(createLocalTypeFilter<NavData::Person>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<SpatialControl>(this,
                                                               &SpatialControl::newPersonData));    
  
  addChangeFilter(createLocalTypeFilter<NavData::Person>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<SpatialControl>(this,
                                                               &SpatialControl::deletePersonData));   

  addChangeFilter(createGlobalTypeFilter<ptz::SetPTZPoseCommand>(cdl::ADD),
		  new MemberFunctionChangeReceiver<SpatialControl>(this,
                                                               &SpatialControl::newPanTiltCommand));   
  addChangeFilter(createGlobalTypeFilter<ptz::SetPTZPoseCommand>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<SpatialControl>(this,
                                                               &SpatialControl::overwrittenPanTiltCommand));   
               /*                             
		// connecting Pan-Tilt server                   
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
 */

m_RobotServer = getIceServer<Robotbase::RobotbaseServer>(m_RobotServerName);
	
  if(m_sendPTZCommands) {	
  	m_ptzInterface = getIceServer<ptz::PTZInterface>("ptz.server");
  	ptz::PTZReading reading = m_ptzInterface->getPose();
  

    checkPTZPose(reading.pose);

    m_lastPtzNavPoseCompletion = getCASTTime();
  }


  log("SpatialControl started");
  
}

void 
SpatialControl::changeCurrentCommandCompletion (const NavData::Completion &value, 
                                            const NavData::StatusError &status)
{
  debug("Changing completion to %d", value);
  
  // Ok, nobody else is expected to overwrite the command.
  // And it should not be removed before we set a terminal completion 
  // (life is so beautiful)
  shared_ptr<CASTData<NavData::InternalNavCommand> > oobj =
    getWorkingMemoryEntry<NavData::InternalNavCommand>(m_CurrentCmdAddress);
  
  NavData::InternalNavCommandPtr cmd = oobj->getData();
  cmd->comp = value;
  cmd->status = status;
  debug("just before overwriting");
  overwriteWorkingMemory<NavData::InternalNavCommand>(m_CurrentCmdAddress.id, cmd); 
  debug("just after overwriting");
}

// This three methods cannot overwrite the work memory because it can
// cause a race condition with the odometry push receiver

void SpatialControl::abortTask(int taskId) {
  // If a command is aborted, that means that another NavCtrlCommand
  // is in the game, so the SpatialControl is aware of it.
  // Therefore, nothing need to be done here.
  log("abortTask");
}

void SpatialControl::doneTask(int taskId) {
  log("doneTask");

  IceUtil::Mutex::Lock lock(m_taskStatusMutex);

  m_TolRot = Cure::HelpFunctions::deg2rad(5);
  if(taskId == m_taskId - 1){ // else, it is an old command
  log("this is an old command...");
    if(!m_currentTaskIsExploration){ // @see explorationDone
      m_CurrentCmdFinalCompletion = NavData::SUCCEEDED;
      m_CurrentCmdFinalStatus = NavData::NONE;
      m_taskStatus = TaskFinished;
    }
  }
}


void SpatialControl::failTask(int taskId, int error) {
  log("failTask");
  m_TolRot = Cure::HelpFunctions::deg2rad(5);

  IceUtil::Mutex::Lock lock(m_taskStatusMutex);

  if(taskId == m_taskId - 1){ // else, it is an old command
    m_CurrentCmdFinalCompletion = NavData::FAILED;
    m_CurrentCmdFinalStatus = NavData::UNKNOWN;
    m_taskStatus = TaskFinished;
  }
}

// FrontierExplorer exploration done overload function
void SpatialControl::explorationDone(int taskId, int status)
{

  log("explorationDone");

  IceUtil::Mutex::Lock lock(m_taskStatusMutex);

  m_TolRot = Cure::HelpFunctions::deg2rad(5);
  if(taskId == m_taskId - 1){ // else, it is an old command
    m_CurrentCmdFinalCompletion = NavData::SUCCEEDED;
    m_CurrentCmdFinalStatus = NavData::NONE;
    m_taskStatus = TaskFinished;
  }
}

void SpatialControl::blitHeightMap(Cure::LocalGridMap<unsigned char>& lgm, Cure::LocalGridMap<double>* heightMap, int minX, int maxX, int minY, int maxY, double obstacleMinHeight, double obstacleMaxHeight)
{
  int xi, yi;

  if (minX <= -lgm.getSize())
    minX = -lgm.getSize() + 1;
  if (maxX >= lgm.getSize())
    maxX = lgm.getSize() - 1;
  if (minY <= -lgm.getSize())
    minY = -lgm.getSize() + 1;
  if (maxY >= lgm.getSize())
    maxY = lgm.getSize() - 1;

  /* Project the height map on to the local map */
  for (yi = minY; yi <= maxY; yi++) {
    for (xi = minX; xi <= maxX; xi++) {
      if ((*heightMap)(xi, yi) > obstacleMinHeight && (*heightMap)(xi, yi) < obstacleMaxHeight) {
        lgm(xi, yi) = '1';
      }
      else if ((*heightMap)(xi, yi) != FLT_MAX) {
        // Don't overwrite obstacles seen by the laser with (possibly old)
        // free space from the point cloud
        if (lgm(xi, yi) == '1')
          continue;
        lgm(xi, yi) = '0';
      }
      else if ((m_UsePointCloud) && (lgm(xi, yi) != '1'))
        lgm(xi, yi) = '2';
      else if ((!m_UsePointCloud) && (lgm(xi, yi) != '1')){
        double cellsize = m_lgm->getCellSize();
        double al = -m_currentPTZPose.pan;
        double nx = xi*cellsize * cos(al) - yi*cellsize * sin(al);
        double ny = xi*cellsize * sin(al) + yi*cellsize * cos(al);
        if ((nx > 1.8) || (0.535 * nx - ny < 0) || (-0.535 * nx - ny > 0)) lgm(xi, yi) = '2';
      }
    }
  }
  for (yi = minY+1; yi < maxY; yi++) {
    for (xi = minX+1; xi < maxX; xi++) {
      if (lgm(xi,yi) == '1' && 
          lgm(xi-1,yi) != '1' && lgm(xi+1,yi) != '1' &&
          lgm(xi-1,yi-1) != '1' && lgm(xi+1,yi-1) != '1' &&
          lgm(xi-1,yi+1) != '1' && lgm(xi+1,yi+1) != '1' &&
          lgm(xi,yi-1) != '1' && lgm(xi,yi+1) != '1') {
        lgm(xi,yi) = '0';
      }
    }
  }
}

class ComparePoints {
  public:
    bool operator()(const PointCloud::SurfacePoint& lhs, const PointCloud::SurfacePoint& rhs) const {
      return lhs.p.z < rhs.p.z;
    }
};

bool SpatialControl::isPointVisible(const cogx::Math::Vector3 &pos)
{
  Video::CameraParameters params;
  if (!getCameraParameters(camId, params))
    return false;

  return Video::isPointVisible(params, pos);
}

void SpatialControl::updateGridMaps(){
  Cure::Pose3D scanPose;
  Cure::Pose3D LscanPose;
  Cure::Pose3D lpW;
  int xi,yi;

  /* Update a temporary local map so we don't hog the lock */
  Cure::LocalGridMap<unsigned char> *tmp_lgm;

  tmp_lgm = m_lgm; //new Cure::LocalGridMap<unsigned char>(*m_lgm);
  Cure::GridLineRayTracer<unsigned char> tmp_glrt(*tmp_lgm);
  
  /* Local reference so we don't have to dereference the pointer all the time */
  Cure::LocalGridMap<double>& lgmKH = *m_lgmKH;

  /* Bounding box for laser scan */
  double laserMinX = INT_MAX, laserMaxX = INT_MIN;
  double laserMinY = INT_MAX, laserMaxY = INT_MIN;


  /* Add all queued laser scans */
  {
    SCOPED_TIME_LOG;
    m_ScanQueueMutex.lock();
  }
  while (!m_LScanQueue.empty()){
    Cure::LaserScan2d scan = m_LScanQueue.front();
    m_LScanQueue.pop();
    m_ScanQueueMutex.unlock();

    int status = 1;
    {
      if (m_TOPP.isTransformDefined()) {
	      status = m_TOPP.getPoseAtTime(scan.getTime(), LscanPose);
      }
    }
    
    if (status == 0) {
      lpW.add(LscanPose, m_LaserPoseR);		
      if (m_usePeekabot){
        SCOPED_TIME_LOG;        
        m_ProxyScan.clear_vertices();
        double angStep = scan.getAngleStep();
        double startAng = scan.getStartAngle();
				peekabot::VertexSet vs;
        for (int i = 0; i < scan.getNPts(); i++) {
          float x,y;
          x = cos(startAng + i * angStep) * scan.getRange(i);
          y = sin(startAng + i * angStep) * scan.getRange(i);
          vs.add(x,y,0);
        }
				m_ProxyScan.add_vertices(vs);
        m_ProxyScan.set_pose(lpW.getX(), lpW.getY(), lpW.getZ(), lpW.getTheta(), 0, 0);
      }
      {
        SCOPED_TIME_LOG;
        tmp_glrt.addScan(scan, lpW, m_MaxExplorationRange);
        tmp_lgm->setValueInsideCircle(LscanPose.getX(), LscanPose.getY(),
            0.55*Cure::NewNavController::getRobotWidth(), '0');                                  
      }
      m_firstScanAdded = true;

      /* Update bounding box */
      if (lpW.getX() < laserMinX)
        laserMinX = lpW.getX();
      if (lpW.getX() > laserMaxX)
        laserMaxX = lpW.getX();
      if (lpW.getY() < laserMinY)
        laserMinY = lpW.getY();
      if (lpW.getY() > laserMaxY)
        laserMaxY = lpW.getY();
    }
    {
      SCOPED_TIME_LOG;
      m_ScanQueueMutex.lock();
    }
  }
  m_ScanQueueMutex.unlock();
/* Only proceed if we got any new scans */
  if (laserMinX == INT_MAX || laserMinY == INT_MAX || laserMaxX == INT_MIN || laserMaxY == INT_MIN) {
    return;
  }
  laserMinX -= m_MaxExplorationRange;
  laserMinY -= m_MaxExplorationRange;
  laserMaxX += m_MaxExplorationRange;
  laserMaxY += m_MaxExplorationRange;

  int laserMinXi, laserMinYi, laserMaxXi, laserMaxYi;
  if (m_lgm->worldCoords2Index(laserMinX, laserMinY, laserMinXi, laserMinYi) != 0) {
    laserMinXi = -m_lgm->getSize() + 1;
    laserMinYi = -m_lgm->getSize() + 1;
  }
  if (m_lgm->worldCoords2Index(laserMaxX, laserMaxY, laserMaxXi, laserMaxYi) != 0) {
    laserMaxXi = m_lgm->getSize() - 1;
    laserMaxYi = m_lgm->getSize() - 1;
  }

  if (m_UsePointCloud) {
    SCOPED_TIME_LOG;
    /* Bounding box for new point cloud data */
    int pointcloudMinXi = INT_MAX, pointcloudMaxXi = INT_MIN;
    int pointcloudMinYi = INT_MAX, pointcloudMaxYi = INT_MIN;

    /* Update height map */
    cdl::CASTTime frameTime = getCASTTime();
    double time = frameTime.s+frameTime.us/1000000.0;
    double lastptztime = m_lastPtzNavPoseCompletion.s+m_lastPtzNavPoseCompletion.us/1000000.0;
    bool hasScanPose;
    {
      IceUtil::Mutex::Lock lock(m_PPMutex);

      hasScanPose = m_TOPP.getPoseAtTime(Cure::Timestamp(frameTime.s, frameTime.us), scanPose) == 0;
    }

    /* WARNING (FIXME):
     * For some reason we seem to get old kinect data when we have _just_ moved
     * the pantilt to -45 degrees some times, causing garbage to appear on the
     * map. For now, we wait for 1 second before doing anything with the kinect
     * data to make sure we get fresh data. */
    if (hasScanPose && m_ptzInMappingPose && time-lastptztime > KINECT_PANTILT_DELAY) {
      PointCloud::SurfacePointSeq points;
      {
      SCOPED_TIME_LOG;
      getPoints(true, 640/4, points);
      }
      {
      SCOPED_TIME_LOG;
      std::sort(points.begin(), points.end(), ComparePoints());
      }
      {
      SCOPED_TIME_LOG;
     
      for (PointCloud::SurfacePointSeq::iterator it = points.begin(); it != points.end(); ++it) {
        /* Ignore points not in the current view cone */
//        if (!isPointVisible(it->p))
//        	continue;
//        if (!isPointInViewCone(it->p))
//          continue;
        /* Transform point in cloud with regards to the robot pose */
        Cure::Vector3D from(it->p.x, it->p.y, it->p.z);
        Cure::Vector3D to;
        scanPose.invTransform(from, to);
        double pX = to.X[0];
        double pY = to.X[1];
        double pZ = to.X[2];
        if (m_lgmKH->worldCoords2Index(pX, pY, xi, yi) == 0) {
          /* Check if we can safely remove an old obstacle */
          bool oldObstacle = (lgmKH(xi, yi) > m_obstacleMinHeight && lgmKH(xi, yi) < m_obstacleMaxHeight);
          bool newObstacle = (pZ > m_obstacleMinHeight && pZ < m_obstacleMaxHeight);
          
          bool updateHeightMap = true;
          if (oldObstacle && !newObstacle) {
//          /* Undo robot pose transform since it is not known by the point cloud */
//            Cure::Vector3D old(pX, pY, lgmKH(xi, yi));
//            scanPose.transform(old, to);
//            cogx::Math::Vector3 oldPoint;
//            oldPoint.x = to.X[0];
//            oldPoint.y = to.X[1];
//            oldPoint.z = to.X[2];
//            /* If the old obstable is not currently in view don't remove it */
//            double al = -m_currentPTZPose.pan;
//            double nx = it->p.x * cos(al) - it->p.y * sin(al);
//            double ny = it->p.x * sin(al) + it->p.y * cos(al);
//            if ((nx < 1.8) && (0.535 * nx - ny > 0) && (-0.535 * nx - ny < 0)) updateHeightMap = false;
            updateHeightMap = false;
          }
          /* Don't place a free space if it is out of safety range */

          if (pZ <= m_obstacleMinHeight){
            double al = -m_currentPTZPose.pan;
            double nx = it->p.x * cos(al) - it->p.y * sin(al);
            double ny = it->p.x * sin(al) + it->p.y * cos(al);
            if ((nx > 1.8) || (0.535 * nx - ny < 0) || (-0.535 * nx - ny > 0)) updateHeightMap = false;
          }

          /* If the above tests passed update the height map */ 
          if(updateHeightMap)
            lgmKH(xi, yi) = pZ;

          /* Update bounding box */
          if (xi < pointcloudMinXi)
            pointcloudMinXi = xi;
          if (xi > pointcloudMaxXi)
            pointcloudMaxXi = xi;
          if (yi < pointcloudMinYi)
            pointcloudMinYi = yi;
          if (yi > pointcloudMaxYi)
            pointcloudMaxYi = yi;
        }
      }
      }
      m_lastPointCloudTime = getCASTTime();
    }
    {
      /* Project the height map onto the 2D obstacle map */ 
      SCOPED_TIME_LOG;
      blitHeightMap(*tmp_lgm, m_lgmKH, min(pointcloudMinXi,laserMinXi), max(pointcloudMaxXi,laserMaxXi), min(pointcloudMinYi,laserMinYi), max(pointcloudMaxYi,laserMaxYi), m_obstacleMinHeight, m_obstacleMaxHeight);
    }
  }
  else {

    int minX = laserMinXi;
    int maxX = laserMaxXi;
    int minY = laserMinYi;
    int maxY = laserMaxYi;

    if (minX <= -m_lgm->getSize())
      minX = -m_lgm->getSize() + 1;
    if (maxX >= m_lgm->getSize())
      maxX = m_lgm->getSize() - 1;
    if (minY <= -m_lgm->getSize())
      minY = -m_lgm->getSize() + 1;
    if (maxY >= m_lgm->getSize())
      maxY = m_lgm->getSize() - 1;

    for (int yi = minY; yi <= maxY; yi++) {
      for (int xi = minX; xi <= maxX; xi++) {
        if ((*tmp_lgm)(xi, yi) != '1'){
          double cellsize = m_lgm->getCellSize();
          double al = -(lpW.getTheta()+m_currentPTZPose.pan);
          double nx = (xi*cellsize-lpW.getX()) * cos(al) - (yi*cellsize-lpW.getY()) * sin(al);
          double ny = (xi*cellsize-lpW.getX()) * sin(al) + (yi*cellsize-lpW.getY()) * cos(al);
					if (m_simulateKinect &&
           ((nx > 1.8) || (0.535 * nx - ny < 0) || (-0.535 * nx - ny > 0)))
						(*tmp_lgm)(xi, yi) = '2';
        }
      }
    }
  
  }
  const int deltaN = 3;
  double d = m_lgm->getCellSize()/deltaN;
  int maxcellstocheck = int (5.0/d);
  double xWT,yWT;
  double theta;


  Cure::Pose3D currPose;
  {
    IceUtil::Mutex::Lock lock(m_PPMutex);
    currPose = m_TOPP.getPose();		
  }
  /* Update the nav map */
  m_LMap.clearMap();
  tmp_lgm->setValueInsideCircle(currPose.getX(), currPose.getY(), 0.55*Cure::NewNavController::getRobotWidth(), '0');                                  
  for (int i = 0; i < m_Npts; i++) {
    theta = m_StartAngle + m_AngleStep * i;
    for (int j = 1; j < deltaN*maxcellstocheck; j++){
      xWT = currPose.getX()+j*d*cos(theta);
      yWT = currPose.getY()+j*d*sin(theta);
      if(m_lgm->worldCoords2Index(xWT,yWT,xi,yi)==0){
	      if((*tmp_lgm)(xi,yi) == '1'){
	        m_LMap.addObstacle(xWT, yWT, 1);
	        break;    
	      }
      }
    }
  }

  /* Copy the temporary map */
//  IceUtil::Mutex::Lock lock(m_MapsMutex);
//  *m_lgm = *tmp_lgm;
//  delete tmp_lgm;
}

void SpatialControl::runComponent() 
{
  setupPushScan2d(*this, 0.1);
  setupPushOdometry(*this);

    if(m_usePeekabot){
        while(!m_PeekabotClient.is_connected() && (m_RetryDelay > -1)){
            sleep(m_RetryDelay);
            connectPeekabot();
        }
        CreateGridMap(); 
  
    }
  int count = 0;
  while(isRunning()){
    if(count  == 12){
      if (m_saveLgm) SaveGridMap();
      count = 0;
    }
    count++;

    {
      m_OdomQueueMutex.lock();
    }
    if (m_odometryQueue.size() > 0) 
      //    while (m_odometryQueue.size() > 0) 
    {
      Cure::Pose3D cureOdom = m_odometryQueue.front();
      m_odometryQueue.pop_front();
      m_OdomQueueMutex.unlock();

      {
	SCOPED_TIME_LOG;
	processOdometry(cureOdom);
      }
      m_OdomQueueMutex.lock();
    }
    m_OdomQueueMutex.unlock();

    {

      //FIXME use robot pose


      {
	SCOPED_TIME_LOG;
	updateGridMaps();
      }

      if(m_usePeekabot){
	SCOPED_TIME_LOG;
	UpdateGridMap();
      }


    }
    //FIXME Too slow!
    {
      Cure::Pose3D currentPose = m_TOPP.getPose();
      {
	if (m_Displaylgm) {
	  m_Displaylgm->updateDisplay(&currentPose,
	      &m_NavGraph, 
	      &m_Frontiers);
	}
      }

      {
	if(m_displayBinaryMap)
	  m_displayBinaryMap->updateDisplay(&currentPose);


	if(m_DisplayCureObstacleMap) {
	  // Clear out obstacle map (that's used for visualization only)
	  int radius = m_obstacleMap->getSize();
	  for(int i = -radius; i < radius; ++i) {
	    for(int j = -radius; j < radius; ++j) {
	      (*m_obstacleMap)(i,j) = '0';
	    }
	  }
	  for(unsigned int i = 0; i < m_LMap.nObst(); i++) {
	    ObstPt obspt = m_LMap.obstRef(i);
	    (*m_obstacleMap)((obspt.x)/0.05, (obspt.y)/0.05) = '1';
	  }

	  m_displayObstacleMap->updateDisplay(&currentPose);
	}
      }
    }

    if (m_visualExplorationOngoing && m_waitingForPTZCommandID == "") {
      // If we've gotten at least one cloud since we finished moving the
      // PTU, we can move on to the next phase
      error("last Point cloud time: %f", m_lastPointCloudTime.s+m_lastPointCloudTime.us*1e-6);
      error("last PTU time: %f", m_lastPtzNavPoseCompletion.s+m_lastPtzNavPoseCompletion.us*1e-6);
      if (m_lastPointCloudTime > m_lastPtzNavPoseCompletion) {
	//      cdl::CASTTime diff = getCASTTime() - m_lastPtzNavPoseCompletion;
	////FIXME time is negative !
	//    error("alex time diff %f",(double)diff.s + (double)diff.us*(1e-6));
	//    error("alex m_visualExplorationPhase = %d",m_visualExplorationPhase);
	//    if (fabs((double)diff.s + (double)diff.us*1e-6) > 2.0) 
	if (m_visualExplorationPhase == 1) {
	  error("alex m_visualExplorationPhase == 1");
	  startMovePanTilt(M_PI/3, -M_PI/4, 0);
	  m_visualExplorationPhase = 2;
	}
	else if (m_visualExplorationPhase == 2) {
	  error("alex m_visualExplorationPhase == 2");
	  startMovePanTilt(0, -M_PI/4, 0);
	  m_visualExplorationPhase = 0;
	  m_visualExplorationOngoing = false;
	  try {
	    NavData::VisualExplorationCommandPtr cmd = 
	      getMemoryEntry<NavData::VisualExplorationCommand>(m_visualExplorationCommand);
	    cmd->comp = NavData::SUCCEEDED;
	    overwriteWorkingMemory<NavData::VisualExplorationCommand>(m_visualExplorationCommand, cmd);
	    error("alex cmd->comp = NavData::SUCCEEDED;");
	  }
	  catch (DoesNotExistOnWMException) {
	    error("Could not find visual exploration command for overwriting!");
	  }
	}
      }
    }	  
    //    if (m_odometryQueue.size() == 0) {
    //      usleep(250000);
    //    }
  }
} 


void SpatialControl::newNavGraph(const cdl::WorkingMemoryChange &objID){
//  m_Mutex.lock();
	
  m_NavGraph.clear();
  bool gateway = false;
  shared_ptr<CASTData<NavData::NavGraph> > oobj =
    getWorkingMemoryEntry<NavData::NavGraph>(objID.address);
  
  if (oobj != 0) {
    
    NavData::NavGraphPtr ng = oobj->getData();
        
    for (unsigned int i=0;i<ng->fNodes.size();i++){
      
      double x = ng->fNodes[i]->x;
      double y = ng->fNodes[i]->y;
      double theta = ng->fNodes[i]->theta;
      int areaId = (int) ng->fNodes[i]->areaId;
      string areaType = "";
      if (!ng->fNodes[i]->type.empty()) {
        areaType = ng->fNodes[i]->type[0].name;
      }
      int nodeId = (int) ng->fNodes[i]->nodeId;

      const double maxSpeed = 2;
      
      if (ng->fNodes[i]->gateway){ // add gateway node
               
	double width = 1;
        if (!ng->fNodes[i]->width.empty()) {
          width = ng->fNodes[i]->width[0];
        }
	m_NavGraph.addDoorToNodeList(nodeId, areaId, x, y, theta, 
                                     width, areaType, maxSpeed);
	gateway = true;
      }

      else { // add ordinary node

	m_NavGraph.addNodeToNodeList(nodeId, areaId, x, y, theta, 
                                      areaType, maxSpeed);

      }
      
    }
    
    for (unsigned int i=0;i<ng->aEdges.size();i++){ // add edges
      
      int n1 = (int) ng->aEdges[i]->startNodeId;
      int n2 = (int) ng->aEdges[i]->endNodeId;
      
      m_NavGraph.addEdgeToEdgeList(n1, n2);
      
    }
    
    m_NavGraph.connectNodes();

    debug("Got a new graph with %d doors and a total of %d nodes", m_NavGraph.m_Gateways.size(), m_NavGraph.m_Nodes.size());
    
    if (!m_ready) m_ready = true;

    if (m_ready) debug("m_ready is true");
    else debug("m_ready is false");    
    
  }

//  m_Mutex.unlock();
}

void SpatialControl::newPersonData(const cdl::WorkingMemoryChange &objID)
{
  debug("newPersonData called");

  // Person entries can be removed at any time

  try {
    shared_ptr<CASTData<NavData::Person> > oobj =
      getWorkingMemoryEntry<NavData::Person>(objID.address);

    const NavData::PersonPtr p = oobj->getData();

    bool addNewPerson = true;

    IceUtil::Mutex::Lock lock(m_PeopleMutex);

    // Check if the person already exists, otherwise add it
    for (unsigned int i = 0; i < m_People.size(); i++) {
      if (m_People[i].m_data->id == p->id) {
	// Update it
	m_People[i].m_data = p;
	addNewPerson = false;
	break;
      }
    }

    if (addNewPerson) {
      SpatialControl::PersonData pd;
      pd.m_WMid = objID.address.id;
      pd.m_data = p;
      m_People.push_back(pd);
    }

  }catch(DoesNotExistOnWMException){}
}
  
void SpatialControl::deletePersonData(const cdl::WorkingMemoryChange &objID)
{
  log("deletePersonData called");

  int i = 0;

  IceUtil::Mutex::Lock lock(m_PeopleMutex);

  for (std::vector<SpatialControl::PersonData>::iterator pi = m_People.begin();
       pi != m_People.end(); pi++, i++) {

    if (objID.address.id == pi->m_WMid) {
      if (i == m_CurrPerson) {
        log("Deleting the person we are currently following");
        changeCurrentCommandCompletion(NavData::ABORTED,
                                       NavData::PERSONNOTFOUND);
        m_CurrPerson = -1;
      } else if (i < m_CurrPerson) {
        // since m_CurrPerson gives the index in the list of people we
        // need to decrement it when a person with an index lower than
        // this is removed from the vector
        m_CurrPerson -= 1;
        log("Deleting person (not being followed) and had to fix index");
      } else {
        log("Deleting person (not being followed)");
      }
      m_People.erase(pi);
      break;
    }
  }
}


void SpatialControl::newVisualExplorationCommand(const cdl::WorkingMemoryChange &objID) 
{
  log("Received new VisualExplorationCommand");
  try {
    NavData::VisualExplorationCommandPtr obj =
      getMemoryEntry<NavData::VisualExplorationCommand>(objID.address);

    IceUtil::Mutex::Lock lock(m_taskStatusMutex);

    if (m_taskStatus == NothingToDo) {
      m_visualExplorationOngoing = true;
      m_visualExplorationPhase = 1;
      error("alex (1) m_visualExplorationPhase = %d", m_visualExplorationPhase);
      m_visualExplorationCommand = objID.address.id;
      startMovePanTilt(-M_PI/3, -M_PI/4, 0);
    }
    else {
      log("SpatialControl busy; cannot execute VisualExplorationCommand");
      obj->comp = NavData::FAILED;
      overwriteWorkingMemory<NavData::VisualExplorationCommand>(objID.address.id, obj);
    }
  }
  catch (DoesNotExistOnWMException) {
    log("Could not find VisualExplorationCommand on WM!");
  }
}
  
void SpatialControl::newInhibitor(const cdl::WorkingMemoryChange &objID) 
{
  m_NumInhibitors++;
  log("Got new inhibitor, now has %d", m_NumInhibitors);
}

void SpatialControl::deleteInhibitor(const cdl::WorkingMemoryChange &objID) 
{
  m_NumInhibitors--;
  log("Deleted an inhibitor now has %d", m_NumInhibitors);
}

void SpatialControl::newRobotPose(const cdl::WorkingMemoryChange &objID) 
{
//  static cast::cdl::CASTTime oldTime = getCASTTime();
//  cast::cdl::CASTTime newTime = getCASTTime();
//  long int diff = (newTime.s-oldTime.s)*1000000l+(newTime.us-oldTime.us);
//  oldTime = newTime;
//  if (diff > 500000) {
//    error("SpatialControl::newRobotPose - interval: %f s", ((double)diff)*1e-6);
//  }
//  else if (diff > 100000) {
//    log("SpatialControl::newRobotPose - interval: %f s", ((double)diff)*1e-6);
//  }
//  else {
//    log("SpatialControl::newRobotPose - interval: %f s", ((double)diff)*1e-6);
//  }

  debug("SpatialControl::newRobotPose");
  NavData::RobotPose2dPtr oobj =
    getMemoryEntry<NavData::RobotPose2d>(objID.address);

  //FIXME
  m_SlamRobotPose.setTime(Cure::Timestamp(oobj->time.s,
                                          oobj->time.us));
  m_SlamRobotPose.setX(oobj->x);
  m_SlamRobotPose.setY(oobj->y);
  m_SlamRobotPose.setTheta(oobj->theta);
		//log("time of newRobtoPose() = %d.%ld s",(long int)oobj->getData()->time.s, (long int)oobj->getData()->time.us);
  
  m_lastSLAMPoseTime = oobj->time;

  Cure::Pose3D cp = m_SlamRobotPose;

  IceUtil::Mutex::Lock lock(m_PPMutex);

  m_TOPP.defineTransform(cp);
  debug("defined: %i", m_TOPP.isTransformDefined());
}


void SpatialControl::newNavCtrlCommand(const cdl::WorkingMemoryChange &objID) 
{
  // This component only manages one nav ctrl command at a time
  log("newNavCtrlCommand called");
  
  //REMOVEME
  SaveGridMap();

  shared_ptr<CASTData<NavData::InternalNavCommand> > oobj =
    getWorkingMemoryEntry<NavData::InternalNavCommand>(objID.address);
  
  
  if (oobj != 0){

    IceUtil::Mutex::Lock lock(m_taskStatusMutex);

    log("read new ctrlCommand");

    if(m_taskStatus != NothingToDo){
      // cancel ongoing command
      
      // if it was following, cancel
      if(m_commandType == NavData::lFOLLOWPERSON){
      	m_CurrPerson = -1;
      	// ### is this enough to cancel the chase?
      }
      
      changeCurrentCommandCompletion(NavData::ABORTED,
                                     NavData::REPLACEDBYNEWCMD);
    }
    m_CurrentCmdAddress = objID.address;

    if (m_sendPTZCommands) {
      startMovePanTilt(0, -M_PI/4, 0);
    }
    
    m_commandType = oobj->getData()->cmd;
    m_commandX = oobj->getData()->x;
    m_commandY = oobj->getData()->y;
    m_commandR = oobj->getData()->r;
    m_commandTheta = oobj->getData()->theta;
    m_commandDistance = oobj->getData()->distance;
    m_commandAreaId = oobj->getData()->areaId;
    m_commandNodeId = oobj->getData()->nodeId;
    ExplorationConfinedByGateways = oobj->getData()->SetExplorerConfinedByGateways;
    m_TolPos = m_DefTolPos;
    m_TolRot = m_DefTolRot;
    if (m_commandType == NavData::lGOTOXYA) {
      if (oobj->getData()->tolerance.size() > 1) {
        m_TolRot = oobj->getData()->tolerance[1];
      }
      if (oobj->getData()->tolerance.size() > 0) {
        m_TolPos = oobj->getData()->tolerance[0];
      }
    } else if (m_commandType == NavData::lGOTOXY ||
               m_commandType == NavData::lGOTOXYROUGH ||
               m_commandType == NavData::lGOTOPOLAR ||
               m_commandType == NavData::lGOTONODE ||
               m_commandType == NavData::lBACKOFF) {
      if (oobj->getData()->tolerance.size() > 0) {
        m_TolPos = oobj->getData()->tolerance[0];
      }      
    } else if (m_commandType == NavData::lROTATEREL ||
               m_commandType == NavData::lROTATEABS) {
      if (oobj->getData()->tolerance.size() > 0) {
        m_TolRot = oobj->getData()->tolerance[0];
      }
    }


    // If we are supposed to follow a person we pick the currently
    // closest one
    if (m_commandType == NavData::lFOLLOWPERSON) {
      double minD = 1e10;
      m_CurrPerson = -1;

      IceUtil::Mutex::Lock lock2(m_PeopleMutex);

      for (unsigned int i = 0; i < m_People.size(); i++) {
        double d = hypot(m_CurrPose.getY() - m_People[i].m_data->y,
                         m_CurrPose.getX() - m_People[i].m_data->x);
        if (d < minD) {
          m_CurrPerson = i;
          minD = d;
        }
      }

      if (m_CurrPerson >= 0) {
        char buf[256];
        sprintf(buf, "Found person to follow at xW=%.2fm yW=%.2fm (d=%.2fm)\n",
                m_People[m_CurrPerson].m_data->y,
                m_People[m_CurrPerson].m_data->x,
                minD);
        log(buf);
      } else {
        log("Ordered to follow a person but found no person to follow");
        m_CurrPerson = -1;
        m_taskStatus = NothingToDo;
        changeCurrentCommandCompletion(NavData::ABORTED,
                                       NavData::PERSONNOTFOUND);
        return;
      }
    }
    
    log("read new ctrlCommand");
    m_taskStatus = NewTask;
  }
}


void SpatialControl::receiveOdometry(const Robotbase::Odometry &castOdom)
{
  Cure::Pose3D cureOdom;
  CureHWUtils::convOdomToCure(castOdom, cureOdom);

  //Can't defer this; odometry must be in the TOPP before
  //the new RobotPose arrives
  {
    debug("lock receiveOdometry");
    IceUtil::Mutex::Lock lock(m_PPMutex); //Don't allow any interface calls while processing a callback
    debug("lock acquired");
    m_TOPP.addOdometry(cureOdom); 

    m_CurrPose = m_TOPP.getPose();
    if (m_usePeekabot) {
      m_OdomPoseProxy.set_pose(m_CurrPose.getX(), m_CurrPose.getY(), 2.3, m_CurrPose.getTheta(), 0, 0);
    }
    debug("unlock receiveOdometry");
  }
	
  {
    IceUtil::Mutex::Lock lock(m_OdomQueueMutex);
    if (m_odometryQueue.size() == 0) {
        m_odometryQueue.push_back(cureOdom);
    }
    else {
      m_odometryQueue.front() = cureOdom;
    }
  }
}

void SpatialControl::processOdometry(Cure::Pose3D cureOdom)
{
  log("Got odometry x=%.2f y=%.2f a=%.4f t=%.6f",
        cureOdom.getX(), cureOdom.getY(), cureOdom.getTheta(),
        cureOdom.getTime().getDouble());
  log("CASTTime: %f",getCASTTime().s+1e-6*getCASTTime().us);
  
  if (m_ready || m_bNoNavGraph) { // have to get a first nav graph 
                 // to be ready
    
    {
      {
	IceUtil::Mutex::Lock lock(m_taskStatusMutex); // acquire mutex!
      }

      if(m_taskStatus == TaskFinished){
	// report final status
	changeCurrentCommandCompletion(m_CurrentCmdFinalCompletion,
	    m_CurrentCmdFinalStatus);
	m_taskStatus = NothingToDo;

      }else if(m_taskStatus == NewTask 
	  && (m_commandType == NavData::lSTOPROBOT || m_waitingForPTZCommandID == ""))
      {

	//      m_Mutex.lock();

	// Result of m_NavCtrl.operationX
	int ret = -1; // -1:operation not done, 0:ok, >0:error
	m_currentTaskIsExploration = false; // set later

	// Task id
	int currentTaskId = m_taskId++;

	// GOTO_XYA        
	if ((m_commandType == NavData::lGOTOXYA)) {

	  log("executing command GOTOXYA");  	

	  Cure::NewNavController::setPositionToleranceFinal(m_TolPos);
	  Cure::NewNavController::setOrientationTolerance(m_TolRot);
	  ret = Cure::NewNavController::gotoXYA
	    (currentTaskId, m_commandX, m_commandY, m_commandTheta);
	}

	// GOTO_XY_ROUGH

	else if ((m_commandType == NavData::lGOTOXYROUGH)){ 

	  log("executing command GOTOXYROUGH"); 

	  Cure::NewNavController::setPositionToleranceFinal(m_TolPos);
	  Cure::NewNavController::setOrientationTolerance(m_TolRot);
	  ret = Cure::NewNavController::gotoXY(currentTaskId, m_commandX, m_commandY);
	}


	// GOTO_XY

	else if ((m_commandType == NavData::lGOTOXY)){ 

	  log("executing command GOTOXY"); 

	  Cure::NewNavController::setPositionToleranceFinal(m_TolPos);
	  Cure::NewNavController::setOrientationTolerance(m_TolRot);
	  ret = Cure::NewNavController::gotoXY(currentTaskId, m_commandX, m_commandY);
	  //Clean out path; use only final waypoint
	  if(!m_Path.empty()) {
	    Cure::NavGraphNode lastNode = m_Path.back();
	    m_Path.clear();
	    m_Path.push_back(lastNode);
	  }
	  log("sent command.");
	}

	// GOTO_POLAR

	else if ((m_commandType == NavData::lGOTOPOLAR)) {

	  log("executing command GOTOPOLAR"); 

	  Cure::NewNavController::setPositionToleranceFinal(m_TolPos);
	  Cure::NewNavController::setOrientationTolerance(m_TolRot);
	  ret = Cure::NewNavController::gotoPolar(currentTaskId,m_commandTheta,m_commandR);
	}

	// GOTO_AREA

	else if ((m_commandType == NavData::lGOTOAREA)) {

	  log("executing command GOTOAREA"); 

	  Cure::NewNavController::setPositionToleranceFinal(m_TolPos);
	  Cure::NewNavController::setOrientationTolerance(m_TolRot);
	  ret = Cure::NewNavController::gotoArea(currentTaskId, m_commandAreaId);
	}


	// GOTO_NODE

	else if ((m_commandType == NavData::lGOTONODE)) {

	  log("executing command GOTONODE"); 

	  Cure::NewNavController::setPositionToleranceFinal(m_TolPos);
	  Cure::NewNavController::setOrientationTolerance(m_TolRot);
	  ret = Cure::NewNavController::gotoNode(currentTaskId, m_commandNodeId);
	}

	// ROTATE_REL

	else if ((m_commandType == NavData::lROTATEREL)) {  

	  log("executing command ROTATEREL"); 

	  Cure::NewNavController::setPositionToleranceFinal(m_TolPos);
	  Cure::NewNavController::setOrientationTolerance(m_TolRot);
	  ret = Cure::NewNavController::rotateRel(currentTaskId, m_commandTheta);

	}


	// ROTATE_ABS

	else if ((m_commandType == NavData::lROTATEABS)) {

	  log("executing command ROTATEABS"); 

	  Cure::NewNavController::setPositionToleranceFinal(m_TolPos);
	  Cure::NewNavController::setOrientationTolerance(m_TolRot);
	  ret = Cure::NewNavController::rotateAbs(currentTaskId, m_commandTheta);

	}

	// BACK_OFF

	else if ((m_commandType == NavData::lBACKOFF)) {

	  log("executing command BACKOFF"); 

	  Cure::NewNavController::setPositionToleranceFinal(m_TolPos);
	  Cure::NewNavController::setOrientationTolerance(m_TolRot);
	  ret = Cure::NewNavController::backOff(currentTaskId, m_commandDistance);

	}

	// STOP

	else if ((m_commandType == NavData::lSTOPROBOT)) {

	  log("executing command STOPROBOT"); 

	  // This command is special:
	  // It is always accomplished at the moment, by this thread,
	  // and does not raise a doneTask calling (but can raise
	  // abortTask).
	  // 
	  ret = Cure::NewNavController::stop();

	}

	// EXPLORE


	// Change completion now

	// First treat stop case
	if(m_commandType == NavData::lSTOPROBOT){
	  changeCurrentCommandCompletion(NavData::SUCCEEDED,
	      NavData::UNKNOWN);
	  m_taskStatus = NothingToDo;

	} else if(m_taskStatus != TaskFinished){

	  if ((m_commandType == NavData::lFOLLOWPERSON)) {

	    IceUtil::Mutex::Lock lock2(m_PeopleMutex);

	    if (m_CurrPerson < 0 || m_CurrPerson > (int)m_People.size()-1) {

	      m_CurrPerson = -1;
	      m_taskStatus = NothingToDo;
	      changeCurrentCommandCompletion(NavData::ABORTED,
		  NavData::PERSONNOTFOUND);

	      log("Lost the person we were tracking");



	    } else {

	      ret = Cure::NewNavController::followPerson(currentTaskId,
		  m_People[m_CurrPerson].m_data->x,
		  m_People[m_CurrPerson].m_data->y,
		  m_People[m_CurrPerson].m_data->direction,
		  m_People[m_CurrPerson].m_data->speed,
		  0);

	    }

	  } else {
	    // this means that the m_navCtrl events were not triggered off,
	    // so we have to send a completion now
	    if(ret == 0){
	      m_taskStatus = ExecutingTask;
	      changeCurrentCommandCompletion(NavData::INPROGRESS,
		  NavData::UNKNOWN);
	    }else if(ret > 0){
	      m_taskStatus = NothingToDo;
	      changeCurrentCommandCompletion(NavData::FAILED,
		  NavData::UNKNOWN);
	    }
	  }
	}
	//      m_Mutex.unlock();
      }

    }

    {
      m_LMap.moveRobot(m_CurrPose);
    }

  static cast::cdl::CASTTime oldTime = getCASTTime();
  cast::cdl::CASTTime newTime = getCASTTime();
  long int diff = (newTime.s-oldTime.s)*1000000l+(newTime.us-oldTime.us);
  oldTime = newTime;
  if (diff > 500000) {
    error("SpatialControl::updateCtrl - interval: %f s", ((double)diff)*1e-6);
  }
  else if (diff > 100000) {
    log("SpatialControl::updateCtrl - interval: %f s", ((double)diff)*1e-6);
  }
  else {
    debug("SpatialControl::updateCtrl - interval: %f s", ((double)diff)*1e-6);
  }

  diff = (newTime.s-m_lastSLAMPoseTime.s)*1000000l+(newTime.us-m_lastSLAMPoseTime.us);
  if (diff > 2000000) {
    error("SpatialControl::updateCtrl - SLAM pose age: %f s", ((double)diff)*1e-6);
  }
  else if (diff > 1000000) {
    log("SpatialControl::updateCtrl - SLAM pose age: %f s", ((double)diff)*1e-6);
  }
  else {
    debug("SpatialControl::updateCtrl - SLAM pose age: %f s", ((double)diff)*1e-6);
  }

    {
      SCOPED_TIME_LOG;
    Cure::NewNavController::updateCtrl();
    }

  } // if (m_ready)    
}

void SpatialControl::receiveScan2d(const Laser::Scan2d &castScan)
{
	
  debug("lock receiveScan2d");
//  lockComponent(); //Don't allow any interface calls while processing a callback
//  debug("lock acquired");
  debug("Got scan with n=%d and t=%ld.%06ld",
        castScan.ranges.size(), 
        (long)castScan.time.s, (long)castScan.time.us);
	
  Cure::LaserScan2d cureScan;
  CureHWUtils::convScan2dToCure(castScan, cureScan);

//  if (!m_UsePointCloud) {
//
//    Cure::Pose3D scanPose;
//    int status = 1;
//    {
//      IceUtil::Mutex::Lock lock(m_PPMutex);
//
//
//      if (m_TOPP.isTransformDefined()) {
//	status = m_TOPP.getPoseAtTime(cureScan.getTime(), scanPose);
//      }
//    }
//
//    if (status == 0) {
//      IceUtil::Mutex::Lock lock(m_MapsMutex);
//
//      m_LMap.addScan(cureScan, m_LaserPoseR, scanPose);
//
//      Cure::Pose3D lpW;
//      m_lgm->setValueInsideCircle(scanPose.getX(), scanPose.getY(),
//	  0.5*Cure::NewNavController::getRobotWidth(), 
//	  '0');
//      lpW.add(scanPose, m_LaserPoseR);
//      m_Glrt->addScan(cureScan, lpW, m_MaxExplorationRange);      
//      m_firstScanAdded = true;
//    }
//  }
//  else 
  {
    IceUtil::Mutex::Lock lock(m_ScanQueueMutex);
    if (m_LScanQueue.size() == 0) {
      m_LScanQueue.push(cureScan);
    }
    else {
      m_LScanQueue.front() = cureScan;
    }
  }

  /* Person following stuff */    
  NavData::PersonFollowedPtr p = new NavData::PersonFollowed;
  static long last_id_sent = -1;
  {
    IceUtil::Mutex::Lock lock(m_PeopleMutex);

    if (m_CurrPerson >= 0 && m_CurrPerson < (int)m_People.size()) {
      p->id = m_People[m_CurrPerson].m_data->id;
    } else {
      p->id = -1;
    }
    if(last_id_sent != p->id) {
      last_id_sent = p->id;
      p->time = castScan.time;
      if (m_CurrPersonWMid == "") {
	// Create the entry in the working memory for the 
	m_CurrPersonWMid = newDataID();
	addToWorkingMemory<NavData::PersonFollowed>(m_CurrPersonWMid, p);
	log("added id of person to follow");
      }else{
	debug("Updating id of person to follow");
	overwriteWorkingMemory<NavData::PersonFollowed>(m_CurrPersonWMid, p);
	debug("updated id of person to follow");
      }
    }
  }
//  unlockComponent();
//  debug("unlock receiveScan2d");
}

void 
SpatialControl::execCtrl(Cure::MotionAlgorithm::MotionCmd &cureCmd) 
{
  debug("execCtrl(type=%d, dir=%fdeg, v=%fm/s, w=%frad/s) called", 
        cureCmd.type, 180.0/M_PI*cureCmd.dir, cureCmd.v, cureCmd.w);

  Robotbase::MotionCommand cmd;

  if (m_NumInhibitors > 0) {    
    if (!m_SentInhibitStop) {
      log("Sending stop after being inhibited, and then not doing anything");
      cmd.speed = 0;
      cmd.rotspeed = 0;
      m_RobotServer->execMotionCommand(cmd);    
      m_SentInhibitStop = true;
    } else {
      debug("Got inhibitors in WM, not doing anything");
    }
    return;
  } else {
    m_SentInhibitStop = false;
  }

  if (cureCmd.type == Cure::MotionAlgorithm::CMD_TYPE_STOP) {

    cmd.speed = 0;
    cmd.rotspeed = 0;

  } else if (cureCmd.type == Cure::MotionAlgorithm::CMD_TYPE_VW) {

    // pure speed command
    cmd.speed = cureCmd.v;
    cmd.rotspeed = cureCmd.w;

  } else if (cureCmd.type == Cure::MotionAlgorithm::CMD_TYPE_VA) {

    // translation speed and motion direction

    Cure::Pose3D cp = m_CurrPose;
    double da = Cure::HelpFunctions::angleDiffRad(cureCmd.dir, cp.getTheta());
    cmd.rotspeed = 0.5 * da;
    Cure::HelpFunctions::limitAndSetValueSymm(cmd.rotspeed, 0.5);

    // We limit the translation speed when we need to turn much
    cmd.speed = cureCmd.v * exp(-da*da/(0.3*0.3));

  } else {

    println("No support for motion command type %d\n", cureCmd.type);
    cmd.speed = 0;
    cmd.rotspeed = 0;

  }   

  debug("execCtrl sending (v=%fm/s,w=%frad/s) to RobotServer", cmd.speed, cmd.rotspeed);
  
  SCOPED_TIME_LOG;
  m_RobotServer->execMotionCommand(cmd);
}

/* Fills map with an expanded version of gridmap, where unknown space is
   also set as obstacles */
void SpatialControl::getExpandedBinaryMap(const Cure::LocalGridMap<unsigned char>* gridmap, Cure::BinaryMatrix &map) const {
  
//  if(lockMapsMutex) {
//
//    m_MapsMutex.lock();
//    }

  //FIXME: Don't reallocate every time!

  int gridmapSize = gridmap->getSize();

  /* The Cure documentation recommends keeping columns as a multiple
     of 32 to be able to exploit the performance improvements of doing
     calculations on 32 bit longs. However, we use both 32 and 64 bit
     operating systems now and longs on unix systems are usually different
     sizes for these architectures. If anyone knows the correct way of
     handling this, please modify the code below. */
  Cure::BinaryMatrix ungrown_map;
  ungrown_map.reallocate(2*gridmapSize, 2*gridmapSize);
  ungrown_map = 0; // Set all cells to zero

  // Transfer obstacles for the local gridmap
  for(int x = -gridmapSize; x < gridmapSize; ++x) {
    for(int y = -gridmapSize; y < gridmapSize; ++y) {
      if((*gridmap)(x,y) == '1') {
        ungrown_map.setBit(x + gridmapSize, y + gridmapSize, true);
      }
    }
  }

  // Grow each occupied cell to account for the size of the robot.
  ungrown_map.growInto(map, 0.5*Cure::NewNavController::getRobotWidth() / m_lgm->getCellSize());

  /* Set unknown space as obstacles, since we don't want to find paths
  going through space we don't know anything about */
  for(int x = -gridmapSize; x < gridmapSize; ++x) {
    for(int y = -gridmapSize; y < gridmapSize; ++y) {
      if((*gridmap)(x,y) == '2') {
        map.setBit(x + gridmapSize, y + gridmapSize, true);
      }
    }
  }

//  if(lockMapsMutex) {
//    m_MapsMutex.unlock();
//  }
}

void SpatialControl::setFrontierReachability(std::list<Cure::FrontierPt> &frontiers) {
  Cure::BinaryMatrix map;

//  //Mutual exclusion vs. changes to m_lgm in updateGridMaps
//  IceUtil::Mutex::Lock lock(m_MapsMutex); 

  getExpandedBinaryMap(m_lgm, map);

  for(std::list<Cure::FrontierPt>::iterator it = frontiers.begin();
      it != frontiers.end(); ++it) {
    it->m_State = Cure::FrontierPt::FRONTIER_STATUS_UNREACHABLE;
  }

  /* Update the map used for visualization of the binarymap.
     Note that we don't lock the map mutex here, since m_binaryMap
     is not important for robot behaviour, only visualization.
     Not waiting for a lock takes precedence to always getting the
     correct visualization */
  for(int x = -m_binaryMap->getSize(); x < m_binaryMap->getSize(); ++x) {
    for(int y = -m_binaryMap->getSize(); y < m_binaryMap->getSize(); ++y) {
      (*m_binaryMap)(x,y) = map(x+m_binaryMap->getSize(),y+m_binaryMap->getSize()) ? '1' : '0';
    }
  }
  
  int rX, rY; // Robot position index

  // Get robot position
  Cure::Pose3D robotPose = m_TOPP.getPose();
  if(m_lgm->worldCoords2Index(robotPose.getX(), robotPose.getY(), rX, rY) != 0) {
    log("Robot position is outside of gridmap! Returning.");
    return;
  }

  // Offset the indices so that top left is (0,0).
  rX += m_lgm->getSize();
  rY += m_lgm->getSize();

  /* Do a BFS over the reachable parts of the map to see if the coorinates in
     points are reachable */
  std::queue<std::pair<int, int> > toVisit;
  toVisit.push(make_pair(rX,rY));

  while (!toVisit.empty()) {
    std::pair<int,int> coord = toVisit.front();
    toVisit.pop();

    int x = coord.first;
    int y = coord.second;

    map.setBit(x,y, true); // Set as visited

    // Check if this coordinate is in points
    for(list<Cure::FrontierPt>::iterator it = frontiers.begin();
        it != frontiers.end(); ++it) {
      int pX, pY;
      if(m_lgm->worldCoords2Index(it->getX(),it->getY(), pX,pY) != 0) {
        log("Coordinate is outside of map.");
        continue;
      }

      // Offset the indices so that top left is (0,0)
      pX += m_lgm->getSize();
      pY += m_lgm->getSize();

      if(pX == x && pY == y)
        it->m_State = Cure::FrontierPt::FRONTIER_STATUS_OPEN;
    }

    // Add neighbors if they are in free space, has not been visited before
    // and are not out of bounds
    if(y-1 >= 0 && !map(x  , y-1)) {
      toVisit.push(make_pair(x  , y-1));
      map.setBit(x,y-1,true);
    }
    if(x-1 >= 0 && !map(x-1, y  )){
      toVisit.push(make_pair(x-1, y  ));
      map.setBit(x-1,y,true);
    }
    if(x+1 < map.Columns-1 && !map(x+1, y  )){
      toVisit.push(make_pair(x+1, y  ));
      map.setBit(x+1,y,true);
    }
    if(y+1 < map.Rows-1 && !map(x  , y+1)) {
      toVisit.push(make_pair(x  , y+1));
      map.setBit(x,y+1,true);
    }
  }
}

/* Finds the node with the shortest path from (x,y) in an expanded binarymap
   Returns -1 on error of if there are no nodes to search or if no node
   was found */
int SpatialControl::findClosestNode(double x, double y) {
  double maxDist = 20; // The first maximum distance to try
  Cure::BinaryMatrix map;

  // Mutual exclusion vs. changes to m_lgm in the main loop (updateGridMaps)
//  IceUtil::Mutex::Lock lock(m_MapsMutex);

  // Get the expanded binary map used to search 
  getExpandedBinaryMap(m_lgm, map);

  // Get all the navigation nodes
  vector<NavData::FNodePtr> nodes;
  getMemoryEntries<NavData::FNode>(nodes, 0);

  if(nodes.size() == 0)
    return -1;

  int xi, yi;
  if(m_lgm->worldCoords2Index(x, y, xi, yi) != 0) {
    return -1;
  }
  
  // Offset the indices so that top left is (0,0).
  xi += m_lgm->getSize();
  yi += m_lgm->getSize();

  int closestNodeId = -1;
  double minDistance = FLT_MAX;
  Cure::ShortMatrix path;
  while(closestNodeId == -1) {
    for(vector<NavData::FNodePtr>::iterator nodeIt = nodes.begin(); nodeIt != nodes.end(); ++nodeIt) {
      try {

        int nodexi, nodeyi;
        if(m_lgm->worldCoords2Index((*nodeIt)->x,(*nodeIt)->y, nodexi, nodeyi) != 0)
          continue;

        // Offset the indices so that top left is (0,0).
        nodexi += m_lgm->getSize();
        nodeyi += m_lgm->getSize();

        double dist = map.path(xi,yi,nodexi,nodeyi, path, maxDist);
        if(dist >= 0) { // If a path was found.. 
          if(dist < minDistance) {
            closestNodeId = (*nodeIt)->nodeId;
            minDistance = dist;
          }
        }
      } catch(IceUtil::NullHandleException e) {
        log("Node suddenly disappeared..");
      }
    }

    if(maxDist > map.Columns*map.Rows)
      return -1;

    maxDist *= 2; // Double the maximum distance to search for the next loop
  }

  return closestNodeId;
}

/* Finds the node with the shortest path from (x,y) in an expanded binarymap
   Returns -1 on error of if there are no nodes to search or if no node
   was found */
int SpatialControl::findClosestPlace(double x, double y, const SpatialData::NodeIDSeq& nodeids) {
  double maxDist = 20; // The first maximum distance to try
  Cure::BinaryMatrix map;

  // Get the expanded binary map used to search 
  getExpandedBinaryMap(m_lgm, map);

  // Get all the navigation nodes
  vector<NavData::FNodePtr> nodes;
  getMemoryEntries<NavData::FNode>(nodes, 0);

  if(nodes.size() == 0)
    return -1;

  int xi, yi;
  if(m_lgm->worldCoords2Index(x, y, xi, yi) != 0) {
    return -1;
  }
  
  // Offset the indices so that top left is (0,0).
  xi += m_lgm->getSize();
  yi += m_lgm->getSize();

  int closestNodeId = -1;
  double minDistance = FLT_MAX;
  Cure::ShortMatrix path;

  while(closestNodeId == -1) {
    for(vector<NavData::FNodePtr>::iterator nodeIt = nodes.begin(); nodeIt != nodes.end(); ++nodeIt) {
      try {
          bool b=true;
          for (size_t g = 0; (g < nodeids.size()) && b; g++)
              if (nodeids[g]==((*nodeIt)->nodeId)) b=false;

          if (!b) {
            int nodexi, nodeyi;
            if(m_lgm->worldCoords2Index((*nodeIt)->x,(*nodeIt)->y, nodexi, nodeyi) != 0)
              continue;

            // Offset the indices so that top left is (0,0).
            nodexi += m_lgm->getSize();
            nodeyi += m_lgm->getSize();

            double dist = map.path(xi,yi,nodexi,nodeyi, path, maxDist);
            if(dist >= 0) { // If a path was found.. 
              if(dist < minDistance) {
                closestNodeId = (*nodeIt)->nodeId;
                minDistance = dist;
              }
          }
        } 
      } catch(IceUtil::NullHandleException e) {
        log("Node suddenly disappeared..");
      }
    }

    if(maxDist > map.Columns*map.Rows)
      return -1;

    maxDist *= 2; // Double the maximum distance to search for the next loop
  }

  return closestNodeId;
}


void SpatialControl::getBoundedMap(SpatialData::LocalGridMap &map, const Cure::LocalGridMap<unsigned char>* gridmap, double minx, double maxx, double miny, double maxy) const {
  int minxi, minyi, maxxi, maxyi; // Cure::LocalGridMap indices
  int lgmsize = gridmap->getSize(); // Size of real gridmap

  // Get the bounds as indices of gridmap
  gridmap->worldCoords2Index(minx,miny, minxi, minyi);
  gridmap->worldCoords2Index(maxx,maxy, maxxi, maxyi);

  // Set map metadata
  map.xCenter = (minx+maxx)/2;
  map.yCenter = (miny+maxy)/2;
  map.cellSize = gridmap->getCellSize();

  int sizeX = (maxxi-minxi)/2;
  int sizeY = (maxyi-minyi)/2;
  int newSize = sizeX > sizeY ? sizeX : sizeY;
  map.size = newSize;

  // Get the square that we will actually loop over
  int xiCenter, yiCenter;
  gridmap->worldCoords2Index(map.xCenter, map.yCenter, xiCenter, yiCenter);
  minxi = xiCenter-newSize;
  minyi = yiCenter-newSize;
  maxxi = xiCenter+newSize;
  maxyi = yiCenter+newSize;

  // Set the map data
  map.data.clear();
  map.data.reserve((maxxi-minxi)*(maxxi-minxi)+(maxxi-minxi)*2+1);

  for(int x = minxi; x <= maxxi; ++x) {
    for(int y = minyi; y <= maxyi; ++y) {
      // Make sure we only get data that was requested. Pad with 'unknown'.
      if(x < -sizeX+xiCenter || x > sizeX+xiCenter ||
           y < -sizeY+yiCenter || y > sizeY+yiCenter ||
           x < -lgmsize || x > lgmsize ||
           y < -lgmsize || y > lgmsize) {
        map.data.push_back('2');
      } else {
        map.data.push_back((*gridmap)(x,y));
      }
    }
  }
}

FrontierInterface::FrontierPtSeq
SpatialControl::getFrontiers()
{
  log("SpatialControl::getFrontiers() called");

  // Mutual exclusion vs. changes to m_lgm in main thread
//  m_MapsMutex.lock();

  while (!m_firstScanAdded) {
    log("  Waiting for first scan to be added...");
//    m_MapsMutex.unlock();
    usleep(1000000);
//    m_MapsMutex.lock();
  }

  m_Frontiers.clear();
  debug("calling findFrontiers");
  m_FrontierFinder->findFrontiers(0.8,2.0,m_Frontiers);
//  m_FrontierFinderKinect->findFrontiers(0.8,2.0,m_Frontiers);

  setFrontierReachability(m_Frontiers);


  FrontierInterface::FrontierPtSeq outArray;
  log("m_Frontiers contains %i frontiers", m_Frontiers.size());
  for (list<Cure::FrontierPt>::iterator it =  m_Frontiers.begin();
      it != m_Frontiers.end(); it++) {
    FrontierInterface::FrontierPtPtr newPt = new FrontierInterface::FrontierPt;
    newPt->mWidth = it->m_Width;
    switch (it->m_State) {
      case Cure::FrontierPt::FRONTIER_STATUS_OPEN:
        newPt->mState = FrontierInterface::FRONTIERSTATUSOPEN;
        break;
      case Cure::FrontierPt::FRONTIER_STATUS_CURRENT:
        newPt->mState = FrontierInterface::FRONTIERSTATUSCURRENT;
        break;
      case Cure::FrontierPt::FRONTIER_STATUS_UNREACHABLE:
        newPt->mState = FrontierInterface::FRONTIERSTATUSUNREACHABLE;
        break;
      case Cure::FrontierPt::FRONTIER_STATUS_PATHBLOCKED:
        newPt->mState = FrontierInterface::FRONTIERSTATUSPATHBLOCKED;
        break;
      case Cure::FrontierPt::FRONTIER_STATUS_GATEWAYBLOCKED:
        newPt->mState = FrontierInterface::FRONTIERSTATUSGATEWAYBLOCKED;
        break;
      case Cure::FrontierPt::FRONTIER_STATUS_UNKNOWN:
      default:
        newPt->mState = FrontierInterface::FRONTIERSTATUSUNKNOWN;
    }
    newPt->x = it->getX();
    newPt->y = it->getY();
    outArray.push_back(newPt);
  }
//  m_MapsMutex.unlock();
  log("exit getFrontiers");
  return outArray;
}
