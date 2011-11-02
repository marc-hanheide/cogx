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

#include <utility>
#include <queue>

using namespace cast;
using namespace std;
using namespace boost;
using namespace spatial;

#define CAM_ID_DEFAULT 0

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

SpatialControl::SpatialControl()
  :NavController(m_NavGraph, m_LMap),
   NavControllerEventListener("SpatialControl"),
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

  m_RobotServerHost = "localhost";

  m_waitingForPTZCommandID = "";

  m_NumInhibitors = 0;
  m_SentInhibitStop = false;
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
}

void SpatialControl::newPanTiltCommand(const cdl::WorkingMemoryChange &objID) {
  m_ptzInNavigationPose = false; // Wait until the command is done before checking the pose
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

    // Check if the tilt is now in the 'navigation pose' (-45 degrees)
    if(overwritten->pose.tilt < -M_PI/4 + 0.05 && overwritten->pose.tilt > -M_PI/4 -0.05)
      m_ptzInNavigationPose = true;
    else 
      m_ptzInNavigationPose = false;

    log("PTZ in navigation pose: %d", m_ptzInNavigationPose);

    m_lastPtzNavPoseCompletion = getCASTTime();
  }
  catch (DoesNotExistOnWMException e)
  {
    log ("Error: SetPTZPoseCommand went missing! "); 
  }

  if (objID.address.id == m_waitingForPTZCommandID)
      m_waitingForPTZCommandID = "";
}

SpatialControl::~SpatialControl() 
{ }

//REMOVEME
void SpatialControl::SaveGridMap(){
  log("Saving node gridmaps");
  ofstream fout("NodeGridMaps.txt");
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

void SpatialControl::configure(const map<string,string>& _config) 
{
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

  m_bNoNavGraph = false;
  if(_config.find("--no-graph") != _config.end()){
    m_bNoNavGraph = true;
  }

  if (Cure::NavController::config(configfile)) {
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

  m_MaxCatExplorationRange = 5.6;
  it = _config.find("--cat-explore-range");
  if (it != _config.end()) {
    m_MaxCatExplorationRange = (atof(it->second.c_str()));
  }

  it = _config.find("--robot-server-host");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_RobotServerHost;
  }

  m_lgm = new Cure::LocalGridMap<unsigned char>(300, 0.05, '2', Cure::LocalGridMap<unsigned char>::MAP1);
  m_lgmKH = new Cure::LocalGridMap<double>(300, 0.05, FLT_MAX, Cure::LocalGridMap<double>::MAP1);

  m_categoricalMap = new Cure::LocalGridMap<unsigned char>(300, 0.05, '2', Cure::LocalGridMap<unsigned char>::MAP1);
  m_categoricalKHMap = new Cure::LocalGridMap<double>(300, 0.05, FLT_MAX, Cure::LocalGridMap<double>::MAP1);
  m_binaryMap = new Cure::LocalGridMap<unsigned char>(300, 0.05, '2', Cure::LocalGridMap<unsigned char>::MAP1);

  m_DisplayCureObstacleMap = false;
  if (_config.find("--display-cure-obstacle-map") != _config.end()) {
    m_DisplayCureObstacleMap = true;
    m_obstacleMap = new Cure::LocalGridMap<unsigned char>(300, 0.05, 1, Cure::LocalGridMap<unsigned char>::MAP1);
    m_displayObstacleMap = new Cure::XDisplayLocalGridMap<unsigned char>(*m_obstacleMap);
  }

  m_Glrt  = new Cure::GridLineRayTracer<unsigned char>(*m_lgm);

  m_FrontierFinder = new Cure::FrontierFinder<unsigned char>(*m_lgm);

  if (_config.find("--no-x-window") == _config.end()) {
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

  if(_config.find("--show-categorical-map") != _config.end()) {
    m_displayCategoricalMap = new Cure::XDisplayLocalGridMap<unsigned char>(*m_categoricalMap);
  } else {
    m_displayCategoricalMap = 0;
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

  Cure::NavController::addEventListener(this);
  Cure::NavController::setTurnAngleIntoSpeed(true, 0.5);
  Cure::NavController::setMinNonzeroSpeeds(0.04, 
                                           Cure::HelpFunctions::deg2rad(10));
  Cure::NavController::setApproachTolerances(0.5, 
                                             Cure::HelpFunctions::deg2rad(10));
  Cure::NavController::setUsePathTrimming(false);
  Cure::NavController::setMaxPathTrimDist(3);
  Cure::NavController::setProgressTimeout(20);
  Cure::NavController::setGotoMaxSpeeds(maxGotoV, maxGotoW);
  Cure::NavController::setGatewayMaxSpeeds(0.3, 0.3);
  
  Cure::NavController::setFollowDistances(0.8, 0.4);
  Cure::NavController::setFollowTolerances(0.1, 
                                           Cure::HelpFunctions::deg2rad(10));

  Cure::NavController::setPoseProvider(m_TOPP);

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

  m_RobotServer = RobotbaseClientUtils::getServerPrx(*this,
                                                     m_RobotServerHost);

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

	
  if(m_sendPTZCommands) {	
  	m_ptzInterface = getIceServer<ptz::PTZInterface>("ptz.server");
  	ptz::PTZReading reading = m_ptzInterface->getPose();
  

  if(reading.pose.tilt < -M_PI/4 + 0.05 && reading.pose.tilt > -M_PI/4 -0.05)
    m_ptzInNavigationPose = true;
  else
    m_ptzInNavigationPose = false;

  log("PTZ in navigation pose: %d", m_ptzInNavigationPose);

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
  m_taskStatusMutex.lock();
  m_TolRot = Cure::HelpFunctions::deg2rad(5);
  if(taskId == m_taskId - 1){ // else, it is an old command
  log("this is an old command...");
    if(!m_currentTaskIsExploration){ // @see explorationDone
      m_CurrentCmdFinalCompletion = NavData::SUCCEEDED;
      m_CurrentCmdFinalStatus = NavData::NONE;
      m_taskStatus = TaskFinished;
    }
  }
  m_taskStatusMutex.unlock();
}


void SpatialControl::failTask(int taskId, int error) {
  log("failTask");
  m_TolRot = Cure::HelpFunctions::deg2rad(5);
  m_taskStatusMutex.lock();
  if(taskId == m_taskId - 1){ // else, it is an old command
    m_CurrentCmdFinalCompletion = NavData::FAILED;
    m_CurrentCmdFinalStatus = NavData::UNKNOWN;
    m_taskStatus = TaskFinished;
  }
  m_taskStatusMutex.unlock();
}

// FrontierExplorer exploration done overload function
void SpatialControl::explorationDone(int taskId, int status)
{

  log("explorationDone");
  m_taskStatusMutex.lock();
  m_TolRot = Cure::HelpFunctions::deg2rad(5);
  if(taskId == m_taskId - 1){ // else, it is an old command
    m_CurrentCmdFinalCompletion = NavData::SUCCEEDED;
    m_CurrentCmdFinalStatus = NavData::NONE;
    m_taskStatus = TaskFinished;
  }
  m_taskStatusMutex.unlock();
}

const Cure::LocalGridMap<unsigned char>& SpatialControl::getLocalGridMap()
{
  return *m_lgm;
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
  for (xi = minX; xi <= maxX; xi++) {
    for (yi = minY; yi <= maxY; yi++) {
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
    }
  }

  /* Filtering */
  for (xi = minX+1; xi < maxX; xi++) {
    for (yi = minY+1; yi < maxY; yi++) {
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

void SpatialControl::updateGridMaps()
{
  double maxCatHeight = 0.3;
  Cure::Pose3D scanPose;
	Cure::Pose3D LscanPose;
  Cure::Pose3D lpW;
 	int xi,yi;

  /* Update a temporary local map so we don't hog the lock */
  Cure::LocalGridMap<unsigned char> tmp_lgm(*m_lgm);
  Cure::GridLineRayTracer<unsigned char> tmp_glrt(tmp_lgm);

  Cure::LocalGridMap<unsigned char> tmp_catlgm(*m_categoricalMap);
  Cure::GridLineRayTracer<unsigned char> tmp_catglrt(tmp_catlgm);
  

  /* Local reference so we don't have to dereference the pointer all the time */
  Cure::LocalGridMap<double>& lgmKH = *m_lgmKH;
  Cure::LocalGridMap<double>& lgmcatKH = *m_categoricalKHMap;

  /* Bounding box for laser scan */
  double laserMinX = INT_MAX, laserMaxX = INT_MIN;
  double laserMinY = INT_MAX, laserMaxY = INT_MIN;

  double laserCatMinX = INT_MAX, laserCatMaxX = INT_MIN;
  double laserCatMinY = INT_MAX, laserCatMaxY = INT_MIN;

  /* Add all queued laser scans */
  m_ScanQueueMutex.lock();
  while (!m_LScanQueue.empty()){
    if (m_TOPP.isTransformDefined() && m_TOPP.getPoseAtTime(m_LScanQueue.front().getTime(), LscanPose) == 0) {
      lpW.add(LscanPose, m_LaserPoseR);		
      tmp_glrt.addScan(m_LScanQueue.front(), lpW, m_MaxExplorationRange);
      tmp_lgm.setValueInsideCircle(LscanPose.getX(), LscanPose.getY(),
          0.55*Cure::NavController::getRobotWidth(), '0');                                  

      tmp_catglrt.addScan(m_LScanQueue.front(), lpW, m_MaxCatExplorationRange);
      tmp_catlgm.setValueInsideCircle(LscanPose.getX(), LscanPose.getY(),
          0.55*Cure::NavController::getRobotWidth(), '0');                                  

      m_firstScanAdded = true;

      /* Update bounding box */
      if (LscanPose.getX() < laserMinX)
        laserMinX = LscanPose.getX();
      if (LscanPose.getX() > laserMaxX)
        laserMaxX = LscanPose.getX();
      if (LscanPose.getY() < laserMinY)
        laserMinY = LscanPose.getY();
      if (LscanPose.getY() > laserMaxY)
        laserMaxY = LscanPose.getY();

      if (LscanPose.getX() < laserCatMinX)
        laserCatMinX = LscanPose.getX();
      if (LscanPose.getX() > laserCatMaxX)
        laserCatMaxX = LscanPose.getX();
      if (LscanPose.getY() < laserCatMinY)
        laserCatMinY = LscanPose.getY();
      if (LscanPose.getY() > laserCatMaxY)
        laserCatMaxY = LscanPose.getY();
    }
    m_LScanQueue.pop();
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

  laserCatMinX -= m_MaxCatExplorationRange;
  laserCatMinY -= m_MaxCatExplorationRange;
  laserCatMaxX += m_MaxCatExplorationRange;
  laserCatMaxY += m_MaxCatExplorationRange;

  int laserMinXi, laserMinYi, laserMaxXi, laserMaxYi;
  if (tmp_lgm.worldCoords2Index(laserMinX, laserMinY, laserMinXi, laserMinYi) != 0) {
    laserMinXi = -tmp_lgm.getSize() + 1;
    laserMinYi = -tmp_lgm.getSize() + 1;
  }
  if (tmp_lgm.worldCoords2Index(laserMaxX, laserMaxY, laserMaxXi, laserMaxYi) != 0) {
    laserMaxXi = tmp_lgm.getSize() - 1;
    laserMaxYi = tmp_lgm.getSize() - 1;
  }

  int laserCatMinXi, laserCatMinYi, laserCatMaxXi, laserCatMaxYi;
  if(tmp_catlgm.worldCoords2Index(laserCatMinX, laserCatMinY, laserCatMinXi, laserCatMinYi) != 0) {
    laserCatMinXi = -tmp_catlgm.getSize() + 1;
    laserCatMinYi = -tmp_catlgm.getSize() + 1;
  }
  if(tmp_catlgm.worldCoords2Index(laserCatMaxX, laserCatMaxY, laserCatMaxXi, laserCatMaxYi) != 0) {
    laserCatMaxXi = -tmp_catlgm.getSize() - 1;
    laserCatMaxYi = -tmp_catlgm.getSize() - 1;
  }

  /* Bounding box for new point cloud data */
  int pointcloudMinXi = INT_MAX, pointcloudMaxXi = INT_MIN;
  int pointcloudMinYi = INT_MAX, pointcloudMaxYi = INT_MIN;

  /* Update height map */
  cdl::CASTTime frameTime = getCASTTime();
  double time = frameTime.s+frameTime.us/1000000.0;
  double lastptztime = m_lastPtzNavPoseCompletion.s+m_lastPtzNavPoseCompletion.us/1000000.0;
  bool hasScanPose = m_TOPP.getPoseAtTime(Cure::Timestamp(frameTime.s, frameTime.us), scanPose) == 0;
  /* WARNING (FIXME):
   * For some reason we seem to get old kinect data when we have _just_ moved
   * the pantilt to -45 degrees some times, causing garbage to appear on the
   * map. For now, we wait for 1 second before doing anything with the kinect
   * data to make sure we get fresh data. */
  if (hasScanPose && m_ptzInNavigationPose && time-lastptztime > 1) {
    PointCloud::SurfacePointSeq points;
    getPoints(true, 640/4, points);
    std::sort(points.begin(), points.end(), ComparePoints());
    for (PointCloud::SurfacePointSeq::iterator it = points.begin(); it != points.end(); ++it) {
      /* Ignore points not in the current view cone */
      if (!isPointVisible(it->p))
	continue;
//      if (!isPointInViewCone(it->p))
//        continue;
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

        bool oldcatObstacle = (lgmcatKH(xi, yi) > m_obstacleMinHeight  && lgmcatKH(xi,yi) < maxCatHeight);
        bool newcatObstacle = (pZ > m_obstacleMinHeight && pZ < maxCatHeight);

        bool updateHeightMap = true;
        if (oldObstacle && !newObstacle) {
          /* Undo robot pose transform since it is not known by the point cloud */
          Cure::Vector3D old(pX, pY, lgmKH(xi, yi));
          scanPose.transform(old, to);
          cogx::Math::Vector3 oldPoint;
          oldPoint.x = to.X[0];
          oldPoint.y = to.X[1];
          oldPoint.z = to.X[2];
          /* If the old obstable is not currently in view don't remove it */
          if (!isPointInViewCone(oldPoint))
            updateHeightMap = false;
        }
        /* If the above tests passed update the height map */ 
        if(updateHeightMap)
          lgmKH(xi, yi) = pZ;

        // Again for categorical gridmap
        updateHeightMap = true;
        if (oldcatObstacle && !newcatObstacle) {
          /* Undo robot pose transform since it is not known by the point cloud */
          Cure::Vector3D old(pX, pY, lgmcatKH(xi, yi));
          scanPose.transform(old, to);
          cogx::Math::Vector3 oldPoint;
          oldPoint.x = to.X[0];
          oldPoint.y = to.X[1];
          oldPoint.z = to.X[2];
          /* If the old obstable is not currently in view don't remove it */
          if (!isPointInViewCone(oldPoint))
            updateHeightMap = false;
        }
        /* If the point is above the limit we care about, ignore it */
        if (pZ > maxCatHeight)
          updateHeightMap = false;

        /* If the above tests passed update the height map */ 
        if(updateHeightMap)
          lgmcatKH(xi, yi) = pZ;

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

    /* Project the height map onto the 2D obstacle map */ 
    if (pointcloudMinXi != INT_MAX && pointcloudMinYi != INT_MAX && pointcloudMaxXi != INT_MIN && pointcloudMaxYi != INT_MIN) {
      blitHeightMap(tmp_lgm, m_lgmKH, pointcloudMinXi, pointcloudMaxXi, pointcloudMinYi, pointcloudMaxYi, m_obstacleMinHeight, m_obstacleMaxHeight);
      tmp_lgm.setValueInsideCircle(scanPose.getX(), scanPose.getY(),
          0.55*Cure::NavController::getRobotWidth(), '0'); 

      blitHeightMap(tmp_catlgm, m_categoricalKHMap, pointcloudMinXi, pointcloudMaxXi, pointcloudMinYi, pointcloudMaxYi, m_obstacleMinHeight, maxCatHeight);
      tmp_catlgm.setValueInsideCircle(scanPose.getX(), scanPose.getY(),
          0.55*Cure::NavController::getRobotWidth(), '0'); 
    }
  }

  /* Project the height map onto the 2D obstacle map */ 
  blitHeightMap(tmp_lgm, m_lgmKH, laserMinXi, laserMaxXi, laserMinYi, laserMaxYi, m_obstacleMinHeight, m_obstacleMaxHeight);

  blitHeightMap(tmp_catlgm, m_categoricalKHMap, laserCatMinXi, laserCatMaxXi, laserCatMinYi, laserCatMaxYi, m_obstacleMinHeight, maxCatHeight);

  const int deltaN = 3;
  double d = tmp_lgm.getCellSize()/deltaN;
  int maxcellstocheck = int (5.0/d);
  double xWT,yWT;
  double theta;

  /* Update the nav map */
  m_Mutex.lock();
  m_LMap.clearMap();
  Cure::Pose3D currPose = m_TOPP.getPose();		
  tmp_lgm.setValueInsideCircle(currPose.getX(), currPose.getY(),
      0.55*Cure::NavController::getRobotWidth(), '0');                                  
  for (int i = 0; i < m_Npts; i++) {
    theta = m_StartAngle + m_AngleStep * i;
    for (int j = 1; j < deltaN*maxcellstocheck; j++){
      xWT = currPose.getX()+j*d*cos(theta);
      yWT = currPose.getY()+j*d*sin(theta);
      if(tmp_lgm.worldCoords2Index(xWT,yWT,xi,yi)==0){
        if(tmp_lgm(xi,yi) == '1'){
          m_LMap.addObstacle(xWT, yWT, 1);
          break;    
        }
      }
    }
  }
  m_Mutex.unlock();

  /* Copy the temporary map */
  m_MapsMutex.lock();
  *m_lgm = tmp_lgm;
  *m_categoricalMap = tmp_catlgm;
  m_MapsMutex.unlock();
}

void SpatialControl::runComponent() 
{
  setupPushScan2d(*this, 0.1);
  setupPushOdometry(*this);

  while(isRunning()){
    if (m_UsePointCloud) {
      updateGridMaps();
    }	

		m_MapsMutex.lock();
    Cure::Pose3D currentPose = m_TOPP.getPose();
		if (m_Displaylgm) {
			m_Displaylgm->updateDisplay(&currentPose,
						                      &m_NavGraph, 
						                      &m_Frontiers);
		}

    if(m_displayBinaryMap)
      m_displayBinaryMap->updateDisplay(&currentPose);

    if(m_displayCategoricalMap)
      m_displayCategoricalMap->updateDisplay(&currentPose);

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
		m_MapsMutex.unlock();	
	  
    usleep(250000);
  }
} 


void SpatialControl::newNavGraph(const cdl::WorkingMemoryChange &objID){
  m_Mutex.lock();
	
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

  m_Mutex.unlock();
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
  shared_ptr<CASTData<NavData::RobotPose2d> > oobj =
    getWorkingMemoryEntry<NavData::RobotPose2d>(objID.address);

  //FIXME
  m_SlamRobotPose.setTime(Cure::Timestamp(oobj->getData()->time.s,
                                          oobj->getData()->time.us));
  m_SlamRobotPose.setX(oobj->getData()->x);
  m_SlamRobotPose.setY(oobj->getData()->y);
  m_SlamRobotPose.setTheta(oobj->getData()->theta);
		//log("time of newRobtoPose() = %d.%ld s",(long int)oobj->getData()->time.s, (long int)oobj->getData()->time.us);
  
  Cure::Pose3D cp = m_SlamRobotPose;
  m_TOPP.defineTransform(cp);
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
    m_Mutex.lock();

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
        m_Mutex.unlock();
        return;
      }

    }
    m_Mutex.unlock();
    
    log("read new ctrlCommand");
    m_taskStatus = NewTask;
  }
}


void SpatialControl::receiveOdometry(const Robotbase::Odometry &castOdom)
{
  debug("lock receiveOdometry");
  lockComponent(); //Don't allow any interface calls while processing a callback
  debug("lock acquired");
  Cure::Pose3D cureOdom;
  CureHWUtils::convOdomToCure(castOdom, cureOdom);

  debug("Got odometry x=%.2f y=%.2f a=%.4f t=%.6f",
        cureOdom.getX(), cureOdom.getY(), cureOdom.getTheta(),
        cureOdom.getTime().getDouble());
  
  m_TOPP.addOdometry(cureOdom);
  
  m_CurrPose = m_TOPP.getPose();
	
  if (m_ready || m_bNoNavGraph) { // have to get a first nav graph 
                 // to be ready
    
    m_taskStatusMutex.lock(); // acquire mutex!
    
    if(m_taskStatus == TaskFinished){
      // report final status
      changeCurrentCommandCompletion(m_CurrentCmdFinalCompletion,
                                     m_CurrentCmdFinalStatus);
      m_taskStatus = NothingToDo;
      
    }else if(m_taskStatus == NewTask 
	&& (m_commandType == NavData::lSTOPROBOT || m_waitingForPTZCommandID == ""))
    {

      m_Mutex.lock();
      
      // Result of m_NavCtrl.operationX
      int ret = -1; // -1:operation not done, 0:ok, >0:error
      m_currentTaskIsExploration = false; // set later
      
      // Task id
      int currentTaskId = m_taskId++;
      
      // GOTO_XYA        
      if ((m_commandType == NavData::lGOTOXYA)) {
        
        log("executing command GOTOXYA");  	
        
        Cure::NavController::setPositionToleranceFinal(m_TolPos);
        Cure::NavController::setOrientationTolerance(m_TolRot);
        ret = Cure::NavController::gotoXYA
          (currentTaskId, m_commandX, m_commandY, m_commandTheta);
      }
      
      // GOTO_XY_ROUGH
      
      else if ((m_commandType == NavData::lGOTOXYROUGH)){ 
        
        log("executing command GOTOXYROUGH"); 
        
        Cure::NavController::setPositionToleranceFinal(m_TolPos);
        Cure::NavController::setOrientationTolerance(m_TolRot);
        ret = Cure::NavController::gotoXY(currentTaskId, m_commandX, m_commandY);
      }
      
      
      // GOTO_XY
      
      else if ((m_commandType == NavData::lGOTOXY)){ 
        
        log("executing command GOTOXY"); 
        
        Cure::NavController::setPositionToleranceFinal(m_TolPos);
        Cure::NavController::setOrientationTolerance(m_TolRot);
        ret = Cure::NavController::gotoXY(currentTaskId, m_commandX, m_commandY);
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
        
        Cure::NavController::setPositionToleranceFinal(m_TolPos);
        Cure::NavController::setOrientationTolerance(m_TolRot);
        ret = Cure::NavController::gotoPolar(currentTaskId,m_commandTheta,m_commandR);
      }
      
      // GOTO_AREA
      
      else if ((m_commandType == NavData::lGOTOAREA)) {
        
        log("executing command GOTOAREA"); 
        
        Cure::NavController::setPositionToleranceFinal(m_TolPos);
        Cure::NavController::setOrientationTolerance(m_TolRot);
        ret = Cure::NavController::gotoArea(currentTaskId, m_commandAreaId);
      }
      
      
      // GOTO_NODE
      
      else if ((m_commandType == NavData::lGOTONODE)) {
        
        log("executing command GOTONODE"); 
        
        Cure::NavController::setPositionToleranceFinal(m_TolPos);
        Cure::NavController::setOrientationTolerance(m_TolRot);
        ret = Cure::NavController::gotoNode(currentTaskId, m_commandNodeId);
      }
      
      // ROTATE_REL
      
      else if ((m_commandType == NavData::lROTATEREL)) {  
        
        log("executing command ROTATEREL"); 
        
        Cure::NavController::setPositionToleranceFinal(m_TolPos);
        Cure::NavController::setOrientationTolerance(m_TolRot);
        ret = Cure::NavController::rotateRel(currentTaskId, m_commandTheta);
        
      }
      
      
      // ROTATE_ABS
      
      else if ((m_commandType == NavData::lROTATEABS)) {
        
        log("executing command ROTATEABS"); 
        
        Cure::NavController::setPositionToleranceFinal(m_TolPos);
        Cure::NavController::setOrientationTolerance(m_TolRot);
        ret = Cure::NavController::rotateAbs(currentTaskId, m_commandTheta);
        
      }
      
      // BACK_OFF
      
      else if ((m_commandType == NavData::lBACKOFF)) {
        
        log("executing command BACKOFF"); 
        
        Cure::NavController::setPositionToleranceFinal(m_TolPos);
        Cure::NavController::setOrientationTolerance(m_TolRot);
        ret = Cure::NavController::backOff(currentTaskId, m_commandDistance);
        
      }
      
      // STOP
      
      else if ((m_commandType == NavData::lSTOPROBOT)) {
        
        log("executing command STOPROBOT"); 
        
        // This command is special:
        // It is always accomplished at the moment, by this thread,
        // and does not raise a doneTask calling (but can raise
        // abortTask).
        // 
        ret = Cure::NavController::stop();
        
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
          if (m_CurrPerson < 0 || m_CurrPerson > (int)m_People.size()-1) {
            
            m_CurrPerson = -1;
            m_taskStatus = NothingToDo;
            changeCurrentCommandCompletion(NavData::ABORTED,
                                           NavData::PERSONNOTFOUND);

            log("Lost the person we were tracking");
            


          } else {

            ret = Cure::NavController::followPerson(currentTaskId,
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
      m_Mutex.unlock();
    }
    
    m_taskStatusMutex.unlock(); // release mutex!
    
    m_Mutex.lock();
    m_LMap.moveRobot(m_CurrPose);
    Cure::NavController::updateCtrl();
    m_Mutex.unlock();
    
  } // if (m_ready)    
  unlockComponent();
  debug("unlock receiveOdometry");
}

void SpatialControl::receiveScan2d(const Laser::Scan2d &castScan)
{
	
  debug("lock receiveScan2d");
  lockComponent(); //Don't allow any interface calls while processing a callback
  debug("lock acquired");
  debug("Got scan with n=%d and t=%ld.%06ld",
        castScan.ranges.size(), 
        (long)castScan.time.s, (long)castScan.time.us);
	
  Cure::LaserScan2d cureScan;
  CureHWUtils::convScan2dToCure(castScan, cureScan);

  if (!m_UsePointCloud) {
    if (m_TOPP.isTransformDefined()) {

      Cure::Pose3D scanPose;
      if (m_TOPP.getPoseAtTime(cureScan.getTime(), scanPose) == 0) {
        m_Mutex.lock();
        m_LMap.addScan(cureScan, m_LaserPoseR, scanPose);
        m_Mutex.unlock();

        m_MapsMutex.lock();
        Cure::Pose3D lpW;
        m_lgm->setValueInsideCircle(scanPose.getX(), scanPose.getY(),
            0.5*Cure::NavController::getRobotWidth(), 
            '0');
        lpW.add(scanPose, m_LaserPoseR);
        m_Glrt->addScan(cureScan, lpW, m_MaxExplorationRange);      
        m_firstScanAdded = true;
        m_MapsMutex.unlock();
      }
    }
  }
  else {
    m_ScanQueueMutex.lock();
    m_LScanQueue.push(cureScan);
    m_ScanQueueMutex.unlock();
  }

  /* Person following stuff */    
  NavData::PersonFollowedPtr p = new NavData::PersonFollowed;
  static long last_id_sent = -1;
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
  unlockComponent();
  debug("unlock receiveScan2d");
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
  
  m_RobotServer->execMotionCommand(cmd);
}

/* Fills map with an expanded version of gridmap, where unknown space is
   also set as obstacles */
void SpatialControl::getExpandedBinaryMap(Cure::LocalGridMap<unsigned char>* gridmap, Cure::BinaryMatrix &map, bool lockMapsMutex = true) const {
  
  if(lockMapsMutex)
    m_MapsMutex.lock();

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
  ungrown_map.growInto(map, 0.5*Cure::NavController::getRobotWidth() / m_lgm->getCellSize());

  /* Set unknown space as obstacles, since we don't want to find paths
  going through space we don't know anything about */
  for(int x = -gridmapSize; x < gridmapSize; ++x) {
    for(int y = -gridmapSize; y < gridmapSize; ++y) {
      if((*gridmap)(x,y) == '2') {
        map.setBit(x + gridmapSize, y + gridmapSize, true);
      }
    }
  }

  if(lockMapsMutex)
    m_MapsMutex.unlock();
}

void SpatialControl::setFrontierReachability(std::list<Cure::FrontierPt> &frontiers) {
  Cure::BinaryMatrix map;
  getExpandedBinaryMap(m_lgm, map, false);

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

vector<double> SpatialControl::getGridMapRaytrace(double startAngle, double angleStep, unsigned int beamCount) {
  vector<double> raytrace;
  raytrace.resize(beamCount);

  // Get current pose
  double xytheta[3];
  m_TOPP.getPose().getXYTheta(xytheta);
  double theta = xytheta[2];

  // Raytrace in the gridmap
  double cellSize = m_categoricalMap->getCellSize();
  double angle = startAngle + theta;
  for(size_t i=0; i<beamCount; ++i)
  {
    raytrace[i] = m_MaxCatExplorationRange;
    for(double r=0; r<=m_MaxCatExplorationRange; r+=(cellSize/2.0))
    {
      double x = r * cos(angle) + xytheta[0];
      double y = r * sin(angle) + xytheta[1];

      int xCell = static_cast<int>(round(x/cellSize));
      int yCell = static_cast<int>(round(y/cellSize));

      if(xCell < -300 || xCell > 300 || yCell < -300 || yCell > 300)
        printf("From raytrace. x: %f, y: %f\n", x,y);

      if((*m_categoricalMap)(xCell,yCell) != '0')
      {
        raytrace[i] = r;
        break;
      }
    }
    angle+=angleStep;
  }

  return raytrace;
}

void SpatialControl::getBoundedMap(SpatialData::LocalGridMap &map, Cure::LocalGridMap<unsigned char>* gridmap, double minx, double maxx, double miny, double maxy) {
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

  m_MapsMutex.lock();

  while (!m_firstScanAdded) {
    log("  Waiting for first scan to be added...");
    m_MapsMutex.unlock();
    usleep(1000000);
    m_MapsMutex.lock();
  }

  m_Frontiers.clear();
  m_FrontierFinder->findFrontiers(0.8,2.0,m_Frontiers);
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
  m_MapsMutex.unlock();
  log("exit getFrontiers");
  return outArray;
}
