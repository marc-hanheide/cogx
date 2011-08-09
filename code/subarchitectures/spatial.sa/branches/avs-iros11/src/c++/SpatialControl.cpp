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

#include <ctime>

using namespace cast;
using namespace std;
using namespace boost;
using namespace spatial;

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

bool SpatialControl::MapServer::isPointReachable(double xW, double yW, const Ice::Current &_context) {
  return m_pOwner->isPointReachable(xW, yW);
}

SpatialData::BoolSeq SpatialControl::MapServer::arePointsReachable(const SpatialData::CoordinateSeq& points, const Ice::Current &_context) {
  return m_pOwner->arePointsReachable(points);
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

  m_NumInhibitors = 0;
  m_SentInhibitStop = false;
}

SpatialControl::~SpatialControl() 
{ }



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

  it = _config.find("--robot-server-host");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_RobotServerHost;
  }

  m_lgm = new Cure::LocalGridMap<unsigned char>(200, 0.05, '2', Cure::LocalGridMap<unsigned char>::MAP1);
  m_lgmKH = new Cure::LocalGridMap<double>(200, 0.05, FLT_MAX, Cure::LocalGridMap<double>::MAP1);

  m_binaryMap = new Cure::LocalGridMap<unsigned char>(200, 0.05, '2', Cure::LocalGridMap<unsigned char>::MAP1);
  m_displayBinaryMap = new Cure::XDisplayLocalGridMap<unsigned char>(*m_binaryMap);

  m_Glrt  = new Cure::GridLineRayTracer<unsigned char>(*m_lgm);

  m_Explorer = new Cure::FrontierExplorer(*this,*m_lgm);
  //m_Explorer->setExplorationConfinedByGateways(true);
  m_Explorer->addEventListener(this);


  if (_config.find("--no-x-window") == _config.end()) {
    m_Displaylgm = new Cure::XDisplayLocalGridMap<unsigned char>(*m_lgm);
    println("Will use X window to show the exploration map");
  } else {
    m_Displaylgm = 0;
    println("Will NOT use X window to show the exploration map");
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

void SpatialControl::blitHeightMap(Cure::LocalGridMap<unsigned char>& lgm, int minX, int maxX, int minY, int maxY)
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
      if ((*m_lgmKH)(xi, yi) > m_obstacleMinHeight && (*m_lgmKH)(xi, yi) < m_obstacleMaxHeight) {
        lgm(xi, yi) = '1';
      }
      else if ((*m_lgmKH)(xi, yi) != FLT_MAX) {
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
void SpatialControl::updateGridMaps()
{
  Cure::Pose3D scanPose;
	Cure::Pose3D LscanPose;
  Cure::Pose3D lpW;
 	int xi,yi;

  /* Update a temporary local map so we don't hog the lock */
  Cure::LocalGridMap<unsigned char> tmp_lgm(*m_lgm);
  Cure::GridLineRayTracer<unsigned char> tmp_glrt(tmp_lgm);

  /* Local reference so we don't have to dereference the pointer all the time */
  Cure::LocalGridMap<double>& lgmKH = *m_lgmKH;

  /* Bounding box for laser scan */
  double laserMinX = INT_MAX, laserMaxX = INT_MIN;
  double laserMinY = INT_MAX, laserMaxY = INT_MIN;

  /* Add all queued laser scans */
  m_ScanQueueMutex.lock();
  while (!m_LScanQueue.empty()){
    if (m_TOPP.isTransformDefined() && m_TOPP.getPoseAtTime(m_LScanQueue.front().getTime(), LscanPose) == 0) {
      lpW.add(LscanPose, m_LaserPoseR);		
      tmp_glrt.addScan(m_LScanQueue.front(), lpW, m_MaxExplorationRange);
      tmp_lgm.setValueInsideCircle(LscanPose.getX(), LscanPose.getY(),
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
  int laserMinXi, laserMinYi, laserMaxXi, laserMaxYi;
  if (tmp_lgm.worldCoords2Index(laserMinX, laserMinY, laserMinXi, laserMinYi) != 0) {
    laserMinXi = -tmp_lgm.getSize() + 1;
    laserMinYi = -tmp_lgm.getSize() + 1;
  }
  if (tmp_lgm.worldCoords2Index(laserMaxX, laserMaxY, laserMaxXi, laserMaxYi) != 0) {
    laserMaxXi = tmp_lgm.getSize() - 1;
    laserMaxYi = tmp_lgm.getSize() - 1;
  }

  /* Bounding box for new point cloud data */
  int pointcloudMinXi = INT_MAX, pointcloudMaxXi = INT_MIN;
  int pointcloudMinYi = INT_MAX, pointcloudMaxYi = INT_MIN;

  /* Update height map */
  cdl::CASTTime frameTime = getCASTTime();
  bool hasScanPose = m_TOPP.getPoseAtTime(Cure::Timestamp(frameTime.s, frameTime.us), scanPose) == 0;
  if (hasScanPose) {
    PointCloud::SurfacePointSeq points;
    getPoints(true, 0 /* unused */, points);
    std::sort(points.begin(), points.end(), ComparePoints());
    for (PointCloud::SurfacePointSeq::iterator it = points.begin(); it != points.end(); ++it) {
      /* Ignore points not in the current view cone */
      if (!isPointInViewCone(it->p))
        continue;
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
            continue;
        }
        /* If the above tests passed update the height map */ 
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

    /* Project the height map onto the 2D obstacle map */ 
    if (pointcloudMinXi != INT_MAX && pointcloudMinYi != INT_MAX && pointcloudMaxXi != INT_MIN && pointcloudMaxYi != INT_MIN) {
      blitHeightMap(tmp_lgm, pointcloudMinXi, pointcloudMaxXi, pointcloudMinYi, pointcloudMaxYi);
      tmp_lgm.setValueInsideCircle(scanPose.getX(), scanPose.getY(),
          0.55*Cure::NavController::getRobotWidth(), '0'); 
    }
  }

  /* Project the height map onto the 2D obstacle map */ 
  blitHeightMap(tmp_lgm, laserMinXi, laserMaxXi, laserMinYi, laserMaxYi);

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
		if (m_Displaylgm) {
			Cure::Pose3D currentPose = m_TOPP.getPose();
			m_Displaylgm->updateDisplay(&currentPose,
						                      &m_NavGraph, 
						                      &m_Explorer->m_Fronts);
      m_displayBinaryMap->updateDisplay(&currentPose);
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
    if(gateway) // if a new gateway is detected, let Cure::FrontierExplorer know so that it can back off (if confinedbygateways is set).
    { 
      m_Explorer->newGateway( ( *m_NavGraph.m_Gateways.back() ) );
      gateway = false;
    }
    
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
      
    }else if(m_taskStatus == NewTask){

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
				Cure::NavGraphNode lastNode = m_Path.back();
				m_Path.clear();
				m_Path.push_back(lastNode);
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

      else if ((m_commandType == NavData::lEXPLORE)) {
        
        log("executing command EXPLORE");
         m_TolRot = Cure::HelpFunctions::deg2rad(45);
        m_Explorer->setExplorationConfinedByGateways(ExplorationConfinedByGateways);
        ret = m_Explorer->startNextExplorationStep(currentTaskId);
        m_currentTaskIsExploration = true;

        // The EXPLORE command which calls Frontier Explorer from CURE
        
      }
      else if ((m_commandType == NavData::lEXPLOREFLOOR)) {
      	log("executing command EXPLOREFLOOR"); 
  	m_TolRot = Cure::HelpFunctions::deg2rad(45);
      	m_Explorer->setExplorationConfinedByGateways(ExplorationConfinedByGateways);
        ret = m_Explorer->startNextExplorationStep(currentTaskId);
        m_currentTaskIsExploration = true;
       
      	
	}
      
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
        m_MapsMutex.lock();
        m_LMap.addScan(cureScan, m_LaserPoseR, scanPose);

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

std::vector<bool> SpatialControl::arePointsReachable(std::vector<std::vector<double> > points) {
  std::vector<bool> ret;
  Cure::BinaryMatrix map;
  
  debug("lock arePointsReachable");
  m_MapsMutex.lock();

  /* The Cure documentation recommends keeping columns as a multiple
     of 32 to be able to exploit the performance improvements of doing
     calculations on 32 bit longs. However, we use both 32 and 64 bit
     operating systems now and longs on unix systems are usually different
     sizes for these architectures. If anyone knows the correct way of
     handling this, please modify the code below. */
  map.reallocate(2*m_lgm->getSize(), 2*m_lgm->getSize());
  map = 0; // Set all cells to zero

  // Transfer obstacles for the local gridmap
  for(int x = -m_lgm->getSize(); x < m_lgm->getSize(); ++x) {
    for(int y = -m_lgm->getSize(); y < m_lgm->getSize(); ++y) {
      if((*m_lgm)(x,y) == '1') {
        map.setBit(x + m_lgm->getSize(), y + m_lgm->getSize(), true);
      }
    }
  }

  // Grow each occupied cell to account for the size of the robot.
  Cure::BinaryMatrix grown_map;
  map.growInto(grown_map, 0.5*Cure::NavController::getRobotWidth() / m_lgm->getCellSize());

  /* Set unknown space as obstacles, since we don't want to find paths
  going through space we don't know anything about */
  for(int x = -m_lgm->getSize(); x < m_lgm->getSize(); ++x) {
    for(int y = -m_lgm->getSize(); y < m_lgm->getSize(); ++y) {
      if((*m_lgm)(x,y) == '2') {
        grown_map.setBit(x + m_lgm->getSize(), y + m_lgm->getSize(), true);
      }
    }
  }

  m_MapsMutex.unlock();
  debug("unlock arePointsReachable");

  for(int x = -m_binaryMap->getSize(); x < m_binaryMap->getSize(); ++x) {
    for(int y = -m_binaryMap->getSize(); y < m_binaryMap->getSize(); ++y) {
      (*m_binaryMap)(x,y) = grown_map(x+m_binaryMap->getSize(),y+m_binaryMap->getSize()) ? '1' : '0';
    }
  }
  
  int rX, rY; // Robot position index

  Cure::Pose3D robotPose = m_TOPP.getPose();
  if(m_lgm->worldCoords2Index(robotPose.getX(), robotPose.getY(), rX, rY) != 0) {
    log("Robot position is outside of gridmap! Returning.");
    printf("returned prematurely\n");
    return ret;
  }

  // Offset the indices so that top left is (0,0).
  rX += m_lgm->getSize();
  rY += m_lgm->getSize();

  Cure::ShortMatrix path;
  /* d will be -1 if there is no path from the robot to the placeholder.
     The 20*size*size value is taken from AdvancedObjectSearch.cpp. Not sure
     if it's a good length. */
  for(std::vector<std::vector<double> >::iterator pointsIt = points.begin(); pointsIt != points.end(); pointsIt++) { 
    int pX, pY;  // Point position index
    if(m_lgm->worldCoords2Index((*pointsIt)[0], (*pointsIt)[1], pX, pY) != 0) {
      log("Point is outside of map.");
      ret.push_back(false);
    } else {
      pX += m_lgm->getSize();
      pY += m_lgm->getSize();
      double d = (grown_map.path(rX, rY, pX, pY, path, 20* m_lgm->getSize()) *
          m_lgm->getCellSize());
      ret.push_back(d >= 0);
    }
  }

  return ret;
}

bool SpatialControl::isPointReachable(double xW, double yW) {
  std::vector<std::vector<double> > vec;
  std::vector<double> coord;
  coord.push_back(xW);
  coord.push_back(yW);
  vec.push_back(coord);
  
  return arePointsReachable(vec).front();
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

  m_Explorer->updateFrontiers();

  m_MapsMutex.unlock();

  FrontierInterface::FrontierPtSeq outArray;
  log("m_Fronts contains %i frontiers", m_Explorer->m_Fronts.size());
  for (list<Cure::FrontierPt>::iterator it =  m_Explorer->m_Fronts.begin();
      it != m_Explorer->m_Fronts.end(); it++) {
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
  return outArray;
}
