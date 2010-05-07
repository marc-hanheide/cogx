//
// = FILENAME
//    NavController.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//    Chandana Paul
//    Dorian Galvez Lopez
//
// = COPYRIGHT
//    Copyright (c) 2007 Chandana Paul
//                  2007 Dorian Galvez Lopez
//                  2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "NavControl.hpp"
#include <CureHWUtils.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

#include <AddressBank/ConfigFileReader.hh>
#include <RobotbaseClientUtils.hpp>

using namespace cast;
using namespace std;
using namespace boost;
using namespace navsa;

 
/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new NavControl();
  }
}

NavControl::NavControl()
  :NavController(m_NavGraph, m_LMap),
   NavControllerEventListener("NavControl"),
   FrontierExplorerEventListener("NavControl")
{
  m_CurrPerson = -1;
  m_CurrPersonWMid = "";

  m_CurrentCmdFinalStatus = NavData::UNKNOWN;
  
  m_RobotServerHost = "localhost";

  m_NumInhibitors = 0;
  m_SentInhibitStop = false;
}

NavControl::~NavControl() 
{ }



void NavControl::configure(const map<string,string>& _config) 
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

  if (Cure::NavController::config(configfile)) {
    println("configure(...) Failed to config with \"%s\", use -c option\n",
            configfile.c_str());
    std::abort();
  } 

  if (cfg.getSensorPose(1, m_LaserPoseR)) {
    println("configure(...) Failed to get sensor pose");
    std::abort();
  } 

  m_MaxExplorationRange = 1;
  it = _config.find("--explore-range");
  if (it != _config.end()) {
    m_MaxExplorationRange = (atof(it->second.c_str()));
  }

  it = _config.find("--robot-server-host");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_RobotServerHost;
  }

  m_lgm = new Cure::LocalGridMap<unsigned char>(200, 0.1, '2', Cure::LocalGridMap<unsigned char>::MAP1);
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
  Cure::NavController::setUsePathTrimming(true);
  Cure::NavController::setMaxPathTrimDist(3);
  Cure::NavController::setProgressTimeout(15);
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
} 

void NavControl::start() 
{ 
  addChangeFilter(createLocalTypeFilter<NavData::InternalNavCommand>(cdl::ADD), 
		  new MemberFunctionChangeReceiver<NavControl>(this,
								  &NavControl::newNavCtrlCommand));  

  addChangeFilter(createLocalTypeFilter<NavData::InhibitNavControl>(cdl::ADD),
		  new MemberFunctionChangeReceiver<NavControl>(this,
								  &NavControl::newInhibitor));
	        
  addChangeFilter(createLocalTypeFilter<NavData::InhibitNavControl>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<NavControl>(this,
								  &NavControl::deleteInhibitor));
	        
  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
		  new MemberFunctionChangeReceiver<NavControl>(this,
								  &NavControl::newRobotPose));

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<NavControl>(this,
								  &NavControl::newRobotPose));  
  
  addChangeFilter(createLocalTypeFilter<NavData::NavGraph>(cdl::ADD),
		  new MemberFunctionChangeReceiver<NavControl>(this,
								  &NavControl::newNavGraph));
  
  addChangeFilter(createLocalTypeFilter<NavData::NavGraph>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<NavControl>(this,
								  &NavControl::newNavGraph));    
  
  addChangeFilter(createLocalTypeFilter<NavData::Person>(cdl::ADD),
		  new MemberFunctionChangeReceiver<NavControl>(this,
                                                               &NavControl::newPersonData));
  
  addChangeFilter(createLocalTypeFilter<NavData::Person>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<NavControl>(this,
                                                               &NavControl::newPersonData));    
  
  addChangeFilter(createLocalTypeFilter<NavData::Person>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<NavControl>(this,
                                                               &NavControl::deletePersonData));    
  
  log("NavControl started");
  
}

void 
NavControl::changeCurrentCommandCompletion (const NavData::Completion &value, 
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

void NavControl::abortTask(int taskId) {
  // If a command is aborted, that means that another NavCtrlCommand
  // is in the game, so the NavControl is aware of it.
  // Therefore, nothing need to be done here.
  log("abortTask");
}

void NavControl::doneTask(int taskId) {
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


void NavControl::failTask(int taskId, int error) {
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
void NavControl::explorationDone(int taskId, int status)
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

void NavControl::runComponent() 
{
  setupPushScan2d(*this, 0.1);
  setupPushOdometry(*this);

  log("I am running!");
  
  while(isRunning()){
    if (m_Displaylgm) {
      Cure::Pose3D currentPose = m_TOPP.getPose();
      m_Displaylgm->updateDisplay(&currentPose,
                                  &m_NavGraph, 
                                  &m_Explorer->m_Fronts);
    }

    usleep(250000);
  }
}

void NavControl::newNavGraph(const cdl::WorkingMemoryChange &objID){

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

void NavControl::newPersonData(const cdl::WorkingMemoryChange &objID)
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
	    NavControl::PersonData pd;
	    pd.m_WMid = objID.address.id;
	    pd.m_data = p;
	    m_People.push_back(pd);
	  }

  }catch(DoesNotExistOnWMException){}
}
  
void NavControl::deletePersonData(const cdl::WorkingMemoryChange &objID)
{
  log("deletePersonData called");

  int i = 0;
  for (std::vector<NavControl::PersonData>::iterator pi = m_People.begin();
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
  
void NavControl::newInhibitor(const cdl::WorkingMemoryChange &objID) 
{
  m_NumInhibitors++;
  log("Got new inhibitor, now has %d", m_NumInhibitors);
}

void NavControl::deleteInhibitor(const cdl::WorkingMemoryChange &objID) 
{
  m_NumInhibitors--;
  log("Deleted an inhibitor now has %d", m_NumInhibitors);
}

void NavControl::newRobotPose(const cdl::WorkingMemoryChange &objID) 
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


void NavControl::newNavCtrlCommand(const cdl::WorkingMemoryChange &objID) 
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


void NavControl::receiveOdometry(const Robotbase::Odometry &castOdom)
{
  Cure::Pose3D cureOdom;
  CureHWUtils::convOdomToCure(castOdom, cureOdom);

  debug("Got odometry x=%.2f y=%.2f a=%.4f t=%.6f",
        cureOdom.getX(), cureOdom.getY(), cureOdom.getTheta(),
        cureOdom.getTime().getDouble());
  
  m_TOPP.addOdometry(cureOdom);
  
  m_CurrPose = m_TOPP.getPose();
	
  if (m_ready) { // have to get a first nav graph 
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
}

void NavControl::receiveScan2d(const Laser::Scan2d &castScan)
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

      /*      
      static int id = 0;
      std::fstream fs;
      char buf[128];
      sprintf(buf, "lm%02d.txt", id);
      fs.open(buf, std::ios::out);
      for (unsigned int i = 0; i < m_LMap.nObst(); i++) {
        fs << m_LMap.obstRef(i).x << " "
           << m_LMap.obstRef(i).y << " ";
      }
      fs << std::endl;
      fs.close();
      println("Saved %s, map has %d obstacles", buf, m_LMap.nObst());
      id = (id + 1) % 100;
      */

      Cure::Pose3D lpW;
      m_lgm->setValueInsideCircle(scanPose.getX(), scanPose.getY(),
                                  0.5*Cure::NavController::getRobotWidth(), 
                                  '0');
      lpW.add(scanPose, m_LaserPoseR);
      m_Glrt->addScan(cureScan, lpW, m_MaxExplorationRange);      
    }
  }
    
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
}

void 
NavControl::execCtrl(Cure::MotionAlgorithm::MotionCmd &cureCmd) 
{
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
  
  m_RobotServer->execMotionCommand(cmd);
}
