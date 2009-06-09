//
// = FILENAME
//    TranslationProcess.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Chandana Paul
//    Dorian Galvez
//
// = COPYRIGHT
//    Copyright (c) 2007 Chandana Paul
//
/*----------------------------------------------------------------------*/

#include "TranslationProcess.hpp"
#include <NavData.hpp>
#include <Rendezvous.h>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <list>
#include <cmath>

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
    return new TranslationProcess();
  }
}

// ----------------------------------------------------------------------------

TranslationProcess::TranslationProcess() {

  //setOntology(NavOntologyFactory::getOntology());
  
  pthread_cond_init(&m_MutexCond, NULL);

  m_navGraphChanged = true;
 
}

// ----------------------------------------------------------------------------

TranslationProcess::~TranslationProcess() {
}

// ----------------------------------------------------------------------------

void TranslationProcess::start() {
  ManagedComponent::start();

  addChangeFilter(createLocalTypeFilter<NavData::NavCommand>(cdl::ADD),
		  new MemberFunctionChangeReceiver<TranslationProcess>(this,
								       &TranslationProcess::newNavCommand));
	        
  addChangeFilter(createLocalTypeFilter<NavData::NavGraph>(cdl::ADD),
		  new MemberFunctionChangeReceiver<TranslationProcess>(this,
								       &TranslationProcess::newNavGraph));
	        
  addChangeFilter(createLocalTypeFilter<NavData::NavGraph>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<TranslationProcess>(this,
								       &TranslationProcess::owtNavGraph));

  log("TranslationProcess started");

}

// ----------------------------------------------------------------------------

void TranslationProcess::stop(){
  ManagedComponent::stop();
  pthread_cond_broadcast(&m_MutexCond);
}

// ----------------------------------------------------------------------------

void TranslationProcess::runComponent() {

  while(isRunning()){
    log("I am running, but I wait until getting work to do");
		
    std::vector< boost::shared_ptr<CASTData<NavData::RobotPose2d> > >poseVector;
    while (poseVector.empty()) {
      getWorkingMemoryEntries<NavData::RobotPose2d>(0, poseVector);	
      sleepComponent(200); // wait a little...
    }
    std::vector< boost::shared_ptr<CASTData<NavData::NavGraph> > > graphVector;
    while (graphVector.empty()) {
      getWorkingMemoryEntries<NavData::NavGraph>(0, graphVector);	
      sleepComponent(200); // wait a little...
    }

    log("Got the data needed to start, now wating for task");

    m_Tasks.lock();
    while(isRunning() && m_Tasks.size() == 0){
      pthread_cond_wait(&m_MutexCond, &m_Tasks.m_Mutex);
    }

    log("I am awake");

    if(isRunning()){
      // ok, work to do
      tpNavCommandWithId cmd = m_Tasks.front();
      m_Tasks.pop_front(); // we take out the task
      m_Tasks.m_Abort = false;
      m_Tasks.m_Executing = true;
      m_Tasks.m_ExecutingId = cmd.first;
      m_Tasks.unlock();
			
      if(cmd.second->cmd == NavData::BLOCKCONTROL) 
	// This command is special
	executeBlockCommand(cmd);
      else
	// Normal command
	executeCommand(cmd);
    }else
      m_Tasks.unlock();
  }
}

// ----------------------------------------------------------------------------

void TranslationProcess::executeCommand(const tpNavCommandWithId &cmd){
  // Note: this function must listen to abort events and to m_Tasks.m_Abort
	
  // Lets see if we can carry out the command
  NavData::InternalNavCommandPtr ctrl = new NavData::InternalNavCommand;
  NavData::Completion result;
  NavData::StatusError status;
  if(translateCommand(cmd.second, *ctrl, status)){
    // ok, execution in progress
    result = NavData::INPROGRESS;
  }else{
    // fail :(
    result = NavData::FAILED;
  }
	
  // Report fail/in_progress
  m_Tasks.lock();
  if(!m_Tasks.m_Abort){
    // This way we make sure we dont overwrite an "abort" completion
    changeNavCmdCompletion(cmd.first, result, status);
  }
  if(result == NavData::FAILED) m_Tasks.m_Executing = false;
  m_Tasks.unlock();

  // If task failed, dont go on
  if(result == NavData::FAILED){
    return;
  }
	
  // Post the nav ctrl command then
  string navCtrlCmdId = newDataID();
  string navCmdId = cmd.first;
		
  Rendezvous *rv = new Rendezvous(*this);
	
  rv->addChangeFilter(
		      createChangeFilter<NavData::InternalNavCommand>(cdl::OVERWRITE,
								  "", // src
								  navCtrlCmdId, // change id
								  subarchitectureID(), // change sa
								  cdl::LOCALSA)); // local

  // This listens to the current nav command completion field
  rv->addChangeFilter(
		      createChangeFilter<NavData::NavCommand>(cdl::OVERWRITE,
							      "", navCmdId, getSubarchitectureID(), cdl::LOCALSA));
	
	
  bool some_error = false;
  bool aborted = false;
	
  // This is in case the abort signal arrives before adding the last filter
  if(!m_Tasks.m_Abort){	
    // Send command
    addToWorkingMemory<NavData::InternalNavCommand>(navCtrlCmdId, ctrl);
		
    // Wait until resolution...
    cdl::WorkingMemoryChange change;
    bool finished = false;
    while(!aborted && !some_error && !finished){
      debug("before rv wait...");
      change = rv->wait();
      debug("after rv wait");
			
      debug("Change:");
      debug("::type");
      debug(change.type);
      debug("::op");
      switch(change.operation){
      case cdl::ADD: debug("add"); break;
      case cdl::OVERWRITE: debug("overwrite"); break;
      case cdl::DELETE: debug("delete"); break;
      case cdl::GET: debug("get"); break;
      default: debug("wildcard??"); break;
      }
      debug("::src");
      debug(change.src);
			
      string type = change.type;
      if(type == typeName<NavData::NavCommand>()){
	debug("abort?");
	// abort got?
	try {
	  shared_ptr<CASTData<NavData::NavCommand> > pcmd = 
	    getWorkingMemoryEntry<NavData::NavCommand>(navCmdId);

	  if(pcmd){
	    aborted = 
	      (pcmd->getData()->comp == NavData::ABORTED);
	  }else{
	    log("The NavCommand suddenly disappeared...");
	    some_error = true;
	  }
	} catch (DoesNotExistOnWMException) {
	  log("The NavCommand suddenly disappeared...");
	  some_error = true;
	}				
	debug(aborted? "yes": "no");
			
      }else if(type == typeName<NavData::InternalNavCommand>()){
	debug("nav ctrl cmd finished?");
	// nav ctrl cmd finished?
	shared_ptr<CASTData<NavData::InternalNavCommand> > pcmd = 
	  getWorkingMemoryEntry<NavData::InternalNavCommand>(navCtrlCmdId);
			
	if(pcmd){
	  switch(pcmd->getData()->comp){
	  case NavData::SUCCEEDED:
	    finished = true;
	    break;
	  case NavData::ABORTED:
	  case NavData::FAILED:
	    some_error = true;
	    status = NavData::TARGETUNREACHABLE;
	    break;
	  default: break;
	  }
	  debug(finished? "yes": "no");
	}else{
	  log("The InternalNavCommand suddenly disappeared...");
	  some_error = true;
	}
      }
    } // while(...)
		
    // make sure you delete rv before cancelCurrentTask, in case this rv
    // and that function have similar filters...
    delete rv;
		
    debug("before cancel current task");
    if(aborted) cancelCurrentTask(true, navCtrlCmdId);
    debug("after cancel current task");
		
  }else{ // if(!m_Tasks.m_Abort)
    delete rv;
  }
	
  // Disable executing flag
  debug("before lock");
  m_Tasks.lock();
  m_Tasks.m_Executing = false; // this means it cannot be marked as aborted
  if(m_Tasks.m_Abort) aborted = true;
  m_Tasks.unlock();
  debug("after lock");
	
  if(!aborted){
    // Post the final status of the NavCommand
    if(some_error){
      result = NavData::FAILED;
    }else{
      result = NavData::SUCCEEDED;
    }

    changeNavCmdCompletion(navCmdId, result, status);
		
  } // else, the ABORT completion was filled by cancelQueueTasks

  log("Task execution finished");
	
}

// ----------------------------------------------------------------------------

void TranslationProcess::executeBlockCommand(const tpNavCommandWithId &cmd){

  log("Executing block command");
	
  // Listen to abortions or deleting of the block command
  Rendezvous rv = Rendezvous(*this);
	
  rv.addChangeFilter(createChangeFilter<NavData::NavCommand>(cdl::OVERWRITE, 
							     "", // srcdsa
							     cmd.first, // address id
							     subarchitectureID(), // address sa
							     cdl::LOCALSA)); // local

  rv.addChangeFilter(createChangeFilter<NavData::NavCommand>(cdl::DELETE, 
							     "", // src
							     cmd.first, // change id
							     subarchitectureID(), // change sa 
							     cdl::LOCALSA)); // local (object.sa, probably)

  // Mark the command as in_progress	
  m_Tasks.lock();
  if(!m_Tasks.m_Abort){
    // This way we make sure we dont overwrite an "abort" completion
		
    // If the abortion and the removing are done exactly now,
    // changeNavCmdCompletion will take care of it
    changeNavCmdCompletion(cmd.first, 
			   NavData::INPROGRESS, NavData::NONE);
    debug("Blocking command marked as In progress");
  }
  m_Tasks.unlock();
	
  debug("Waiting until something happens on the blocking command...");
	
  // And wait until something happens...
  bool finished = m_Tasks.m_Abort;	
  while(!finished){
    rv.wait();
		
    if(existsOnWorkingMemory(cmd.first)){
      shared_ptr<CASTData<NavData::NavCommand> > oobj =
	getWorkingMemoryEntry<NavData::NavCommand>(cmd.first);
				
      if(oobj)
	finished = (oobj->getData()->comp == NavData::ABORTED);
      else
	finished = true;
    }else
      finished = true;
  }
	
  // Command finished, disable executing flag
  m_Tasks.lock();
  m_Tasks.m_Executing = false; // this means it cannot be marked as aborted
  m_Tasks.unlock();
	
  // if aborted, the ABORT completion was filled by cancelQueueTasks

  log("Block command execution finished");	
	
}

// ----------------------------------------------------------------------------

void TranslationProcess::cancelCurrentTask(bool stop_robot, string navCtrlCmdId)
{
	
  // Send a stop InternalNavCommand
  if(stop_robot){
    NavData::InternalNavCommandPtr stop = new NavData::InternalNavCommand;
    stop->cmd = NavData::lSTOPROBOT;
		
    Rendezvous rv(*this);
    string aux_id = newDataID();
		
    rv.addChangeFilter(createChangeFilter<NavData::InternalNavCommand>
		       (cdl::OVERWRITE, "", aux_id, subarchitectureID(), cdl::LOCALSA));
    addToWorkingMemory<NavData::InternalNavCommand>(aux_id, 
						stop);
				
    cdl::WorkingMemoryChange change;
    bool terminal = false;
    while(!terminal){
      debug("cancel current task: before wait");
      change = rv.wait();
      debug("cancel current task: after wait");
			
      shared_ptr<CASTData<NavData::InternalNavCommand> > oobj =
	getWorkingMemoryEntry<NavData::InternalNavCommand>(change.address);
			
      if(oobj){
	switch(oobj->getData()->comp){
	case NavData::FAILED:
	case NavData::SUCCEEDED:
	case NavData::ABORTED:
	  terminal = true;
	  break;
	default:
	  break;
	}
      }else
	terminal = true;
    }

    // Remove the stop command
    deleteFromWorkingMemory(aux_id);

  } // if(stop_robot)
	
  if(navCtrlCmdId != ""){
    deleteFromWorkingMemory(navCtrlCmdId);
    debug("Deleting this nav ctrl command");
    debug(navCtrlCmdId.c_str());
  }
	
}

// ----------------------------------------------------------------------------

void TranslationProcess::newNavCommand(const cdl::WorkingMemoryChange & objID){

  log("New NavCommand got");
	
  shared_ptr<CASTData<NavData::NavCommand> > oobj =
    getWorkingMemoryEntry<NavData::NavCommand>(objID.address);

  if (oobj != 0){
    string navId = objID.address.id;
    // this adds the task and signals the m_MutexCond
    addTaskToQueue(navId, oobj->getData());
  }
}		

// ----------------------------------------------------------------------------

void TranslationProcess::addTaskToQueue
(const std::string &id, const NavData::NavCommandPtr &cmd)
{
  tpNavCommandWithId idded(id, cmd);
  bool first = true;
	
  m_Tasks.lock();
  switch(cmd->prio){
  case NavData::NORMAL:
    log("Adding task at the end of the queue");
    first = (m_Tasks.size() == 0);
    m_Tasks.push_back(idded);
    break;
  case NavData::HIGH:
    log("Adding task at the front of the queue");
    m_Tasks.push_front(idded);
    break;
  case NavData::URGENT:
    log("Adding task after cancelling all the others");
    m_Tasks.m_Abort = true;
    cancelQueueTasks();
    m_Tasks.push_front(idded);
    break;
  }
	
  // if the task is not going to be executed now, we mark it as pending
  if(m_Tasks.m_Executing || (!first && m_Tasks.size() > 1)){
    changeNavCmdCompletion(id, NavData::PENDING, cmd->status);
  }
	
  pthread_cond_signal(&m_MutexCond);
  m_Tasks.unlock();
}

// ----------------------------------------------------------------------------

void TranslationProcess::cancelQueueTasks(){
  // Remember: m_Tasks is locked and the ongoing task is being stopped
  tpQueue::iterator it;
  for(it = m_Tasks.begin(); it != m_Tasks.end(); it++){
    // Cancel NavCommand on WM 
    changeNavCmdCompletion(it->first, NavData::ABORTED, NavData::NONE);
  }
  m_Tasks.clear();
	
  // Now the ongoing task, if any
  if(m_Tasks.m_Executing){
    // actually, this is only true when the action is finished in
    // executeCommand or executeBlockCommand
    m_Tasks.m_Executing = false; 
    changeNavCmdCompletion(m_Tasks.m_ExecutingId, 
			   NavData::ABORTED, NavData::NONE);
  }
}

// ----------------------------------------------------------------------------

void TranslationProcess::newNavGraph(const cdl::WorkingMemoryChange &objID){
  // Get the first navgraph
	
  shared_ptr<CASTData<NavData::NavGraph> > oobj = 
    getWorkingMemoryEntry<NavData::NavGraph>(objID.address);
	
  if(oobj){
    m_NavGraphMutex.lock();
    m_TmpNavGraph = oobj->getData();
    m_navGraphChanged = false;
    m_NavGraphMutex.unlock();
  }
	
}

// ----------------------------------------------------------------------------

void TranslationProcess::owtNavGraph(const cdl::WorkingMemoryChange &objID){
  m_NavGraphMutex.lock();
  m_navGraphChanged = true;
  m_NavGraphMutex.unlock();
}

// ----------------------------------------------------------------------------
		
bool TranslationProcess::translateCommand(const NavData::NavCommandPtr &nav, 
					  NavData::InternalNavCommand &ctrl, 
                                          NavData::StatusError &status)
{
  status = NavData::NONE;	

  if(nav->cmd == NavData::BLOCKCONTROL){
    log("read navcommand: BLOCKCONTROL");
    // nothing needs being done
		
  }else if (nav->cmd == NavData::GOTOPOSITION){

    log("read navcommand: GOTOPOSITION");
	
    if (nav->pose.size() < 2) {
      println("cmd syntax error, need NavCommand.pose (size 2)");
      status = NavData::CMDMALFORMATTED;
      return false;
    } else {
      ctrl.cmd = NavData::lGOTOXY;
      ctrl.x = nav->pose[0];
      ctrl.y = nav->pose[1];
      ctrl.tolerance = nav->tolerance;
    }
      
  }else if (nav->cmd == NavData::GOTONODE){

    log("read navcommand: GOTONODE");
	
    if (nav->destId.size() < 1) {
      println("cmd syntax error, need NavCommand.destId (size 1)");
      status = NavData::CMDMALFORMATTED;
      return false;
    } else {
      ctrl.cmd = NavData::lGOTONODE;
      ctrl.nodeId = nav->destId[0];
      ctrl.tolerance = nav->tolerance;
    }
      
  }else if (nav->cmd == NavData::GOTOAREA){

    log("read navcommand: GOTOAREA");
	
    if (nav->destId.empty()) {
      println("cmd syntax error, need NavCommand.destId (size 1)");
    } else {
      ctrl.cmd = NavData::lGOTOAREA;
      ctrl.areaId = nav->destId[0];
    }
	
  }else if (nav->cmd == NavData::GOFORWARD) {

    log("read navcommand: GOFORWARD");

    // find the position of the robot
    NavData::RobotPose2dPtr pose;
    if(!getRobotPose(pose) && pose) {
      println("Could not find robot pose and cannot GOFORWARD");
      status = NavData::CMDMALFORMATTED;
      return false;
    }

    if (nav->distance.empty()) {
      println("Need distance in GOFOWARD command");
      status = NavData::CMDMALFORMATTED;
      return false;
    }

    // add a small increment in the forward direction
    // send the x,y,theta to NavController
    ctrl.cmd = NavData::lGOTOXYA;
    ctrl.x = pose->x + nav->distance[0]*cos(pose->theta);
    ctrl.y = pose->y + nav->distance[0]*sin(pose->theta);
    ctrl.theta = pose->theta;
    ctrl.tolerance = nav->tolerance;

  }else if (nav->cmd == NavData::GOBACK) {

    log("read navcommand: GO_BACK");
		
    if (nav->distance.empty()) {
      println("Need distance in GOBACK command");
      status = NavData::CMDMALFORMATTED;
      return false;
    }
    
    ctrl.cmd = NavData::lBACKOFF;
    ctrl.distance = nav->distance[0];
    ctrl.tolerance = nav->tolerance;

  }else if (nav->cmd == NavData::TURN) {

    log("read navcommand: TURN");

    if (!nav->angle.empty()) {
      ctrl.cmd = NavData::lROTATEREL;		
      ctrl.theta = nav->angle[0];
      ctrl.tolerance = nav->tolerance;    
    } else if (!nav->target.empty()) {

      // interpret the string which says "left" or "right"
      string direction = nav->target[0];
      
      //send the command to the NavController
      ctrl.cmd = NavData::lROTATEREL;
      if(direction == "left")
        ctrl.theta = M_PI*45.0/180.0;
      else if(direction == "right")
        ctrl.theta = -M_PI*45.0/180.0;
      else{
        status = NavData::TARGETUNREACHABLE;
        return false;
      }
      ctrl.tolerance = nav->tolerance;
    } else {
      println("NavCommand TURN lacks target spec");
      status = NavData::CMDMALFORMATTED;
      return false;
    }

  }else if (nav->cmd == NavData::TURNTO) {

    log("read navcommand: TURNTO");

    if (nav->angle.empty()) {
      println("NavCommand TURNTO lacks angle spec");
      status = NavData::CMDMALFORMATTED;
      return false;
    }

    //send the command to the NavController
    ctrl.cmd = NavData::lROTATEABS;		
    ctrl.theta = nav->angle[0];
    ctrl.tolerance = nav->tolerance;

  }else if (nav->cmd == NavData::PERSONFOLLOW) {
	
    // call follow person, which wil write InternalNavCommand 
    // to working memory
    ctrl.cmd = NavData::lFOLLOWPERSON;		

    log("read navcommand: PERSON_FOLLOW");

  }else if (nav->cmd == NavData::EXPLORE) {
	
    log("read navcommand: EXPLORE");

    // call explore process, which will write InternalNavCommands
    // to working memory
    ctrl.cmd = NavData::lEXPLORE;
    ctrl.SetExplorerConfinedByGateways = true;


  }else if (nav->cmd == NavData::STOP) {
    log("read navcommand: STOP");
	
    // send the stop command to the NavController
    ctrl.cmd = NavData::lSTOPROBOT;
    ctrl.x = 0.0;
    ctrl.y = 0.0;
    ctrl.theta = 0.0;
  }
  else if (nav->cmd == NavData::EXPLOREFLOOR){
    log("read navcommand: EXPLORE FLOOR");
    ctrl.cmd = NavData::lEXPLOREFLOOR;
    ctrl.SetExplorerConfinedByGateways = false;
  }
		

  return true;
}

// ----------------------------------------------------------------------------

bool TranslationProcess::getRobotPose(NavData::RobotPose2dPtr &pose){

  vector<shared_ptr<CASTData<NavData::RobotPose2d> > > v;
  getWorkingMemoryEntries<NavData::RobotPose2d>(1, v);
  if(v.size() == 0)
    return false;
  else{
    pose = v[0]->getData();
    return true;
  }
}

// ----------------------------------------------------------------------------

NavData::NavGraphPtr TranslationProcess::getNavGraph(){
  if(m_navGraphChanged){
    vector<shared_ptr<CASTData<NavData::NavGraph> > > v;
    getWorkingMemoryEntries<NavData::NavGraph>(1, v);
    m_NavGraphMutex.lock();
    if(v.size() > 0){
      m_TmpNavGraph = v[0]->getData();
      m_navGraphChanged = false;
    }
    m_NavGraphMutex.unlock();
  }
  return m_TmpNavGraph;
}

// ----------------------------------------------------------------------------

bool TranslationProcess::findObjectNode(const std::string &objectName,
					const NavData::NavGraphPtr &graph, double &x, double &y)
{
  for(unsigned long i = 0; i < graph->objects.size(); i++){
    if(graph->objects[i]->category == objectName){
      x = graph->objects[i]->x;
      y = graph->objects[i]->y;
      return true;
    }
  }
  return false;
}

// ----------------------------------------------------------------------------

bool TranslationProcess::findObjectBestNode(const std::string &objectName,
					    const NavData::NavGraphPtr &graph, 
                                            long &nodeId)
{
  long areaId = -1;
  nodeId = -1;
	
  NavData::ObjDataPtr objData;
  for(unsigned long i = 0; i < graph->objects.size(); i++){
    if(graph->objects[i]->category == objectName){
      areaId = graph->objects[i]->areaId;
      objData = graph->objects[i];
      break;
    }
  }
	
  if(areaId != -1){
    double minD = 1e06;
    for(unsigned long i = 0; i < graph->fNodes.size(); i++){
      if(graph->fNodes[i]->areaId == areaId){
	const NavData::FNode &node = *(graph->fNodes[i]);
	double dx = node.x - objData->x;
	double dy = node.y - objData->y;
	double d = sqrt(dx*dx + dy*dy);
				
	if(d < minD){
	  nodeId = graph->fNodes[i]->nodeId;
	  minD = d;
	}
      }
    }	
  }
	
  return(nodeId != -1);
}

// ----------------------------------------------------------------------------


void TranslationProcess::changeNavCmdCompletion(const std::string &id, 
						const NavData::Completion &completion, const NavData::StatusError &status)
{
  shared_ptr<CASTData<NavData::NavCommand> > pcmd;
  NavData::NavCommandPtr newcmd;
  bool stop = false;

  try {
    pcmd = getWorkingMemoryEntry<NavData::NavCommand>(id);	
  } catch (DoesNotExistOnWMException) {
    debug("changeNavCmdCompletion called for nonexistent NavCommand");
  }
  while(!stop){
    newcmd = new NavData::NavCommand(*pcmd->getData());
    newcmd->comp = completion;
    newcmd->status = status;

    try {
      overwriteWorkingMemory<NavData::NavCommand>(id, newcmd);
      log(std::string("Overwrote Nav Command: ") + id);
      stop = true;
    }catch(ConsistencyException){

      // repeat only if the completion we are setting is 
      // more important than the one set				
      pcmd = getWorkingMemoryEntry<NavData::NavCommand>(id);
      stop = (completion <= pcmd->getData()->comp);
    }catch(DoesNotExistOnWMException){
      // we can get an exception for trying to update an already
      // deleted entry (because it was aborted)
      stop = true;
    }
  }
}

// ----------------------------------------------------------------------------

