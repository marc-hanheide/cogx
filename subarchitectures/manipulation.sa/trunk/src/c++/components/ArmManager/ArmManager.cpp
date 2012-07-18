/**
 * @author Alen Vrecko
 * @date September 2011
 * (with many code snippets from ss11 abuse of ObjectRecognizer3DDriver by Tom Mörwald)
 */

#include <iostream>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "ArmManager.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::ArmManager();
   }
}

using namespace std;
using namespace cast;
using namespace manipulation::slice;
using namespace VisionData;
using namespace cogx::Math;

namespace cogx
{

  ArmManager::ArmManager() : m_lastStatus(MCREQUESTED)
{
#ifdef FEAT_VISUALIZATION
  //display.setClientData(this);
#endif
}

void ArmManager::configure(const map<string, string> &_config)
{
  map<string,string>::const_iterator it;

  if((it = _config.find("--playerhost")) != _config.end())
  {
    //  playerHost = it->second;
  }
  if((it = _config.find("--playerport")) != _config.end())
  {
    //  istringstream str(it->second);
    //  str >> playerPort;
  }
}

void ArmManager::start()
{
  m_halt_arm = true;
  m_repeat_arm_movement = false;
  m_pointing_now = false;
  m_pointingOffsetVer = POINTING_OFFSET_VER;
  m_pointingOffsetHor = POINTING_OFFSET_HOR;

  //  addChangeFilter(createGlobalTypeFilter<VisualObject>(cdl::ADD),
  //    new MemberFunctionChangeReceiver<ArmManager>(this, &ArmManager::receiveNewObject));

  //  addChangeFilter(createGlobalTypeFilter<VisualObject>(cdl::DELETE),
  //    new MemberFunctionChangeReceiver<ArmManager>(this, &ArmManager::receiveDeletedObject));
  addChangeFilter(createGlobalTypeFilter<ArmMovementTask>(cdl::ADD),
      new MemberFunctionChangeReceiver<ArmManager>(this, &ArmManager::receiveNewCommand));

  addChangeFilter(createGlobalTypeFilter<FarArmMovementCommand>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<ArmManager>(this,
        &ArmManager::overwriteFarArmMovementCommand));

  addChangeFilter(createGlobalTypeFilter<MoveArmToHomePositionCommand>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<ArmManager>(this,
        &ArmManager::overwriteMoveToHomeCommand));

  addChangeFilter(createGlobalTypeFilter<MoveArmToPose>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<ArmManager>(this,
        &ArmManager::overwriteMoveToPose));
}


void ArmManager::runComponent()
{
  while(isRunning())
  {
    std::queue<cdl::WorkingMemoryAddress> actionItems;
    {
      // SYNC: Lock the monitor
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_queueMonitor);

      // SYNC: If queue is empty, unlock the monitor and wait for notify() or timeout
      if (m_actionQueue.size() < 1)
        m_queueMonitor.timedWait(IceUtil::Time::seconds(2));

      // SYNC: Continue with a locked monitor
      while (!actionItems.empty()) actionItems.pop(); // clear, just in case
      while (!m_actionQueue.empty()) {
        actionItems.push(m_actionQueue.front());
        m_actionQueue.pop();
      }

      // SYNC: unlock the queue monitor; items in actionItems will be processed while
      // new ones can be added to the m_actionQueue.
    }

    while (!actionItems.empty())
    {
      cdl::WorkingMemoryAddress addr;

      addr = actionItems.front();
      actionItems.pop();

      ArmMovementTaskPtr task;
      try {
        task = getMemoryEntry<ArmMovementTask>(addr);
      }
      catch (DoesNotExistOnWMException e) {
        println("WARNING: the arm movement task entry ID %s was removed before it could be processed.", addr.id.c_str());
      }

      switch (task->taskType) {
        case POINTOBJ0:
          //log(""" **** 11 (%s - %s)""", 
          //    task->objPointerSeq[0]->address.subarchitecture.c_str(),
          //    task->objPointerSeq[0]->address.id.c_str());
          //      	  m_pointing_now = addFarArmMovementCommand(addr);
          assert(task->objPointerSeq[0]->type.find("VisualObject") != string::npos);
          task->status = pointAtObject(task->objPointerSeq[0]->address);

          break;
        case RETRACTARM:
          //if(!m_pointing_now) {
          //  task->status = MCFAILED;
          //  break;
          //} 
	  task->status = addMoveToHomeCommand();
          if(task->status == MCSUCCEEDED) {
            m_pointing_now = false;
          } 

          break;
        default:
          error("Arm command not known");
          task->status = MCFAILED;
      }

      try {
        overwriteWorkingMemory(addr, task);
      }
      catch (DoesNotExistOnWMException e) {
        println("WARNING: could not overwrite WM entry ID '%s'.", addr.id.c_str());
      }
    }
  }
}

void ArmManager::destroy()
{
}

Pose3 ArmManager::pointingPose(const Pose3 objPose)
{
  /*Pose3 pointingPose = objPose;

  double dist = sqrt(sqr(pointingPose.pos.x) + sqr(pointingPose.pos.y));
  double fact = (dist - m_pointingOffsetHor)/dist;
  debug("Calculation pointing pose x:%f y:%f dist: %f fact:%f xp:%f yp:%f",
      pointingPose.pos.x, pointingPose.pos.y, dist, fact,
      pointingPose.pos.x*fact, pointingPose.pos.y*fact);
  
  pointingPose.pos.x *= fact;
  pointingPose.pos.y *= fact;
  pointingPose.pos.z += m_pointingOffsetVer;
  
  double sin = pointingPose.pos.z + m_pointingOffsetVer;
  double tan = sqrt(sqr(m_pointingOffsetHor) + sqr(sin));
  
  pointingPose.rot.m11 = m_pointingOffsetHor/tan;
  pointingPose.rot.m12 = sin/tan;
  pointingPose.rot.m21 = -sin/tan;
  pointingPose.rot.m22 = m_pointingOffsetHor/tan;*/
  Pose3 pointingPose;
  setIdentity(pointingPose);
  pointingPose.pos = objPose.pos;
  double dist = sqrt(sqr(pointingPose.pos.x) + sqr(pointingPose.pos.y));
  double fact = (dist - m_pointingOffsetHor)/dist;
  pointingPose.pos.x *= fact;
  pointingPose.pos.y *= fact;
  pointingPose.pos.z += m_pointingOffsetVer;
  Vector3 y = objPose.pos;
  y.z = 0.;
  normalise(y);
  Vector3 z = vector3(0., 0., 1.);
  Vector3 x = cross(y, z);
  pointingPose.rot.m00 = x.x;
  pointingPose.rot.m10 = x.y;
  pointingPose.rot.m20 = x.z;

  pointingPose.rot.m01 = y.x;
  pointingPose.rot.m11 = y.y;
  pointingPose.rot.m21 = y.z;

  pointingPose.rot.m02 = z.x;
  pointingPose.rot.m12 = z.y;
  pointingPose.rot.m22 = z.z;

  return pointingPose;
}

ManipulationTaskStatus ArmManager::pointAtObject(cdl::WorkingMemoryAddress addr)
{
  Pose3 objPose;
  setIdentity(objPose);

  try {
    VisualObjectPtr obj = getMemoryEntry<VisualObject>(addr);
    objPose.pos = obj->pose.pos;
  }
  catch(DoesNotExistOnWMException e) {
    println("pointAtObject: the object does not exist!");
    return MCFAILED;
  }
  
 // addCloseGripperCommand();

  return addMoveArmToPose(pointingPose(objPose));

}

void ArmManager::receiveNewCommand(const cdl::WorkingMemoryChange &_wmc)
{	
  debug("received a new ArmMovementTask");

  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_queueMonitor);

  m_actionQueue.push(_wmc.address);
  m_queueMonitor.notify();

  //addFarArmMovementCommand(_wmc.address);
  //FarArmMovementCommandPtr cmd = new FarArmMovementCommand();
  //cmd->targetObjectAddr = _wmc.address;
  //VisualObjectPtr obj = getMemoryEntry<VisualObject>(_wmc.address);
  //MoveArmToPosePtr cmd = new MoveArmToPose();
  //cmd->targetPose.pos.x=0.4;
  //cmd->targetPose.pos.y=1;
  //cmd->targetPose.pos.z=0.5;
  //cmd->targetPose.rot.m00=1;
  //cmd->targetPose.rot.m01=0;
  //cmd->targetPose.rot.m02=0;
  //cmd->targetPose.rot.m10=0;
  //cmd->targetPose.rot.m11=1;
  //cmd->targetPose.rot.m12=0;
  //cmd->targetPose.rot.m20=0;
  //cmd->targetPose.rot.m21=0;
  //cmd->targetPose.rot.m22=1;
  //addToWorkingMemory(newDataID(), cmd);

  debug("added movement command");
}

//void ArmManager::receiveDeletedObject(const cdl::WorkingMemoryChange &_wmc)
//{
//try {
//log("received a deleted VisualObject");

//IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_queueMonitor);
//armAction action;
//action.type = RETRACTARM;
////	action.objAddr = _wmc.address;
//m_actionQueue.push(action);
//m_queueMonitor.notify();

//log("added retract command");
//}
//catch (const std::exception& e) {
//log("exception: %s",e.what());
//}
//}

ManipulationTaskStatus ArmManager::addMoveArmToPose(cogx::Math::Pose3 pose) //, cogx::Math::Vector3 offset)
{
  log("Starting to move the arm (MoveArmToPose).");
  MoveArmToPosePtr moveArmCom = new MoveArmToPose();
  moveArmCom->comp = COMPINIT;
  moveArmCom->status = NEW;
  moveArmCom->targetPose = pose;
  //	farArmMovementCom->offset = offset;
  string data_id = newDataID();
  m_repeat_arm_movement = true;
  m_lastStatus = MCREQUESTED;
  m_lastStatus = MCREQUESTED;

  while(m_repeat_arm_movement) {
    m_repeat_arm_movement = false;
    addToWorkingMemory(data_id, moveArmCom);
    // Wait for arm to finish
    debug("Waiting for arm to finish movement");
    while(m_halt_arm && isRunning())
      sleepComponent(100);
    m_halt_arm = true;

    if(!isRunning())
      return MCFAILED;
    deleteFromWorkingMemory(data_id);	
  }
  log("Arm movement finished.");
  return m_lastStatus;
}

bool ArmManager::addFarArmMovementCommand(cast::cdl::WorkingMemoryAddress wma) //, cogx::Math::Vector3 offset)
{
  log("Starting to move the arm (FarArmMovementCommand).");
  FarArmMovementCommandPtr farArmMovementCom = new FarArmMovementCommand();
  farArmMovementCom->comp = COMPINIT;
  farArmMovementCom->status = NEW;
  farArmMovementCom->targetObjectAddr = wma;
  //	farArmMovementCom->offset = offset;
  string data_id = newDataID();
  m_repeat_arm_movement = true;

  while(m_repeat_arm_movement){
    m_repeat_arm_movement = false;
    addToWorkingMemory(data_id, farArmMovementCom);
    // Wait for arm to finish
    debug("Waiting for arm to finish movement");
    while(m_halt_arm && isRunning())
      sleepComponent(100);
    m_halt_arm = true;

    if(!isRunning())
      return false;
    deleteFromWorkingMemory(data_id);	
  }
  log("Arm movement finished.");
  return true;
}

ManipulationTaskStatus ArmManager::addMoveToHomeCommand()
{
  log("Starting to move the arm (MoveArmToHomePositionCommand).");

  MoveArmToHomePositionCommandPtr moveToHomeCom = new MoveArmToHomePositionCommand();
  moveToHomeCom->comp = COMPINIT;
  moveToHomeCom->status = NEW;
  //	moveToHomeCom->targetObjectAddr = wma;
  //	farArmMovementCom->offset = offset;
  string data_id = newDataID();
  m_repeat_arm_movement = true;
  m_lastStatus = MCREQUESTED;

  while(m_repeat_arm_movement){
    m_repeat_arm_movement = false;
    addToWorkingMemory(data_id, moveToHomeCom);
    // Wait for arm to finish
    debug("Waiting for arm to finish movement");
    while(m_halt_arm && isRunning())
      sleepComponent(100);
    m_halt_arm = true;
    if(!isRunning())
      return MCFAILED;
    deleteFromWorkingMemory(data_id);
  }
  log("Arm movement finished.");
  return m_lastStatus;
}


bool ArmManager::addCloseGripperCommand()
{
  log("Close gripper.");

  CloseGripperCommandPtr closeGripperCom = new CloseGripperCommand();
  closeGripperCom->comp = COMPINIT;
  closeGripperCom->status = NEW;
  //	moveToHomeCom->targetObjectAddr = wma;
  //	farArmMovementCom->offset = offset;
  string data_id = newDataID();
  m_repeat_arm_movement = true;

  while(m_repeat_arm_movement){
    m_repeat_arm_movement = false;
    addToWorkingMemory(data_id, closeGripperCom);
    // Wait for arm to finish
    debug("Waiting for gripper to close");
    while(m_halt_arm && isRunning())
      sleepComponent(100);
    m_halt_arm = true;
    if(!isRunning())
      return false;
    deleteFromWorkingMemory(data_id);
  }
  log("Arm movement finished.");
  return true;
}


bool ArmManager::addOpenGripperCommand()
{
  log("Open gripper.");
  
  OpenGripperCommandPtr openGripperCom = new OpenGripperCommand();
  openGripperCom->comp = COMPINIT;
  openGripperCom->status = NEW;
  string data_id = newDataID();
  m_repeat_arm_movement = true;

  while(m_repeat_arm_movement){
    m_repeat_arm_movement = false;
    addToWorkingMemory(data_id, openGripperCom);
    // Wait for arm to finish
    debug("Waiting for gripper to open");
    while(m_halt_arm && isRunning())
      sleepComponent(100);
    m_halt_arm = true;
    if(!isRunning())
      return false;
    deleteFromWorkingMemory(data_id);
  }
  log("Arm movement finished.");
  return true;
}

void ArmManager::overwriteFarArmMovementCommand(const cdl::WorkingMemoryChange & _wmc)
{
  FarArmMovementCommandPtr cmd = getMemoryEntry<FarArmMovementCommand>(_wmc.address);

  debug("FarArmMovementCommand overwritten");

  if(cmd->status == manipulation::slice::FINISHED){
    log("Arm movement finished");
    m_halt_arm =false;
  }
  if(cmd->status == manipulation::slice::COMMANDFAILED){
    log("Arm movement failed");
    m_halt_arm =false;
    //		m_repeat_arm_movement = true;
  }
  if (m_halt_arm)
    log("m_halt_arm true");
  else
    log("m_halt_arm false");
}

void ArmManager::overwriteMoveToHomeCommand(const cdl::WorkingMemoryChange & _wmc)
{
  MoveArmToHomePositionCommandPtr cmd = getMemoryEntry<MoveArmToHomePositionCommand>(_wmc.address);

  log("MoveArmToHomePositionCommand overwritten");

  if(cmd->status == manipulation::slice::FINISHED){
    log("Arm movement finished");
    m_halt_arm =false;
    m_lastStatus = MCSUCCEEDED;
  }
  else if(cmd->status == manipulation::slice::COMMANDFAILED){
    log("Arm movement failed");
    m_halt_arm =false;
    //		m_repeat_arm_movement = true;
    m_lastStatus = MCFAILED;
  }
  if (m_halt_arm)
    log("m_halt_arm true");
  else
    log("m_halt_arm false");
}

void ArmManager::overwriteMoveToPose(const cdl::WorkingMemoryChange & _wmc)
{
  MoveArmToPosePtr cmd = getMemoryEntry<MoveArmToPose>(_wmc.address);

  log("MoveArmToPose overwritten");

  if(cmd->status == manipulation::slice::FINISHED){
    log("Arm movement finished");
    m_halt_arm =false;
    m_lastStatus = MCSUCCEEDED;
  }
  else if(cmd->status == manipulation::slice::COMMANDFAILED){
    log("Arm movement failed");
    m_halt_arm =false;
    //		m_repeat_arm_movement = true;
    m_lastStatus = MCFAILED;

  }

  if (m_halt_arm)
    log("m_halt_arm true");
  else
    log("m_halt_arm false");
}

}
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et :vim
