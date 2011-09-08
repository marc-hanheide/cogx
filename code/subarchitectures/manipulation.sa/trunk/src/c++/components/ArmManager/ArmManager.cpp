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

namespace cogx
{

using namespace std;
using namespace cast;
using namespace manipulation::slice;
using namespace VisionData;
using namespace cogx::Math;

ArmManager::ArmManager()
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

  addChangeFilter(createGlobalTypeFilter<VisualObject>(cdl::ADD),
    new MemberFunctionChangeReceiver<ArmManager>(this, &ArmManager::receiveNewObject));
    
  addChangeFilter(createGlobalTypeFilter<VisualObject>(cdl::DELETE),
    new MemberFunctionChangeReceiver<ArmManager>(this, &ArmManager::receiveDeletedObject));
    
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
  	// SYNC: Lock the monitor
  	IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_queueMonitor);
  	// SYNC: If queue is empty, unlock the monitor and wait for notify() or timeout
    if (m_actionQueue.size() < 1)
        m_queueMonitor.timedWait(IceUtil::Time::seconds(2));

    // SYNC: Continue with a locked monitor


    while (!m_actionQueue.empty())
    {
      cdl::WorkingMemoryAddress addr;
      
      armAction action = m_actionQueue.front();
      m_actionQueue.pop();
      
      switch (action.type) {
      	case POINT_OBJ:
      	  addr = action.objAddr;
      	  m_pointing_now = pointAtObject(addr);
//      	  m_pointing_now = addFarArmMovementCommand(addr);
      	  break;
      	case RETRACT:
      	  if(m_pointing_now)
      	  	m_pointing_now = addMoveToHomeCommand();
      	  break;
      	default:
      	  error("Arm command not known");
      };
    }
  }
}

void ArmManager::destroy()
{
}

Pose3 ArmManager::pointingPose(const Pose3 objPose)
{
	Pose3 pointingPose = objPose;
	
	double dist = sqrt(sqr(pointingPose.pos.x) + sqr(pointingPose.pos.y));
	double fact = (dist - POINTING_OFFSET)/dist;
	log("Calculation pointing pose x:%f y:%f dist: %f fact:%f xp:%f yp:%f",
		pointingPose.pos.x, pointingPose.pos.y, dist, fact,
		pointingPose.pos.x*fact, pointingPose.pos.y*fact);
	pointingPose.pos.x*=fact;
	pointingPose.pos.y*=fact;
	
	return pointingPose;
}

bool ArmManager::pointAtObject(cdl::WorkingMemoryAddress addr)
{
	VisualObjectPtr obj = getMemoryEntry<VisualObject>(addr);
	
	Pose3 objPose;
	setIdentity(objPose);
	objPose.pos = obj->pose.pos;
	
	return addMoveArmToPose(pointingPose(objPose));
		
}

void ArmManager::receiveNewObject(const cdl::WorkingMemoryChange &_wmc)
{
  try {
	log("received a new VisualObject");
	
	IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_queueMonitor);
	armAction action;
	action.type = POINT_OBJ;
	action.objAddr = _wmc.address;
  	m_actionQueue.push(action);
    m_queueMonitor.notify();
    
//	addFarArmMovementCommand(_wmc.address);
/*	
	FarArmMovementCommandPtr cmd = new FarArmMovementCommand();
	
	cmd->targetObjectAddr = _wmc.address;

	VisualObjectPtr obj = getMemoryEntry<VisualObject>(_wmc.address);
	MoveArmToPosePtr cmd = new MoveArmToPose();
	
	cmd->targetPose.pos.x=0.4;
	cmd->targetPose.pos.y=1;
	cmd->targetPose.pos.z=0.5;
	cmd->targetPose.rot.m00=1;
	cmd->targetPose.rot.m01=0;
	cmd->targetPose.rot.m02=0;
	cmd->targetPose.rot.m10=0;
	cmd->targetPose.rot.m11=1;
	cmd->targetPose.rot.m12=0;
	cmd->targetPose.rot.m20=0;
	cmd->targetPose.rot.m21=0;
	cmd->targetPose.rot.m22=1;


	addToWorkingMemory(newDataID(), cmd);
*/	
	log("added movement command");
  }
  catch (const std::exception& e) {
        log("exception: %s",e.what());
  }
}

void ArmManager::receiveDeletedObject(const cdl::WorkingMemoryChange &_wmc)
{
  try {
	log("received a deleted VisualObject");
	
	IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_queueMonitor);
	armAction action;
	action.type = RETRACT;
//	action.objAddr = _wmc.address;
  	m_actionQueue.push(action);
    m_queueMonitor.notify();

	log("added retract command");
  }
  catch (const std::exception& e) {
        log("exception: %s",e.what());
  }
}

bool ArmManager::addMoveArmToPose(cogx::Math::Pose3 pose) { //, cogx::Math::Vector3 offset){
	
	MoveArmToPosePtr moveArmCom = new MoveArmToPose();
	moveArmCom->comp = COMPINIT;
	moveArmCom->status = NEW;
	moveArmCom->targetPose = pose;
//	farArmMovementCom->offset = offset;
	string data_id = newDataID();
	m_repeat_arm_movement = true;
	
	while(m_repeat_arm_movement){
		m_repeat_arm_movement = false;
		addToWorkingMemory(data_id, moveArmCom);
			// Wait for arm to finish
			log("Waiting for arm to finish movement");
			while(m_halt_arm && isRunning())
				sleepComponent(100);
			m_halt_arm = true;
			
			if(!isRunning())
				return false;
		deleteFromWorkingMemory(data_id);	
	}
	log("Arm movement finished");
	return true;
}

bool ArmManager::addFarArmMovementCommand(cast::cdl::WorkingMemoryAddress wma) { //, cogx::Math::Vector3 offset){
	
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
			log("Waiting for arm to finish movement");
			while(m_halt_arm && isRunning())
				sleepComponent(100);
			m_halt_arm = true;
			
			if(!isRunning())
				return false;
		deleteFromWorkingMemory(data_id);	
	}
	log("Arm movement finished");
	return true;
}

bool ArmManager::addMoveToHomeCommand() {
	
	MoveArmToHomePositionCommandPtr moveToHomeCom = new MoveArmToHomePositionCommand();
	moveToHomeCom->comp = COMPINIT;
	moveToHomeCom->status = NEW;
//	moveToHomeCom->targetObjectAddr = wma;
//	farArmMovementCom->offset = offset;
	string data_id = newDataID();
	m_repeat_arm_movement = true;
	
	while(m_repeat_arm_movement){
		m_repeat_arm_movement = false;
		addToWorkingMemory(data_id, moveToHomeCom);
			// Wait for arm to finish
			log("Waiting for arm to finish movement");
			while(m_halt_arm && isRunning())
				sleepComponent(100);
			m_halt_arm = true;
			if(!isRunning())
				return false;
		deleteFromWorkingMemory(data_id);
	}
	log("Arm movement finished");
	return true;
}


void ArmManager::overwriteFarArmMovementCommand(const cdl::WorkingMemoryChange & _wmc){
	FarArmMovementCommandPtr cmd = getMemoryEntry<FarArmMovementCommand>(_wmc.address);

	log("FarArmMovementCommand overwritten");

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

void ArmManager::overwriteMoveToHomeCommand(const cdl::WorkingMemoryChange & _wmc){
	MoveArmToHomePositionCommandPtr cmd = getMemoryEntry<MoveArmToHomePositionCommand>(_wmc.address);

	log("MoveArmToHomePositionCommand overwritten");

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

void ArmManager::overwriteMoveToPose(const cdl::WorkingMemoryChange & _wmc){
	MoveArmToPosePtr cmd = getMemoryEntry<MoveArmToPose>(_wmc.address);

	log("MoveArmToPose overwritten");

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

}
