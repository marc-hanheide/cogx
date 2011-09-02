/**
 * @author Alen Vrecko
 * @date September 2011
*/

#include <iostream>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "PointingTest.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cogx::PointingTest();
  }
}

namespace cogx
{

using namespace std;
using namespace cast;
using namespace manipulation::slice;
using namespace VisionData;
using namespace cogx::Math;

PointingTest::PointingTest()
{
#ifdef FEAT_VISUALIZATION
//display.setClientData(this);
#endif
}

void PointingTest::configure(const map<string, string> &_config)
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

void PointingTest::start()
{
  m_halt_arm = true;
  m_repeat_arm_movement = false;

  addChangeFilter(createGlobalTypeFilter<VisualObject>(cdl::ADD),
    new MemberFunctionChangeReceiver<PointingTest>(this, &PointingTest::receiveNewObject));
    
  addChangeFilter(createGlobalTypeFilter<FarArmMovementCommand>(cdl::OVERWRITE),
	new MemberFunctionChangeReceiver<PointingTest>(this,
		&PointingTest::overwriteFarArmMovementCommand));
}

void PointingTest::destroy()
{
}

void PointingTest::receiveNewObject(const cdl::WorkingMemoryChange &_wmc)
{
  try {
	log("received a new VisualObject");

	addFarArmMovementCommand(_wmc.address);
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

void PointingTest::addFarArmMovementCommand(cast::cdl::WorkingMemoryAddress wma) { //, cogx::Math::Vector3 offset){
	
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
//			while(m_halt_arm && isRunning())
//				sleepComponent(100);
			m_halt_arm = true;
			if(!isRunning())
				return;
//		deleteFromWorkingMemory(data_id);
	}
	log("Arm movement finished");
}

void PointingTest::overwriteFarArmMovementCommand(const cdl::WorkingMemoryChange & _wmc){
	FarArmMovementCommandPtr cmd = getMemoryEntry<FarArmMovementCommand>(_wmc.address);

	log("Got overwriteFarArmMovementCommand");

	if(cmd->status == manipulation::slice::FINISHED){
		log("Arm movement finished");
		m_halt_arm =false;
	}
	if(cmd->status == manipulation::slice::COMMANDFAILED){
		log("Arm movement failed");
		m_halt_arm =false;
//		m_repeat_arm_movement = true;
	}
}
}
