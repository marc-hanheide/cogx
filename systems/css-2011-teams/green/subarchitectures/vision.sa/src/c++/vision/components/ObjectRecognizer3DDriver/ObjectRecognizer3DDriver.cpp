/**
 * @author Thomas Mörwald
 * @date February 2010
 *
 * Just a dummy component to drive some vision functionality.
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "ObjectRecognizer3DDriver.h"
#include <VideoUtils.h>


using namespace cogx;
using namespace Tracking;
using namespace manipulation::slice;
using namespace ptz;

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ObjectRecognizer3DDriver();
  }
}

using namespace cast;
using namespace std;
using namespace VisionData;

void ObjectRecognizer3DDriver::receiveLookForObjectCommand(const cdl::WorkingMemoryChange & _wmc){
	m_look_cmd = getMemoryEntry<VisionData::LookForObjectCommand>(_wmc.address);

	m_pan = m_look_cmd->pan;
	m_tilt = m_look_cmd->tilt;

	log("received LookForObject command at %e %e", m_pan, m_tilt);

	m_look = true;

//	look_cmd->status = VisionData::PENDING;
//	overwriteWorkingMemory(_wmc.address, look_cmd);

}

void ObjectRecognizer3DDriver::doLooking(){

	if(	m_pan >  1.57 ||
		m_pan < -1.57 ||
		m_tilt >  0.0 ||
		m_tilt < -1.1 )
	{
		log("WARNING: look pose for PTZ unit out of range!");
	}else{
		addPTZCommand(m_pan, m_tilt);
		if(!isRunning()) return;
	}

//	m_rec_visualObjectIDs.clear();

	// recognize all objects given
	std::string modelID;  // empty string: causes the ObjectRecognizer3D to generate a VisualObject for a label
	for(unsigned i=0; i<m_labels.size(); i++)
		addRecognizer3DCommand(VisionData::RECOGNIZE, m_labels[i], modelID);

//	// put found objects into LookForObjectCommand
//	look_cmd->foundVisualObjects.clear();
//	cast::cdl::WorkingMemoryAddress wma;
//	wma.subarchitecture = getSubarchitectureID();
//	for(unsigned i=0; i<m_rec_visualObjectIDs.size(); i++){
//		wma.id = m_rec_visualObjectIDs[i];
//		look_cmd->foundVisualObjects.push_back(wma);
//	}
//
//	// Update status and completion
//	if(look_cmd->foundVisualObjects.empty())
//		look_cmd->comp = VisionData::FAILED;
//	else
//		look_cmd->comp = VisionData::SUCCEEDED;
//
//	look_cmd->status = VisionData::FINISHED;
//
//	overwriteWorkingMemory(_wmc.address, look_cmd);
//
//	addPTZCommand(0.0, 0.0);

	m_look = false;

}

void ObjectRecognizer3DDriver::receiveGraspForObjectCommand(const cdl::WorkingMemoryChange & _wmc){
	m_grasp_cmd = getMemoryEntry<VisionData::GraspForObjectCommand>(_wmc.address);
	m_grasp_wma = _wmc.address;
	log("received GraspForObject command");

	m_grasp = true;

}

void ObjectRecognizer3DDriver::doGrasping(){

	if(m_manipulation_sa.empty()){
		error("No manipulation.sa given in cast file!");
		return;
	}

	addPTZCommand(0.0, -1.0472);
	if(!isRunning()) return;

	cogx::Math::Vector3 vOffset = cogx::Math::vector3(0.0,0.0,0.0);

	// Delete all visual objects so far ...
	log("delete all visual objects recognized so far");
	cast::cdl::WorkingMemoryAddress wma;
	wma.subarchitecture = getSubarchitectureID();
	for(unsigned i=0; i>m_visualObjectIDs.size(); i++){
		wma.id = m_visualObjectIDs[i];
		deleteFromWorkingMemory(wma);
	}
	m_visualObjectIDs.clear();

	// ... and recognize object to grasp
	log("recognizing object to grasp");
	std::string modelID;
	addRecognizer3DCommand(VisionData::RECOGNIZE, m_grasp_cmd->label, modelID);
	wma.id = m_latest_visualObjectID;
	wma.subarchitecture = getSubarchitectureID();
	if(!isRunning()) return;

	log("get visual object from working memory");
	VisionData::VisualObjectPtr visObj = getMemoryEntry<VisionData::VisualObject>(wma);
	if(visObj->identDistrib[0] < 0.03){
		log("cannot recognize object for grasping");
		m_grasp_cmd->comp = VisionData::FAILED;
		m_grasp_cmd->status = VisionData::COMMANDFAILED;
		overwriteWorkingMemory(m_grasp_wma, m_grasp_cmd);
		return;
	}


	// Move Arm in front of visual object position
	log("move arm to object pose -0.15");
	vOffset = cogx::Math::vector3(-0.15,0.0,0.0);
	addFarArmMovementCommand(wma, vOffset);
	if(!isRunning()) return;

	// Move Arm toward visual object center
	log("move arm forward 0.1");
	vOffset = cogx::Math::vector3(-0.05,0.0,0.0);
	addFarArmMovementCommand(wma, vOffset);
	if(!isRunning()) return;

	// Close Gripper
	addCloseGripperCommand();
	if(!isRunning()) return;

	// Lift arm
	log("lift object");
	vOffset = cogx::Math::vector3(-0.05,0.0,0.2);
	addFarArmMovementCommand(wma, vOffset);
	if(!isRunning()) return;

	// look left
	addPTZCommand(1.5, 0.0);
	if(!isRunning()) return;

	// Move to save pose
	log("move to save pose");
	cogx::Math::Pose3 save_pose;
	save_pose.pos = cogx::Math::vector3(0.0, 0.4, 0.9);
	save_pose.rot.m00 = 0.9;	save_pose.rot.m01 = 0.1;	save_pose.rot.m02 = -0.5;
	save_pose.rot.m10 = -0.3;	save_pose.rot.m11 = 0.9;	save_pose.rot.m12 = -0.4;
	save_pose.rot.m20 = 0.4;	save_pose.rot.m21 = 0.5;	save_pose.rot.m22 = 0.8;
	addMoveArmToPoseCommand(save_pose);
	if(!isRunning()) return;

	// open gripper
	addOpenGripperCommand();
	if(!isRunning()) return;

	// look straight
	addPTZCommand(0.0, 0.0);
	if(!isRunning()) return;

	log("Cannot recognize object for grasping");
	m_grasp_cmd->comp = VisionData::SUCCEEDED;
	m_grasp_cmd->status = VisionData::FINISHED;
	overwriteWorkingMemory(m_grasp_wma, m_grasp_cmd);

	m_grasp = false;

}

void ObjectRecognizer3DDriver::addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, std::string label, std::string visualObjectID){
	VisionData::Recognizer3DCommandPtr rec_cmd = new VisionData::Recognizer3DCommand;
	rec_cmd->cmd = cmd;
	rec_cmd->label = label;
	rec_cmd->visualObjectID = visualObjectID;
	std::string new_data_id = newDataID();
	log("Add Recognizer3DCommand: '%s'", rec_cmd->label.c_str());
	addToWorkingMemory(new_data_id, rec_cmd);

	log("Waiting for recognizer to finish");
	while(m_halt_rec && isRunning())
		sleepComponent(100);
	m_halt_rec = true;

	deleteFromWorkingMemory(new_data_id);
}


void ObjectRecognizer3DDriver::addPTZCommand(double pan, double tilt) {
	SetPTZPoseCommandPtr ptz_cmd = new SetPTZPoseCommand;

	PTZPose pose;
	pose.pan = pan;
	pose.tilt = tilt;
	pose.zoom = 0;

	ptz_cmd->pose = pose;
	ptz_cmd->comp = ptz::COMPINIT;

	log("Add SetPTZPoseCommand: %e, %e, 0", pan, tilt);
	std::string data_id = newDataID();
	addToWorkingMemory(data_id, ptz_cmd);
	log("Waiting for arm to finish movement");
		while(m_halt_ptz && isRunning())
			sleepComponent(100);
		m_halt_ptz = true;
	deleteFromWorkingMemory(data_id);

}

void ObjectRecognizer3DDriver::addTrackingCommand(VisionData::TrackingCommandType cmd){
	VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
	track_cmd->cmd = cmd;
	addToWorkingMemory(newDataID(), track_cmd);
	log("Add TrackingCommand");
}

void ObjectRecognizer3DDriver::addFarArmMovementCommand(cast::cdl::WorkingMemoryAddress wma, cogx::Math::Vector3 vOffset){
	FarArmMovementCommandPtr farArmMovementCom = new FarArmMovementCommand();
	farArmMovementCom->comp = manipulation::slice::COMPINIT;
	farArmMovementCom->status = manipulation::slice::NEW;
	farArmMovementCom->targetObjectAddr = wma;
	farArmMovementCom->offset = vOffset;
	std::string data_id = newDataID();
	m_repeat_arm_movement = true;
	while(m_repeat_arm_movement){
		m_repeat_arm_movement = false;
		addToWorkingMemory(data_id, m_manipulation_sa, farArmMovementCom);
			// Wait for arm to finish
			log("Waiting for arm to finish movement");
			while(m_halt_arm && isRunning())
				sleepComponent(100);
			m_halt_arm = true;
			if(!isRunning())
				return;
		deleteFromWorkingMemory(data_id, m_manipulation_sa);
	}
}

void ObjectRecognizer3DDriver::addMoveArmToPoseCommand(cogx::Math::Pose3 pose){
	MoveArmToPosePtr moveArmToPose = new MoveArmToPose();
	moveArmToPose->comp = manipulation::slice::COMPINIT;
	moveArmToPose->status = manipulation::slice::NEW;
	moveArmToPose->targetPose = pose;
	std::string data_id = newDataID();
	addToWorkingMemory(data_id, m_manipulation_sa, moveArmToPose);
		// Wait for arm to finish
		log("Waiting for arm to finish movement");
		while(m_halt_arm && isRunning())
			sleepComponent(100);
		m_halt_arm = true;
		if(!isRunning())
			return;
	deleteFromWorkingMemory(data_id, m_manipulation_sa);
}

void ObjectRecognizer3DDriver::addOpenGripperCommand(){
	log("open gripper");
	OpenGripperCommandPtr openGripperCmd = new OpenGripperCommand();
	openGripperCmd->comp = manipulation::slice::COMPINIT;
	openGripperCmd->status = manipulation::slice::NEW;
	std::string data_id = newDataID();
	addToWorkingMemory(data_id, m_manipulation_sa, openGripperCmd);
		// Wait for arm to finish
		log("Waiting for gripper to open");
		while(m_halt_arm && isRunning())
			sleepComponent(100);
		m_halt_arm = true;
		if(!isRunning())
			return;
	deleteFromWorkingMemory(data_id, m_manipulation_sa);

}

void ObjectRecognizer3DDriver::addCloseGripperCommand(){
	log("close gripper");
	CloseGripperCommandPtr closeGripperCmd = new CloseGripperCommand();
	closeGripperCmd->comp = manipulation::slice::COMPINIT;
	closeGripperCmd->status = manipulation::slice::NEW;
	closeGripperCmd->graspStatus = GRASPINGSTATUSINIT;
	std::string data_id = newDataID();
	addToWorkingMemory(data_id, m_manipulation_sa, closeGripperCmd);
		// Wait for arm to finish
		log("Waiting for gripper to close");
		while(m_halt_arm && isRunning())
			sleepComponent(100);
		m_halt_arm = true;
		if(!isRunning())
			return;
	deleteFromWorkingMemory(data_id, m_manipulation_sa);

}

void ObjectRecognizer3DDriver::configure(const map<string,string> & _config){
	map<string,string>::const_iterator it;

	if((it = _config.find("--labels")) != _config.end())
  {
    istringstream istr(it->second);
    string label;
    while(istr >> label){
      m_labels.push_back(label);
      m_sumDetections[label] = 0;
      m_sumConfidence[label] = 0.0;
    }
  }

	if((it = _config.find("--mode")) != _config.end())
  {
    m_mode = VisionData::RECOGNIZE;
    if(it->second == "LEARN")
      m_mode = VisionData::RECLEARN;

    if(it->second == "LOOKGRASP")
    	m_mode = LOOKGRASP;
    // else mode is "VisionData::RECOGNIZE"
  }
  
  if((it = _config.find("--manipulation_sa")) != _config.end())
  { 
    istringstream istr(it->second);
    istr >> m_manipulation_sa;
  }

  if((it = _config.find("--Loops")) != _config.end())
	{
    istringstream istr(it->second);
    istr >> m_loops;
	}else{
		m_loops = 1;
	}

  ostringstream ostr;
  for(size_t i = 0; i < m_labels.size(); i++)
    ostr << " '" << m_labels[i] << "'";
  if(m_mode == VisionData::RECOGNIZE)
    log("Recognizing objects: %s", ostr.str().c_str());
  else if(m_mode == VisionData::RECLEARN)
    log("Learning objects: %s", ostr.str().c_str());
}

void ObjectRecognizer3DDriver::start(){

   addChangeFilter(createLocalTypeFilter<VisionData::LookForObjectCommand>(cdl::ADD),
       new MemberFunctionChangeReceiver<ObjectRecognizer3DDriver>(this,
         &ObjectRecognizer3DDriver::receiveLookForObjectCommand));

   addChangeFilter(createLocalTypeFilter<VisionData::GraspForObjectCommand>(cdl::ADD),
          new MemberFunctionChangeReceiver<ObjectRecognizer3DDriver>(this,
            &ObjectRecognizer3DDriver::receiveGraspForObjectCommand));

	addChangeFilter(createLocalTypeFilter<VisionData::Recognizer3DCommand>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<ObjectRecognizer3DDriver>(this,
        &ObjectRecognizer3DDriver::overwriteRecognizer3DCommand));

	addChangeFilter(createGlobalTypeFilter<FarArmMovementCommand>(cdl::OVERWRITE),
	  new MemberFunctionChangeReceiver<ObjectRecognizer3DDriver>(this,
		&ObjectRecognizer3DDriver::overwriteFarArmMovementCommand));

	addChangeFilter(createGlobalTypeFilter<MoveArmToPose>(cdl::OVERWRITE),
	  new MemberFunctionChangeReceiver<ObjectRecognizer3DDriver>(this,
		&ObjectRecognizer3DDriver::overwriteMoveArmToPose));

	addChangeFilter(createGlobalTypeFilter<CloseGripperCommand>(cdl::OVERWRITE),
	  new MemberFunctionChangeReceiver<ObjectRecognizer3DDriver>(this,
		&ObjectRecognizer3DDriver::overwriteCloseGripperCommand));

	addChangeFilter(createGlobalTypeFilter<SetPTZPoseCommand>(cdl::OVERWRITE),
	  new MemberFunctionChangeReceiver<ObjectRecognizer3DDriver>(this,
		&ObjectRecognizer3DDriver::overwriteSetPTZPoseCommand));

}

void ObjectRecognizer3DDriver::runComponent(){

	m_pan = 0.0;
	m_tilt = 0.0;
	m_halt_rec = true;
	m_halt_arm = true;
	m_grasp = false;
	m_look = false;
	m_halt_ptz = false;
	m_repeat_arm_movement = false;
	m_obj_distance = 1000.0;
	m_rec_objects = 0;
	std::string modelID;

	sleepProcess(1000);  // HACK: the nav visualisation might crash if we send it
						 // object observations too soon.

	m_timer.Update();

	if(m_mode == VisionData::LOOKGRASP){

		VisionData::LookForObjectCommandPtr look_cmd = new VisionData::LookForObjectCommand();
		look_cmd->comp = VisionData::COMPINIT;
		look_cmd->status = VisionData::NEW;
		look_cmd->pan = 0.5;
		look_cmd->tilt = -0.5;
		log("add to working memory: look_cmd %e %e", look_cmd->pan, look_cmd->tilt);
		addToWorkingMemory(newDataID(), look_cmd);
//		VisionData::GraspForObjectCommandPtr grasp_cmd = new VisionData::GraspForObjectCommand();
//		grasp_cmd->comp = VisionData::COMPINIT;
//		grasp_cmd->status = VisionData::NEW;
//		grasp_cmd->label = m_labels[0];
//		addToWorkingMemory(newDataID(), grasp_cmd);

		while(isRunning()){

			if(m_grasp)
				doGrasping();

			if(m_look)
				doLooking();

			sleepComponent(100);
		}

	}

	if(m_mode == VisionData::RECOGNIZE){
		for(unsigned i=0; i<m_labels.size() && isRunning(); i++)
			addRecognizer3DCommand(VisionData::RECOGNIZE, m_labels[i], modelID);
	}

	if(m_mode == VisionData::RECLEARN){
		for(unsigned i=0; i<m_labels.size() && isRunning(); i++)
			addRecognizer3DCommand(VisionData::RECLEARN, m_labels[i], modelID);
	}
	
	log("done");

}

void ObjectRecognizer3DDriver::overwriteRecognizer3DCommand(const cdl::WorkingMemoryChange & _wmc){
	VisionData::Recognizer3DCommandPtr rec_cmd = getMemoryEntry<VisionData::Recognizer3DCommand>(_wmc.address);

	log("receive overwrite Recognizer3DCommand");

	m_rec_objects++;

	// add to visual object list if it is not contained already
	bool push = true;
	for(unsigned i=0; i<m_visualObjectIDs.size(); i++){
		if(rec_cmd->visualObjectID.compare(m_visualObjectIDs[i]) == 0)
			push = false;
	}
	if(push)
		m_visualObjectIDs.push_back(rec_cmd->visualObjectID);

	log("overwriteRecognizer3DCommand testing");
	m_latest_visualObjectID = rec_cmd->visualObjectID;

	m_halt_rec = false;

}

void ObjectRecognizer3DDriver::overwriteFarArmMovementCommand(const cdl::WorkingMemoryChange & _wmc){
	m_arm_cmd = getMemoryEntry<FarArmMovementCommand>(_wmc.address);

	log("Got overwriteFarArmMovementCommand");

	if(m_arm_cmd->status == manipulation::slice::FINISHED){
		log("Arm movement finished");
		m_halt_arm =false;
	}
	if(m_arm_cmd->status == manipulation::slice::COMMANDFAILED){
		log("Arm movement failed");
		m_halt_arm =false;
		m_repeat_arm_movement = true;
	}
}

void ObjectRecognizer3DDriver::overwriteMoveArmToPose(const cdl::WorkingMemoryChange & _wmc){
	m_moveto_cmd = getMemoryEntry<MoveArmToPose>(_wmc.address);

	log("Got overwriteMoveArmToPoseCommand");

	if(m_moveto_cmd->status == manipulation::slice::FINISHED){
		log("Arm movement finished");
		m_halt_arm =false;
	}

}

void ObjectRecognizer3DDriver::overwriteCloseGripperCommand(const cdl::WorkingMemoryChange & _wmc){
	manipulation::slice::CloseGripperCommandPtr gripper_cmd = getMemoryEntry<CloseGripperCommand>(_wmc.address);

	log("Got overwriteCloseGripperCommand");

	if(gripper_cmd->graspStatus == manipulation::slice::GRASPING){
		log("gripper closed");
		m_halt_arm =false;
	}
}

void ObjectRecognizer3DDriver::overwriteOpenGripperCommand(const cdl::WorkingMemoryChange & _wmc){
	manipulation::slice::OpenGripperCommandPtr gripper_cmd = getMemoryEntry<OpenGripperCommand>(_wmc.address);

	log("Got overwriteOpenGripperCommand");

	if(gripper_cmd->status == manipulation::slice::FINISHED){
		log("gripper opened");
		m_halt_arm =false;
	}
}



void ObjectRecognizer3DDriver::overwriteSetPTZPoseCommand(const cdl::WorkingMemoryChange & _wmc){
  SetPTZPoseCommandPtr ptz_cmd = getMemoryEntry<SetPTZPoseCommand>(_wmc.address);

  log("Received PTZ confirmation");

  m_halt_ptz = false;
}


