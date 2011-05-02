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

void ObjectRecognizer3DDriver::loadVisualModelToWM(std::string filename, std::string& modelID, Math::Pose3 pose){
 	log("Loading PLY model");
	ModelLoader modelloader;
	Model model;
	modelloader.LoadPly(model, filename.c_str());

	VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
  obj->model = new VisionData::GeometryModel;
	convertModel2Geometry(model, obj->model);
	obj->identLabels.push_back("Testobject");
	obj->identLabels.push_back("unknown");
	obj->identDistrib.push_back(1.0);
	obj->identDistrib.push_back(0.0);
	obj->identAmbiguity = 0.0;
	obj->pose = pose;

  log("Add model to working memory: '%s'", obj->identLabels[0].c_str());
  modelID = newDataID();
  addToWorkingMemory(modelID, obj);
}

void ObjectRecognizer3DDriver::addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, std::string label, std::string visualObjectID){
	VisionData::Recognizer3DCommandPtr rec_cmd = new VisionData::Recognizer3DCommand;
  rec_cmd->cmd = cmd;
  rec_cmd->label = label;
  rec_cmd->visualObjectID = visualObjectID;
  addToWorkingMemory(newDataID(), rec_cmd);
  log("Add Recognizer3DCommand: '%s'", rec_cmd->label.c_str());
}


void ObjectRecognizer3DDriver::addPTZCommand(double pan, double tilt) {
  SetPTZPoseCommandPtr ptz_cmd = new SetPTZPoseCommand;
  
  PTZPose pose;
  pose.pan = pan;
  pose.tilt = tilt;
  pose.zoom = 0;
  
  ptz_cmd->pose = pose;
  ptz_cmd->comp = ptz::COMPINIT;
  
  addToWorkingMemory(newDataID(), ptz_cmd);
  log("Add SetPTZPoseCommand: %d, %d, 0", pan, tilt);
}

void ObjectRecognizer3DDriver::addTrackingCommand(VisionData::TrackingCommandType cmd){
	VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
	track_cmd->cmd = cmd;
	addToWorkingMemory(newDataID(), track_cmd);
	log("Add TrackingCommand");
}

void ObjectRecognizer3DDriver::doFarArmMovement(cast::cdl::WorkingMemoryAddress wma, cogx::Math::Vector3 vOffset){
	FarArmMovementCommandPtr farArmMovementCom = new FarArmMovementCommand();
	farArmMovementCom->comp = manipulation::slice::COMPINIT;
	farArmMovementCom->status = manipulation::slice::NEW;
	farArmMovementCom->targetObjectAddr = wma;
	farArmMovementCom->offset = vOffset;
	std::string data_id = newDataID();
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

void ObjectRecognizer3DDriver::doMoveArmToPose(cogx::Math::Pose3 pose){
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

void ObjectRecognizer3DDriver::doOpenGripper(){
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

void ObjectRecognizer3DDriver::doCloseGripper(){
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
    m_mode = RECOGNIZE;
    if(it->second == "LEARN")
      m_mode = RECLEARN;
    // else mode is "RECOGNIZE"
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
  if(m_mode == RECOGNIZE)
    log("Recognizing objects: %s", ostr.str().c_str());
  else if(m_mode == RECLEARN)
    log("Learning objects: %s", ostr.str().c_str());
}

void ObjectRecognizer3DDriver::start(){

//   addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
//       new MemberFunctionChangeReceiver<ObjectRecognizer3DDriver>(this,
//         &ObjectRecognizer3DDriver::receiveVisualObject));

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

}

void ObjectRecognizer3DDriver::runComponent(){
	sleepProcess(1000);  // HACK: the nav visualisation might crash if we send it
					   // object observations too soon.

	std::string data_id;

	m_halt_rec = true;
	m_halt_arm = true;
	m_ptz = false;
	m_obj_distance = 1000.0;
	m_rec_objects = 0;

	// Load PLY model to working memory
	std::string modelID;
	//   loadVisualModelToWM(m_plyfile, modelID, Math::Pose3());
	//   m_visualObjectIDs.push_back(modelID);

	m_timer.Update();

	addPTZCommand(0.0, -1.0472);
//	while(!m_ptz && isRunning())
		sleepComponent(5000);
	if(!isRunning())
		return;

  // trigger Recognizer3D
 // for(int j=0; j<m_loops && isRunning(); j++){

 // 	log("*** Loop %d/%d ***", j, m_loops);
//   	addTrackingCommand(RELEASEMODELS);

	// Recognize all objects
	for(int i=0; i<m_labels.size(); i++){

		if(m_mode == RECOGNIZE){
			addRecognizer3DCommand(RECOGNIZE, m_labels[i], modelID);

			while(m_halt_rec && isRunning())
				sleepComponent(100);
			m_halt_rec = true;
			if(!isRunning())
				return;

		}else if(m_mode = RECLEARN){
			addRecognizer3DCommand(RECLEARN, m_labels[i], modelID);
		}

	}
			
	log("%s %f", m_rec_cmd->label.c_str(), m_rec_cmd->confidence);

	if(m_mode == RECOGNIZE && !m_manipulation_sa.empty()){
		if(m_rec_cmd->confidence > 0.03){

			cogx::Math::Vector3 vOffset = cogx::Math::vector3(0.0,0.0,0.0);


			log("Add FarArmMovementCommand to Working Memory.");

			cast::cdl::WorkingMemoryAddress wma;
			wma.id = m_rec_cmd->visualObjectID;
			wma.subarchitecture = getSubarchitectureID();

			// Move Arm in front of visual object position
			log("move arm to object pose -0.15");
			vOffset = cogx::Math::vector3(-0.15,0.0,0.0);
			doFarArmMovement(wma, vOffset);

			// Move Arm toward visual object center
			log("move arm forward 0.1");
			vOffset = cogx::Math::vector3(-0.05,0.0,0.0);
			doFarArmMovement(wma, vOffset);

			// Close Gripper
			doCloseGripper();

			// Lift arm
			log("lift object");
			vOffset = cogx::Math::vector3(-0.05,0.0,0.2);
			doFarArmMovement(wma, vOffset);

			// look left
			addPTZCommand(1.5, 0.0);

			// Move to save pose
			log("move to save pose");
			cogx::Math::Pose3 save_pose;
			save_pose.pos = cogx::Math::vector3(0.0, 0.4, 0.9);
			save_pose.rot.m00 = 0.9;	save_pose.rot.m01 = 0.1;	save_pose.rot.m02 = -0.5;
			save_pose.rot.m10 = -0.3;	save_pose.rot.m11 = 0.9;	save_pose.rot.m12 = -0.4;
			save_pose.rot.m20 = 0.4;	save_pose.rot.m21 = 0.5;	save_pose.rot.m22 = 0.8;
			doMoveArmToPose(save_pose);

			// Close Gripper
			doOpenGripper();


		}else{

			log("Confidence to low %f < 0.03", m_rec_cmd->confidence);

		}
	}

	// look straight
	addPTZCommand(0.0, 0.0);
//	while(!m_ptz && isRunning())
		sleepComponent(5000);

// 		log("Taking Screenshot");
// 		addTrackingCommand(SCREENSHOT);

//	}
	
	

//	printf("Results: %f\n", m_timer.Update()/(m_loops*m_labels.size()));
//	for(int i=0; i<m_labels.size(); i++){
//		printf("  %s %f %f\n", m_labels[i].c_str(), 100*float(m_sumDetections[m_labels[i]])/m_loops, 100*m_sumConfidence[m_labels[i]]/m_loops);
//	}

	log("Done");

}



void ObjectRecognizer3DDriver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc){
  VisionData::VisualObjectPtr obj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);

	log("Received Visual Object");
}

void ObjectRecognizer3DDriver::overwriteRecognizer3DCommand(const cdl::WorkingMemoryChange & _wmc){
	VisionData::Recognizer3DCommandPtr rec_cmd = getMemoryEntry<VisionData::Recognizer3DCommand>(_wmc.address);

//	m_sumConfidence[m_rec_cmd->label] += m_rec_cmd->confidence;
//	if(rec_cmd->confidence > 0.03)
//		m_sumDetections[rec_cmd->label] += 1;

	log("Receive Recognizer3DCommand update");
	cast::cdl::WorkingMemoryAddress wma;
	wma.subarchitecture = getSubarchitectureID();
	wma.id = rec_cmd->visualObjectID;
	VisionData::VisualObjectPtr obj = getMemoryEntry<VisionData::VisualObject>(wma);
	double dist = (obj->pose.pos.x*obj->pose.pos.x + obj->pose.pos.y*obj->pose.pos.y);


	  if(m_rec_objects==0){
		  if(dist != 0.0){
			  m_obj_distance = dist;
			  m_rec_cmd = rec_cmd;
		  }
	  }else if(dist < m_obj_distance){
		  if(dist != 0.0){
			  m_obj_distance = dist;
			  m_rec_cmd = rec_cmd;
		  }
	  }
	  m_rec_objects++;
	  m_halt_rec =false;
}

void ObjectRecognizer3DDriver::overwriteFarArmMovementCommand(const cdl::WorkingMemoryChange & _wmc){
	m_arm_cmd = getMemoryEntry<FarArmMovementCommand>(_wmc.address);

	log("Got overwriteFarArmMovementCommand");

	if(m_arm_cmd->status == manipulation::slice::FINISHED){
		log("Arm movement finished");
		m_halt_arm =false;
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

  if(ptz_cmd->comp == ptz::SUCCEEDED){
	  deleteFromWorkingMemory(_wmc.address.id);
	  m_ptz = true;
  }


}


