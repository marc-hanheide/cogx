/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "DummyDriver.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::DummyDriver();
  }
}

namespace cast
{

using namespace std;
using namespace cast;
using namespace manipulation::slice;
using namespace ptz;
using namespace VisionData;
using namespace SpatialData;

void DummyDriver::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  if((it = _config.find("--labels")) != _config.end())
  {
    istringstream istr(it->second);
    string label;
    while(istr >> label)
      labels.push_back(label);

    ostringstream ostr;
    for(size_t i = 0; i < labels.size(); i++)
      ostr << " '" << labels[i] << "'";
    log("detecting objects: %s", ostr.str().c_str());
  }
}

void DummyDriver::start()
{
  // we want to receive detected objects
  addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<DummyDriver>(this,
        &DummyDriver::receiveVisualObject));
  // .. and when they change
  addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DummyDriver>(this,
        &DummyDriver::receiveVisualObject));
        
  addChangeFilter(createGlobalTypeFilter<NavCommand>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DummyDriver>(this,
        &DummyDriver::overwriteNavCommand));
        
//  addChangeFilter(createGlobalTypeFilter<SetPTZPoseCommand>(cdl::OVERWRITE),
//      new MemberFunctionChangeReceiver<DummyDriver>(this,
//        &DummyDriver::overwriteSetPTZPoseCommand));
        
  addChangeFilter(createGlobalTypeFilter<GraspForObjectCommand>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DummyDriver>(this,
        &DummyDriver::overwriteGraspCommand));
        
  addChangeFilter(createGlobalTypeFilter<LookForObjectCommand>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DummyDriver>(this,
        &DummyDriver::overwriteLook4ObjCommand));
}

void DummyDriver::runComponent()
{
  sleepProcess(10000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.
  // and initiate detection
  
  addLook4ObjCommand(0, -0.5);
  log("executed ptz");
  
  sleepProcess(4000);
// addNavCommand(5, 5, 1);
   log("move");
  getGraspPoses(m_poses);
  log("got poses");
  if(m_poses.size() > 0)
  {	
  	log("issuing mover command");
  	addNavCommand(m_poses[0]->robotPose.x, m_poses[0]->robotPose.y, m_poses[0]->robotPose.z);
  }
}

void DummyDriver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::VisualObjectPtr obj =
    getMemoryEntry<VisionData::VisualObject>(_wmc.address);
    
  if(obj->identLabels.empty() && obj->identLabels[0].substr(0, 5) == "table.")
  {
  	string lbl = obj->identLabels[0];
  	
  }
  else
  	return;
/*  
  if(obj->detectionConfidence >= 0.5)
    log("ok, detected '%s'", obj->identLabels[0].c_str());
  else
    log("nah, did not detect '%s'", obj->identLabels[0].c_str());

  VisionData::DetectionCommandPtr cmd = new VisionData::DetectionCommand;
  cmd->labels.push_back(obj->identLabels[0]);
  addToWorkingMemory(newDataID(), cmd);
  */
}

// Move PTZ

bool DummyDriver::addPTZCommand(double pan, double tilt) {
  SetPTZPoseCommandPtr ptz_cmd = new SetPTZPoseCommand;
  
  PTZPose pose;
  pose.pan = pan;
  pose.tilt = tilt;
  pose.zoom = 0;
  
  ptz_cmd->pose = pose;
  ptz_cmd->comp = ptz::COMPINIT;
  
  m_ptz = ptz::COMPINIT;
  
  addToWorkingMemory(newDataID(), ptz_cmd);
  log("Add SetPTZPoseCommand: %d, %d, 0", pan, tilt);
  
  while(m_ptz == ptz::COMPINIT)
		sleepComponent(60);
		
  return (m_ptz == ptz::SUCCEEDED);
}

// Receive PTZ completion signal

void DummyDriver::overwriteSetPTZPoseCommand(const cdl::WorkingMemoryChange & _wmc){
  SetPTZPoseCommandPtr ptz_cmd = getMemoryEntry<SetPTZPoseCommand>(_wmc.address);

  log("Received PTZ confirmation");

  m_ptz = ptz_cmd->comp;
  assert(m_ptz != ptz::COMPINIT);
//  deleteFromWorkingMemory(_wmc.address.id);
}

// Look4Obj functions	
bool DummyDriver::addLook4ObjCommand(double pan, double tilt) {
  LookForObjectCommandPtr cmd = new LookForObjectCommand;
  
  
  cmd->status = VisionData::NEW;
  cmd->comp = VisionData::COMPINIT;
  cmd->pan = pan;
  cmd->tilt = tilt;
  
  m_viscomp = VisionData::COMPINIT;
  
  addToWorkingMemory(newDataID(), cmd);
  log("Added LookForObjects command");
  
  while(m_viscomp == VisionData::COMPINIT)
		sleepComponent(50);
		
  return (m_viscomp == VisionData::SUCCEEDED);

}
  
void DummyDriver::overwriteLook4ObjCommand(const cdl::WorkingMemoryChange & _wmc) {
  LookForObjectCommandPtr cmd = getMemoryEntry<LookForObjectCommand>(_wmc.address);

  log("Received LookForObject confirmation");

  m_viscomp = cmd->comp;
  assert(m_viscomp != VisionData::COMPINIT);
//  deleteFromWorkingMemory(_wmc.address.id);
}
  
// Grasp functions	
bool DummyDriver::addGraspCommand(string label) {
  GraspForObjectCommandPtr cmd = new GraspForObjectCommand;
  
  
  cmd->status = VisionData::NEW;
  cmd->comp = VisionData::COMPINIT;
  
  m_viscomp = VisionData::COMPINIT;
  
  addToWorkingMemory(newDataID(), cmd);
  log("Added GraspForObject command");
  
  while(m_viscomp == VisionData::COMPINIT)
		sleepComponent(50);
		
  return (m_viscomp == VisionData::SUCCEEDED);
}

void DummyDriver::overwriteGraspCommand(const cdl::WorkingMemoryChange & _wmc) {
  GraspForObjectCommandPtr cmd = getMemoryEntry<GraspForObjectCommand>(_wmc.address);

  log("Received GraspForObject confirmation");

  m_viscomp = cmd->comp;
  assert(m_viscomp != VisionData::COMPINIT);
//  deleteFromWorkingMemory(_wmc.address.id);
}

// Nav Commands	
bool DummyDriver::addNavCommand(double x, double y, double angle) {

	NavCommandPtr nc = new NavCommand();
//	                nc->pose = new vector<double>;
	   nc->pose.push_back(x);
 	   nc->pose.push_back(y);
	   nc->pose.push_back(angle);
	
//	                nc->tolerance=new vector<double>;
	   nc->tolerance.push_back(0.1);
	   nc->tolerance.push_back(0.1);
	   nc->tolerance.push_back(0.175);
	   
	   nc->destId.push_back(0);
	   nc->distance.push_back(0);
	   nc->angle.push_back(0);
	   
	   nc->cmd = SpatialData::GOTOPOSITION;
	   nc->comp = SpatialData::COMMANDPENDING;
	   nc->prio = SpatialData::URGENT;
	   nc->status = SpatialData::NONE;
	
	m_nav = SpatialData::COMMANDPENDING;
	addToWorkingMemory(newDataID(), string("spatial.sa"), nc);
	while(m_nav == SpatialData::COMMANDINPROGRESS || m_nav == SpatialData::COMMANDPENDING)
		sleepComponent(50);
		
	return (m_nav == SpatialData::COMMANDSUCCEEDED);
}

bool DummyDriver::addNavCommand(cogx::Math::Vector3 pose) {		
	return addNavCommand(pose.x, pose.y, pose.z);
}

bool DummyDriver::addNavCommand(long place) {

	NavCommandPtr nc = new NavCommand();
//	                nc->pose = new vector<double>;
	   nc->pose.push_back(0);
 	   nc->pose.push_back(0);
	   nc->pose.push_back(0);
	
//	                nc->tolerance=new vector<double>;
	   nc->tolerance.push_back(0.1);
	   nc->tolerance.push_back(0.1);
	   nc->tolerance.push_back(0.175);
	   
	   nc->destId.push_back(place);
	   nc->distance.push_back(0);
	   nc->angle.push_back(0);
	   
	   nc->cmd = SpatialData::GOTOPLACE;
	   nc->comp = SpatialData::COMMANDPENDING;
	   nc->prio = SpatialData::URGENT;
	   nc->status = SpatialData::NONE;
	
	m_nav = SpatialData::COMMANDPENDING;
	addToWorkingMemory(newDataID(), string("spatial.sa"), nc);
	while(m_nav == SpatialData::COMMANDINPROGRESS || m_nav == SpatialData::COMMANDPENDING)
		sleepComponent(50);
		
	return (m_nav == SpatialData::COMMANDSUCCEEDED);
}


void DummyDriver::overwriteNavCommand(const cdl::WorkingMemoryChange & _wmc) {
	NavCommandPtr cmd = getMemoryEntry<NavCommand>(_wmc.address);

  log("Received Nav confirmation");

  m_nav = cmd->comp;

//  deleteFromWorkingMemory(_wmc.address.id);
}

void DummyDriver::getGraspPoses(vector<ManipulationPosePtr> wmposes) {

	wmposes.clear();
	getMemoryEntries(wmposes, string("manipulation.sa"));
}

ManipulationPosePtr DummyDriver::bestPose(vector<ManipulationPosePtr> poses) {
	double min=56565444;
	ManipulationPosePtr best;
	int pos;
	
	for (int it=0 ; it < poses.size(); it++ )
		if(poses[it]->distance < min) {
  			best = poses[it]);
  			pos=it;
  		}
  		
  	poses.erase(pos);
  	return best;
}

 DummyDriver::purgePoses(string label, vector<ManipulationPosePtr> poses) {
	
	vector<ManipulationPosePtr> purged;
//	vector<ManipulationPosePtr>::iterator it;
	for (int it=0 ; it < poses.size(); it++ ) {
		if(poses[it]->label != label)
  			purged.push_back(poses[it]);
  	}
  	
  	return purged;
}


}


