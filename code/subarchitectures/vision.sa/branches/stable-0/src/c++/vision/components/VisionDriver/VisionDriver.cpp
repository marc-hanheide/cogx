/**
 * @author Thomas Mörwald
 * @date June 2009
 *
 * Just a dummy component to drive some vision functionality.
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "VisionDriver.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::VisionDriver();
  }
}

using namespace cast;

using namespace std;

void VisionDriver::configure(const map<string,string> & _config)
{
//  map<string,string>::const_iterator it;
//
//  if((it = _config.find("--labels")) != _config.end())
//  {
//    istringstream istr(it->second);
//    string label;
//    while(istr >> label)
//      labels.push_back(label);

//    ostringstream ostr;
//    for(size_t i = 0; i < labels.size(); i++)
//      ostr << " '" << labels[i] << "'";
//    log("detecting objects: %s", ostr.str().c_str());
//  }
	tracking = false;
	running = true;
}

void VisionDriver::start()
{
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<VisionDriver>(this,
        &VisionDriver::receiveVisualObject));
        
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<VisionDriver>(this,
        &VisionDriver::receiveVisualObjectPoseChange));
}

void VisionDriver::runComponent()
{
  sleepProcess(1000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.
  
	VisionData::ObjectDetectionCommandPtr detect_cmd = new VisionData::ObjectDetectionCommand;
		
	while(running){
		// start detection
		detect_cmd->cmd = VisionData::DSTART;
		addToWorkingMemory(newDataID(), detect_cmd);
		log("detection start-command sent!");
		
  	sleepProcess(2000);	// detection time

		// stop detection
		detect_cmd->cmd = VisionData::DSTOP;
		addToWorkingMemory(newDataID(), detect_cmd);
		log("detection stop-command sent!");
  
  	sleepProcess(10000); // idle
  }
}

void VisionDriver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::VisualObjectPtr obj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);

  log("object detected: '%s'", obj->label.c_str());
  
   // Object detected send tracking command (if not already tracking)
	if(!tracking){
	  VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
		track_cmd->cmd = VisionData::START;
		addToWorkingMemory(newDataID(), track_cmd);
		log("tracking start-command sent!");
		tracking = true;
	}
}

void VisionDriver::receiveVisualObjectPoseChange(const cdl::WorkingMemoryChange & _wmc)
{
	VisionData::VisualObjectPtr obj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
	/*
	log("Change of pose of VisualObject '%s' detected: %f %f %f", 
		obj->label.c_str(), 
		obj->pose.pos.x,
		obj->pose.pos.y,
		obj->pose.pos.z);
*/
}




