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
	timedout = false;
}

void VisionDriver::start()
{
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<VisionDriver>(this,
        &VisionDriver::receiveVisualObject));
}

void VisionDriver::runComponent()
{
  sleepProcess(1000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.
  
  // Send start detection command
  VisionData::ObjectDetectionCommandPtr detect_cmd = new VisionData::ObjectDetectionCommand;
  detect_cmd->cmd = VisionData::DSTART;
  addToWorkingMemory(newDataID(), detect_cmd);
  log("detection start-command sent!");
  timedout = true;
  
  sleepProcess(60000);	// timeout time
  
  if(timedout){
  	log("detection timeout");
  	// Stop detecting
    VisionData::ObjectDetectionCommandPtr detect_cmd = new VisionData::ObjectDetectionCommand;
	detect_cmd->cmd = VisionData::DSTOP;
	addToWorkingMemory(newDataID(), detect_cmd);
	log("detection stop-command sent!");
  }

}

void VisionDriver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::VisualObjectPtr obj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);

  if(obj->detectionConfidence >= 0.5){
    log("object detected: '%s'", obj->label.c_str());
    timedout = false;
    
    // Stop detecting
    VisionData::ObjectDetectionCommandPtr detect_cmd = new VisionData::ObjectDetectionCommand;
	detect_cmd->cmd = VisionData::DSTOP;
	addToWorkingMemory(newDataID(), detect_cmd);
	log("detection stop-command sent!");
    
    // Object detected send tracking command
    VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
  	track_cmd->cmd = VisionData::START;
  	addToWorkingMemory(newDataID(), track_cmd);
  	log("tracking start-command sent!");
  }else{
  	log("detection confidence to low, stop");
  }
}




