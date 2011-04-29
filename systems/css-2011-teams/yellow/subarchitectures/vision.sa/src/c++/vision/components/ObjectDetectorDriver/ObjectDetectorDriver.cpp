/**
 * @author Andreas Richtsfeld
 * @date May 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "ObjectDetectorDriver.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ObjectDetectorDriver();
  }
}

using namespace cast;

using namespace std;

void ObjectDetectorDriver::configure(const map<string,string> & _config)
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
  
 
}

void ObjectDetectorDriver::start()
{
//  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
//      new MemberFunctionChangeReceiver<ObjectTrackerDriver>(this,
//        &ObjectTrackerDriver::receiveVisualObject));
}

void ObjectDetectorDriver::runComponent()
{
  sleepProcess(1000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.
                    
  while(isRunning())
  {   
    // Send start detection command
    VisionData::ObjectDetectionCommandPtr detect_cmd = new VisionData::ObjectDetectionCommand;
    detect_cmd->cmd = VisionData::DSTART;
    addToWorkingMemory(newDataID(), detect_cmd);
    log("detection start-command sent!");
  
    // Detect for 10 seconds
    sleepComponent(30000);
  
    // Send stop tracking command
    detect_cmd->cmd = VisionData::DSTOP;
    addToWorkingMemory(newDataID(), detect_cmd);
    log("detection stop command sent!");

    // Sleep for 10 seconds
    sleepComponent(5000);
  }
}

void ObjectDetectorDriver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::VisualObjectPtr obj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);

  if(obj->detectionConfidence >= 0.5)
    log("Cube detected: '%s'", obj->label.c_str());
  else
    log("nah, did not detect '%s'", obj->label.c_str());
}




