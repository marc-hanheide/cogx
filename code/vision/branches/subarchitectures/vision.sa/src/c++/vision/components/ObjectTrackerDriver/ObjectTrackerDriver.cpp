/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <ChangeFilterFactory.hpp>
#include "ObjectTrackerDriver.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ObjectTrackerDriver();
  }
}

namespace cast
{

using namespace std;

void ObjectTrackerDriver::configure(const map<string,string> & _config)
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

void ObjectTrackerDriver::start()
{
  // we want to receive detected objects
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectTrackerDriver>(this,
        &ObjectTrackerDriver::receiveVisualObject));
  // .. and when they change
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<ObjectTrackerDriver>(this,
        &ObjectTrackerDriver::receiveVisualObject));
}

void ObjectTrackerDriver::runComponent()
{
  sleepProcess(1000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.
  
  // Load Geometry to Tracker
  VisionData::GeometryModelPtr geom = new VisionData::GeometryModel;
  geom->label = "box";
  addToWorkingMemory(newDataID(), geom);
  
  // Start Tracking
  VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
  track_cmd->cmd = VisionData::START;
  addToWorkingMemory(newDataID(), track_cmd);
  
  // for 10 seconds
  sleepComponent(10000);
  
  // Stop Tracking
  track_cmd->cmd = VisionData::STOP;
  addToWorkingMemory(newDataID(), track_cmd);
  
}

void ObjectTrackerDriver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::VisualObjectPtr obj =
    getMemoryEntry<VisionData::VisualObject>(_wmc.address);
  if(obj->detectionConfidence >= 0.5)
    log("ok, detected '%s'", obj->label.c_str());
  else
    log("nah, did not detect '%s'", obj->label.c_str());

  VisionData::DetectionCommandPtr cmd = new VisionData::DetectionCommand;
  cmd->labels.push_back(obj->label);
  addToWorkingMemory(newDataID(), cmd);
}

}

