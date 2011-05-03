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
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<DummyDriver>(this,
        &DummyDriver::receiveVisualObject));
  // .. and when they change
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DummyDriver>(this,
        &DummyDriver::receiveVisualObject));
        
  addChangeFilter(createLocalTypeFilter<SetPTZPoseCommand>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DummyDriver>(this,
        &DummyDriver::overwriteSetPTZPoseCommand));
}

void DummyDriver::runComponent()
{
  sleepProcess(2000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.
  // and initiate detection
  VisionData::DetectionCommandPtr cmd = new VisionData::DetectionCommand;
  cmd->labels = labels;
  addToWorkingMemory(newDataID(), cmd);
}

void DummyDriver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::VisualObjectPtr obj =
    getMemoryEntry<VisionData::VisualObject>(_wmc.address);
  if(obj->detectionConfidence >= 0.5)
    log("ok, detected '%s'", obj->identLabels[0].c_str());
  else
    log("nah, did not detect '%s'", obj->identLabels[0].c_str());

  VisionData::DetectionCommandPtr cmd = new VisionData::DetectionCommand;
  cmd->labels.push_back(obj->identLabels[0]);
  addToWorkingMemory(newDataID(), cmd);
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
		sleepComponent(50);
		
  return (m_ptz == ptz::SUCCEEDED);
}

// Receive PTZ completion signal

void DummyDriver::overwriteSetPTZPoseCommand(const cdl::WorkingMemoryChange & _wmc){
  SetPTZPoseCommandPtr ptz_cmd = getMemoryEntry<SetPTZPoseCommand>(_wmc.address);

  log("Received PTZ confirmation");

  m_ptz = ptz_cmd->comp;
  deleteFromWorkingMemory(_wmc.address.id);

}
}

