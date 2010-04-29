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
  
  halt = false;
}

void DummyDriver::start()
{
  // we want to receive detected objects
//   addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
//       new MemberFunctionChangeReceiver<DummyDriver>(this,
//         &DummyDriver::receiveDetectionCommand));
  // .. and when they change
  addChangeFilter(createLocalTypeFilter<VisionData::DetectionDone>(cdl::ADD),
      new MemberFunctionChangeReceiver<DummyDriver>(this,
        &DummyDriver::receiveDetectionDone));
}

void DummyDriver::runComponent()
{
  sleepProcess(2000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.
  // and initiate detection
  VisionData::DetectionCommandPtr cmd = new VisionData::DetectionCommand;
  while(isRunning()){
    cmd->labels = labels;
    addToWorkingMemory(newDataID(), cmd);
    halt = true;
    
    while(isRunning() && halt){
      sleepComponent(500);
    }
    
  }
}

void DummyDriver::receiveDetectionDone(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::DetectionDonePtr dd =
    getMemoryEntry<VisionData::DetectionDone>(_wmc.address);
  if(dd->done)
    halt = false;
  
  deleteFromWorkingMemory(_wmc.address);
  

//   VisionData::DetectionCommandPtr cmd = new VisionData::DetectionCommand;
//   cmd->labels.push_back(obj->label);
//   addToWorkingMemory(newDataID(), cmd);
}

}

