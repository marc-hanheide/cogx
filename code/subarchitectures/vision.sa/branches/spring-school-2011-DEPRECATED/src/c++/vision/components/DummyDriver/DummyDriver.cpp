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

}

