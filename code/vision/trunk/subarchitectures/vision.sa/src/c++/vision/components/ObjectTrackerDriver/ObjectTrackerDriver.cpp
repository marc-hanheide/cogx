/* Dummy driver for the object tracker component
*
*  @author: Thomas Mörwald
*  @date: April 2009
*
*  This component is an example on how to control the
*  object tracker by loading a model from ply-file
*  to the working memory and calling several tracking commands.
*
*/

#include <cast/architecture/ChangeFilterFactory.hpp>
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

using namespace cast;

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
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<ObjectTrackerDriver>(this,
        &ObjectTrackerDriver::receiveVisualObject));
}

void ObjectTrackerDriver::runComponent()
{
  sleepProcess(1000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.
                       
  // Load geometry from ply-file
  log("loading model 'box_blender.ply'");
  m_model.load("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/model/jasmin.ply");
    
  // Generate VisualObject
  VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
  obj->model = new VisionData::GeometryModel;
  if(!convertTrackerModel(&m_model, obj->model))
	log("no geometry model in Visual Object");
  obj->label = "box";
  obj->detectionConfidence = 0.0;
  Particle p = Particle(0.0);
  convertParticle2Pose(p, obj->pose);  
  	
  // Add VisualObject to working memory
  log("add model to working memory: '%s'", obj->label.c_str());
  addToWorkingMemory(newDataID(), obj);
  
  // Send start tracking command
  log("send tracking command: START");
  VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
  track_cmd->cmd = VisionData::START;
  addToWorkingMemory(newDataID(), track_cmd);
  
  // Track for 10 seconds
  log("tracking 20 seconds");
  sleepComponent(120000);
  
  // Send stop tracking command
  log("send tracking command: STOP");
  track_cmd->cmd = VisionData::STOP;
  addToWorkingMemory(newDataID(), track_cmd);
  
}

void ObjectTrackerDriver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::VisualObjectPtr obj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
  
  /*
  if(obj->detectionConfidence >= 0.5)
    log("ok, detected '%s'", obj->label.c_str());
  else
    log("nah, did not detect '%s'", obj->label.c_str());
    */
}



