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
using namespace Tracking;

void ObjectTrackerDriver::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
  if((it = _config.find("--model")) != _config.end())
  {
  	istringstream istr(it->second);
    istr >> model;
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
//   sleepComponent(1000);  // HACK: the nav visualisation might crash if we send it
//                        // object observations too soon.
//                        
//   // Load geometry from ply-file
//   log("loading model '%s'", model.c_str());
//   m_modelloader.LoadPly(m_model, model.c_str());
//     
//   // Generate VisualObject
//   VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
//   obj->model = new VisionData::GeometryModel;
//   if(!convertTrackerModel(&m_model, obj->model))
// 		log("no geometry model in Visual Object");
//   obj->label = model.c_str();
//   obj->detectionConfidence = 0.0;
//   Particle p = Particle(0.0);
//   p.translate(0.1, 0.1, 0.0);
// 	p.rotate(0.0,0.0,0.78);
//   convertParticle2Pose(p, obj->pose);  
//   	
//   // Add VisualObject to working memoryabs(trajectory[i].rot.m00 - obj->pose.rot.m00) +
//   log("add model to working memory: '%s'", obj->label.c_str());
//   addToWorkingMemory(newDataID(), obj);
//   
//   sleepComponent(1000);
//   
//   // Send start tracking command
//   log("send tracking command: START");
//   VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
//   track_cmd->cmd = VisionData::START;
//   addToWorkingMemory(newDataID(), track_cmd);
  
}

void ObjectTrackerDriver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  
}



