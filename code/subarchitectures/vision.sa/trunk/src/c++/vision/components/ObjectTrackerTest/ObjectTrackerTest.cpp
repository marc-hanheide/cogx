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
#include "ObjectTrackerTest.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ObjectTrackerTest();
  }
}

using namespace cast;

using namespace std;

void ObjectTrackerTest::configure(const map<string,string> & _config)
{
    
  timerstarted = false;
  m_error_pos = 0.0;
  m_error_rot = 0.0;
  
}

void ObjectTrackerTest::start()
{
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<ObjectTrackerTest>(this,
        &ObjectTrackerTest::receiveVisualObject));
}

void ObjectTrackerTest::runComponent()
{
  int id=0;
  
  sleepProcess(1000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.
             
  // Load geometry
	
  
  // Generate VisualObject
  VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
  obj->model = new VisionData::GeometryModel;
  if(!convertTrackerModel(m_model, obj->model))
	log("no geometry model in Visual Object");
  obj->label = "red box";
  obj->detectionConfidence = 0.0;
  Particle p = Particle(0.0);
  convertParticle2Pose(p, obj->pose);  
  	
  // Add VisualObject to working memoryabs(trajectory[i].rot.m00 - obj->pose.rot.m00) +
  log("add model to working memory: '%s'", obj->label.c_str());
  addToWorkingMemory(newDataID(), obj);
  
  sleepProcess(1000);
  
  // Send start tracking command
  log("send tracking command: START");
  VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
  track_cmd->cmd = VisionData::START;
  addToWorkingMemory(newDataID(), track_cmd);
  
  // Track for 10 seconds
  log("tracking 20 seconds (20 images @ 10Hz)");
  sleepComponent(20000);
  
  // Send stop tracking command
  log("send tracking command: STOP");
  track_cmd->cmd = VisionData::STOP;
  addToWorkingMemory(newDataID(), track_cmd);
  
  sleepComponent(1000);
  
  delete(g_Resources);
}

void ObjectTrackerTest::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::VisualObjectPtr obj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
  trajectory.push_back(obj->pose);
  
  
}



