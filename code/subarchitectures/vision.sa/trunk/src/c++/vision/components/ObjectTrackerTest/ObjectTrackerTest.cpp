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

using namespace Tracking;
using namespace cast;
using namespace std;

void ObjectTrackerTest::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
  
  if((it = _config.find("--plyfile")) != _config.end()){
		m_plyfile = it->second;
		log("ply file: '%s'", m_plyfile.c_str());
	}
  
}

void ObjectTrackerTest::start()
{
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<ObjectTrackerTest>(this,
        &ObjectTrackerTest::receiveVisualObject));
}

void ObjectTrackerTest::runComponent()
{
  // Load geometry
  log("Loading ply model");
	ModelLoader modelloader;
	Model model;
	modelloader.LoadPly(model, m_plyfile.c_str());
	
	VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
  obj->model = new VisionData::GeometryModel;
	convertModel2Geometry(model, obj->model);
	obj->label = "Testobject";
	obj->detectionConfidence = 0.0;
	Tracking::Pose tPose;	
	tPose.translate(0.0,0.0,0.05);
	convertParticle2Pose(tPose, obj->pose); 
	 
  log("Add model to working memory: '%s'", obj->label.c_str());
  addToWorkingMemory(newDataID(), obj);
  
  sleepProcess(1000);
  
  log("Send tracking command: START");
  VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
  track_cmd->cmd = VisionData::START;
  addToWorkingMemory(newDataID(), track_cmd);

//   
//   // Track for 10 seconds
//   log("tracking 20 seconds (20 images @ 10Hz)");
//   sleepComponent(20000);
//   
//   // Send stop tracking command
//   log("send tracking command: STOP");
//   track_cmd->cmd = VisionData::STOP;
//   addToWorkingMemory(newDataID(), track_cmd);
//   
//   sleepComponent(1000);
//   
//   delete(g_Resources);
}

void ObjectTrackerTest::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
//   VisionData::VisualObjectPtr obj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
}



