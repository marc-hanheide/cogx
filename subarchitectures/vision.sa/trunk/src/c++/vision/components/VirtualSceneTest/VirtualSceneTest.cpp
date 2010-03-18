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
#include "VirtualSceneTest.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::VirtualSceneTest();
  }
}

using namespace cast;
using namespace std;

void VirtualSceneTest::configure(const map<string,string> & _config){
  map<string,string>::const_iterator it;
  if((it = _config.find("--model")) != _config.end())
  {
    istringstream istr(it->second);
    string filename;
    while(istr >> filename){
      m_modelfile.push_back(filename);
    }

    ostringstream ostr;
    for(size_t i = 0; i < m_modelfile.size(); i++)
      ostr << " '" << m_modelfile[i] << "'";
    log("Recognizing objects: %s", ostr.str().c_str());
  }	
  
}

void VirtualSceneTest::start(){
//   addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
//       new MemberFunctionChangeReceiver<VirtualSceneTest>(this,
//         &VirtualSceneTest::receiveVisualObject));
}

void VirtualSceneTest::runComponent(){
  sleepComponent(1000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.
  
  for(int i=0; i<m_modelfile.size(); i++){
		VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
		obj->model = new VisionData::GeometryModel;
		Tracking::Model model;
		
		// Load geometry from ply-file
		log("Loading model '%s'", m_modelfile[i].c_str());
		m_modelloader.LoadPly(model, m_modelfile[i].c_str());
  	
  	// Convert Model
  	log("Converting model to geometry");
  	convertModel2Geometry(model, obj->model);
  	
  	// 
  	obj->label = m_modelfile[i].c_str();
		obj->detectionConfidence = 0.0;
		Tracking::Pose p;
		p.translate(0.1+0.5*rand()/RAND_MAX, 0.1+0.5*rand()/RAND_MAX, 0.0);
		p.rotate(0.0,0.0,2*PI*rand()/RAND_MAX);
		convertParticle2Pose(p, obj->pose); 
		
		log("Add model to working memory: '%s'", obj->label.c_str());
  	addToWorkingMemory(newDataID(), obj);
  }
  
  
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

//   
//   sleepComponent(1000);
//   
//   // Send start tracking command
//   log("send tracking command: START");
//   VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
//   track_cmd->cmd = VisionData::START;
//   addToWorkingMemory(newDataID(), track_cmd);
  
}

void VirtualSceneTest::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc){
  
}



