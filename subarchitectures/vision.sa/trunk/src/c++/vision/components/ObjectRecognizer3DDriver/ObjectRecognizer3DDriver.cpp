/**
 * @author Thomas Mörwald
 * @date February 2010
 *
 * Just a dummy component to drive some vision functionality.
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "ObjectRecognizer3DDriver.h"
#include <VideoUtils.h>

using namespace cogx;
using namespace Tracking;

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ObjectRecognizer3DDriver();
  }
}

using namespace cast;
using namespace std;
using namespace VisionData;

void ObjectRecognizer3DDriver::loadVisualModelToWM(std::string filename, std::string& modelID, Math::Pose3 pose){
 	log("Loading PLY model");
	ModelLoader modelloader;
	Model model;
	modelloader.LoadPly(model, filename.c_str());
	
	VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
  obj->model = new VisionData::GeometryModel;
	convertModel2Geometry(model, obj->model);
	obj->label = filename.c_str();
	obj->detectionConfidence = 0.0;
	obj->pose = pose;
	 
  log("Add model to working memory: '%s'", obj->label.c_str());
  modelID = newDataID();
  addToWorkingMemory(modelID, obj);
}

void ObjectRecognizer3DDriver::addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, std::string label, std::string visualObjectID){
	VisionData::Recognizer3DCommandPtr rec_cmd = new VisionData::Recognizer3DCommand;
  rec_cmd->cmd = cmd;
  rec_cmd->label = label;
  rec_cmd->visualObjectID = visualObjectID;
  addToWorkingMemory(newDataID(), rec_cmd);
}

void ObjectRecognizer3DDriver::configure(const map<string,string> & _config){
	map<string,string>::const_iterator it;
	
	if((it = _config.find("--labels")) != _config.end())
  {
    istringstream istr(it->second);
    string label;
    while(istr >> label)
      m_labels.push_back(label);

    ostringstream ostr;
    for(size_t i = 0; i < m_labels.size(); i++)
      ostr << " '" << m_labels[i] << "'";
    log("Recognizing objects: %s", ostr.str().c_str());
  }	
}

void ObjectRecognizer3DDriver::start(){

  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectRecognizer3DDriver>(this,
        &ObjectRecognizer3DDriver::receiveVisualObject));
  
}

void ObjectRecognizer3DDriver::runComponent(){
  sleepProcess(1000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.
  
  // Load PLY model to working memory
  std::string modelID;
//   loadVisualModelToWM(m_plyfile, modelID, Math::Pose3());
//   m_visualObjectIDs.push_back(modelID);
  
  // trigger Recognizer3D
  for(int i=0; i<m_labels.size(); i++){
  	addRecognizer3DCommand(RECLEARN, m_labels[i], modelID);
//   	addRecognizer3DCommand(RECOGNIZE, m_labels[i], modelID);
	}

}



void ObjectRecognizer3DDriver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc){
  VisionData::VisualObjectPtr obj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);

  
}




