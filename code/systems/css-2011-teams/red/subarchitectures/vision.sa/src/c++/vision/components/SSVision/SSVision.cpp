/**
 * @author Andreas Richtsfeld
 * @date April 2011
 * @brief Vision system for the spring school system
 */

#include "SSVision.h"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "cogxmath.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::SSVision();
  }
}

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;

static double CONFIDENCE_THRESHOLD = 0.2;

SSVision::SSVision()                  /// TODO Add the other labels!
{
  stopRecognizeObjects = false;
  stopDetectingTables = false;
  objectFound = false;
  objectFoundID = "";
  
  std::string label = "chocos";
  labels.push_back(label);
  label = "weetabix";
  labels.push_back(label);
}

/**
 * @brief Configure
 * @param _config Configuration
 */
void SSVision::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
}


/**
 * @brief start component
 */
void SSVision::start()
{
  // add change filter for vision commands
  addChangeFilter(createLocalTypeFilter<VisionData::SSVisionCommand>(cdl::ADD),
    new MemberFunctionChangeReceiver<SSVision>(this, &SSVision::receivedVisionCommand));

  // add change filter for ProtoObject changes
  addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject>(cdl::ADD),
    new MemberFunctionChangeReceiver<SSVision>(this, &SSVision::newVisualObject));

//   // add change filter for ProtoObject changes
//   addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
//     new MemberFunctionChangeReceiver<ManipulationPlanner>(this, &SSVision::newVisualObject));
}

/**
 * @brief runComponent
 */
void SSVision::runComponent()
{}

void SSVision::receivedVisionCommand(const cdl::WorkingMemoryChange & _wmc)
{
  log("detection command received");

  VisionData::SSVisionCommandPtr command = getMemoryEntry< VisionData::SSVisionCommand>(_wmc.address);
  
  if(command->cmd == VisionData::SSVRECOGNIZE)
    RecognizeObject();
  
  if(command->cmd == VisionData::SSVDETECTTABLE)
    stopRecognizeObjects = true;

  if(command->cmd == VisionData::SSVDETECTTABLE)
    DetectTable();
}


void SSVision::RecognizeObject()
{
  log("recognize objects start");
  while(!objectFound || stopRecognizeObjects)
  {
    // start objectrecognizer and wait for object???
    std::string objectID = newDataID();
    VisionData::DetectionCommandPtr command = new VisionData::DetectionCommand;
    command->labels = labels;
    addToWorkingMemory(objectID, "vision.sa", command);
    sleepComponent(1000);
  }  
  
  log("recognition ended: send command back");
  if(stopRecognizeObjects) WriteCommandToWM(0, false, objectFoundID);
  else WriteCommandToWM(0, true, objectFoundID);
}


void SSVision::DetectTable()
{
  log("detect now a table");
  
  // start planePopOut and wait for SOI???

  
  log("detection ended: send command back");
  WriteCommandToWM(0, true, objectFoundID);
}


void SSVision::newVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  log("new visual object received: check if correct one?");
  VisionData::VisualObjectPtr newobj = getMemoryEntry< VisionData::VisualObject>(_wmc.address);

  if(newobj->detectionConfidence > CONFIDENCE_THRESHOLD)
  {
    log("object recognized");
    objectFound = true;
    objectFoundID = _wmc.address.id;
  }
  else
  {
    // delete visual object
    log("delete visual object: confidence value is too low");
    deleteFromWorkingMemory(_wmc.address.id);
  }
}


/**
 * @brief Send command back
 * cmd 0 ... SSVRECOGNIZEEND
 * cmd 1 ... SSVDETECTTABLEEND
 */
void SSVision::WriteCommandToWM(int cmd, bool succeed, std::string id)
{
  log("write command to wm: %i", cmd);
  
  if (cmd == 0) // Recognition ended
  {
    std::string objectID = newDataID();
    VisionData::SSVisionCommandPtr command = new VisionData::SSVisionCommand;
    command->cmd = VisionData::SSVRECOGNIZEEND;
    command->succeed = succeed;
    command->objID = id;
    addToWorkingMemory(objectID, "vision.sa", command);
  }
  else if (cmd == 1) // Table detection ended
  {
    std::string objectID = newDataID();
    VisionData::SSVisionCommandPtr command = new VisionData::SSVisionCommand;
    command->cmd = VisionData::SSVDETECTTABLEEND;
    command->succeed = succeed;
    command->objID = id;
    addToWorkingMemory(objectID, "vision.sa", command);
  }
  sleepComponent(200);

  log("wrote command to wm");
}

}









