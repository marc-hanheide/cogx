/**
 * @file StereoFlapDetectorDriver.cpp
 * @author Andreas Richtsfeld
 * @date September 2009
 * @brief Driver for detecting flaps with stereo for cogx implementation in cast.
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "StereoFlapDetectorDriver.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::StereoFlapDetectorDriver();
  }
}

using namespace cast;
using namespace std;

void StereoFlapDetectorDriver::configure(const map<string,string> & _config)
{
}

void StereoFlapDetectorDriver::start()
{
}

void StereoFlapDetectorDriver::runComponent()
{
  sleepProcess(1000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.
                    
  while(1)
  {   
    // Send start detection command
    VisionData::StereoFlapDetectionCommandPtr detect_cmd = new VisionData::StereoFlapDetectionCommand;
    detect_cmd->cmd = VisionData::SFSTART;
    addToWorkingMemory(newDataID(), detect_cmd);
    log("stereo flap detection start-command sent!");
  
    // Send start tracking command
    VisionData::TrackingCommandPtr tracker_cmd = new VisionData::TrackingCommand;
    tracker_cmd->cmd = VisionData::START;
    addToWorkingMemory(newDataID(), tracker_cmd);
    log("stereo flap tracking start command sent!");

    // Detect for some seconds
    sleepComponent(100000);
  
//     // Send stop detection command
//     detect_cmd->cmd = VisionData::SFSTOP;
//     addToWorkingMemory(newDataID(), detect_cmd);
//     log("stereo flap detection stop command sent!");

/*
    // Track for some seconds
    sleepComponent(10000);

    // Send start tracking command
    tracker_cmd->cmd = VisionData::STOP;
    addToWorkingMemory(newDataID(), tracker_cmd);
    log("stereo flap tracking stop command sent!");
*/

  }
}

void StereoFlapDetectorDriver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
}




