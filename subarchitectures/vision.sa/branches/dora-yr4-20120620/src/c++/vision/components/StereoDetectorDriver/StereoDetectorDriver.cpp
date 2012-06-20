/**
 * @file StereoDetectorDriver.cpp
 * @author Andreas Richtsfeld
 * @date October 2009
 * @brief Driver for detecting objects with a stereo rig for cogx project.
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "StereoDetectorDriver.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::StereoDetectorDriver();
  }
}

using namespace cast;
using namespace std;

void StereoDetectorDriver::configure(const map<string,string> & _config)
{
	detecting = false;
}

void StereoDetectorDriver::start()
{
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoDetectorDriver>(this, &StereoDetectorDriver::receiveVisualObject));
}

void StereoDetectorDriver::runComponent()
{
	sleepProcess(100);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.

// 	VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
// 	track_cmd->cmd = VisionData::START;
// 	addToWorkingMemory(newDataID(), track_cmd);
// 	log("tracking start-command sent!");
//   
// 	// Send start detection command
	VisionData::StereoDetectionCommandPtr detect_cmd = new VisionData::StereoDetectionCommand;
// 	detect_cmd->cmd = VisionData::SDSTART;
// 	addToWorkingMemory(newDataID(), detect_cmd);
// 	detecting = true;
// 	log("stereo detection start-command sent!");


	while(isRunning())
	{   
		// Sleep for some time
		sleepComponent(3000);

		if(!detecting)
		{
			// start single detection
			detect_cmd->cmd = VisionData::SDSINGLE;
			addToWorkingMemory(newDataID(), detect_cmd);
			log("single stereo detection command sent: SDSINGLE");

			sleepComponent(5000);

// 			// start single detection
// 			detect_cmd->cmd = VisionData::SDSINGLEHR;
// 			addToWorkingMemory(newDataID(), detect_cmd);
// 			log("single stereo detection command sent: SDSINGLEHR");
// 
// 			sleepComponent(5000);
		}
	}
}

void StereoDetectorDriver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
	// stop detection
	if(detecting)
	{
		// Send stop detection command
		VisionData::StereoDetectionCommandPtr detect_cmd = new VisionData::StereoDetectionCommand;
		detect_cmd->cmd = VisionData::SDSTOP;
		addToWorkingMemory(newDataID(), detect_cmd);
		detecting = false;
		log("stereo detection stop-command sent!");
	}
}




