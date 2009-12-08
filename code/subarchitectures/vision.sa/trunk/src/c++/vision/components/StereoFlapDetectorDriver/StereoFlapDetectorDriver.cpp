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
  tracking = false;
	detecting = false;
  running = true;
	not_found = true;
}

void StereoFlapDetectorDriver::start()
{
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoFlapDetectorDriver>(this, &StereoFlapDetectorDriver::receiveVisualObject));
}

void StereoFlapDetectorDriver::runComponent()
{
  sleepProcess(1000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.

	if(!tracking)
	{
	  VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
		track_cmd->cmd = VisionData::START;
		addToWorkingMemory(newDataID(), track_cmd);
		log("tracking start-command sent!");
		tracking = true;
	}
	
	while(running)
	{
		sleepProcess(2000);	// detection time

		if(not_found)
		{
			// Send start detection command
			VisionData::StereoFlapDetectionCommandPtr detect_cmd = new VisionData::StereoFlapDetectionCommand;
			detect_cmd->cmd = VisionData::SFSINGLE;
			addToWorkingMemory(newDataID(), detect_cmd);
			log("single stereo flap detection command sent!");
		}
 	}

// 	// Send start detection command
// 	VisionData::StereoFlapDetectionCommandPtr detect_cmd = new VisionData::StereoFlapDetectionCommand;
// 	detect_cmd->cmd = VisionData::SFSTART;
// 	addToWorkingMemory(newDataID(), detect_cmd);
// 	detecting = true;
// 	log("stereo flap detection start-command sent!");
// 
// 	while(running)
// 	{
// 		sleepProcess(3000);	// detection time
// 
// 		if(!detecting)
// 		{
// 			// Send start detection command
// 			VisionData::StereoFlapDetectionCommandPtr detect_cmd = new VisionData::StereoFlapDetectionCommand;
// 			detect_cmd->cmd = VisionData::SFSINGLE;
// 			addToWorkingMemory(newDataID(), detect_cmd);
// 			log("single stereo flap detection command sent!");
// 		}
//  }
}

void StereoFlapDetectorDriver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
// 	// Send stop detection command
// 	VisionData::StereoFlapDetectionCommandPtr detect_cmd = new VisionData::StereoFlapDetectionCommand;
// 	detect_cmd->cmd = VisionData::SFSTOP;
// 	addToWorkingMemory(newDataID(), detect_cmd);
// 	detecting = false;
// 	log("stereo flap detection stop-command sent!");

	not_found = false;
}




