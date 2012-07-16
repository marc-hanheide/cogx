/** @file TrackerTest.cpp
 *
 *
 * Program which test the tracker thread
 * 
 * @author	Manuel Noll - DFKI
 * @version 1.0
 *
 */

#include <scenario/TrackerScenario.h>
// #include <scenario/TrackerThread.h>
#include <unistd.h>

using namespace smlearning;

int main(int argc, char *argv[]) {

	// TrackerThread th (std::string("tracking.ini"), std::string("cam.cal"), std::string("pose.cal"));
	
	// th.SetThreadType (ThreadTypeIntervalDriven,1000);
	// th.Event ();
	// th. run ();
	// th.start ();
	// sleep (2);
	// th.OnTask ();
	TrackerScenarioApp ().main(argc, argv);
	sleep (2);

	return 0;
}
