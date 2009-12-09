/** @file ensemble_off_experiment.cpp
 *
 * vHanz02
 *
 * Program which moves the arm along a straight line
 * using reactive trajectory planner and collision detection
 * simulating a pushing action on a polyflap
 * (also see DemoReacPlannerPhys and DemoRobotFinger).
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the arm simulators
 *
 * data are stored that can be used for offline training RNNs
 *
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk/~msk">The University Of Birmingham</A>
 * @author      Sergio Roa - DFKI
 * @author      Jan Hanzelka - DFKI
 * @version 1.0
 *
 */

#include "metalearning/Scenario.h"

using namespace smlearning;

int main(int argc, char *argv[]) {

	try {
		Scenario learningScenario;
		//if (!learningScenario.setup (argc, argv))
		//return 1;
		string arg = "/usr/local/bin/SMLearning/offline_experiment";
		strcpy (argv[0], arg.c_str());
		if (argc == 2) {
			for (int i=1; i<=18; i++)
				if (!learningScenario.runSimulatedOfflineExperiment (argc, argv, atoi(argv[1]), i))
					return 1;
		}
			

	}
	catch (const MsgStreamFile &msg) {
		std::cout << msg << std::endl;
	}
	catch (const std::exception &ex) {
		std::cout << Message(Message::LEVEL_CRIT, "C++ exception: %s", ex.what()) << std::endl;
	}

	return 0;
}
