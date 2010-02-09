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
 * @author      Sergio Roa - DFKI
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk/~msk">The University Of Birmingham</A>
 * @author      Jan Hanzelka - DFKI
 * @version 1.0
 *
 */

#include "metalearning/Scenario.h"

using namespace smlearning;

int main(int argc, char *argv[]) {

	string arg1 = string("/usr/local/bin/SMLearning/offline_experiment.xml");
	if (argc == 2) {
		for (int i=1; i<=18; i++) {
			vector<string> args;
			args.push_back (string(argv[0]));
			args.push_back (arg1);
			args.push_back (string(argv[1]));
			std::stringstream out;
			out << i;
			string s = out.str();
			args.push_back (s);
			char** newargv = new char*[args.size()];
			for (int i=0; i<args.size(); i++) {
				newargv[i] = new char[args[i].size()+1];
				strncpy(newargv[i], args[i].c_str(), args[i].size());
				newargv[i][args[i].size()] = '\0';
			}
			PushingApplication().main(argc+1, newargv);
			// free dynamic memory
			for (int i=0; i<args.size(); i++)
				delete[] newargv[i];
			delete[] newargv;
			args.clear();
		}
	}

			
	return 0;
}
