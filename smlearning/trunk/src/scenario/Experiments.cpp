/** @file Experiments.cpp
 *
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
 * @author	Manuel Noll - DFKI
 * @version 1.0
 *
 */

#include <scenario/PushingApplication.h>


using namespace smlearning;

int main(int argc, char *argv[]) {

	PushingApplication<Scenario, Scenario::Desc>().main(argc, argv);

	return 0;
}
