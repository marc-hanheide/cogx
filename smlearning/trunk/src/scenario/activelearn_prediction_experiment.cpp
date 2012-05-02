/** @file activelearn_prediction_experiment.cpp
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
 * a Substochastic Sequential Machine is used for prediction
 *
 * @author      Sergio Roa - DFKI
 * @version 2.0 beta
 *
 */

#include <scenario/PushingApplication.h>


using namespace smlearning;

int main(int argc, char *argv[]) {

	PushingApplication<PredictingActiveLearnScenario, PredictingActiveLearnScenario::Desc>().main(argc, argv);

	return 0;
}
