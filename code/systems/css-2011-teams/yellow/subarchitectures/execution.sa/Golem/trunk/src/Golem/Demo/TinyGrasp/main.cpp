/** @file main.cpp
 * 
 * Program demonstrating Tiny Golem interface
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Demo/TinyGrasp/TinyGrasp.h>
#include <iostream>

using namespace golem;
using namespace golem::tiny;

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	try {
		TinyGrasp robot("GolemDeviceKatana300Sim");
		
		const Mat34 home = robot.read();

		Mat34 globalPose;
		globalPose.setId();
		globalPose.R.rotX(2.1);
		globalPose.p.y = Real(0.55);
		globalPose.p.z = Real(0.45);
		Vec3 dimensions(Real(0.02), Real(0.08), Real(0.11));
		robot.setObject(globalPose, dimensions);

		GraspPose::Seq graspPoses = robot.getGraspPoses();
		GraspPose::Seq::const_iterator best;
		GraspPose actual = robot.graspTry(graspPoses, best);
		// TODO before execution, test here how good is the best grasp pose
		robot.graspExec(); // go to the last tried pose

		Mat34 target = home;
		target.p.set(Real(0.1), Real(0.6), Real(0.7));
		(void)robot.moveTry(target); // ignore returned actual/best target pose
		robot.moveExec(); // go to the last tried pose
		robot.graspRelease();

		(void)robot.moveTry(home); // ignore returned actual/best target pose
		robot.moveExec(); // go to the last tried pose
	}
	catch (const Message& ex) {
		std::cerr << ex.str() << std::endl;
	}

	return 0;
}
