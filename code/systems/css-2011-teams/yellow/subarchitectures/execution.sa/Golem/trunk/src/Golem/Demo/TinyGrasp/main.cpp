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
		
		const Mat34 home = robot.read(); // set current pose as home
		
		// setup object
		Mat34 globalPose;
		globalPose.setId();
		globalPose.R.rotY(1.1);
		globalPose.p.x = Real(0.6);
		globalPose.p.z = Real(0.6);
		Vec3 dimensions(Real(0.08), Real(0.02), Real(0.11));
		RigidBody* object = robot.createObject(globalPose, dimensions);

		robot.setGraspObject(object); // set object to be grasped
		GraspPose::Seq graspPoses = robot.getGraspPoses(); // read all grasp poses
		std::pair<GraspPose, GraspPose::Seq::const_iterator> actual = robot.graspTry(graspPoses); // find the best one
		// TODO before execution, test here how good is the best grasp pose
		robot.graspExec(); // go to the last tried pose

		Mat34 target = robot.read();
		target.p.set(Real(0.4), -Real(0.4), Real(0.7));
		(void)robot.moveTry(target); // ignore returned actual/best target pose
		robot.moveExec(); // go to the last tried pose
		//PerfTimer::sleep(5.0); // wait for a while
		robot.graspRelease(); // and release the object

		(void)robot.moveTry(home); // ignore returned actual/best target pose
		robot.moveExec(); // go to the last tried pose
	}
	catch (const Message& ex) {
		std::cerr << ex.str() << std::endl;
	}

	return 0;
}
