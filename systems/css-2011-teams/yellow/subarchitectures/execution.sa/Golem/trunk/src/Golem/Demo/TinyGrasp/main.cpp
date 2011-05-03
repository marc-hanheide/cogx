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
		
		// setup obstacle
		Mat34 obstaclePose;
		obstaclePose.setId();
		obstaclePose.p.x = Real(0.6);
		obstaclePose.p.z = Real(0.48);
		Vec3 obstacleDimensions(Real(0.2), Real(0.3), Real(0.01));
		RigidBody* obstacle = robot.createObject(obstaclePose, obstacleDimensions);

		// setup object
		Mat34 objectPose;
		objectPose.setId();
		objectPose.R.rotY(0.3);
		objectPose.p.x = Real(0.55);
		objectPose.p.z = Real(0.6);
		Vec3 objectDimensions(Real(0.08), Real(0.02), Real(0.11));
		RigidBody* object = robot.createObject(objectPose, objectDimensions);

		robot.setGraspObject(object); // set object to be grasped
		GraspPose::Seq graspPoses = robot.getGraspPoses(); // read all grasp poses
		if (robot.graspTry(graspPoses)) { // test is the object is graspable
			if (robot.graspExec()) { // grasp the object, return false if grasping failed
				Mat34 target = robot.read();
				target.p.set(Real(0.4), -Real(0.4), Real(0.7));
				(void)robot.moveTry(target); // ignore returned actual/best target pose
				robot.moveExec(); // go to the last tried pose
				//PerfTimer::sleep(5.0); // wait for a while
				robot.graspRelease(); // and release the object
			}
		}
		else {
			Mat23Seq robotPoses = robot.getRobotPoses(graspPoses);
			// TODO move the robot to robotPoses, repeat grasping procedure
		}

		(void)robot.moveTry(home); // ignore returned actual/best target pose
		robot.moveExec(); // go to the last tried pose
	}
	catch (const Message& ex) {
		std::cerr << ex.str() << std::endl;
	}

	return 0;
}
