/** @file Planner.cpp
 * 
 * Demonstration program which moves the arm to random poses with
 * trajectory planner and collision detection.
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the Katana arm simulator
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/XMLData.h>
#include <Golem/Phys/Application.h>
#include <Golem/Phys/Data.h>
#include <Golem/PhysCtrl/PhysPlanner.h>
#include <Golem/PhysCtrl/Creator.h>
#include <Golem/Demo/Common/Tools.h>

using namespace golem;

//------------------------------------------------------------------------------

void setupObjects(Scene* scene) {
	// Creator
	Creator creator(*scene);
	Actor::Desc *pActorDesc;
	
	// Create ground plane.
	pActorDesc = creator.createGroundPlaneDesc();
	scene->createObject(*pActorDesc);
	
	// "Left post"
	pActorDesc = creator.createBoxDesc(Real(0.05), Real(0.3), Real(0.25));
	pActorDesc->nxActorDesc.globalPose.t.set(NxReal(0.25), NxReal(-0.05), NxReal(0.25));
	scene->createObject(*pActorDesc);

	// "Right post"
	pActorDesc = creator.createBoxDesc(Real(0.07), Real(0.1), Real(0.25));
	pActorDesc->nxActorDesc.globalPose.t.set(NxReal(-0.15), NxReal(0.05), NxReal(0.25));
	scene->createObject(*pActorDesc);
	
	// "Crossbar"
	pActorDesc = creator.createBoxDesc(Real(0.3), Real(0.05), Real(0.025));
	pActorDesc->nxActorDesc.globalPose.t.set(NxReal(0.0), NxReal(0.15), NxReal(0.525));
	scene->createObject(*pActorDesc);
}

void setupPlanner(PhysPlanner::Desc &desc, XMLContext* xmlcontext, golem::Context* context) {
	// some planner parameter tuning
	desc.pPlannerDesc->pHeuristicDesc->distJointcoordMax[4] = Real(1.0)*REAL_PI;// last joint
}

//------------------------------------------------------------------------------

Real getLinearDist(const Vec3& v0, const Vec3& v1) {
	return v0.distance(v1);
}

Real getAngularDist(const Quat& q0, const Quat& q1) {
	const Real d = q0.dot(q1);
	return REAL_ONE - Math::abs(d);
}

//------------------------------------------------------------------------------

/** MyApplication */
class MyApplication : public Application {
protected:
	/** Runs MyApplication */
	virtual void run(int argc, char *argv[]) {
		printf("Use the arrow keys to move the camera.\n");
		printf("Use the mouse to rotate the camera.\n");
		printf("Press p to pause simulations.\n");
		printf("Press pgup/pgdn/space to switch between simulations.\n");
		printf("Press v to show Actors reference frames.\n");
		printf("Use z, x, c to change randering mode.\n");
		printf("Use F1-F12 to display program specific debug information:\n");
		printf("\tF1 to display/hide the current destination pose.\n");
		printf("\tF2 to display/hide the current trajectory.\n");
		printf("\tF3 to display/hide the goal/desired pose.\n");
		printf("\tF4 to display/hide the global waypoint graph nodes.\n");
		printf("\tF5 to display/hide the global waypoint path.\n");
		printf("\tF6 to display/hide the local waypoint graph nodes.\n");
		printf("\tF7 to display/hide the local waypoint path.\n");
		printf("\tF8 to display/hide the optimised waypoint path.\n");
		printf("Press esc to exit.\n");
		
		// Random number generator seed
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Random number generator seed %d", context()->getRandSeed()._U32[0]);

		// Get arm driver name
		std::string driver;
		XMLData("driver", driver, xmlcontext()->getContextFirst("arm"));
		// Load driver and setup planner
		PhysPlanner::Desc physPlannerDesc;
		physPlannerDesc.pArmDesc = Arm::Desc::load(*context(), driver);
		setupPlanner(physPlannerDesc, xmlcontext(), context());
		
		// Create PhysPlanner
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Initialising planner...");
		PhysPlanner *pPhysPlanner = dynamic_cast<PhysPlanner*>(scene()->createObject(physPlannerDesc));
		if (pPhysPlanner == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create Planner");
		
		// Display arm information
		armInfo(pPhysPlanner->getArm());

		// Scene objects setup
		setupObjects(scene());
		
		// Move the arm to random destination poses
		Rand rand(context()->getRandSeed());
		while (!universe()->interrupted()) {
			Planner &planner = pPhysPlanner->getPlanner();

			// generate random end pose
			golem::GenWorkspaceState end;
			// random position
			Vec3 position(
				rand.nextUniform(Real(-0.4), Real(+0.4)),
				rand.nextUniform(Real(-0.4), Real(+0.4)),
				rand.nextUniform(Real(+0.05), Real(+0.5))
			);
			// random orientation
			Vec3 orientation(
				rand.nextUniform(Real(-0.25)*REAL_PI, Real(+0.25)*REAL_PI),
				rand.nextUniform(Real(-0.25)*REAL_PI, Real(+0.25)*REAL_PI),
				-Math::atan2(position.v1, position.v2)// + rand.nextUniform(Real(-0.2)*REAL_PI, Real(+0.2)*REAL_PI)
			);
			fromCartesianPose(end.pos, position, orientation);
			end.vel.setZero();

			// arm state at a time t and finishes at some time later
			golem::GenConfigspaceState begin;
			pPhysPlanner->getArm().lookupCommand(begin, SEC_TM_REAL_INF); // last sent trajectory waypoint
			begin.vel.setZero();
			begin.acc.setZero();
			begin.t += pPhysPlanner->getArm().getTimeDeltaAsync(); // concatenate trajectories
			end.t = begin.t + SecTmReal(5.0); // minimum trajectory duration
			
			// Velocity and acceleration limits
			//planner.setVelocity(Real(1.0));
			//planner.setAcceleration(Real(1.0));
			
			// Main ON/OFF collision detection switch
			//planner.getHeuristic()->setCollisionDetection(false);
			
			// All bounds are treated as obstacles
			pPhysPlanner->setCollisionBoundsGroup(Bounds::GROUP_ALL);

			// find target
			GenConfigspaceState cend;
			if (!planner.findTarget(cend, begin, end)) {
				continue;
			}

			Planner::Trajectory trajectory;
			// find trajectory
			if (!planner.findTrajectory(trajectory, trajectory.begin(), begin, cend, &end)) {
				continue;
			}
			
			// wait until the arm stops moving
			PerfTimer::sleep(std::max(SEC_TM_REAL_ZERO, trajectory.begin()->t - context()->getTimer().elapsed()));
			
			// print pose error, note that the target is always computed with respect to the reference pose in the tool frame (the last joint)
			Mat34 actual;
			planner.getArm().forwardTransform(actual, cend.pos);
			actual.multiply(actual, planner.getArm().getReferencePose());
			context()->getMessageStream()->write(Message::LEVEL_INFO, "Pose error = (%f, %f)", getLinearDist(end.pos.p, actual.p), getAngularDist(Quat(actual.R), Quat(end.pos.R)));

			// send to the arm controller
			if (!planner.send<golem::GenConfigspaceState>(trajectory.begin(), trajectory.end())) {
				continue;
			}
		}

		context()->getMessageStream()->write(Message::LEVEL_INFO, "Good bye!");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
