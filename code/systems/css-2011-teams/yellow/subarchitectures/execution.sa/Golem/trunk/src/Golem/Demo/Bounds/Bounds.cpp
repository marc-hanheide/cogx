/** @file Bounds.cpp
 * 
 * Program demonstrates how to add/remove/modify shape of arm joints.
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

//#include <Golem/Tools/Debug.h>
#include <Golem/Ctrl/Arm.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Phys/Application.h>
#include <Golem/Phys/Data.h>
#include <Golem/PhysCtrl/PhysReacPlanner.h>
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
//	pActorDesc = creator.createBoxDesc(Real(0.07), Real(0.1), Real(0.25));
//	pActorDesc->nxActorDesc.globalPose.t.set(NxReal(-0.15), NxReal(0.05), NxReal(0.25));
//	scene->createObject(*pActorDesc);
	
	// "Crossbar"
//	pActorDesc = creator.createBoxDesc(Real(0.3), Real(0.05), Real(0.025));
//	pActorDesc->nxActorDesc.globalPose.t.set(NxReal(0.0), NxReal(0.15), NxReal(0.525));
//	scene->createObject(*pActorDesc);
}

//------------------------------------------------------------------------------

template <typename _PVEC, typename _MAT> void trn(_PVEC begin, _PVEC end, const _MAT &pose) {
	for (_PVEC v = begin; v != end; ++v) pose.multiply(*v, *v);
}

// create mace
void createMace(std::vector<Bounds::Desc::Ptr> &boundsDescSeq, Mat34 &referencePose, const Mat34 &pose) {
	// mace characteristic dimensions
	const Real length = Real(0.15);
	const Real begin = Real(0.01);
	const Real end = Real(0.025);
	const Real diam = Real(0.02);
	const Real height = Real(0.05);
	const Real pos = length - end;

	// reference pose at the end (on Y-axis)
	referencePose.setId();
	referencePose.p.v2 += length;
	referencePose.multiply(pose, referencePose);
	
	boundsDescSeq.clear();

	// objects data
	BoundingConvexMesh::Desc hilt;
	hilt.vertices.resize(8);
	hilt.bCook = true;
	BoundingConvexMesh::Desc spike;
	spike.vertices.resize(5);
	spike.bCook = true;

	// create hilt (along Y-axis)
	hilt.vertices[0].set(-begin, Real(0.0), -begin);
	hilt.vertices[1].set(+begin, Real(0.0), -begin);
	hilt.vertices[2].set(-begin, Real(0.0), +begin);
	hilt.vertices[3].set(+begin, Real(0.0), +begin);
	hilt.vertices[4].set(-end, length, -end);
	hilt.vertices[5].set(+end, length, -end);
	hilt.vertices[6].set(-end, length, +end);
	hilt.vertices[7].set(+end, length, +end);
	trn(hilt.vertices.begin(), hilt.vertices.end(), pose);
	boundsDescSeq.push_back(hilt.clone());

	// create spike #1
	spike.vertices[0].set(-diam, pos - diam, end);
	spike.vertices[1].set(+diam, pos - diam, end);
	spike.vertices[2].set(-diam, pos + diam, end);
	spike.vertices[3].set(+diam, pos + diam, end);
	spike.vertices[4].set(Real(0.0), pos, end + height);
	trn(spike.vertices.begin(), spike.vertices.end(), pose);
	boundsDescSeq.push_back(spike.clone());
	
	// create spike #2
	spike.vertices[0].set(-diam, pos - diam, -end);
	spike.vertices[1].set(+diam, pos - diam, -end);
	spike.vertices[2].set(-diam, pos + diam, -end);
	spike.vertices[3].set(+diam, pos + diam, -end);
	spike.vertices[4].set(Real(0.0), pos, - end - height);
	trn(spike.vertices.begin(), spike.vertices.end(), pose);
	boundsDescSeq.push_back(spike.clone());
	
	// create spike #3
	spike.vertices[0].set(end, pos - diam, -diam);
	spike.vertices[1].set(end, pos - diam, +diam);
	spike.vertices[2].set(end, pos + diam, -diam);
	spike.vertices[3].set(end, pos + diam, +diam);
	spike.vertices[4].set(end + height, pos, Real(0.0));
	trn(spike.vertices.begin(), spike.vertices.end(), pose);
	boundsDescSeq.push_back(spike.clone());
	
	// create spike #4
	spike.vertices[0].set(-end, pos - diam, -diam);
	spike.vertices[1].set(-end, pos - diam, +diam);
	spike.vertices[2].set(-end, pos + diam, -diam);
	spike.vertices[3].set(-end, pos + diam, +diam);
	spike.vertices[4].set(-end - height, pos, Real(0.0));
	trn(spike.vertices.begin(), spike.vertices.end(), pose);
	boundsDescSeq.push_back(spike.clone());
}

// create finger
void createFinger(std::vector<Bounds::Desc::Ptr> &boundsDescSeq, Mat34 &referencePose, const Mat34 &pose) {
	// finger characteristic dimensions
	const Real length = Real(0.1);
	const Real begin = Real(0.015);
	const Real end = Real(0.01);

	// reference pose at the end (on Y-axis)
	referencePose.setId();
	referencePose.p.v2 += length;
	referencePose.multiply(pose, referencePose);
	
	boundsDescSeq.clear();
	
	// objects data
	BoundingConvexMesh::Desc finger;
	finger.vertices.resize(8);
	finger.bCook = true;

	// create finger (along Y-axis)
	finger.vertices[0].set(-begin, Real(0.0), -begin);
	finger.vertices[1].set(+begin, Real(0.0), -begin);
	finger.vertices[2].set(-begin, Real(0.0), +begin);
	finger.vertices[3].set(+begin, Real(0.0), +begin);
	finger.vertices[4].set(-end, length, -end);
	finger.vertices[5].set(+end, length, -end);
	finger.vertices[6].set(-end, length, +end);
	finger.vertices[7].set(+end, length, +end);
	trn(finger.vertices.begin(), finger.vertices.end(), pose);
	boundsDescSeq.push_back(finger.clone());
}

//------------------------------------------------------------------------------

// Add new bounds to the Joint Actor.
void addBounds(Actor* actor, Bounds::ConstSeq &boundsSeq, const Bounds::Desc::Seq &boundsDescSeq) {
	ASSERT(actor)
	boundsSeq.clear();
	for (Bounds::Desc::Seq::const_iterator i = boundsDescSeq.begin(); i != boundsDescSeq.end(); i++)
		boundsSeq.push_back(actor->createBounds(*i));
}

// Remove bounds of the Joint Actor.
void removeBounds(Actor* actor, Bounds::ConstSeq &boundsSeq) {
	ASSERT(actor)
	for (Bounds::ConstSeq::const_iterator i = boundsSeq.begin(); i != boundsSeq.end(); i++)
		actor->releaseBounds(**i);
}

//------------------------------------------------------------------------------

void setupPlanner(PhysReacPlanner::Desc &desc, XMLContext* xmlcontext, golem::Context* context) {
	// some planner parameter tuning
	desc.pPlannerDesc->pHeuristicDesc->distJointcoordMax[4] = Real(1.0)*REAL_PI;// last joint
	// Disable signal synchronization
	desc.pReacPlannerDesc->signalSync = false;
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
		PhysReacPlanner::Desc physReacPlannerDesc;
		physReacPlannerDesc.pArmDesc = Arm::Desc::load(*context(), driver);
		setupPlanner(physReacPlannerDesc, xmlcontext(), context());

		// Create PhysReacPlanner
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Initialising reactive planner...");
		PhysReacPlanner *pPhysReacPlanner = dynamic_cast<PhysReacPlanner*>(scene()->createObject(physReacPlannerDesc));
		if (pPhysReacPlanner == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create ReacPlanner");
		
		// some useful pointers
		ReacPlanner &reacPlanner = pPhysReacPlanner->getReacPlanner();
		Planner &planner = pPhysReacPlanner->getPlanner();
		Arm &arm = pPhysReacPlanner->getArm();
		
		// Display arm information
		armInfo(arm);

		// Scene objects setup
		setupObjects(scene());

		// Create bounds to be attached to the end-effector (the last joint) 
		Bounds::Desc::Seq boundsDescSeq;
		const Mat34 initReferencePose = arm.getReferencePose();
		Mat34 referencePose;
		createMace(boundsDescSeq, referencePose, initReferencePose);
		//createFinger(boundsDescSeq, referencePose, initReferencePose);
		// set new arm reference pose
		arm.setReferencePose(referencePose);
		// Modify shape of the joint by adding new bounds to the Actor representing the end-effector.
		Bounds::ConstSeq boundsSeq;
		addBounds(pPhysReacPlanner->getJointActors().back(), boundsSeq, boundsDescSeq);

		// (Synchronous) Time Delta - a minimum elapsed time between two consecutive waypoints sent to the controller
		SecTmReal timeDelta = reacPlanner.getTimeDelta();
		// Asynchronous Time Delta - a minimum elapsed time between the current time and the first waypoint sent to the controller
		// (no matter what has been sent before)
		SecTmReal timeDeltaAsync = reacPlanner.getTimeDeltaAsync();
		
		// Define the initial pose in the Cartesian workspace
		Vec3 position(Real(0.1), Real(0.2), Real(0.1));
		Vec3 orientation(Real(-0.5)*REAL_PI, Real(0.0), Real(0.0));
		// and set target waypoint
		golem::GenWorkspaceState target;
		fromCartesianPose(target.pos, position, orientation);
		target.vel.setZero(); // it doesn't move
		target.t = context()->getTimer().elapsed() + timeDeltaAsync + SecTmReal(5.0); // i.e. the movement will last at least 5 sec
		
		// set the initial pose of the arm, force the global movement (with planning in the entire arm workspace)
		reacPlanner.send(target, ReacPlanner::ACTION_GLOBAL);
		// wait for completion of the action (until the arm moves to the initial pose)
		reacPlanner.waitForEnd();

		//bool bPresent = true;
		//U32 n = 0;

		// Move the arm to random destination poses,
		// plan entire path if the distance between current pose and the target pose is larger than a threshold,
		// otherwise use local planner
		Rand rand(context()->getRandSeed());
		while (!universe()->interrupted()) {
			// incrementaly generate next random target pose in Cartesian coordinates
			// random position
			position.set(
				Math::clamp(position.v1 + rand.nextUniform(Real(-0.15), Real(+0.15)), Real(-0.4), Real(+0.4)),
				Math::clamp(position.v2 + rand.nextUniform(Real(-0.15), Real(+0.15)), Real(-0.4), Real(+0.4)),
				Math::clamp(position.v3 + rand.nextUniform(Real(-0.15), Real(+0.15)), Real(+0.05), Real(+0.5))
			);
			// random orientation
			orientation.set(
				Math::clamp(orientation.v1 + rand.nextUniform(Real(-0.1)*REAL_PI, Real(+0.1)*REAL_PI), Real(-0.5)*REAL_PI, Real(+0.25)*REAL_PI),
				Math::clamp(orientation.v2 + rand.nextUniform(Real(-0.1)*REAL_PI, Real(+0.1)*REAL_PI), Real(-0.5)*REAL_PI, Real(+0.25)*REAL_PI),
				-Math::atan2(position.v1, position.v2)// + rand.nextUniform(Real(-0.1)*REAL_PI, Real(+0.1)*REAL_PI)
			);
			
			// setup target waypoint 
			fromCartesianPose(target.pos, position, orientation);
			target.vel.setZero(); // it doesn't move
			target.t = context()->getTimer().elapsed() + timeDeltaAsync; // fast movement

			// Maximum distance without global re-planning
			//reacPlanner.setDistLinearMax(Real(0.2));
			//reacPlanner.setDistAngularMax(Real(0.5));
			
			// ON/OFF collision detection
			//planner.getHeuristic()->setCollisionDetection(false);
			// Velocity and acceleration limits
			//planner.setVelocity(Real(1.0));
			//planner.setAcceleration(Real(1.0));
			
			// send target pose, automatically decide wheter to use local or global planner
			if (!reacPlanner.send(target)) {
				continue;
			}
			
			// wait some random amount of time
			PerfTimer::sleep(rand.nextUniform(Real(1.0), Real(4.0)));
		}

		context()->getMessageStream()->write(Message::LEVEL_INFO, "Good bye!");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
