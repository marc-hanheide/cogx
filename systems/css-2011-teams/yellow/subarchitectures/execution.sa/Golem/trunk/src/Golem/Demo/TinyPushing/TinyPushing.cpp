/** @file TinyPushing.cpp
 * 
 * Program demonstrating Tiny Golem interface
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tiny/Tiny.h>
#include <Golem/Math/Rand.h>
#include <Golem/Math/Quat.h>
#include <iostream>

using namespace golem;
using namespace golem::tiny;

//------------------------------------------------------------------------------

double getLinearDist(const Vec3& v0, const Vec3& v1) {
	return v0.distance(v1);
}

double getAngularDist(const Quat& q0, const Quat& q1) {
	const double d = q0.dot(q1);
	return REAL_ONE - ::fabs(d);
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	try {
		Tiny tiny(argc, argv);

		// create arm
		//KatanaArmDesc* pArmDesc = new KatanaArmDesc; // specialised Katana 300/450 description
		ArmDesc* pArmDesc = new ArmDesc; // generic description
		pArmDesc->path = "GolemDeviceKatana300Sim"; // specify driver path
		tiny.print("Creating the arm...");
		Arm* pArm = dynamic_cast<Arm*>(tiny.createActor(ActorDescPtr(pArmDesc)));

		// attached a finger to the end-effector (the last joint)
		Joint* pEffector = pArm->getJoints().back();
		// get the end-effector reference pose
		Mat34 referencePose = pArm->getReferencePose();
		// construct a finger from a box and a sphere in local end-effector coordinates (arm at reference configuration stretched along Y-axis)
		const double fingerLength = 0.1;
		const double fingerDiam = 0.005;
		const double fingerTipRadius = 0.015;
		BoxShapeDesc* pFingerRodShapeDesc = new BoxShapeDesc;
		pFingerRodShapeDesc->dimensions.set(Real(fingerDiam/2.0), Real(fingerLength/2.0), Real(fingerDiam/2.0));
		pFingerRodShapeDesc->localPose = referencePose;
		pFingerRodShapeDesc->localPose.p.v2 += Real(fingerLength/2.0);
		Shape* pFingerRodShape = pEffector->createShape(ShapeDescPtr(pFingerRodShapeDesc));
		SphereShapeDesc* pFingerTipShapeDesc = new SphereShapeDesc;
		pFingerTipShapeDesc->radius = fingerTipRadius;
		pFingerTipShapeDesc->localPose = referencePose;
		pFingerTipShapeDesc->localPose.p.v2 += Real(fingerLength);
		Shape* pFingerTipShape = pEffector->createShape(ShapeDescPtr(pFingerTipShapeDesc));
		// change reference pose, so the end-effector pose will be further referred to the finger tip
		referencePose.p.v2 += Real(fingerLength);
		pArm->setReferencePose(referencePose);

		// trajectory
		GenConfigspaceStateSeq trajectory;
		// movement begin and end position in join configuration space
		GenConfigspaceState cbegin, cend;
		// initial configuration (it is the current joint configuration)
		GenConfigspaceState initial = pArm->recvGenConfigspaceState(tiny.getTime());

		// setup home end-effector pose (the joint configuration is not known)
		GenWorkspaceState home;
		home.pos.R.rotX(Real(-0.5)*REAL_PI); // end-effector pointing downwards
		home.pos.p.set(Real(0.0), Real(0.12), Real(0.1));
		
		// create ground plane using plane shape
		RigidBodyDesc* pGroundPlaneDesc = new RigidBodyDesc;
		PlaneShapeDesc* pGroundPlaneShapeDesc = new PlaneShapeDesc;
		pGroundPlaneDesc->shapes.push_back(ShapeDescPtr(pGroundPlaneShapeDesc));
		RigidBody* pGroundPlane = dynamic_cast<RigidBody*>(tiny.createActor(ActorDescPtr(pGroundPlaneDesc)));

		// Object pointer
		RigidBody* pObject = NULL;

		// Random number generator
		Rand frand;
		frand.setRandSeed(RandSeed());
		
		// Experiment main loop
		while (!tiny.interrupted()) {
			// setup movement to home position and compute movement end/target in joint configuration space
			cbegin = pArm->recvGenConfigspaceState(tiny.getTime());
			home.t = cbegin.t + pArm->getTimeDeltaAsync() + 1.0; // movement will last no shorter than 1 sec
			cend = pArm->findTarget(cbegin, home);
			// compute trajectory using path planning with collision detection
			trajectory = pArm->findTrajectory(cbegin, cend);
			// move the arm and wait until it stops
			tiny.print("Moving to home position...");
			pArm->send(trajectory, numeric_const<double>::INF);

			// setup an object (polyflap) as a set of two boxes
			RigidBodyDesc* pObjectDesc = new RigidBodyDesc;
			const double objectWidthY = 0.07;
			const double objectWidthZ = 0.07;
			const double objectLength = 0.07;
			const double objectHeight = 0.07;
			const double objectAngle = numeric_const<double>::PI_2;
			const double objectThickness = 0.0001;
			// Y-up shape
			BoxShapeDesc* pYShapeDesc = new BoxShapeDesc;
			pYShapeDesc->dimensions.set(Real(objectWidthY), Real(objectLength), Real(objectThickness));
			pYShapeDesc->localPose.p.set(Real(0.0), Real(objectLength), Real(objectThickness));
			pObjectDesc->shapes.push_back(ShapeDescPtr(pYShapeDesc));
			// Z-up shape
			BoxShapeDesc* pZShapeDesc = new BoxShapeDesc;
			double objectSin, objectCos;
			Math::sinCos(objectAngle, objectSin, objectCos);
			pZShapeDesc->dimensions.set(Real(objectWidthZ), Real(objectHeight), Real(objectThickness));
			pZShapeDesc->localPose.p.set(Real(0.0), Real(objectCos*objectHeight), Real(objectSin*objectHeight + objectThickness));
			pZShapeDesc->localPose.R.rotX(objectAngle);
			pObjectDesc->shapes.push_back(ShapeDescPtr(pZShapeDesc));
			// global pose
			pObjectDesc->globalPose.p.v2 += Real(0.25);
			pObjectDesc->globalPose.R.rotZ(REAL_2_PI*frand.nextUniform<Real>());
			// delete previous object, create a new one. Optionally only global pose can be set
			if (pObject != NULL)
				tiny.releaseActor(pObject);
			pObject = dynamic_cast<RigidBody*>(tiny.createActor(ActorDescPtr(pObjectDesc)));

			// setup end-effector begin pose
			GenWorkspaceState begin;
			begin.pos.R.rotX(Real(-0.5)*REAL_PI); // end-effector pointing downwards
			begin.pos.p.set(
				frand.nextUniform(Real(0.10), Real(0.15)),
				frand.nextUniform(Real(0.22), Real(0.28)),
				frand.nextUniform(Real(0.05), Real(0.15))
			);
			begin.t = tiny.getTime() + pArm->getTimeDeltaAsync() + 1.0; // movement will last no shorter than 1 sec
			// compute movement end/target in joint configuration space
			cbegin = pArm->recvGenConfigspaceState(tiny.getTime());
			cend = pArm->findTarget(cbegin, begin);
			// print pose error, note that the target is always computed with respect to the reference pose in the tool frame (the last joint)
			Mat34 actual;
			actual.multiply(pArm->getForwardTransform(cend.pos).back(), pArm->getReferencePose());
			tiny.print("Pose error = (%f, %f)", getLinearDist(begin.pos.p, actual.p), getAngularDist(Quat(begin.pos.R), Quat(actual.R)));
			// compute trajectory using path planning with collision detection
			trajectory = pArm->findTrajectory(cbegin, cend);
			// move the arm and wait until it stops
			tiny.print("Moving to the begin position...");
			pArm->send(trajectory, numeric_const<double>::INF);

			// setup end-effector end pose
			GenWorkspaceState end;
			end.pos.R.rotX(Real(-0.5)*REAL_PI); // end-effector pointing downwards
			end.pos.p.set(
				frand.nextUniform(Real(-0.10), Real(-0.15)),
				frand.nextUniform(Real(0.22), Real(0.28)),
				frand.nextUniform(Real(0.05), Real(0.15))
			);
			end.t = tiny.getTime() + pArm->getTimeDeltaAsync() + frand.nextUniform(Real(2.0), Real(5.0)); // random movement duration
			// turn off collision detection - clear collision group mask
			pArm->setCollisionGroup(0x0);
			// compute line-shaped trajectory
			cbegin = pArm->recvGenConfigspaceState(tiny.getTime());
			trajectory = pArm->findWorkspaceTrajectory(cbegin, end);
			// move the arm and wait until it stops
			tiny.print("Moving to the end position...");
			pArm->send(trajectory, numeric_const<double>::INF);

			// reset collision detection - fill collision group mask with 1s to indicate shapes with all possible group masks
			pArm->setCollisionGroup(0xFFFFFFFF);
		}

		// setup initial configuration
		initial.t = tiny.getTime() + pArm->getTimeDeltaAsync() + 5.0; // movement will last no shorter than 5 sec
		// compute movement end/target in joint configuration space
		cbegin = pArm->recvGenConfigspaceState(tiny.getTime());
		cend = initial;
		// compute trajectory using path planning with collision detection
		trajectory = pArm->findTrajectory(cbegin, cend);
		// move the arm and wait until it stops
		tiny.print("Moving back to the initial configuration...");
		pArm->send(trajectory, numeric_const<double>::INF);
		
		tiny.print("Good bye!");
	}
	catch (const ExTiny& ex) {
		std::cerr << ex.str() << std::endl;
	}

	return 0;
}
