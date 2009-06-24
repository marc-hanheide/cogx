/** @file Movement.cpp
 *
 * vHanz02
 *
 * Demonstration program which moves the arm along a straight line
 * using reactive trajectory planner and collision detection
 * (also see DemoReacPlannerPhys and DemoRobotFinger).
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the arm simulators
 * 
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk/~msk">The University Of Birmingham</A>
 *
 * @version 1.0
 *
 */

#include "Common.h"
#include "Tools.h"
#include "Creator.h"
#include "controller/PhysReacPlanner.h"
#include "controller/Katana.h"
#include "controller/Simulator.h"
#include <iostream>
#include "XMLParser.h"
#include "XMLData.h"
#include "Actuator.h"
#include "tools/data_handling.h"
// #include "embodiment/Embodiment.h"

//using namespace stl;
using namespace std;
using namespace msk;
using namespace msk::ctrl;
using namespace msk::phys;
using namespace msk::demo;
// using namespace msk::embod;

//------------------------------------------------------------------------------

template <typename Desc> void setupPlanner(Desc &desc, XMLContext* xmlContext, msk::Context& context) {
	// some planner parameter tuning
	desc.plannerDesc.pHeuristicDesc->distJointcoordMax.j[4] = Real(1.0*MATH_PI);// last joint
	// Enable signal synchronization (default value)
	//desc.reacPlannerDesc.signalSync = true;
}

//------------------------------------------------------------------------------

Actor *polyFlapActor;


int pos = 1;

class MyPRMPlanner : public PRMPlanner {

	
protected:	
	/** Planner constructor */
	
	MyPRMPlanner(msk::ctrl::Arm &arm) : PRMPlanner::PRMPlanner(arm) {}
	
public:	

	virtual bool find(Path &path, Path::iterator iter, const msk::ctrl::GenJointState &begin, const msk::ctrl::GenWorkspaceState &wend) {
		//while (!PRMPlanner::find (path, iter, begin, wend)) {
		while (true) {
			if (PRMPlanner::find (path, iter, begin, wend)) {
				break;
			}
		
			//cout << "unable to find path... trying again..." << endl;
			context.getLogger()->post(PRMPlannerMsg(StdMsg::LEVEL_INFO, "unable to find path... trying again..."));
		}
		
	}

	virtual bool find(Path &path, Path::iterator iter, const msk::ctrl::GenJointState &begin, const msk::ctrl::GenJointState &jend) {
		//while (!PRMPlanner::find (path, iter, begin, jend)) {
		while (true) {
			if (PRMPlanner::find (path, iter, begin, jend)) {
				break;
			}
			context.getLogger()->post(PRMPlannerMsg(StdMsg::LEVEL_INFO, "unable to find path... trying again..."));
		}
	}
};


bool setupObjects(Scene &scene, Vec3 position, Vec3 rotation, Vec3 dimensions, msk::Context &context) {

//bool setupObjects(Scene &scene, Vec3 position, Vec3 rotation, Vec3 dimensions, msk::Context &context, Actor *polyFlapActor) {


// Creator
	Creator creator(scene);
	Actor::Desc *pActorDesc;
	
	// Create ground plane.
	pActorDesc = creator.createGroundPlaneDesc();
	scene.createObject(*pActorDesc);
	
	// Create polyflap
	pActorDesc = creator.createSimple2FlapDesc(Real(dimensions.v1*0.5), Real(dimensions.v1*0.5), Real(dimensions.v1*0.5), Real(0.0002), REAL_PI_2);
	//-sets coordinates
	pActorDesc->nxActorDesc.globalPose.t.set(NxReal(position.v1), NxReal(position.v2), NxReal(position.v3));	

	Mat34 pose;
	pose.R.fromEuler(
		rotation.v1, 
		rotation.v2, 
		rotation.v3 
	);


	//-sets rotations	
	pActorDesc->nxActorDesc.globalPose.M.setRowMajor(&pose.R._m._11);	
	
	//-density
	pActorDesc->nxActorDesc.density = NxReal(10.0);	


	//scene.createObject(*pActorDesc);
	polyFlapActor = dynamic_cast<Actor*>(scene.createObject(*pActorDesc));


	Mat34 p2;
	Mat34 p3;

	msk::obj_ptr<msk::BoundsSet> set = polyFlapActor->getBounds();
	
	p2 = set->get().front()->getPose();
	p3 = set->get().back()->getPose();



	context.getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f, %f, %f", p2.p.v1, p2.p.v2, p2.p.v3/**, mojepose2.R.v1, mojepose2.R.v2, mojepose2.R.v3*/));

	context.getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f, %f, %f", p3.p.v1, p3.p.v2, p3.p.v3/**, mojepose2.R.v1, mojepose2.R.v2, mojepose2.R.v3*/));

	context.getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%d", set->get().size()));

	
}



template <typename Desc>
void setupFingerActuator(Desc &desc) {
	

/**
if (xmlContext == NULL)	{
		context.getLogger()->post(DemoMsg(StdMsg::LEVEL_CRIT, "NULL Actuator context"));
		return;
	}
	
	XMLData(desc.fingerDesc.fingerCtrlDesc.reacPlannerDesc.pTransmitterDesc->initSignal.gws.pos.p, xmlContext->getContextFirst("finger init_pos"));
*/

	
	Mat33 initTrnR;
	Vec3 tmp;
	desc.fingerDesc.fingerCtrlDesc.reacPlannerDesc.pTransmitterDesc->initSignal.gws.pos.q.toMat33(initTrnR);
	initTrnR.toEuler(tmp.v1, tmp.v2, tmp.v3); // Y, X, Z
	//XMLData(tmp, xmlContext->getContextFirst("finger init_rot_euler"));
	initTrnR.fromEuler(tmp.v1, tmp.v2, tmp.v3); // Y, X, Z
	desc.fingerDesc.fingerCtrlDesc.reacPlannerDesc.pTransmitterDesc->initSignal.gws.pos.q.fromMat33(initTrnR);




	//XMLData(desc.fingerDesc.fingerActorDesc.appearance.solidColour, xmlContext->getContextFirst("finger colour"));
}

void setupActuator(Actuator::Desc::Ptr& pDesc) {
	//Actuator pActuator = new Actuator
	typedef FingerActuator::Desc<GenSimArm, MyPRMPlanner, ReacPlanner> FingerActuatorDesc;
//  	typedef FingerActuator::Desc<GenSimArm, PRMPlanner, ReacPlanner> FingerActuatorDesc;
	FingerActuatorDesc *pFingerActuatorDesc = new FingerActuatorDesc();
	pDesc.reset(pFingerActuatorDesc);
//	pDesc.appearance.solidColour = phys::RGBA::RED;
	setupFingerActuator(*pFingerActuatorDesc);
	//pActuatorDesc.release();

}

void trn(Vec3 v [], U32 n, const Mat34 &pose) {
	while (n--)
		pose.multiply(v[n], v[n]);
}

void createFinger(std::vector<Bounds::Desc::Ptr> &bounds, const Mat34 &pose, MemoryStream &buffer) {
	// mace characteristic dimensions
	const Real length = Real(0.1);
	const Real begin = Real(0.01);
	const Real end = Real(0.01);
// 	const Real diam = Real(0.02);
// 	const Real height = Real(0.05);
	const Real pos = length - end;
	
	// objects data
	Vec3 hilt[8];
	BoundingConvexMesh::Desc desc;
	desc.verticesStrideBytes = sizeof(Vec3);
	desc.bCook = true;

	// create hilt (along Y-axis)
	hilt[0].set(-begin, Real(0.0), -begin);
	hilt[1].set(+begin, Real(0.0), -begin);
	hilt[2].set(-begin, Real(0.0), +begin);
	hilt[3].set(+begin, Real(0.0), +begin);
	hilt[4].set(-end, length, -end);
	hilt[5].set(+end, length, -end);
	hilt[6].set(-end, length, +end);
	hilt[7].set(+end, length, +end);
	trn(hilt, 8, pose);
	desc.numOfVertices = 8;
	buffer.write(hilt, 8);
	desc.vertices = buffer.get().back();
	bounds.push_back(desc.clone());

}



NxJoint *setupJoint(NxActor *effector, NxActor *tool, const NxVec3 &anchor, const NxVec3 &axis) {
	const NxReal angle = NxReal(Math::degToRad(0.001));
	const NxReal spring = NxReal(0.01);
	const NxReal damping = NxReal(0.0001);
	
// 	tool->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);
	
	NxD6JointDesc jDesc;
	jDesc.actor[0] = effector;
	jDesc.actor[1] = tool;
	jDesc.setGlobalAnchor(anchor);
	jDesc.setGlobalAxis(axis);

	jDesc.xMotion = NX_D6JOINT_MOTION_LOCKED;
	jDesc.yMotion = NX_D6JOINT_MOTION_LOCKED;
	jDesc.zMotion = NX_D6JOINT_MOTION_LOCKED;
	jDesc.linearLimit.value = NxReal(0.0);//3locked/2locked/1locked/0locked = none/line/cylinder/sphere

	jDesc.swing1Motion = NX_D6JOINT_MOTION_LOCKED;
	jDesc.swing1Limit.value = angle;
	jDesc.swing1Limit.spring = spring;
	jDesc.swing1Limit.damping = damping;
	
	jDesc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;
	jDesc.swing2Limit.value = angle;
	jDesc.swing2Limit.spring = spring;
	jDesc.swing2Limit.damping = damping;
	
	jDesc.twistMotion = NX_D6JOINT_MOTION_LOCKED;//NX_D6JOINT_MOTION_LIMITED;
	jDesc.twistLimit.low.value =  -angle;
	jDesc.twistLimit.low.spring = spring;
	jDesc.twistLimit.low.damping = damping;
	jDesc.twistLimit.high.value = angle;
	jDesc.twistLimit.high.spring = spring;
	jDesc.twistLimit.high.damping = damping;

	return effector->getScene().createJoint(jDesc);
}

// Modify shape of the joint by adding a new Actor.
void addFinger(PhysReacPlanner &physReacPlanner, U32 jointIndex, std::vector<Bounds::Desc::Ptr> &bounds, msk::Context::Ptr context) {
	Actor::Desc fingerActorDesc;
	NxBodyDesc nxBodyDesc;
	Arm &arm = physReacPlanner.getArm();


	fingerActorDesc.nxActorDesc.body = &nxBodyDesc;
	fingerActorDesc.nxActorDesc.density = NxReal(10.0);

	for (std::vector<Bounds::Desc::Ptr>::const_iterator i = bounds.begin(); i != bounds.end(); i++) {
		NxShapeDesc *pNxShapeDesc = physReacPlanner.getScene().createNxShapeDesc(*i);
		pNxShapeDesc->density = NxReal(1.0);
		fingerActorDesc.nxActorDesc.shapes.push_back(pNxShapeDesc);
	}

	// Find the current pose of the end-effector
	GenJointState j;
	arm.lookupInp(j, physReacPlanner.getContext().getTimer()->elapsed());
	//arm.lookupInp (j, SEC_TM_REAL_ZERO);
	Mat34 zeroPose;
	arm.forwardTransform(zeroPose, j.pos);

	zeroPose.multiply(zeroPose, arm.getReferencePose()); // reference pose
	fingerActorDesc.nxActorDesc.globalPose.M.setRowMajor(&zeroPose.R._m._11);
	fingerActorDesc.nxActorDesc.globalPose.t.set(&zeroPose.p.v1);
	
	Mat34 pose;
	pose.setId();
	pose.p.v2 = Real(0.1);
	pose.multiply(pose, arm.getReferencePose());
	arm.setReferencePose (pose);
	
	NxMat34 nxZeroPose(false);
	nxZeroPose.t.set(&zeroPose.p.v1);
	nxZeroPose.M.setRowMajor(&zeroPose.R._m._11);
	
	// create finger Actor
	Actor *pFingerActor = dynamic_cast<Actor*>(physReacPlanner.getScene().createObject(fingerActorDesc));
	if (pFingerActor == NULL) {
		return;
	}


	Actor *effector = physReacPlanner.getJointActors()[jointIndex];

	const NxU32 solverAccuracy = (NxU32)255;
	const U32 numOfJointActors = physReacPlanner.getJointActors().size();

	fingerActorDesc.nxActorDesc.globalPose.multiply(nxZeroPose, fingerActorDesc.nxActorDesc.globalPose);
	pFingerActor->getNxActor()->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);
	pFingerActor->getNxActor()->setSolverIterationCount(solverAccuracy);
   
		
	// make the finger a part of the arm body, but not a part rigidly attached to the last joint
	const U32 group = physReacPlanner.getArmBoundsGroup();
	pFingerActor->setBoundsGroup(group);
	for (NxArray<NxShapeDesc*, NxAllocatorDefault>::const_iterator i = fingerActorDesc.nxActorDesc.shapes.begin(); i != fingerActorDesc.nxActorDesc.shapes.end(); i++) {
		Bounds::Desc::Ptr pBoundsDesc =  physReacPlanner.getScene().createBoundsDesc(**i);
		if (pBoundsDesc != NULL) {
			pBoundsDesc->group = group;
			pBoundsDesc->pose.multiply(pBoundsDesc->pose, arm.getReferencePose());
			arm.Arm::addBoundsDesc(numOfJointActors - 1, pBoundsDesc);// HACK: call only Arm::addBoundsDesc() so the bounds will be invisible
		}
	}

	physReacPlanner.getPlanner().getHeuristic()->syncArmBoundsDesc(); // sync new arm bounds
	
	NxVec3 anchor(0.0f);
	fingerActorDesc.nxActorDesc.globalPose.multiply(anchor, anchor);
	NxVec3 axis(NxReal(0.0), NxReal(1.0), NxReal(0.0)); // along Y-axis
	fingerActorDesc.nxActorDesc.globalPose.M.multiply(axis, axis);

	NxJoint *pJoint = setupJoint(effector->getNxActor(), pFingerActor->getNxActor(), anchor, axis);
	if (pJoint == NULL) {
		physReacPlanner.getContext().getLogger()->post(DemoMsg(StdMsg::LEVEL_CRIT, "setupJoint(): Unable to create joint"));
	}

};



Real normalizeJnPos(Real r){
	return Real(r/MATH_PI);
}




Real normalize(int i, Real r, float* maxVel, float* minVel) {
	if (r >= 0.0) {
		Real res = r / maxVel[i];
		return res;
	} else {
		Real res = -r / minVel[i];
		return res;
	}
}

//--------------------------------------------------------------------------------





int main(int argc, char *argv[]) {
	// Determine configuration file name
	std::string cfg;
	if (argc == 1) {
		// default configuration file name
		cfg.assign(argv[0]);
		size_t pos = cfg.rfind(".exe"); // Windows only
		if (pos != std::string::npos) cfg.erase(pos);
		cfg.append(".xml");
	}
	else
		cfg.assign(argv[1]);

	// Create XML parser and load configuration file
	XMLParser::Desc parserDesc;
	XMLParser::Ptr parser = parserDesc.create();
	if (!parser->load(FileReadStream(cfg.c_str()))) {
		printf("unable to load configuration file: %s\n", cfg.c_str());
		printf("%s <configuration_file>\n", argv[0]);
		return 1;
	}

	// Find program XML root context
	XMLContext* xmlContext = parser->getContextRoot()->getContextFirst("golem");
	if (xmlContext == NULL) {
		printf("unknown configuration file: %s\n", cfg.c_str());
		return 1;
	}

	// Create program context
	msk::Context::Desc contextDesc;
	setupContext(contextDesc, xmlContext);
	msk::Context::Ptr context = contextDesc.create();
	if (context == NULL) {
		printf("unable to create program context");
		return 1;
	}

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
	
	// Stream all messages to std::cerr
	Streamer streamer(context->getLogger(), std::cout);
	// Throw exception if the message level at least LEVEL_CRIT but only from the current thread
	context->getLogger()->setExFilter(MessageFilter::Ptr(new ThreadFilter<StdMsg>(StdMsg::LEVEL_CRIT)));
	// Do not display LEVEL_DEBUG messages (only with level at least LEVEL_INFO)
	//context->getLogger()->setInpFilter(MessageFilter::Ptr(new LevelFilter<StdMsg>(StdMsg::LEVEL_ERR)));

	// Random number generator seed
	context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Random number generator seed %d", context->getRandSeed()._U32[0]));

	//-----------------------------------------------------------------------------

	try {
		// Create Universe
		Universe::Desc universeDesc;
		setupUniverse(universeDesc, xmlContext->getContextFirst("universe"), *context);
		universeDesc.name = "Golem (Movement3)";
		universeDesc.argc = argc;
		universeDesc.argv = argv;
		Universe::Ptr pUniverse = universeDesc.create(*context);

		// Create scene
		Scene::Desc sceneDesc;
		setupScene(sceneDesc, xmlContext->getContextFirst("scene"), *context);
		sceneDesc.name = "Robotic arm controller demo";
		Scene *pScene = pUniverse->createScene(sceneDesc);

		// Determine arm type
		std::string armType;
		if (!XMLData(armType, xmlContext->getContextFirst("arm type"))) {
			context->getLogger()->post(DemoMsg(StdMsg::LEVEL_CRIT, "Unspecified arm type"));
			return 1;
		}
		
		// Setup PhysReacPlanner controller description
		obj_ptr<Object::Desc> pPhysReacPlannerDesc;
		if (!armType.compare("kat_serial_arm")) {
			typedef PhysReacPlanner::Desc<KatSerialArm, MyPRMPlanner, ReacPlanner> PhysReacPlannerDesc;
// 			typedef PhysReacPlanner::Desc<KatSerialArm, PRMPlanner, ReacPlanner> PhysReacPlannerDesc;
			PhysReacPlannerDesc *pDesc = new PhysReacPlannerDesc();
			pPhysReacPlannerDesc.reset(pDesc);
			setupPlanner(*pDesc, xmlContext, *context);
			XMLData(pDesc->armDesc.cfgPath, xmlContext->getContextFirst("arm kat_serial_arm path"));
			XMLData(pDesc->armDesc.serialDesc.commPort, xmlContext->getContextFirst("arm kat_serial_arm comm_port"));
		}
		else if (!armType.compare("kat_sim_arm")) {
			typedef PhysReacPlanner::Desc<KatSimArm, MyPRMPlanner, ReacPlanner> PhysReacPlannerDesc;
// 			typedef PhysReacPlanner::Desc<KatSimArm, PRMPlanner, ReacPlanner> PhysReacPlannerDesc;
			PhysReacPlannerDesc *pDesc = new PhysReacPlannerDesc();
			pPhysReacPlannerDesc.reset(pDesc);
			setupPlanner(*pDesc, xmlContext, *context);

		}
		else if (!armType.compare("gen_sim_arm")) {
			typedef PhysReacPlanner::Desc<GenSimArm, MyPRMPlanner, ReacPlanner> PhysReacPlannerDesc;
// 			typedef PhysReacPlanner::Desc<GenSimArm, PRMPlanner, ReacPlanner> PhysReacPlannerDesc;
			PhysReacPlannerDesc *pDesc = new PhysReacPlannerDesc();
			pPhysReacPlannerDesc.reset(pDesc);
			setupPlanner(*pDesc, xmlContext, *context);
		}
		else {
			context->getLogger()->post(DemoMsg(StdMsg::LEVEL_CRIT, "Unknown arm type"));
			return 1;
		}

		// Create PhysReacPlanner
		context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Initialising reactive planner..."));
		PhysReacPlanner *pPhysReacPlanner = dynamic_cast<PhysReacPlanner*>(pScene->createObject(*pPhysReacPlannerDesc));
		if (pPhysReacPlanner == NULL) {
			context->getLogger()->post(DemoMsg(StdMsg::LEVEL_CRIT, "Unable to create ReacPlanner"));
			return 1;
		}

		if (!armType.compare("kat_sim_arm")) {
			// Create finger to be attached to the 5th joint 
			const U32 jointIndex = 4;
			std::vector<Bounds::Desc::Ptr> bounds;
			MemoryWriteStream buffer;
			Mat34 pose;
			pose.setId();
			createFinger(bounds, pose/*arm.getReferencePose()*/, buffer);
			// Modify shape of the joint by adding a new Actor.
			addFinger(*pPhysReacPlanner, jointIndex, bounds, context);
		}

		
		// some useful pointers
		ReacPlanner &reacPlanner = pPhysReacPlanner->getReacPlanner();
		Planner &planner = pPhysReacPlanner->getPlanner();
		Arm &arm = pPhysReacPlanner->getArm();

		
// 		Mat34 p = pRobot->getFinger()->getFingerActor().getBounds()->get().front()->getPose();
		Real roll, pitch, yaw;
// 		p.R.toEuler (roll,pitch,yaw);
// 		context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Getting finger bounds pose..."));
// 		context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f, %f, %f, %f, %f, %f", p.p.v1, p.p.v2, p.p.v3, roll, pitch, yaw));

		Mat34 p;
		//arm.setReferencePose (p);
		//arm.setReferencePose (arm.getGlobalPose());
		p = arm.getReferencePose ();
		p.R.toEuler (roll, pitch, yaw);
		
		context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Getting arm reference pose..."));
		context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f, %f, %f, %f, %f, %f", p.p.v1, p.p.v2, p.p.v3, roll, pitch, yaw));

		p = arm.getGlobalPose ();
		p.R.toEuler (roll, pitch, yaw);
		
		context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Getting arm global pose..."));
		context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f, %f, %f, %f, %f, %f", p.p.v1, p.p.v2, p.p.v3, roll, pitch, yaw));

		
		// Display arm information
		armInfo(arm);
		sleep (1);

		
		
		

		
		const U32 numOfJoints = (U32)arm.getJoints().size();
		float minVelocities [numOfJoints];
		float maxVelocities [numOfJoints];
		for (U32 i = 0; i < numOfJoints; i++) {
			const Joint &joint = *arm.getJoints()[i];
			minVelocities[i] = joint.getMin().vel;
		}
		for (U32 i = 0; i < numOfJoints; i++) {
			const Joint &joint = *arm.getJoints()[i];
			maxVelocities[i] = joint.getMax().vel;
		}


		//Polyflap Position and orientation
		//-------------------------------------------------------
		Vec3 startPolyflapPosition(Real(0.2), Real(0.2), Real(0.0));
		Vec3 startPolyflapRotation(Real(-0.0*REAL_PI), Real(-0.0*REAL_PI), Real(-0.0*REAL_PI));//Y,X,Z
		Vec3 polyflapDimensions(Real(0.1), Real(0.1), Real(0.1)); //w,h,l
		//-------------------------------------------------------
		//Normal vector showing the direction of the lying part of polyflap, and it' orthogonal
		//Vec3 polyflapNormalVec(Real(sin(polyflapRotation.v3)), Real(cos(polyflapRotation.v3)), Real(0.0));
		//Vec3 polyflapOrthogonalVec(Real(polyflapNormalVec.v2), Real(-1.0*polyflapNormalVec.v1), Real(0.0));
		//context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f, %f", polyflapNormalVec.v1, polyflapNormalVec.v2));		


		// Actuator
		//Experiment::Desc experimentDesc(*pScene);
		//Embodiment embDesc = Embodiment(pScene);
		//setupActuator(experimentDesc.pActuatorDesc);


		//Actor *pfPolyflap;
		Mat34 mojepose2;
		Mat34 mojepose3;

		// Setup objects
		setupObjects(*pScene, startPolyflapPosition, startPolyflapRotation, polyflapDimensions, *context);		
		//setupObjects(*pScene, polyflapPosition, polyflapRotation, polyflapDimensions, *context, polyFlapActor);
		//setupObjects(*pScene, polyflapPosition, polyflapRotation, polyflapDimensions, *context, mojepose2, mojepose3);
		
		// Big Bang!
		context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Launching Universe..."));
		pUniverse->launch();

		// Reactive arm controller is capable to make the arm to move on almost arbirtary trajectories
		// (within velocity and acceleration limits) using planner with collision detection
		// Trajectories are created by sequential sending trajectory waypoints to the arm
		// Waypoints can be specified in jointspace or in Cartesian workspace
		// Each waypoints has its own time stamp, and the arm controller takes care of time synchronization between
		// the arm and the program
		
		// Program has its own local time which is the same for all threads
		// and it is the time that has elapsed since the start of the program
		// Each arm controller has two characteristic time constants:
		// (Synchronous) Time Delta - a minimum elapsed time between two consecutive waypoints sent to the controller
		SecTmReal timeDelta = reacPlanner.getTimeDelta();
		// Asynchronous Time Delta - a minimum elapsed time between the current time and the first waypoint sent to the controller
		// (no matter what has been sent before)
		SecTmReal timeDeltaAsync = reacPlanner.getTimeDeltaAsync();
		
		// Define the Home pose in the Cartesian workspace
		Vec3 positionH(Real(0.0), Real(0.1), Real(0.1));
		Vec3 orientationH(Real(-0.5*MATH_PI), Real(0.0*MATH_PI), Real(0.0*MATH_PI));
		// and set target waypoint
		msk::ctrl::GenWorkspaceState home;
		fromCartesianPose(home.pos, positionH, orientationH);
		home.vel.setId(); // it doesn't move
		//home.acc.setId(); // nor accelerate
		home.t = context->getTimer()->elapsed() + timeDeltaAsync + SecTmReal(5.0); // i.e. the movement will last at least 5 sec

		// set the initial pose of the arm, force the global movement (with planning in the entire arm workspace)
		reacPlanner.send(home, ReacPlanner::ACTION_GLOBAL);
		// wait for completion of the action (until the arm moves to the initial pose)
		reacPlanner.wait();



		// Define the initial pose in the Cartesian workspace
		Vec3 orientationT(Real(-0.5*MATH_PI), Real(0.0*MATH_PI), Real(0.0*MATH_PI));





		const int numExperiments = 1;

		for (int i=0; i<numExperiments; i++)
		{

		Sequence seq;



	msk::obj_ptr<msk::BoundsSet> curPol = polyFlapActor->getBounds();
	Mat34 curPolPos1;
	Mat34 curPolPos2;	
	if (curPol->get().front()->getPose().p.v3 > curPol->get().back()->getPose().p.v3) {
	curPolPos1 = curPol->get().front()->getPose();
	curPolPos2 = curPol->get().back()->getPose();
	}
	else {
	curPolPos1 = curPol->get().back()->getPose();
	curPolPos2 = curPol->get().front()->getPose();
	}

	Vec3 polyflapPosition(curPolPos1.p.v1, curPolPos1.p.v2, curPolPos2.p.v3);
		
	context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f, %f, %f", polyflapPosition.v1, polyflapPosition.v2, polyflapPosition.v3));

	Vec3 polyflapNormalVec(Real((curPolPos2.p.v1 - curPolPos1.p.v1)/sqrt(pow(curPolPos2.p.v1 - curPolPos1.p.v1,2) + pow(curPolPos2.p.v2 - curPolPos1.p.v2,2) + 0.0)),
						     Real((curPolPos2.p.v2 - curPolPos1.p.v2)/sqrt(pow(curPolPos2.p.v1 - curPolPos1.p.v1,2) + pow(curPolPos2.p.v2 - curPolPos1.p.v2,2) + 0.0)),
						     Real(0.0));
context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f, %f, %f", polyflapNormalVec.v1, polyflapNormalVec.v2, polyflapNormalVec.v3));


//sleep(10);




//Normal vector showing the direction of the lying part of polyflap, and it' orthogonal
//		Vec3 polyflapNormalVec(Real(sin(polyflapRotation.v3)), Real(cos(polyflapRotation.v3)), Real(0.0));
		Vec3 polyflapOrthogonalVec(Real(polyflapNormalVec.v2), Real(-1.0*polyflapNormalVec.v1), Real(0.0));







			//vertical distance from the ground
			Real over = 0.01;

			//initial target of the arm: the center of the polyflap
			Vec3 positionT(Real(polyflapPosition.v1), Real(polyflapPosition.v2), Real(polyflapPosition.v3 + over));
		
			//chose random point int the vicinity of the polyflap
			int startPosition = rand() % 24 + 1;

			//distance from the front/back of the polyflap
			Real dist = 0.05;
			//distance from the side of the polyflap
			Real side = polyflapDimensions.v1*0.6;
			//center of the poolyflop
			Real center = polyflapDimensions.v2*0.5;
			//distance from the top of the polyflap
			Real top = polyflapDimensions.v2* 1.2;
			
			//set it's coordinates into target
			switch (pos /*startPosition*/) {
			case 1: 
				positionT.v1 += (dist*polyflapNormalVec.v1); 
				positionT.v2 += (dist*polyflapNormalVec.v2); 
				positionT.v1 += (side*polyflapOrthogonalVec.v1); 
				positionT.v2 +=(side*polyflapOrthogonalVec.v2); 
				positionT.v3 += 0.0; 
				context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Front down left (1)")); break;

			case 2: positionT.v1 += (dist*polyflapNormalVec.v1); positionT.v2 += (dist*polyflapNormalVec.v2); positionT.v1 += (0.0*polyflapOrthogonalVec.v1); positionT.v2 += (0.0*polyflapOrthogonalVec.v2); positionT.v3 += 0.0; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Front down middle (2)")); break;

			case 3: positionT.v1 += (dist*polyflapNormalVec.v1); positionT.v2 += (dist*polyflapNormalVec.v2); positionT.v1 += (-side*polyflapOrthogonalVec.v1); positionT.v2 += (-side*polyflapOrthogonalVec.v2); positionT.v3 += 0.0; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Front down right (3)")); break;

			case 4: positionT.v1 += (dist*polyflapNormalVec.v1); positionT.v2 += (dist*polyflapNormalVec.v2); positionT.v1 += (side*polyflapOrthogonalVec.v1); positionT.v2 += (side*polyflapOrthogonalVec.v2); positionT.v3 = center; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Front center left (4)")); break;

			case 5: positionT.v1 += (dist*polyflapNormalVec.v1); positionT.v2 += (dist*polyflapNormalVec.v2); positionT.v1 += (0.0*polyflapOrthogonalVec.v1); positionT.v2 += (0.0*polyflapOrthogonalVec.v2); positionT.v3 = center; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Front center middle (5)")); break;

			case 6: positionT.v1 += (dist*polyflapNormalVec.v1); positionT.v2 += (dist*polyflapNormalVec.v2); positionT.v1 += (-side*polyflapOrthogonalVec.v1); positionT.v2 += (-side*polyflapOrthogonalVec.v2); positionT.v3 = center; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Front center right (6)")); break;

			case 7: positionT.v1 += (dist*polyflapNormalVec.v1); positionT.v2 += (dist*polyflapNormalVec.v2); positionT.v1 += (side*polyflapOrthogonalVec.v1); positionT.v2 += (side*polyflapOrthogonalVec.v2); positionT.v3 = top; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Front up left (7)")); break;

			case 8: positionT.v1 += (dist*polyflapNormalVec.v1); positionT.v2 += (dist*polyflapNormalVec.v2); positionT.v1 += (0.0*polyflapOrthogonalVec.v1); positionT.v2 += (0.0*polyflapOrthogonalVec.v2); positionT.v3 = top; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Front up middle (8)")); break;

			case 9: positionT.v1 += (dist*polyflapNormalVec.v1); positionT.v2 += (dist*polyflapNormalVec.v2); positionT.v1 += (-side*polyflapOrthogonalVec.v1); positionT.v2 += (-side*polyflapOrthogonalVec.v2); positionT.v3 = top; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Front up right (9)")); break;

			case 10: positionT.v1 += (-dist*polyflapNormalVec.v1); positionT.v2 += (-dist*polyflapNormalVec.v2); positionT.v1 += (side*polyflapOrthogonalVec.v1); positionT.v2 += (side*polyflapOrthogonalVec.v2); positionT.v3 += 0.0; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Back down left (10)")); break;

			case 11: positionT.v1 += (-dist*polyflapNormalVec.v1); positionT.v2 += (-dist*polyflapNormalVec.v2); positionT.v1 += (0.0*polyflapOrthogonalVec.v1); positionT.v2 += (0.0*polyflapOrthogonalVec.v2); positionT.v3 += 0.0; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Back down middle (11)")); break;

			case 12: positionT.v1 += (-dist*polyflapNormalVec.v1); positionT.v2 += (-dist*polyflapNormalVec.v2); positionT.v1 += (-side*polyflapOrthogonalVec.v1); positionT.v2 += (-side*polyflapOrthogonalVec.v2); positionT.v3 += 0.0; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Back down right (12)")); break;

			case 13: positionT.v1 += (-dist*polyflapNormalVec.v1); positionT.v2 += (-dist*polyflapNormalVec.v2); positionT.v1 += (side*polyflapOrthogonalVec.v1); positionT.v2 += (side*polyflapOrthogonalVec.v2); positionT.v3 = center; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Back center left (13)")); break;

			case 14: positionT.v1 += (-dist*polyflapNormalVec.v1); positionT.v2 += (-dist*polyflapNormalVec.v2); positionT.v1 += (0.0*polyflapOrthogonalVec.v1); positionT.v2 += (0.0*polyflapOrthogonalVec.v2); positionT.v3 = center; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Back center middle (14)")); break;

			case 15: positionT.v1 += (-dist*polyflapNormalVec.v1); positionT.v2 += (-dist*polyflapNormalVec.v2); positionT.v1 += (-side*polyflapOrthogonalVec.v1); positionT.v2 += (-side*polyflapOrthogonalVec.v2); positionT.v3 = center; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Back center right (15)")); break;

			case 16: positionT.v1 += (-dist*polyflapNormalVec.v1); positionT.v2 += (-dist*polyflapNormalVec.v2); positionT.v1 += (side*polyflapOrthogonalVec.v1); positionT.v2 += (side*polyflapOrthogonalVec.v2); positionT.v3 = top; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Back up left (16)")); break;

			case 17: positionT.v1 += (-dist*polyflapNormalVec.v1); positionT.v2 += (-dist*polyflapNormalVec.v2); positionT.v1 += (0.0*polyflapOrthogonalVec.v1); positionT.v2 += (0.0*polyflapOrthogonalVec.v2); positionT.v3 = top; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Back up middle (17)")); break;

			case 18: positionT.v1 += (-dist*polyflapNormalVec.v1); positionT.v2 += (-dist*polyflapNormalVec.v2); positionT.v1 += (-side*polyflapOrthogonalVec.v1); positionT.v2 += (-side*polyflapOrthogonalVec.v2); positionT.v3 = top; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Back up right (18)")); break;

			case 19: positionT.v1 += 0.0; positionT.v1 += (side*polyflapOrthogonalVec.v1); positionT.v2 += (side*polyflapOrthogonalVec.v2); positionT.v3 += 0.0; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Side down left (19)")); break;

			case 20: positionT.v1 += 0.0; positionT.v1 += (-side*polyflapOrthogonalVec.v1); positionT.v2 += (-side*polyflapOrthogonalVec.v2); positionT.v3 += 0.0; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Side down right (20)")); break;

			case 21: positionT.v1 += 0.0; positionT.v1 += (side*polyflapOrthogonalVec.v1); positionT.v2 += (side*polyflapOrthogonalVec.v2); positionT.v3 = center; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Side center left (21)")); break;

			case 22: positionT.v1 += 0.0; positionT.v1 += (-side*polyflapOrthogonalVec.v1); positionT.v2 += (-side*polyflapOrthogonalVec.v2); positionT.v3 = center; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Side center right (22)")); break;

			case 23:
positionT.v1 += 0.0;
positionT.v1 += (side*polyflapOrthogonalVec.v1);
positionT.v2 += (side*polyflapOrthogonalVec.v2);
positionT.v3 = top; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Side up left (23)")); break;

			case 24: 
positionT.v1 += 0.0;
positionT.v2 += 0.0;
positionT.v1 += (-side*polyflapOrthogonalVec.v1); 
positionT.v2 += (-side*polyflapOrthogonalVec.v2); 
positionT.v3 = top; context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Side up right (24)")); break;

			};

pos++;







			// and set target waypoint
			msk::ctrl::GenWorkspaceState target;
			fromCartesianPose(target.pos, positionT, orientationT);
			target.vel.setId(); // it doesn't mov
			//target.acc.setId(); // nor accelerate
			
			// ON/OFF collision detection
			//planner.getHeuristic()->setCollisionDetection(false);

			target.t = context->getTimer()->elapsed() + timeDeltaAsync + SecTmReal(5.0); // i.e. the movement will last at least 5 sec
			while (true) {
				
				if (reacPlanner.send(target , ReacPlanner::ACTION_GLOBAL)) {
					break;
				}
				context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Unable to find path to polyflap, trying again."));				}

			// wait for completion of the action (until the arm moves to the initial pose)
			reacPlanner.wait();
//sleep(2);
			
			// Trajectory profile can be defined by e.g. a simple 3rd degree polynomial
			Trajectory::Ptr pTrajectory(Polynomial4::Desc().create());
			// It consists of 70 parts
			U32 n = 70;
			// Trajectory duration is a multiplicity of Time Delta [sec]
			SecTmReal duration = timeDelta * n;
			
			// Trajectory end pose equals begin + shift along Y axis
			WorkspaceCoord begin = target.pos, end = target.pos;





			
			//Normal standartized vector of the polyflap
			Vec3 polyflapCenterNormalVec(Real((polyflapPosition.v1 - positionT.v1)/sqrt(pow(polyflapPosition.v1 - positionT.v1,2) + pow(polyflapPosition.v2 - positionT.v2,2) + pow(polyflapDimensions.v2*0.5 - positionT.v3,2))),
						     Real((polyflapPosition.v2 - positionT.v2)/sqrt(pow(polyflapPosition.v1 - positionT.v1,2) + pow(polyflapPosition.v2 - positionT.v2,2) + pow(polyflapDimensions.v2*0.5 - positionT.v3,2))),
						     Real((polyflapDimensions.v2*0.5 - positionT.v3)/sqrt(pow(polyflapPosition.v1 - positionT.v1,2) + pow(polyflapPosition.v2 - positionT.v2,2) + pow(polyflapDimensions.v2*0.5 - positionT.v3,2))));
			//and it's orthogonal
			Vec3 polyflapCenterOrthogonalVec(Real(-1.0*polyflapCenterNormalVec.v2), Real(polyflapCenterNormalVec.v1), Real(0.0));
		




context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f, %f, %f", polyflapCenterNormalVec.v1, polyflapCenterNormalVec.v2, polyflapCenterNormalVec.v3));


//sleep(10);






			//the lenght of the movement
			Real distance = 0.2;
			
			//end.p.v1 += (distance*polyflapCenterNormalVec.v1); 
			//end.p.v2 += (distance*polyflapCenterNormalVec.v2); 
			//end.p.v3 += (distance*polyflapCenterNormalVec.v3); 
		


			//chose random horizontal and vertical angle
			int horizontalAngle = rand() % 7;
			int verticalAngle = rand() % 7;
		
		

			//change the movement end position accordingly
			switch (horizontalAngle) {
			case 0: 

				end.p.v1 += (sin(0.0/6.0*REAL_PI)*(distance*polyflapCenterNormalVec.v1)); 
				end.p.v2 += (sin(0.0/6.0*REAL_PI)*(distance*polyflapCenterNormalVec.v2)); 
				end.p.v1 += (cos(0.0/6.0*REAL_PI)*(distance*polyflapCenterOrthogonalVec.v1)); 
				end.p.v2 += (cos(0.0/6.0*REAL_PI)*(distance*polyflapCenterOrthogonalVec.v2)); 
				end.p.v3 += 0.0;
				context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "0 degree horizontaly (0)")); break;
			
			case 1: 
				end.p.v1 += (sin(1.0/6.0*REAL_PI)*(distance*polyflapCenterNormalVec.v1)); 
				end.p.v2 += (sin(1.0/6.0*REAL_PI)*(distance*polyflapCenterNormalVec.v2)); 
				end.p.v1 += (cos(1.0/6.0*REAL_PI)*(distance*polyflapCenterOrthogonalVec.v1)); 
				end.p.v2 += (cos(1.0/6.0*REAL_PI)*(distance*polyflapCenterOrthogonalVec.v2)); 
				positionT.v3 += 0.0; 
				context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "30 degrees horizontaly (1)")); break;
				
			case 2: 
				end.p.v1 += (sin(2.0/6.0*REAL_PI)*(distance*polyflapCenterNormalVec.v1)); 
				end.p.v2 += (sin(2.0/6.0*REAL_PI)*(distance*polyflapCenterNormalVec.v2)); 
				end.p.v1 += (cos(2.0/6.0*REAL_PI)*(distance*polyflapCenterOrthogonalVec.v1)); 
				end.p.v2 += (cos(2.0/6.0*REAL_PI)*(distance*polyflapCenterOrthogonalVec.v2)); 
				positionT.v3 += 0.0; 
				context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "60 degrees horizontaly (2)")); break;
				
			case 3: 
				end.p.v1 += (sin(3.0/6.0*REAL_PI)*(distance*polyflapCenterNormalVec.v1)); 
				end.p.v2 += (sin(3.0/6.0*REAL_PI)*(distance*polyflapCenterNormalVec.v2)); 
				end.p.v1 += (cos(3.0/6.0*REAL_PI))*(distance*polyflapCenterOrthogonalVec.v1); 
				end.p.v2 += (cos(3.0/6.0*REAL_PI))*(distance*polyflapCenterOrthogonalVec.v2); 
				positionT.v3 += 0.0; 
				context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "90 degrees horizontaly (3)")); break;
				
			case 4: 
				end.p.v1 += (sin(4.0/6.0*REAL_PI)*(distance*polyflapCenterNormalVec.v1)); 
				end.p.v2 += (sin(4.0/6.0*REAL_PI)*(distance*polyflapCenterNormalVec.v2)); 
				end.p.v1 += (cos(4.0/6.0*REAL_PI)*(distance*polyflapCenterOrthogonalVec.v1)); 
				end.p.v2 += (cos(4.0/6.0*REAL_PI)*(distance*polyflapCenterOrthogonalVec.v2)); 
				positionT.v3 += 0.0; 
				context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "120 degree horizontaly (4)")); break;
			
			case 5: 
				end.p.v1 += (sin(5.0/6.0*REAL_PI)*(distance*polyflapCenterNormalVec.v1)); 
				end.p.v2 += (sin(5.0/6.0*REAL_PI)*(distance*polyflapCenterNormalVec.v2)); 
				end.p.v1 += (cos(5.0/6.0*REAL_PI)*(distance*polyflapCenterOrthogonalVec.v1)); 
				end.p.v2 += (cos(5.0/6.0*REAL_PI)*(distance*polyflapCenterOrthogonalVec.v2)); 
				positionT.v3 += 0.0; 
				context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "150 degree horizontaly (5)")); break;
				
			case 6: 
				end.p.v1 += (sin(6.0/6.0*REAL_PI)*(distance*polyflapCenterNormalVec.v1)); 
				end.p.v2 += (sin(6.0/6.0*REAL_PI)*(distance*polyflapCenterNormalVec.v2)); 
				end.p.v1 += (cos(6.0/6.0*REAL_PI)*(distance*polyflapCenterOrthogonalVec.v1)); 
				end.p.v2 += (cos(6.0/6.0*REAL_PI)*(distance*polyflapCenterOrthogonalVec.v2)); 
				positionT.v3 += 0.0; 
				context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "180 degree horizontaly (6)")); break;
			
			};



//sleep(10);



			// Current time is the same for all threads and it is the time that has elapsed since the start of the program
			SecTmReal timeBegin = context->getTimer()->elapsed();
			
			context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Moving on the line..."));
			

			// ON/OFF collision detection
			planner.getHeuristic()->setCollisionDetection(false);







			
			// Generate and send a simple straight line trajectory
			for (U32 i = 0; i <= n; i++) {
				// create a new target waypoint on a straight line at normalized time timeDelta * i
				waypointFromLineTrajectory(target, *pTrajectory, begin, end, duration, timeDelta * i);
				// set the target waypoint absolute time (in future)
				target.t = timeBegin + timeDeltaAsync + timeDelta * i;
			
				// send to the controller, force local movement (without planning in the entire arm workspace)
				// reacPlanner.wait() will wait approx 'timeDelta' seconds, until a next waypoint can be sent
				// reacPlanner.wait() can be used only if signal synchronization is enabled (see 'setupPlanner()' above)
				if (!reacPlanner.send(target, ReacPlanner::ACTION_LOCAL) || !reacPlanner.wait()) {
					// woops something went wrong
				}



				reacPlanner.wait();




				// arm state at a time t and finishes at some time later
				msk::ctrl::GenJointState state;
				arm.lookupInp(state, target.t  /*SEC_TM_REAL_INF)*/); // last sent trajectory waypoint
			
				//msk::demo::armPose(arm, state);
				msk::demo::armState(arm, state);

				//to get polyflap pose information
				msk::obj_ptr<msk::BoundsSet> set = polyFlapActor->getBounds();

				FeatureVector& features = *(new FeatureVector);
				// Read joints
				for (U32 i = 0; i < numOfJoints; i++) {
					//const Joint &joint = *arm.getJoints()[i];
					//Real norm = Real(state.pos.j[i]/ MATH_PI);
					features.push_back(normalizeJnPos(state.pos[i]));
					//context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f", norm));
					//features[2*i] = norm;
					//Real norm2 = Real(normalize(i, state.vel.j[i], maxVelocities, minVelocities));
			//		features.pushBack(normalizeJnVel(i, state.vel.j[i]);
					//context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f", norm2));
					//features[2*i+1] = norm2;
				}
/*
				// add pose of polyflap to features!!

				//context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "2"));

				//	const NxMat34 nxPose = pfPolyflap->getNxActor()->getGlobalPose();
				//	nxPose.M.getRowMajor(&.pose.R._m._11);
				//	nxPose.t.get(&rigidBodyData.pose.p.v1);
				
				mojepose2 = set->get().front()->getPose();
				//context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "3"));
				mojepose3 = set->get().back()->getPose();
				features.push_back(normalizePfPos(mojepose2.p.v1, maxRange);
				features.push_back(normalizePfPos(mojepose2.p.v2, maxRange);
				features.push_back(normalizePfPos(mojepose2.p.v3, maxRange);
				features.push_back(normalizePfPos(mojepose3.p.v1, maxRange);
				features.push_back(normalizePfPos(mojepose3.p.v2, maxRange);
				features.push_back(normalizePfPos(mojepose3.p.v3, maxRange);

				


				//context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f, %f, %f", mojepose2.p.v1, mojepose2.p.v2, mojepose2.p.v3/**, mojepose2.R.v1, mojepose2.R.v2, mojepose2.R.v3/));
					
				//context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f, %f, %f", mojepose3.p.v1, mojepose3.p.v2, mojepose3.p.v3/**, mojepose2.R.v1, mojepose2.R.v2, mojepose2.R.v3/));



				seq.push_back(features);

*/	
 



 

				//begin.vel.setZero();
				//begin.acc.setZero();
				//begin.t += pPhysPlanner->getArm().getTimeDeltaAsync(); // concatenate trajectories
				//end.t = begin.t + SecTmReal(5.0); // minimum trajectory duration
			
			




				//armState(arm, );
				//context->getTimer()->sleep(2);
				//reacPlanner.wait();
				//armState(arm, target);
				//context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "*****************************************************************************************"));

			}
		




			// wait until the arm stops
			context->getTimer()->sleep(timeDeltaAsync - timeDelta);

			// ON/OFF collision detection
			planner.getHeuristic()->setCollisionDetection(true);


				


		Vec3 positionPreH(target.pos.p.v1, target.pos.p.v2, target.pos.p.v3 += (polyflapDimensions.v2*1.1));
		// and set target waypoint
		msk::ctrl::GenWorkspaceState preHome;
		fromCartesianPose(preHome.pos, positionPreH, orientationH);
		home.vel.setId(); // it doesn't move
		//home.acc.setId(); // nor accelerate
		home.t = context->getTimer()->elapsed() + timeDeltaAsync + SecTmReal(5.0); // i.e. the movement will last at least 5 sec

		// set the initial pose of the arm, force the global movement (with planning in the entire arm workspace)
		reacPlanner.send(preHome, ReacPlanner::ACTION_GLOBAL);
		// wait for completion of the action (until the arm moves to the initial pose)
		reacPlanner.wait();






			while (true) {
				if (reacPlanner.send(home, ReacPlanner::ACTION_GLOBAL)) {
					break;
				}
				
				context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Unable to find path home, trying again."));
			}





			context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Moving home..."));
			reacPlanner.wait();
			context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Done"));


			/**
			   ofstream myfile;
			   myfile.open ("home/hanz/Dokumenty/Prace/Golem-0.9.57/Golem/demo/movement/example.txt");
			   for (int i = 0; i < seq.size(); i ++) {
		   
			   for (int a = 0; a < (numOfJoints+6); a++) {
			   //context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f", *seq[i][a]));
			   double res = seq[i][a];
			   context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "%f", res));
			   myfile << res;
			   myfile << ",";
			   }


			   myfile << "\n";
			   }
			   myfile.close();



			   ofstream myfile;
			   myfile.open ("home/hanz/Dokumenty/Prace/Golem-0.9.57/Golem/demo/movement/example.txt", ios::binary);
			*/
		

			//context->getTimer()->sleep(2);
			context->getLogger()->post(DemoMsg(StdMsg::LEVEL_INFO, "Iteration completed!"));
		}
/*	
	int tm_year;
	int tm_mon;
	int tm_mday;
	int tm_hour;
	int tm_min;
	int tm_sec;
	


	cout <<tm_year<< "\n";
	cout <<tm_mon<< "\n";
	cout <<tm_mday<< "\n";
	cout <<tm_hour<< "\n";
	cout <<tm_min<< "\n";
	cout <<tm_sec<< "\n";
	

	stringstream stream;

	stream << (tm_year+1900);

	stream << tm_mon;
	stream << tm_mday;
	stream << tm_hour;
	stream << tm_min;
	stream << tm_sec;
*/



	time_t rawtime;
	struct tm * timeinfo;
  	char buffer [12];

 	time ( &rawtime );
  	timeinfo = localtime ( &rawtime );

  	strftime (buffer,12,"%y%m%d%H%M%s",timeinfo);
  	puts(buffer);

	string name;
	name.append(buffer);
	
	//name = stream.str();
	DataSet data;
	
	cout << name << "\n";

	
	// DOESN'T WORK!!!! compiles, but throws errors by linking executables
	write_dataset(name , data);


	}
	catch (const StdMsg &msg) {
		context->getLogger()->post(msg, false);
	}
	catch (const std::exception &ex) {
		context->getLogger()->post(DemoMsg("C++ exception: %s", ex.what()));
	}

	return 0;
}
