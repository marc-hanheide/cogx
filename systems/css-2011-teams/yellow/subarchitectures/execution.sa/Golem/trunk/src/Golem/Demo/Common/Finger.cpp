/** @file Finger.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Demo/Common/Finger.h>
#include <Golem/Demo/Common/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

WrenchTransmitter::WrenchTransmitter(golem::Arm &arm) :	Transmitter(arm) {
}

//------------------------------------------------------------------------------

bool WrenchTransmitter::create(const Desc& desc) {
	Transmitter::create(desc); // throws

	referencePoseBounds = desc.referencePoseBoundsDesc.create();
	if (referencePoseBounds == NULL)
		throw MsgWrenchTransmitter(Message::LEVEL_CRIT, "Transmitter::create(): Unable to create reference pose bounds");

	pUniverse = desc.pUniverse;

	CriticalSectionWrapper csw(pUniverse->getCSPhysX());
	
	// Setup the actuator Scene
	pNxScene = pUniverse->getSceneList().front()->getNxPhysicsSDK()->createScene(desc.nxSceneDesc);
	if (pNxScene == NULL)
		throw MsgWrenchTransmitter(Message::LEVEL_CRIT, "Transmitter::create(): Unable to create Scene");

	// Create the virtual actuator Actor
	pNxActor = pNxScene->createActor(desc.nxActorDesc);
	if (pNxActor == NULL)
		throw MsgWrenchTransmitter(Message::LEVEL_CRIT, "Transmitter::create(): Unable to create Actor");

	const NxU32 solverAccuracy = (NxU32)255;
	pNxActor->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);
	pNxActor->setSolverIterationCount(solverAccuracy);

	////-------------------------------------------------------------------------
	//anchor.set(nxInitPose.t);
	//axis.set(NxReal(1.0), NxReal(0.0), NxReal(0.0)); // initially along Y-axis
	//nxInitPose.M.multiply(axis, axis); // globally
	//
	//NxDistanceJointDesc jointDesc;
	//jointDesc.actor[0] = NULL;
	//jointDesc.actor[1] = pNxActor;
	//jointDesc.setGlobalAnchor(anchor);
	//jointDesc.setGlobalAxis(axis);
	//jointDesc.minDistance = 0.0f;
	//jointDesc.maxDistance = 0.0f;
	//jointDesc.spring.spring = 0.1f;
	//jointDesc.spring.damper = 0.05f;
	//jointDesc.flags = NX_DJF_MIN_DISTANCE_ENABLED | NX_DJF_MAX_DISTANCE_ENABLED | NX_DJF_SPRING_ENABLED;

	//pNxJoint = pNxScene->createJoint(jointDesc);
	//if (pNxJoint == NULL)
	//	throw MsgWrenchTransmitter(Message::LEVEL_CRIT, "Finger::create(): Unable to create actuator joint");

	//-------------------------------------------------------------------------
	//anchor.set(nxInitPose.t);
	//anchor.z += 1.0f;
	//axis.set(NxReal(1.0), NxReal(0.0), NxReal(0.0)); // initially along Y-axis
	//nxInitPose.M.multiply(axis, axis); // globally
	//
	//NxRevoluteJointDesc jointDesc;
	//jointDesc.actor[0] = NULL;
	//jointDesc.actor[1] = pNxActor;
	//jointDesc.setGlobalAnchor(anchor);
	//jointDesc.setGlobalAxis(axis);
	//jointDesc.spring.spring = 0.5f;
	//jointDesc.spring.damper = 0.5f;
	//jointDesc.spring.targetValue = 0.0f;

	//pNxJoint = pNxScene->createJoint(jointDesc);
	//if (pNxJoint == NULL)
	//	throw MsgWrenchTransmitter(Message::LEVEL_CRIT, "Finger::create(): Unable to create actuator joint");
	// The global pose of the virtual actuator Actor is the inital pose 
	
	//-------------------------------------------------------------------------
	
	// Setup Novodex actuator joint
	NxD6JointDesc nxJointDesc = desc.nxJointDesc;
	nxJointDesc.actor[0] = NULL;
	nxJointDesc.actor[1] = pNxActor;
	nxJointDesc.setGlobalAnchor(NxVec3(NxReal(0.0), NxReal(1.0), NxReal(0.0)));
	nxJointDesc.setGlobalAxis(NxVec3(NxReal(0.0), NxReal(1.0), NxReal(0.0)));

	NxJoint *pNxJoint = pNxScene->createJoint(nxJointDesc);
	if (pNxJoint == NULL)
		throw MsgWrenchTransmitter(Message::LEVEL_CRIT, "WrenchTransmitter::create(): Unable to create Joint");
	
	// Init actor and joint
	Mat34 initPose;
	if (getInitSignal().type == Signal::TYPE_JOINTSPACE) {
		arm.forwardTransform(initPose, getInitSignal().gjs.pos);
		initPose.multiply(initPose, arm.getReferencePose()); // reference pose
	}
	else {
		initPose = getInitSignal().gws.pos;
	}

	NxMat34 nxInitPose(false);
	nxInitPose.t.set(&initPose.p.v1);
	nxInitPose.M.setRowMajor(&initPose.R.m11);
	
	pNxActor->setGlobalPose(nxInitPose);
	
	// Find axis and anchor for the actuator Joint
	NxVec3 anchor, axis;
	anchor.set(nxInitPose.t);
	axis.set(NxReal(0.0), NxReal(1.0), NxReal(0.0)); // initially along Y-axis
	nxInitPose.M.multiply(axis, axis); // globally
	
	// Setup Novodex actuator joint
	pNxJoint->setGlobalAnchor(anchor);
	pNxJoint->setGlobalAxis(axis);

	// Setup limit point
	pNxJoint->setLimitPoint(nxInitPose.t);
	
	// Setup bounding planes
	if (!referencePoseBounds->intersect(initPose.p))
		context.getMessageStream()->write(Message::LEVEL_WARNING, "WrenchTransmitter::create(): Initial pose does not lie within bounds");

	pNxJoint->purgeLimitPlanes();
	NxMat34 nxBoundsPose(false);
	nxBoundsPose.t.set(&referencePoseBounds->getPose().p.v1);
	nxBoundsPose.M.setRowMajor(&referencePoseBounds->getPose().R.m11);	
	for (U32 i = 0; i < 6; i++) {
		NxVec3 point(numeric_const<NxReal>::ZERO);
		point[i/2] = (NxReal)getReferencePoseBounds()->getDimensions()[i/2];
		NxVec3 normal(point);
		normal.normalize();

		if (i%2)
			point.setNegative();
		else
			normal.setNegative();

		point.add(point, nxBoundsPose.t);
		nxBoundsPose.M.multiply(normal, normal);
		
		if (!pNxJoint->addLimitPlane(normal, point))
			context.getMessageStream()->write(Message::LEVEL_ERROR, "WrenchTransmitter::create(): Unable to add limit plane"	);
	}

	return true;
}

//------------------------------------------------------------------------------

bool WrenchTransmitter::set(const void *data) {
	const Wrench &wrench = *(const Wrench*)data;

	CriticalSectionWrapper csw(pUniverse->getCSPhysX());
	
	NxVec3 force;
	force.set(&wrench.getV().v1);
	pNxActor->addLocalForce(force, NX_FORCE);
	
	NxVec3 torque;
	torque.set(&wrench.getW().v1);
	pNxActor->addLocalTorque(torque, NX_FORCE);

	return true;
}

bool WrenchTransmitter::get(Signal &signal) const {
	signal.type = Signal::TYPE_WORKSPACE;
	
	CriticalSectionWrapper csw(pUniverse->getCSPhysX());
	
	// simulate
	pNxScene->simulate((NxReal)arm.getTimeDelta());
	pNxScene->flushStream();
	pNxScene->fetchResults(NX_RIGID_BODY_FINISHED, true); // blocking call

	// Pose
	NxMat34 nxPose = pNxActor->getGlobalPose();
	nxPose.t.get(&signal.gws.pos.p.v1);
	nxPose.M.getRowMajor(&signal.gws.pos.R.m11);
	
	// Linear velocity
	pNxActor->getLinearVelocity().get(&signal.gws.vel.v.v1);
	
	// Angular velocity
	pNxActor->getAngularVelocity().get(&signal.gws.vel.w.v1);

	return true;
}
	
//------------------------------------------------------------------------------

FingerRenderer::FingerRenderer(const golem::ReacPlanner &reacPlanner) :
	reacPlanner(reacPlanner)
{}

bool FingerRenderer::create(const Desc &desc) {
	if (!desc.isValid())
		return false;

	const WrenchTransmitter *pWrenchTransmitter =
		dynamic_cast<const WrenchTransmitter*>(&reacPlanner.getTransmitter());
	
	// reserve memory buffer
	reset();
	
	if (pWrenchTransmitter != NULL && desc.boundsShow)
		reserveLines(12);
	if (pWrenchTransmitter != NULL && desc.initPoseShow)
		reserveAxesInc(1);
	
	if (pWrenchTransmitter != NULL && desc.boundsShow) {
		const BoundingBox &limits = *pWrenchTransmitter->getReferencePoseBounds();
		for (U32 i = 0, j = 0; j < 4; i += 3 - j, j++)
			for (U32 k = 0; k < 3; k++)
				addLine(limits.getEdges()[i], limits.getEdges()[i ^ (1 << k)], desc.boundsColour);
	}
	if (pWrenchTransmitter != NULL && desc.initPoseShow) {
		Mat34 pose;
		const Transmitter::Signal &initSignal = pWrenchTransmitter->getInitSignal();
		if (initSignal.type == Transmitter::Signal::TYPE_JOINTSPACE) {
			reacPlanner.getArm().forwardTransform(pose, initSignal.gjs.pos);
			pose.multiply(pose, reacPlanner.getArm().getReferencePose()); // reference pose
		}
		else {
			pose = initSignal.gws.pos;
		}
		
		addAxes(pose, desc.initPoseSize);
	}

	return true;
}

//------------------------------------------------------------------------------

Finger::Finger(Embodiment &embodiment) : Base(embodiment) {
}

Finger::~Finger() {
}

bool Finger::create(const Desc& desc) {
	Base::create(desc); // throws
	
	pFingerCtrl = dynamic_cast<PhysReacPlanner*>(this->getScene().createObject(desc.fingerCtrlDesc));
	if (pFingerCtrl == NULL)
		throw MsgFinger(Message::LEVEL_CRIT, "Finger::create(): Unknown reactive planner");
	
	fingerActorDesc = desc.fingerActorDesc;
	fingerJointDesc = desc.fingerJointDesc;

	fingerRendererDesc = desc.fingerRendererDesc;
	fingerRendererShow = desc.fingerRendererShow;
		
	// Find the initial global pose of the end-effector (zero pose)
	ASSERT(pFingerCtrl != NULL)
	golem::Arm &arm = pFingerCtrl->getArm();
	golem::GenConfigspaceState j;
	arm.lookupCommand(j, SEC_TM_REAL_ZERO);
	Mat34 zeroPose;
	arm.forwardTransform(zeroPose, j.pos);
	zeroPose.multiply(zeroPose, arm.getReferencePose()); // reference pose

	NxMat34 nxZeroPose(false);
	nxZeroPose.t.set(&zeroPose.p.v1);
	nxZeroPose.M.setRowMajor(&zeroPose.R.m11);

	// take the controller end-effector Actor
	const U32 numOfJointActors = (U32)pFingerCtrl->getJointActors().size();
	golem::Actor *pJointActor = pFingerCtrl->getJointActors()[numOfJointActors - 1];
	if (pJointActor == NULL)
		throw MsgFinger(Message::LEVEL_CRIT, "Finger::create(): End-effector has no body");
	
	// update arm reference pose
	Mat34 pose;
	fingerActorDesc.nxActorDesc.globalPose.t.get(&pose.p.v1);
	fingerActorDesc.nxActorDesc.globalPose.M.getRowMajor(&pose.R.m11);
	pose.multiply(arm.getReferencePose(), pose);
	arm.setReferencePose(pose);

	// create finger Actor
	fingerActorDesc.nxActorDesc.globalPose.multiply(nxZeroPose, fingerActorDesc.nxActorDesc.globalPose);
	pFingerActor = dynamic_cast<golem::Actor*>(this->getScene().createObject(fingerActorDesc));
	if (pFingerActor == NULL)
		throw MsgFinger(Message::LEVEL_CRIT, "Finger::create(): Unable to create finger Actor");

	// make the finger a part of the arm body, but not a part rigidly attached to the last joint
	const U32 group = pFingerCtrl->getArmBoundsGroup();
	pFingerActor->setBoundsGroup(group);
	for (NxArray<NxShapeDesc*, NxAllocatorDefault>::const_iterator i = fingerActorDesc.nxActorDesc.shapes.begin(); i != fingerActorDesc.nxActorDesc.shapes.end(); i++) {
		Bounds::Desc::Ptr pBoundsDesc = this->scene.createBoundsDesc(**i);
		if (pBoundsDesc != NULL) {
			pBoundsDesc->group = group;
			pBoundsDesc->pose.multiply(pBoundsDesc->pose, arm.getReferencePose());
			arm.getJoints()[numOfJointActors - 1]->addBoundsDesc(pBoundsDesc);// HACK: call only Arm::addBoundsDesc() so the bounds will be invisible
		}
	}
	pFingerCtrl->getPlanner().getHeuristic()->syncArmBoundsDesc(); // sync new arm bounds

	{
		CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
		
		const NxU32 solverAccuracy = (NxU32)255;
		pJointActor->getNxActor()->setSolverIterationCount(solverAccuracy);
		pFingerActor->getNxActor()->setSolverIterationCount(solverAccuracy);
		pFingerActor->getNxActor()->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);
	}

	NxVec3 anchor, axis;
	
	// Find axis and anchor for the finger Joint
	anchor.set(NxReal(0.0)); // initially zero
	fingerActorDesc.nxActorDesc.globalPose.multiply(anchor, anchor); // globally
	axis.set(NxReal(0.0), NxReal(1.0), NxReal(0.0)); // initially along Y-axis
	fingerActorDesc.nxActorDesc.globalPose.M.multiply(axis, axis); // globally
	
	// Setup Novodex finger joint
	fingerJointDesc.actor[0] = pJointActor->getNxActor();
	fingerJointDesc.actor[1] = pFingerActor->getNxActor();
	fingerJointDesc.setGlobalAnchor(anchor);
	fingerJointDesc.setGlobalAxis(axis);
	
	{
		CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
		
		NxJoint *pNxJoint = scene.getNxScene()->createJoint(fingerJointDesc);
		if (pNxJoint == NULL || (pFingerJoint = pNxJoint->isD6Joint()) == NULL)
			throw MsgFinger(Message::LEVEL_CRIT, "Finger::create(): Unable to create finger joint");
	}

	pFingerRenderer.reset(new FingerRenderer(pFingerCtrl->getReacPlanner()));
	
	return true;
}

void Finger::release() {
	if (pFingerActor != NULL) {
		this->getScene().releaseObject(*pFingerActor);
		pFingerActor = NULL;
	}
	if (pFingerJoint != NULL) {
		CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
		
		scene.getNxScene()->releaseJoint(*pFingerJoint);
		pFingerJoint = NULL;
	}
	
	Base::release();
}
	
//------------------------------------------------------------------------------

void Finger::postprocess(SecTmReal elapsedTime) {
	pFingerRenderer->create(fingerRendererDesc);
}

void Finger::render() {
	if (fingerRendererShow)
		pFingerRenderer->render();
}

void Finger::keyboardHandler(unsigned char key, int x, int y) {
	switch (key) {
	case 3:// F3
		fingerRendererShow = !fingerRendererShow;
		break;
	}
}

//------------------------------------------------------------------------------
