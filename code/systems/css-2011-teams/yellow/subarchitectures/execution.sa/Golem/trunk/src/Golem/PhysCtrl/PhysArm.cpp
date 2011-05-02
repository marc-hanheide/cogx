/** @file PhysArm.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/PhysCtrl/PhysArm.h>
#include <Golem/PhysCtrl/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

PoseRenderer::PoseRenderer(PhysArm &physArm) : physArm(physArm) {
}

bool PoseRenderer::create(const Desc &desc) {
	if (!desc.isValid())
		return false;

	// reset memory buffer
	reset();

	const Mat34 referencePose = physArm.getArm().getReferencePose();
	Mat34 trn;
	GenConfigspaceState state;
	
	// base frame
	trn = physArm.getArm().getGlobalPose(); // base pose
	addAxes(trn, desc.axesSize);

	// get the current (planned) extended pose
	if (!physArm.getArm().lookupCommand(state, physArm.getContext().getTimer().elapsed()))
		return false;

	// translate pose
	physArm.getArm().forwardTransform(trn, state.pos);
	trn.multiply(trn, referencePose); // reference pose
	addAxes(trn, desc.axesSize);

	// get the destination (planned) extended pose
	if (!physArm.getArm().lookupCommand(state, SEC_TM_REAL_MAX))
		return false;

	// translate pose
	physArm.getArm().forwardTransform(trn, state.pos);
	trn.multiply(trn, referencePose); // reference pose
	addAxes(trn, desc.axesSize);

	return true;
}

//------------------------------------------------------------------------------

PathRenderer::PathRenderer(PhysArm &physArm) : physArm(physArm) {
}

bool PathRenderer::create(const Desc &desc) {
	if (!desc.isValid())
		return false;

	Arm &arm = physArm.getArm();
	const U32 numOfJoints = U32(arm.getJoints().size());	
	const SecTmReal tmCurrent = physArm.getContext().getTimer().elapsed();	
	
	states.resize(desc.trjSegments);

	// get the first and last (planned) extended pose
	const States::const_iterator end = arm.lookupCommand(states.begin(), states.end(), tmCurrent + desc.tmDelta0, tmCurrent + desc.tmDelta1);
	if (end == states.begin())
		return false;

	// reset memory buffer
	reset();

	const Mat34 referencePose = arm.getReferencePose();
	Mat34 trn0, trn1;
	Vec3 p0, p1;
	
	States::const_iterator i = states.begin();
	arm.forwardTransform(trn0, i->pos);
	trn0.multiply(trn0, referencePose); // reference pose
	
	if (desc.axesShow)
		addAxes(trn0, desc.axesSize);

	while (++i != end) {
		arm.forwardTransform(trn1, i->pos);
		trn1.multiply(trn1, referencePose); // reference pose
		
		if (desc.trjShow) {
			if (desc.trjGrid > 0) {
				GenCoordTrj trajectories[CONFIG_SPACE_DIM];

				for (U32 j = 0; j < numOfJoints; j++)
					trajectories[j].set(Real((i-1)->t), Real(i->t), (i-1)->get(j), i->get(j));

				Mat34 trn;
				for (U32 k = 0; k < desc.trjGrid; k++) {
					GenConfigspaceCoord jc;
					for (U32 j = 0; j < numOfJoints; j++)
						jc.set(j, trajectories[j].get(Real((i-1)->t) + Real(k + 1)*Real(i->t - (i-1)->t)/Real(desc.trjGrid + 1)));
					arm.forwardTransform(trn, jc.pos);
					trn.multiply(trn, referencePose); // reference pose
					
					addLine(trn0.p, trn.p, desc.trjColour);
					trn0 = trn;
				}
			}

			addLine(trn0.p, trn1.p, desc.trjColour);
		}
		if (desc.axesShow)
			addAxes(trn1, desc.axesSize);
		
		trn0 = trn1;
	}

	return true;
}

//------------------------------------------------------------------------------

bool JointActor::create(const JointActor::Desc& desc) {
	Actor::create(desc); // throws

	pJoint = desc.pJoint;
	pPhysArm = desc.pPhysArm;

	// install callback interface
	pJoint->setCallback(this);

	return true;
}

JointActor::JointActor(Scene &scene) : Actor(scene) {
}

const Bounds* JointActor::createBounds(Bounds::Desc::Ptr pDesc) {
	const Bounds* pBounds = Actor::createBounds(pDesc);
	
	if (pBounds != NULL) {
		Actor::setBoundsGroup(*pBounds, pPhysArm->getArmBoundsGroup());
		pJoint->addBoundsDesc(pDesc);
	}

	return pBounds;
}

void JointActor::releaseBounds(const Bounds& bounds) {
	const Bounds::Desc* pDesc = Actor::getBoundsDesc(bounds);
	
	if (pDesc != NULL)
		pJoint->removeBoundsDesc(pDesc);
	
	Actor::releaseBounds(bounds);
}

void JointActor::syncJointBoundsDesc() {
	if (pPhysArm->autoSyncArmBoundsDesc)
		pPhysArm->syncArmBoundsDesc();
}

//------------------------------------------------------------------------------

PhysArm::PhysArm(Scene &scene) : Object(scene) {
	pPoseRenderer.reset(new PoseRenderer(*this));
	pPathRenderer.reset(new PathRenderer(*this));
}

PhysArm::~PhysArm() {
}

bool PhysArm::create(const PhysArm::Desc& desc) {
	Object::create(desc); // throws

	autoSyncArmBoundsDesc = desc.autoSyncArmBoundsDesc;

	appearance = desc.appearance;
	poseRendererDesc = desc.poseRendererDesc;
	poseShow = desc.poseShow;
	pathRendererDesc = desc.pathRendererDesc;
	pathShow = desc.pathShow;

	Arm::Desc::Ptr pArmDesc = desc.pArmDesc != NULL ? desc.pArmDesc : Arm::Desc::load(context, desc.driver);
	pArm = pArmDesc->create(context); // throws
	
	armGroup = scene.createBoundsGroup();
	if (armGroup == Bounds::GROUP_UNDEF)
		throw MsgPhysArmBoundsGroupCreate(Message::LEVEL_CRIT, "PhysArm::create(): Unable to create Arm bounds group");
	collisionGroup = Bounds::GROUP_ALL & ~armGroup;
	
	const U32 numJoints = (U32)pArm->getJoints().size();

	jointActors.resize(numJoints);
	jointPoses.resize(numJoints);

	for (U32 j = 0; j < numJoints; j++)
		createJointActor(jointActors[j], pArm->getJoints()[j]); // throws

	// initialise joints pose
	(void)syncJointActorsPose(SEC_TM_REAL_ZERO, false);
	
	return true;
}

void PhysArm::release() {
	for (JointActor::Seq::iterator i = jointActors.begin(); i != jointActors.end(); i++)
		if (*i != NULL) {
//			scene.removeDependency(*i);
			scene.releaseObject(**i);
		}

	jointActors.clear();
	//pArm.release();
}

//------------------------------------------------------------------------------

void PhysArm::createJointActor(JointActor *&pJointActor, Joint* pJoint) {
	// reset pointers to Joint Actors
	pJointActor = NULL;

	// check if the bounds are not empty
	Bounds::Desc::SeqPtr pBoundsDescSeq = pJoint->getBoundsDescSeq();
	if (pBoundsDescSeq->empty())
		return; // nothing to do

	// create Joint Actors
	NxBodyDesc nxBodyDesc;

	JointActor::Desc jointActorDesc;
	jointActorDesc.kinematic = true;
	jointActorDesc.nxActorDesc.body = &nxBodyDesc;
	jointActorDesc.nxActorDesc.density = (NxReal)1.0;
	jointActorDesc.appearance = appearance;
	jointActorDesc.pJoint = pJoint;
	jointActorDesc.pPhysArm = this;
		
	for (Bounds::Desc::Seq::const_iterator i = pBoundsDescSeq->begin(); i != pBoundsDescSeq->end(); i++) {
		// Cloning descriptions results in different pointers in Joint and in Actor, therefore Actor::getBoundsDesc() will point
		// to bounds description which are *not* in Joint.
		// In effect PhysArm::JointActor::releaseBounds() will *not* work, which is fine because these are initial bounds
		// which cannot be released anyway (see Actor::releaseBounds()).
		// On the other hand PhysArm::JointActor::createBounds() and PhysArm::JointActor::releaseBounds() will work fine, because
		// smart pointers are copied to both Joint and Actor.
		NxShapeDesc *pNxShapeDesc = scene.createNxShapeDesc((*i)->clone()); // clone description
		if (pNxShapeDesc == NULL)
			throw MsgPhysArmShapeDescCreate(Message::LEVEL_CRIT, "PhysArm::createJointActor(): Unable to create shape description of %s", pJoint->getName().c_str());

		jointActorDesc.nxActorDesc.shapes.push_back(pNxShapeDesc);
	}

	pJointActor = dynamic_cast<JointActor*>(scene.createObject(jointActorDesc)); // throws
	if (pJointActor == NULL)
		throw MsgPhysArmJointActor(Message::LEVEL_CRIT, "PhysArm::createJointActor(): Unable to create Joint Actor");

	pJointActor->setBoundsGroup(getArmBoundsGroup());
}

bool PhysArm::syncJointActorsPose(SecTmReal t, bool bMove) {
	// get the current time and input pose (input to the controller - the desired pose)
	if (!pArm->lookupCommand(state, t))
		return false;
	
	// compute full forward kinematics
	pArm->forwardTransformEx(&jointPoses.front(), state.pos);

	// update positions of all visible joints
	{
		CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
		
		const U32 numJoints = (U32)pArm->getJoints().size();
		for (U32 i = 0; i < numJoints; i++) {
			JointActor *pJointActor = jointActors[i];
			if (pJointActor != NULL) {
				NxMat34 nxPose(false);
				nxPose.M.setRowMajor(&jointPoses[i].R.m11);
				nxPose.t.set(&jointPoses[i].p.v1);

				if (bMove)
					pJointActor->getNxActor()->moveGlobalPose(nxPose);
				// set the actor pose in any case to avoid peculiar PhysX behavior (at small scale) when moving pose only
				pJointActor->getNxActor()->setGlobalPose(nxPose);
			}
		}
	}

	return true;
}

void PhysArm::keyboardHandler(unsigned char key, int x, int y) {
	switch (key) {
	case 1:// F1
		poseShow = !poseShow;
		break;
	case 2:// F2
		pathShow = !pathShow;
		break;
	}
}

void PhysArm::preprocess(SecTmReal elapsedTime) {
	(void)syncJointActorsPose(context.getTimer().elapsed());
}

void PhysArm::postprocess(SecTmReal elapsedTime) {
	// pose renderer
	if (poseShow && !pPoseRenderer->create(poseRendererDesc))
		context.getMessageStream()->write(Message::LEVEL_WARNING, "PhysArm::postprocess(): Unable to create pose renderer");
	
	// path renderer
	if (pathShow && !pPathRenderer->create(pathRendererDesc))
		context.getMessageStream()->write(Message::LEVEL_WARNING, "PhysArm::postprocess(): Unable to create path renderer");
}

void PhysArm::render() {
	if (poseShow)
		pPoseRenderer->render();
	if (pathShow)
		pPathRenderer->render();
}

//------------------------------------------------------------------------------

Bounds::SeqPtr PhysArm::getCollisionBounds() const {
	const U32 group = getCollisionBoundsGroup() & ~getArmBoundsGroup();
	Bounds::SeqPtr pCollisionBounds(new Bounds::Seq());
	
	CriticalSectionWrapper csw(scene.getCSObjectList());

	const Scene::ObjectList& objectList = scene.getObjectList();
	for (Scene::ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); i++) {
		Actor *pActor = dynamic_cast<Actor*>(*i);
		if (pActor == NULL)
			continue;

		Bounds::SeqPtr pBoundsSeq = pActor->getGlobalBoundsSeq(group);
		pCollisionBounds->insert(pCollisionBounds->end(), pBoundsSeq->begin(), pBoundsSeq->end());
	}
	
	//context.getMessageStream()->write(Message::LEVEL_DEBUG, "arm: %08x, collision: %08x, group: %08x, size: %d", getArmBoundsGroup(), getCollisionBoundsGroup(), group, pCollisionBounds->size());

	return pCollisionBounds;
}

Bounds::SeqPtr PhysArm::getArmBounds() const {
	const U32 group = getArmBoundsGroup();
	Bounds::SeqPtr pArmBounds(new Bounds::Seq());

	for (JointActor::Seq::const_iterator i = jointActors.begin(); i != jointActors.end(); i++) {
		JointActor *pJointActor = *i;
		if (pJointActor == NULL)
			continue;

		Bounds::SeqPtr pBoundsSeq = pJointActor->getGlobalBoundsSeq(group);
		pArmBounds->insert(pArmBounds->end(), pBoundsSeq->begin(), pBoundsSeq->end());
	}

	return pArmBounds;
}

U32 PhysArm::getArmBoundsGroup() const {
	return armGroup;
}

U32 PhysArm::getCollisionBoundsGroup() const {
	return collisionGroup;
}

void PhysArm::setCollisionBoundsGroup(U32 collisionGroup) {
	this->collisionGroup = collisionGroup;
}

//------------------------------------------------------------------------------

