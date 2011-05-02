/** @file Object.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Phys/Object.h>
#include <Golem/Phys/Universe.h>
#include <Golem/Phys/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Object::Object(Scene &scene) : scene(scene), universe(scene.getUniverse()), context(scene.getContext()) {
	pNxPhysicsSDK = scene.getNxPhysicsSDK();
	pNxScene = scene.getNxScene();
}

bool Object::create(const Object::Desc& desc) {
	if (!desc.isValid())
		throw MsgObjectInvalidDesc(Message::LEVEL_CRIT, "Object::create(): Invalid description");

	return true;
}

void Object::release() {
}

//------------------------------------------------------------------------------

Actor::Actor(Scene &scene) : Object(scene) {
}

Actor::~Actor() {
}

bool Actor::create(const Actor::Desc& desc) {
	Object::create(desc); // throws

	CriticalSectionWrapper csw(csBoundsSeq);
	
	boundsDataSeq.clear();
	boundsConstSeq.clear();

	for (NxArray<NxShapeDesc*, NxAllocatorDefault>::const_iterator i = desc.nxActorDesc.shapes.begin(); i != desc.nxActorDesc.shapes.end(); i++) {
		BoundsData boundsData;

		boundsData.pBoundsDesc = scene.createBoundsDesc(**i); // Access to PhysX (for ConvexMesh)
		if (boundsData.pBoundsDesc == NULL)
			throw MsgActorBoundsDescCreate(Message::LEVEL_CRIT, "Actor::create(): Unable to create bounds description");

		boundsData.pBounds = boundsData.pBoundsDesc->create();
		if (boundsData.pBounds == NULL)
			throw MsgActorBoundsCreate(Message::LEVEL_CRIT, "Actor::create(): Unable to create bounds");

		boundsDataSeq.insert(std::pair<const Bounds*, BoundsData>(boundsData.pBounds.get(), boundsData));
		boundsConstSeq.push_back(boundsData.pBounds.get());
	}

	{
		CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
	
		pNxActor = scene.getNxScene()->createActor(desc.nxActorDesc);
		if (pNxActor == NULL)
			throw MsgActorNxActorCreate(Message::LEVEL_CRIT, "Actor::create(): Unable to create NxActor");

		if (desc.kinematic)
			pNxActor->raiseBodyFlag(NX_BF_KINEMATIC);

		const NxU32 nbShapes = pNxActor->getNbShapes();
		const NxShape* const* ppNxShapes = pNxActor->getShapes();

		U32 j = 0;
		for (BoundsConstSeq::const_iterator i = boundsConstSeq.begin(); i != boundsConstSeq.begin(); i++, j++) {
			BoundsData::Seq::iterator pos = boundsDataSeq.find(*i);
			if (pos == boundsDataSeq.end() || j >= nbShapes)
				throw MsgActorBoundsNxShapeMismatch(Message::LEVEL_CRIT, "Actor::createShape(): Bounds and NxShapes mismatch");

			pos->second.pNxShape = NULL;
			pos->second.pConstNxShape = ppNxShapes[j]; // assuming that order of NxActorDesc::shapes is preserved
		}
	}

	appearance = desc.appearance;
	
	return true;
}

void Actor::release() {
	{
		CriticalSectionWrapper csw(csBoundsSeq);
		boundsConstSeq.clear();
		boundsDataSeq.clear();
	}

	if (pNxActor != NULL) {
		CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
		scene.getNxScene()->releaseActor(*pNxActor);
		pNxActor = NULL;
	}

	Object::release();
}

//------------------------------------------------------------------------------

Mat34 Actor::getPose() const {
	NxMat34 nxPose;
	{
		CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
		nxPose = pNxActor->getGlobalPose();
	}

	Mat34 pose;
	nxPose.M.getRowMajor(&pose.R.m11);
	nxPose.t.get(&pose.p.v1);

	return pose;
}

void Actor::setPose(const Mat34& pose) {
	if (!pNxActor->readBodyFlag(NX_BF_KINEMATIC)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Actor::setPose(): Cannot change pose of dynamic actors");
		return;
	}

	NxMat34 nxPose;
	nxPose.M.setRowMajor(&pose.R.m11);
	nxPose.t.set(&pose.p.v1);
	
	CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
	pNxActor->moveGlobalPose(nxPose);
}

const Bounds* Actor::createBounds(Bounds::Desc::Ptr pDesc) {
	BoundsData boundsData;
	
	boundsData.pBoundsDesc = pDesc;
	NxShapeDesc* pNxShapeDesc = scene.createNxShapeDesc(boundsData.pBoundsDesc); // Access to PhysX (for ConvexMesh)
	if (pNxShapeDesc == NULL)
		throw MsgActorNxShapeDescCreate(Message::LEVEL_CRIT, "Actor::createBounds(): Unable to create NxShape description");

	{
		CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
		boundsData.pConstNxShape = NULL;
		boundsData.pNxShape = pNxActor->createShape(*pNxShapeDesc);
		if (boundsData.pNxShape == NULL)
			throw MsgActorNxShapeCreate(Message::LEVEL_CRIT, "Actor::createBounds(): Unable to create NxShape");
	}

	boundsData.pBounds = boundsData.pBoundsDesc->create();
	if (boundsData.pBounds == NULL)
		throw MsgActorBoundsCreate(Message::LEVEL_CRIT, "Actor::createBounds(): Unable to create bounds");

	CriticalSectionWrapper csw(csBoundsSeq);
	boundsDataSeq.insert(std::pair<const Bounds*, BoundsData>(boundsData.pBounds.get(), boundsData));
	boundsConstSeq.push_back(boundsData.pBounds.get());

	return boundsData.pBounds.get();
}

void Actor::releaseBounds(const Bounds& bounds) {
	BoundsData::Seq::iterator pos = boundsDataSeq.find(&bounds);
	if (pos == boundsDataSeq.end()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Actor::releaseBounds(): Unable to find specified bounds");
		return;
	}
	
	if (pos->second.pNxShape == NULL) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Actor::releaseBounds(): Initial Actor bounds cannot be released");
		return;
	}

	{
		CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
		pNxActor->releaseShape(*pos->second.pNxShape);
	}
	
	scene.releaseBoundsDesc(pos->second.pBoundsDesc);
	
	{		
		CriticalSectionWrapper csw(csBoundsSeq);
		boundsDataSeq.erase(pos);
		boundsConstSeq.erase(std::find(boundsConstSeq.begin(), boundsConstSeq.end(), &bounds));// linear search
	}
}

const Bounds::Desc* Actor::getBoundsDesc(const Bounds& bounds) const {
	BoundsData::Seq::const_iterator pos = boundsDataSeq.find(&bounds);
	if (pos == boundsDataSeq.end()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Actor::getBoundsDesc(): Unable to find specified bounds");
		return NULL;
	}
	
	return pos->second.pBoundsDesc.get();
}

const Actor::BoundsConstSeq& Actor::getBoundsSeq() const {
//	CriticalSectionWrapper csw(csBoundsSeq);
	return boundsConstSeq;
}

Bounds::SeqPtr Actor::getLocalBoundsSeq(U32 group) const {
	CriticalSectionWrapper csw(csBoundsSeq);
	Bounds::SeqPtr pBoundsSeq = Bounds::clone(boundsConstSeq.begin(), boundsConstSeq.end(), group);

	return pBoundsSeq;
}

Bounds::SeqPtr Actor::getGlobalBoundsSeq(U32 group) const {
	Mat34 pose = getPose();
	
	CriticalSectionWrapper csw(csBoundsSeq);
	Bounds::SeqPtr pBoundsSeq = Bounds::clone(boundsConstSeq.begin(), boundsConstSeq.end(), group);
	Bounds::multiplyPose(pose, pBoundsSeq->begin(), pBoundsSeq->end());
	
	return pBoundsSeq;
}

//------------------------------------------------------------------------------

void Actor::setBoundsGroup(const Bounds& bounds, U32 group) {
	BoundsData::Seq::iterator pos = boundsDataSeq.find(&bounds);
	if (pos == boundsDataSeq.end()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Actor::setBoundsGroup(): Unable to find specified bounds");
		return;
	}
	
	CriticalSectionWrapper csw(csBoundsSeq);
	pos->second.pBounds->setGroup(group);
}

void Actor::setBoundsGroup(U32 group) {
	CriticalSectionWrapper csw(csBoundsSeq);
	for (BoundsData::Seq::iterator i = boundsDataSeq.begin(); i != boundsDataSeq.end(); i++)
		i->second.pBounds->setGroup(group);
}

//------------------------------------------------------------------------------

Actor::Appearance Actor::getAppearance() const {
	return appearance;
}

void Actor::setAppearance(const Appearance &appearance) {
	this->appearance = appearance;
}

//------------------------------------------------------------------------------

void Actor::render() {
	CriticalSectionWrapper csw(csBoundsSeq);
	if (boundsConstSeq.empty())
		return;
	
	boundsRenderer.setMat(getPose());
	
	Scene::Draw draw;
	scene.getDraw(draw);

	if (draw.solid) {
		boundsRenderer.setSolidColour(appearance.solidColour);
		boundsRenderer.renderSolid(boundsConstSeq.begin(), boundsConstSeq.end());
	}
	if (draw.wire) {
		boundsRenderer.setWireColour(appearance.wireColour);
		boundsRenderer.setLineWidth(appearance.lineWidth);
		boundsRenderer.renderWire(boundsConstSeq.begin(), boundsConstSeq.end());
	}
	if (draw.shadow) {
		boundsRenderer.setShadowColour(appearance.shadowColour);
		boundsRenderer.renderShadow(boundsConstSeq.begin(), boundsConstSeq.end());
	}
}

//------------------------------------------------------------------------------
