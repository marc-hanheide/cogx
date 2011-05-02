/** @file TinyI.cpp
 * 
 * Implementation of Golem Tiny Ice server (source file).
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/TinyIce/Types.h>
#include <Golem/TinyIce/TinyIceI.h>
#include <Golem/Phys/Data.h>
#include <Golem/PhysCtrl/Msg.h>
#include <Golem/PhysCtrl/Creator.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Golem/Device/Katana/Data.h>
#include <IceUtil/UUID.h>

using namespace golem::tinyice;

//------------------------------------------------------------------------------

class golem::tinyice::BoundsDesc {
public:
	golem::Bounds::Desc::Ptr pBoundsDesc;
	BoundsDesc(const golem::Bounds::Desc::Ptr pBoundsDesc) : pBoundsDesc(pBoundsDesc) {}
};

class golem::tinyice::PhysPlannerDesc : public golem::PhysPlanner::Desc {
public:
};

//------------------------------------------------------------------------------

ShapeI::ShapeI(RigidBodyI &rigidBody) : rigidBody(rigidBody), adapter(rigidBody.adapter), pNxShapeDesc(NULL), pBounds(NULL), owner(false) {
	id.name = IceUtil::generateUUID(); 
}

ShapeI::~ShapeI() {
	if (pBounds == NULL || !owner)
		return;
	rigidBody.pActor->releaseBounds(*pBounds);
}

bool ShapeI::create(const ShapeDesc& desc, ShapeType type, const golem::shared_ptr<BoundsDesc>& pBoundsDesc) {
	this->type = type;
	this->density = desc.density;
	this->color = desc.color;

	// pBoundsDesc must be initialized before
	if (pBoundsDesc == NULL)
		throw ExTinyShape("ShapeI::create(): Unable to create bounds description");
	
	this->pBoundsDesc = pBoundsDesc;

	pNxShapeDesc = rigidBody.scene.createNxShapeDesc(this->pBoundsDesc->pBoundsDesc); // throws
	if (pNxShapeDesc == NULL)
		return false;
	pNxShapeDesc->density = NxReal(this->density);
	// pBounds created in container

	return true;
}

ShapePrx ShapeI::activate() {
	return ShapePrx::uncheckedCast(adapter->add(this, id));
}

void ShapeI::deactivate() {
	adapter->remove(id);
}

ShapeType ShapeI::getType(const ::Ice::Current&) const {
	return type;
}

RGBA ShapeI::getColor(const ::Ice::Current&) const {
	return color;
}

Mat34 ShapeI::getLocalPose(const ::Ice::Current&) const {
	return make(pBounds->getPose());
}

::Ice::Int ShapeI::getGroup(const ::Ice::Current&) const {
	return (::Ice::Int)pBounds->getGroup();
}

void ShapeI::setGroup(::Ice::Int group, const ::Ice::Current&) {
	rigidBody.pActor->setBoundsGroup(*pBounds, U32(group));
}

ShapeDescPtr ShapeI::getDesc(const ::Ice::Current&) const {
	return pShapeDesc;
}

//------------------------------------------------------------------------------

PlaneShapeI::PlaneShapeI(RigidBodyI &rigidBody) : ShapeI(rigidBody) {
}
bool PlaneShapeI::create(const PlaneShapeDesc& desc) {
	BoundingPlane::Desc* pDesc = new BoundingPlane::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = make(desc.localPose);
	pDesc->normal = make(desc.normal);
	pDesc->distance = (Real)desc.distance;
	return ShapeI::create(desc, ShapeTypePlane, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

SphereShapeI::SphereShapeI(RigidBodyI &rigidBody) : ShapeI(rigidBody) {
}
bool SphereShapeI::create(const SphereShapeDesc& desc) {
	BoundingSphere::Desc* pDesc = new BoundingSphere::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = make(desc.localPose);
	pDesc->radius = (Real)desc.radius;
	return ShapeI::create(desc, ShapeTypeSphere, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

CylinderShapeI::CylinderShapeI(RigidBodyI &rigidBody) : ShapeI(rigidBody) {
}
bool CylinderShapeI::create(const CylinderShapeDesc& desc) {
	BoundingCylinder::Desc* pDesc = new BoundingCylinder::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = make(desc.localPose);
	pDesc->radius = (Real)desc.radius;
	pDesc->length = (Real)desc.length;
	return ShapeI::create(desc, ShapeTypeCylinder, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

BoxShapeI::BoxShapeI(RigidBodyI &rigidBody) : ShapeI(rigidBody) {
}
bool BoxShapeI::create(const BoxShapeDesc& desc) {
	BoundingBox::Desc* pDesc = new BoundingBox::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = make(desc.localPose);
	pDesc->dimensions = make(desc.dimensions);
	return ShapeI::create(desc, ShapeTypeBox, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

ConvexMeshShapeI::ConvexMeshShapeI(RigidBodyI &rigidBody) : ShapeI(rigidBody) {
}
bool ConvexMeshShapeI::create(const ConvexMeshShapeDesc& desc) {
	BoundingConvexMesh::Desc* pDesc = new BoundingConvexMesh::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = make(desc.localPose);
	for (::golem::tinyice::Vec3Seq::const_iterator i = desc.vertices.begin(); i != desc.vertices.end(); ++i)
		pDesc->vertices.push_back(make(*i));
	pDesc->bCook = true;
	return ShapeI::create(desc, ShapeTypeConvexMesh, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

//------------------------------------------------------------------------------

ActorI::ActorI(TinyI &tiny) : tiny(tiny), adapter(tiny.adapter), context(*tiny.pContext), scene(*tiny.pScene), pXMLContext(tiny.pXMLContext), owner(false) {
	id.name = IceUtil::generateUUID(); 
}

ActorI::~ActorI() {
}

ActorPrx ActorI::activate() {
	return ActorPrx::uncheckedCast(adapter->add(this, id));
}

void ActorI::deactivate() {
	adapter->remove(id);
}

RigidBodyI::RigidBodyI(TinyI &tiny) : ActorI(tiny), pActor(NULL) {
}

RigidBodyI::~RigidBodyI() {
	shapeList.clear(); // release shapes before releasing the Actor
	if (pActor == NULL || !owner)
		return;
	scene.releaseObject(*pActor);
}

ShapeIPtr RigidBodyI::createShape(const ShapeDesc* pDesc) {
	if (!pDesc)
		throw ExTinyShapeCreate("RigidBodyI::createShape(): NULL Shape description");
	else if (dynamic_cast<const PlaneShapeDesc*>(pDesc)) {
		PlaneShapeI* pPlaneShape = new PlaneShapeI(*this);
		ShapeIPtr pShape(pPlaneShape);
		if (pPlaneShape->create(*dynamic_cast<const PlaneShapeDesc*>(pDesc)))
			return pShape;
	}
	else if (dynamic_cast<const SphereShapeDesc*>(pDesc)) {
		SphereShapeI* pSphereShape = new SphereShapeI(*this);
		ShapeIPtr pShape(pSphereShape);
		if (pSphereShape->create(*dynamic_cast<const SphereShapeDesc*>(pDesc)))
			return pShape;
	}
	else if (dynamic_cast<const CylinderShapeDesc*>(pDesc)) {
		CylinderShapeI* pCylinderShape = new CylinderShapeI(*this);
		ShapeIPtr pShape(pCylinderShape);
		if (pCylinderShape->create(*dynamic_cast<const CylinderShapeDesc*>(pDesc)))
			return pShape;
	}
	else if (dynamic_cast<const BoxShapeDesc*>(pDesc)) {
		BoxShapeI* pBoxShape = new BoxShapeI(*this);
		ShapeIPtr pShape(pBoxShape);
		if (pBoxShape->create(*dynamic_cast<const BoxShapeDesc*>(pDesc)))
			return pShape;
	}
	else if (dynamic_cast<const ConvexMeshShapeDesc*>(pDesc)) {
		ConvexMeshShapeI* pConvexMeshShape = new ConvexMeshShapeI(*this);
		ShapeIPtr pShape(pConvexMeshShape);
		if (pConvexMeshShape->create(*dynamic_cast<const ConvexMeshShapeDesc*>(pDesc)))
			return pShape;
	}
	else
		throw ExTinyShapeCreate("RigidBodyI::createShape(): Unknown Shape description");

	throw ExTinyShapeCreate("RigidBodyI::createShape(): Unable to create Shape");
}

ShapeIPtr RigidBodyI::createShape(const golem::Bounds* pBounds) {
	ShapeDescPtr pShapeDesc;
	ShapeIPtr pShape;

	switch (pBounds->getType()) {
	case Bounds::TYPE_PLANE:
		{
			PlaneShapeDesc* pDesc = new PlaneShapeDesc;
			pShapeDesc = pDesc;
			pDesc->normal = make(static_cast<const BoundingPlane*>(pBounds)->getNormal());
			pDesc->distance = (Ice::Double)static_cast<const BoundingPlane*>(pBounds)->getDistance();
		}
		break;
	case Bounds::TYPE_SPHERE:
		{
			SphereShapeDesc* pDesc = new SphereShapeDesc;
			pShapeDesc = pDesc;
			pDesc->radius = (Ice::Double)static_cast<const BoundingSphere*>(pBounds)->getRadius();
		}
		break;
	case Bounds::TYPE_CYLINDER:
		{
			CylinderShapeDesc* pDesc = new CylinderShapeDesc;
			pShapeDesc = pDesc;
			pDesc->radius = (Ice::Double)static_cast<const BoundingCylinder*>(pBounds)->getRadius();
			pDesc->length = (Ice::Double)static_cast<const BoundingCylinder*>(pBounds)->getLength();
		}
		break;
	case Bounds::TYPE_BOX:
		{
			BoxShapeDesc* pDesc = new BoxShapeDesc;
			pShapeDesc = pDesc;
			pDesc->dimensions = make(static_cast<const BoundingBox*>(pBounds)->getDimensions());
		}
		break;
	case Bounds::TYPE_CONVEX_MESH:
		{
			ConvexMeshShapeDesc* pDesc = new ConvexMeshShapeDesc;
			pShapeDesc = pDesc;
			const BoundingConvexMesh* pBoundingConvexMesh = static_cast<const BoundingConvexMesh*>(pBounds);
			for (std::vector<golem::Vec3>::const_iterator i = pBoundingConvexMesh->getVertices().begin(); i != pBoundingConvexMesh->getVertices().end(); ++i)
				pDesc->vertices.push_back(make(*i));
		}
		break;
	}
	if (pShapeDesc == NULL)
		return pShape;

	pShapeDesc->localPose = make(pBounds->getPose());
	pShapeDesc->group = (Ice::Int)pBounds->getGroup();

	pShape = createShape(pShapeDesc.get());// throws
	if (pShape == NULL)
		return pShape;
	pShape->pShapeDesc = pShapeDesc;
	pShape->pBounds = pBounds;
	
	return pShape;
}

bool RigidBodyI::create(const RigidBodyDesc& desc) {
	kinematic = desc.kinematic;

	golem::Actor::Desc actorDesc;
	actorDesc.kinematic = desc.kinematic;
	//actorDesc.appearance.solidColour = ;
	actorDesc.nxActorDesc.globalPose.M.setRowMajor(&desc.globalPose.R.m11);
	actorDesc.nxActorDesc.globalPose.t.set(&desc.globalPose.p.v1);
	NxBodyDesc nxBodyDesc;
	actorDesc.nxActorDesc.body = &nxBodyDesc;
	actorDesc.nxActorDesc.density = NxReal(1.0);

	shapeList.clear();
	for (ShapeDescSeq::const_iterator i = desc.shapes.begin(); i != desc.shapes.end(); i++) {
		ShapeIPtr pShape = createShape((*i).get()); // throws
		if (pShape == NULL || pShape->pNxShapeDesc == NULL)
			return false;
		pShape->pShapeDesc = *i;

		if (pShape->getType() == ShapeTypePlane)
			actorDesc.nxActorDesc.body = NULL; // Actors with Plane Shapes cannot have a body
		actorDesc.nxActorDesc.shapes.push_back(pShape->pNxShapeDesc);
		
		ShapePrx shapePrx = pShape->activate();
		shapeList.push_back(ShapeList::Pair(shapePrx, pShape));
	}

	pActor = dynamic_cast<golem::Actor*>(scene.createObject(actorDesc)); // throws
	if (pActor == NULL)
		return false;
	
	const golem::Actor::BoundsConstSeq& boundsSeq = pActor->getBoundsSeq();
	golem::Actor::BoundsConstSeq::const_iterator i = boundsSeq.begin();
	ShapeList::iterator j = shapeList.begin();
	while (i != boundsSeq.end() && j != shapeList.end())
		(j++)->second->pBounds = *i++; // order is preserved
	
	return true;
}

ActorPrx RigidBodyI::activate() {
	return ActorI::activate();
}

void RigidBodyI::deactivate() {
	ActorI::deactivate();
	
	// deactivate shapes
	for (ShapeList::iterator i = shapeList.begin(); i != shapeList.end(); i++)
		i->second->deactivate();
}

ShapePrx RigidBodyI::createShape(const ::golem::tinyice::ShapeDescPtr& pShapeDesc, const ::Ice::Current&) {
	try {
		if (pShapeDesc == NULL)
			throw ExTinyShapeCreate("RigidBodyI::createShape(): empty description");

		ShapeIPtr pShape = createShape(pShapeDesc.get()); // throws
		if (pShape == NULL)
			return NULL;
		pShape->pShapeDesc = pShapeDesc;
		
		pShape->pBounds = pActor->createBounds(pShape->pBoundsDesc->pBoundsDesc); // throws
		if (pShape->pBounds == NULL)
			return NULL;
		pShape->owner = true;

		ShapePrx shapePrx = pShape->activate();
		shapeList.push_back(ShapeList::Pair(shapePrx, pShape));
		return shapePrx;
	}
	catch (const Message& ex) {
		std::string str("RigidBodyI::createShape(): ");
		str.append(ex.str());
		throw ExTinyShapeCreate(str);
	}
	catch (const std::exception &ex) {
		std::string str("RigidBodyI::createShape(): C++ exception: ");
		str.append(ex.what());
		throw ExTinyShapeCreate(str);
	}

	return ShapePrx();
}

void RigidBodyI::releaseShape(const ShapePrx& shape, const ::Ice::Current&) {
	ShapeList::iterator pos = shapeList.find(shape);
	if (pos == shapeList.end())
		throw ExTinyShapeNotFound("RigidBodyI::releaseShape(): Unable to find specified Shape");
	if (!pos->second->owner)
		throw ExTinyShapeNotRemovable("RigidBodyI::releaseShape(): The specified Shape cannot be removed");

	pos->second->deactivate();
	shapeList.erase(pos);
}

ShapeSeq RigidBodyI::getShapes(const ::Ice::Current&) const {
	ShapeSeq shapeSeq;
	for (ShapeList::const_iterator i = shapeList.begin(); i != shapeList.end(); i++)
		shapeSeq.push_back(i->first);
	return shapeSeq;
}

Mat34 RigidBodyI::getGlobalPose(const ::Ice::Current&) const {
	return make(pActor->getPose());
}

void RigidBodyI::setGlobalPose(const Mat34& pose, const ::Ice::Current&) {
	pActor->setPose(make(pose));
}

void RigidBodyI::setGroup(::Ice::Int group, const ::Ice::Current&) {
	pActor->setBoundsGroup(U32(group));
}

//------------------------------------------------------------------------------

JointI::JointI(ArmI &armI) : RigidBodyI(armI.tiny), armI(armI) {
}

bool JointI::create(const JointDesc& desc, golem::JointActor* pJointActor) {
	ASSERT(pJointActor)
	pActor = pJointActor;
	kinematic = true;
	
	shapeList.clear();
	const golem::Actor::BoundsConstSeq& boundsSeq = pActor->getBoundsSeq();
	for (golem::Actor::BoundsConstSeq::const_iterator i = boundsSeq.begin(); i != boundsSeq.end(); i++) {
		ShapeIPtr pShape = RigidBodyI::createShape(*i); // throws
		if (pShape == NULL)
			return false;
		ShapePrx shapePrx = pShape->activate();
		shapeList.push_back(ShapeList::Pair(shapePrx, pShape));
	}

	return true;
}

ArmI::ArmI(TinyI &tiny) : ActorI(tiny), pPhysPlanner(NULL) {
}

ArmI::~ArmI() {
	jointList.clear(); // joints wrappers must be released before PhysPlanner!
	if (pPhysPlanner == NULL || !owner)
		return;
	scene.releaseObject(*pPhysPlanner);
}

JointIPtr ArmI::createJoint(const JointDesc* pDesc, golem::JointActor* pJointActor) {
	if (!pDesc)
		throw ExTinyActorCreate("ArmI::createJoint(): NULL Joint description");
	else if (dynamic_cast<const JointDesc*>(pDesc)) {
		JointI* pJointI = new JointI(*this);
		JointIPtr pJoint(pJointI);
		if (pJointI->create(*static_cast<const JointDesc*>(pDesc), pJointActor))
			return pJoint;
	}
	else
		throw ExTinyActorCreate("ArmI::createJoint(): Unknown Joint description");

	throw ExTinyActorCreate("ArmI::createJoint(): Unable to create Joint");
}

bool ArmI::create(const ArmDesc& desc) {
	golem::tinyice::PhysPlannerDesc physPlannerDesc;
	physPlannerDesc.pArmDesc = golem::Arm::Desc::load(context, desc.path);
	physPlannerDesc.pArmDesc->globalPose = make(desc.globalPose);

	if (!ArmI::create(physPlannerDesc))
		return false;

	return true;
}

bool ArmI::create(const PhysPlannerDesc& desc) {
	// create controller
	pPhysPlanner = dynamic_cast<PhysPlanner*>(scene.createObject(desc));// throws
	if (pPhysPlanner == NULL)
		return false;

	// create joints
	jointList.clear();
	const JointActor::Seq& jointActorSeq = pPhysPlanner->getJointActors();
	for (JointActor::Seq::const_iterator i = jointActorSeq.begin(); i != jointActorSeq.end(); i++) {
		JointIPtr pJoint;
		if (*i != NULL) { // Joint may have no body
			JointDescPtr pJointDesc(new JointDesc);
			pJoint = createJoint(pJointDesc.get(), *i); // throws
			if (pJoint == NULL)
				return false;
			ActorPrx actorPrx = pJoint->activate();
			jointList.push_back(JointList::Pair(actorPrx, pJoint));
		}
	}

	return true;
}

ActorPrx ArmI::activate() {
	return ActorI::activate();
}

void ArmI::deactivate() {
	ActorI::deactivate();

	// deactivate joints
	for (JointList::iterator i = jointList.begin(); i != jointList.end(); i++)
		i->second->deactivate();
}

GenConfigspaceState ArmI::recvGenConfigspaceState(Ice::Double t, const ::Ice::Current&) {
	golem::GenConfigspaceState gcs;
	pPhysPlanner->getArm().lookupCommand(gcs, t);
	return make(gcs);
}

GenWorkspaceState ArmI::recvGenWorkspaceState(Ice::Double t, const ::Ice::Current&) {
	golem::GenConfigspaceState gcs;
	pPhysPlanner->getArm().lookupCommand(gcs, t);
	golem::GenWorkspaceState gws;
	pPhysPlanner->getArm().forwardTransform(gws.pos, gcs.pos);
	gws.pos.multiply(gws.pos, pPhysPlanner->getArm().getReferencePose());
	gws.vel.set(golem::Vec3(REAL_ZERO), golem::Vec3(REAL_ZERO)); // TODO
	gws.t = gcs.t;
	return make(gws);
}

void ArmI::send(const GenConfigspaceStateSeq& trajectory, double timeWait, const ::Ice::Current&) {
	const golem::Planner::Trajectory tmp(make(trajectory));
	if (!pPhysPlanner->getPlanner().send<golem::GenConfigspaceState>(tmp.begin(), tmp.end()))
		throw ExTinyArmSend("ArmI::sendTrajectory(): send trajectory error");

	golem::GenConfigspaceState gcs;
	pPhysPlanner->getArm().lookupCommand(gcs, golem::SEC_TM_REAL_INF);

	context.getTimer().sleep(Math::clamp(
		gcs.t - context.getTimer().elapsed()/* - pPhysPlanner->getArm().getTimeDeltaAsync()*/, pPhysPlanner->getArm().getTimeDeltaAsync()/*golem::SEC_TM_REAL_ZERO*/, golem::SecTmReal(timeWait)
	));
}

void ArmI::stop(const ::Ice::Current&) {
	if (!pPhysPlanner->getPlanner().stop())
		throw ExTinyArmStop("ArmI::stop(): arm stop error");
}

GenConfigspaceState ArmI::findTarget(const GenConfigspaceState &begin, const GenWorkspaceState& end, const ::Ice::Current&) {
	golem::GenConfigspaceState cend;
	if (!pPhysPlanner->getPlanner().findTarget(cend, make(begin), make(end)))
		throw ExTinyArmFindTarget("ArmI::findTarget(): find target error");

	return make(cend);
}

GenConfigspaceStateSeq ArmI::findTrajectory(const GenConfigspaceState &begin, const GenConfigspaceState &end, const ::Ice::Current&) {
	golem::Planner::Trajectory tmp;
	if (!pPhysPlanner->getPlanner().findTrajectory(tmp, tmp.begin(), make(begin), make(end)))
		throw ExTinyArmFindTrajectory("ArmI::findTrajectory(): find trajectory error");

	return make(tmp);
}

GenConfigspaceStateSeq ArmI::findConfigspaceTrajectory(const GenConfigspaceState &begin, const GenConfigspaceState &end, const ::Ice::Current&) {
	golem::Planner::Trajectory tmp;
	if (!pPhysPlanner->getPlanner().findConfigspaceTrajectory(tmp, tmp.begin(), make(begin), make(end)))
		throw ExTinyArmFindTrajectory("ArmI::findConfigspaceTrajectory(): find trajectory error");

	return make(tmp);
}

GenConfigspaceStateSeq ArmI::findWorkspaceTrajectory(const GenConfigspaceState &begin, const GenWorkspaceState &end, const ::Ice::Current&) {
	golem::Planner::Trajectory tmp;
	if (!pPhysPlanner->getPlanner().findWorkspaceTrajectory(tmp, tmp.begin(), make(begin), make(end)))
		throw ExTinyArmFindTrajectory("ArmI::findWorkspaceTrajectory(): find trajectory error");

	return make(tmp);
}

::golem::tinyice::Mat34Seq ArmI::getForwardTransform(const ::golem::tinyice::ConfigspaceCoord& cc, const ::Ice::Current&) const {
	std::vector<golem::Mat34> seq(pPhysPlanner->getArm().getJoints().size());
	pPhysPlanner->getArm().forwardTransformEx(&seq.front(), make(cc));
	return make(seq);
}

::golem::tinyice::Jacobian ArmI::getJacobian(const ::golem::tinyice::ConfigspaceCoord& cc, const ::Ice::Current&) const {
	golem::Jacobian jacobian;
	pPhysPlanner->getArm().jacobian(jacobian, make(cc));
	return make(jacobian);
}

JointSeq ArmI::getJoints(const ::Ice::Current&) const {
	JointSeq jointSeq;
	for (JointList::const_iterator i = jointList.begin(); i != jointList.end(); i++)
		jointSeq.push_back(JointPrx::uncheckedCast(i->first));
	return jointSeq;
}

::Ice::Int ArmI::getArmGroup(const ::Ice::Current&) const {
	return (::Ice::Int)pPhysPlanner->getArmBoundsGroup();
}

::Ice::Int ArmI::getCollisionGroup(const ::Ice::Current&) const {
	return (::Ice::Int)pPhysPlanner->getCollisionBoundsGroup();
}

void ArmI::setCollisionGroup(::Ice::Int group, const ::Ice::Current&) {
	pPhysPlanner->setCollisionBoundsGroup((int)group);
}

Mat34 ArmI::getGlobalPose(const ::Ice::Current&) const {
	return make(pPhysPlanner->getArm().getGlobalPose());
}

void ArmI::setGlobalPose(const Mat34& pose, const ::Ice::Current&) {
	pPhysPlanner->getArm().setGlobalPose(make(pose));
}

Mat34 ArmI::getReferencePose(const ::Ice::Current&) const {
	return make(pPhysPlanner->getArm().getReferencePose());
}

void ArmI::setReferencePose(const Mat34& pose, const ::Ice::Current&) {
	pPhysPlanner->getArm().setReferencePose(make(pose));
}

::Ice::Double ArmI::getTimeDelta(const ::Ice::Current&) const {
	return (::Ice::Double)pPhysPlanner->getArm().getTimeDelta();
}

::Ice::Double ArmI::getTimeDeltaAsync(const ::Ice::Current&) const {
	return (::Ice::Double)pPhysPlanner->getArm().getTimeDeltaAsync();
}

//------------------------------------------------------------------------------

KatanaArmI::KatanaArmI(TinyI &tiny) : ArmI(tiny) {
}

bool KatanaArmI::create(const KatanaArmDesc& desc) {
	golem::tinyice::PhysPlannerDesc physPlannerDesc;
	physPlannerDesc.pArmDesc = golem::Arm::Desc::load(context, desc.path);
	physPlannerDesc.pArmDesc->globalPose = make(desc.globalPose);

	golem::KatanaGripper::Desc* pDesc = dynamic_cast<golem::KatanaGripper::Desc*>(&*physPlannerDesc.pArmDesc);
	if (pDesc == NULL)
		throw ExTinyKatanaArm("KatanaArmI::create(): Unknown arm type");

	pDesc->bGripper = desc.bGripper;
	pDesc->sensorIndexSet = desc.sensorIndexSet;

	if (!ArmI::create(physPlannerDesc))
		return false;

	pKatanaGripper = dynamic_cast<golem::KatanaGripper*>(&pPhysPlanner->getArm());
	ASSERT(pKatanaGripper) // this should never happen

	return true;
}

KatanaSensorDataSet KatanaArmI::gripperRecvSensorData(double timeOut, const ::Ice::Current&) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent("KatanaArmI::gripperRecvSensorData(): gripper not present");

	golem::KatanaGripper::SensorDataSet data;
	if (!pKatanaGripper->gripperRecvSensorData(data, SecTmRealToMSecTmU32(timeOut)))
		throw ExTinyKatanaGripperIOError("KatanaArmI::gripperRecvSensorData(): gripper IO error");

	return make(data);
}

KatanaGripperEncoderData KatanaArmI::gripperRecvEncoderData(double timeOut, const ::Ice::Current&) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent("KatanaArmI::gripperRecvEncoderData(): gripper not present");

	golem::KatanaGripper::GripperEncoderData data;
	if (!pKatanaGripper->gripperRecvEncoderData(data, SecTmRealToMSecTmU32(timeOut)))
		throw ExTinyKatanaGripperIOError("KatanaArmI::gripperRecvEncoderData(): gripper IO error");

	return make(data);
}

void KatanaArmI::gripperOpen(double timeOut, const ::Ice::Current&) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent("KatanaArmI::gripperOpen(): gripper not present");

	if (!pKatanaGripper->gripperOpen(SecTmRealToMSecTmU32(timeOut)))
		throw ExTinyKatanaGripperIOError("KatanaArmI::gripperOpen(): gripper IO error");
}

void KatanaArmI::gripperClose(const KatanaSensorDataSet& sensorThreshold, double timeOut, const ::Ice::Current&) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent("KatanaArmI::gripperClose(): gripper not present");

	if (!pKatanaGripper->gripperClose(make(sensorThreshold), SecTmRealToMSecTmU32(timeOut)))
		throw ExTinyKatanaGripperIOError("KatanaArmI::gripperClose(): gripper IO error");
}

void KatanaArmI::gripperFreeze(double timeOut, const ::Ice::Current&) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent("KatanaArmI::gripperFreeze(): gripper not initialized");

	if (!pKatanaGripper->gripperFreeze(SecTmRealToMSecTmU32(timeOut)))
		throw ExTinyKatanaGripperIOError("KatanaArmI::gripperFreeze(): gripper IO error");
}

//------------------------------------------------------------------------------

TinyI::TinyI(int argc, char *argv[], Ice::CommunicatorPtr communicator) : communicator(communicator) {
	try {
		// Determine configuration file name
		std::string cfg;
		if (argc == 1) {
			// default configuration file name
			cfg.assign(argv[0]);
#ifdef WIN32
			size_t pos = cfg.rfind(".exe"); // Windows only
			if (pos != std::string::npos) cfg.erase(pos);
#endif
			cfg.append(".xml");
		}
		else
			cfg.assign(argv[1]);

		// Create XML parser and load configuration file
		XMLParser::Desc parserDesc;
		pParser = parserDesc.create();
		try {
			FileReadStream fs(cfg.c_str());
			pParser->load(fs); // throw
		} catch (const Message& msg) {
			std::cerr << msg.str() << std::endl;
			std::cout << "Usage: " << argv[0] << " <configuration_file>" << std::endl;
			return;
		}

		// Find program XML root context
		pXMLContext = pParser->getContextRoot()->getContextFirst("golem");
		if (pXMLContext == NULL) {
			std::string str("Unknown configuration file: ");
			str.append(cfg.c_str());
			throw ExTinyCreate(str);
		}

		// Create program context
		Context::Desc contextDesc;
		XMLData(contextDesc, pXMLContext);
		pContext = contextDesc.create(); // throw
		
		// Create Universe
		golem::Universe::Desc universeDesc;
		universeDesc.name = "Golem (Tiny)";
		XMLData(universeDesc, pXMLContext->getContextFirst("universe"));
		universeDesc.argc = argc;
		universeDesc.argv = argv;
		pUniverse = universeDesc.create(*pContext);

		// Create scene
		golem::Scene::Desc sceneDesc;
		sceneDesc.name = "Ice server";
		XMLData(sceneDesc, pXMLContext->getContextFirst("scene"));
		pScene = pUniverse->createScene(sceneDesc);

		// Launching Universe
		pUniverse->launch();

		// Activate Golem Tiny
		id.name = "GolemTiny";
		XMLData("identity", id.name, pXMLContext->getContextFirst("ice"));
		std::string adapterName = id.name;
		adapterName.append("Adapter");
		std::string adapterTransport = "default -p 8172";
		XMLData("transport", adapterTransport, pXMLContext->getContextFirst("ice"));

		adapter = communicator->createObjectAdapterWithEndpoints(adapterName, adapterTransport);
		activate();
		adapter->activate();
	}
	catch (const ExTiny& ex) {
		throw ex;
	}
	catch (const Message& ex) {
		std::string str("TinyI::createActor(): ");
		str.append(ex.str());
		throw ExTinyActorCreate(str);
	}
	catch (const std::exception &ex) {
		std::string str("TinyI::createActor(): C++ exception: ");
		str.append(ex.what());
		throw ExTinyActorCreate(str);
	}
}

TinyI::~TinyI() {
	actorList.clear();
	pUniverse.release();
}

void TinyI::activate() {
	adapter->add(this, id);
}

void TinyI::deactivate() {
	adapter->remove(id);

	// deactivate actors
	for (ActorList::iterator i = actorList.begin(); i != actorList.end(); i++)
		i->second->deactivate();
}

::Ice::Double TinyI::getTime(const ::Ice::Current&) const {
	return (::Ice::Double)pContext->getTimer().elapsed();
}

void TinyI::sleep(::Ice::Double duration, const ::Ice::Current&) const {
	pContext->getTimer().sleep(SecTmReal(duration));
}

bool TinyI::interrupted(const ::Ice::Current&) {
	return pUniverse->interrupted();
}

ActorIPtr TinyI::createActor(const ActorDesc* pDesc) {
	if (!pDesc)
		throw ExTinyActorCreate("TinyI::createActor(): NULL Actor description");
	else if (dynamic_cast<const KatanaArmDesc*>(pDesc)) {
		KatanaArmI* pKatanaArmI = new KatanaArmI(*this);
		ActorIPtr pActor(pKatanaArmI);
		if (pKatanaArmI->create(*dynamic_cast<const KatanaArmDesc*>(pDesc)))
			return pActor;
	}
	else if (dynamic_cast<const ArmDesc*>(pDesc)) {
		ArmI* pArmI = new ArmI(*this);
		ActorIPtr pActor(pArmI);
		if (pArmI->create(*dynamic_cast<const ArmDesc*>(pDesc)))
			return pActor;
	}
	else if (dynamic_cast<const RigidBodyDesc*>(pDesc)) {
		RigidBodyI* pRigidBodyI = new RigidBodyI(*this);
		ActorIPtr pActor(pRigidBodyI);
		if (pRigidBodyI->create(*dynamic_cast<const RigidBodyDesc*>(pDesc)))
			return pActor;
	}
	else
		throw ExTinyActorCreate("TinyI::createActor(): Unknown Actor description");

	throw ExTinyActorCreate("TinyI::createActor(): Unable to create Actor");
}

ActorPrx TinyI::createActor(const ActorDescPtr& pActorDesc, const ::Ice::Current&) {
	try {
		if (pActorDesc == NULL)
			throw ExTinyActorCreate("TinyI::createActor(): empty description");

		ActorIPtr pActor = createActor(pActorDesc.get()); // throws
		if (pActor == NULL)
			return NULL;
		pActor->owner = true;
		
		ActorPrx actorPrx = pActor->activate();
		actorList.push_back(ActorList::Pair(actorPrx, pActor));
		return actorPrx;
	}
	catch (const Message& ex) {
		std::string str("TinyI::createActor(): ");
		str.append(ex.str());
		throw ExTinyActorCreate(str);
	}
	catch (const std::exception &ex) {
		std::string str("TinyI::createActor(): C++ exception: ");
		str.append(ex.what());
		throw ExTinyActorCreate(str);
	}

	return ActorPrx();
}

void TinyI::releaseActor(const ActorPrx& actor, const ::Ice::Current&) {
	ActorList::iterator pos = actorList.find(actor);
	if (pos == actorList.end())
		throw ExTinyActorNotFound("TinyI::releaseActor(): Unable to find specified Actor");
	if (!pos->second->owner)
		throw ExTinyActorNotRemovable("TinyI::releaseActor(): The specified Actor cannot be removed");

	pos->second->deactivate();
	actorList.erase(pos);
}

ActorSeq TinyI::getActors(const ::Ice::Current&) const {
	ActorSeq actorSeq;
	for (ActorList::const_iterator i = actorList.begin(); i != actorList.end(); i++)
		actorSeq.push_back(i->first);
	return actorSeq;
}

void TinyI::bang(const ::Ice::Current&) {
	pContext->getMessageStream()->write("Bang!");

	Rand rand(pContext->getRandSeed());
	Creator creator(*pScene);
	golem::Actor::Desc *pActorDesc = creator.createTreeDesc(rand.nextUniform(Real(0.07), Real(0.10)));
	pActorDesc->nxActorDesc.globalPose.t.set(
		(NxReal)rand.nextUniform(Real(-0.3), Real(0.3)),
		(NxReal)rand.nextUniform(Real(-0.3), Real(0.3)),
		(NxReal)rand.nextUniform(Real(+0.3), Real(0.9))
	);
	pScene->createObject(*pActorDesc);
}

//------------------------------------------------------------------------------

class TinyIceApp : public Ice::Application {
private:
	// Golem Tiny must be destroyed outside run() and ~Application() because
	// Ice keeps references to all created objects and destroys them after Golem Tiny
	IceInternal::Handle<TinyI> pTiny;

public:
	virtual int run(int argc, char *argv[]) {
		shutdownOnInterrupt();
		pTiny = new TinyI(argc, argv, communicator());
		communicator()->waitForShutdown();
		return 0;
	}
};

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	try {
		return TinyIceApp().main(argc, argv);
	}
	catch (const ExTiny& ex) {
		std::cerr << ex << std::endl;
	}
	return 1;
}
