/** @file Tiny.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/Data.h>
#include <Golem/Tools/Context.h>
#include <Golem/Tools/XMLParser.h>
#include <Golem/PhysCtrl/PhysPlanner.h>
#include <Golem/PhysCtrl/Msg.h>
#include <Golem/PhysCtrl/Creator.h>
#include <Golem/Tiny/Tiny.h>
#include <Golem/Tiny/Types.h>
#include <Golem/Phys/Data.h>
#include <Golem/Device/Katana/Katana.h>

//------------------------------------------------------------------------------

using namespace golem::tiny;

//------------------------------------------------------------------------------

class golem::tiny::BoundsDesc {
public:
	golem::Bounds::Desc::Ptr pBoundsDesc;
	BoundsDesc(const golem::Bounds::Desc::Ptr pBoundsDesc) : pBoundsDesc(pBoundsDesc) {}
};

class golem::tiny::PhysPlannerDesc : public golem::PhysPlanner::Desc {
public:
};

//------------------------------------------------------------------------------

Shape::Shape(RigidBody& rigidBody) : rigidBody(rigidBody), pNxShapeDesc(0), pBounds(0), owner(false) {
}

Shape::~Shape() {
	if (pBounds == 0 || !owner)
		return;
	golem::Mat34 m = pBounds->getPose();
	rigidBody.pActor->releaseBounds(*pBounds);
}

bool Shape::create(const ShapeDesc& desc, ShapeType type, const golem::shared_ptr<BoundsDesc>& pBoundsDesc) {
	this->type = type;
	this->density = desc.density;
	this->color = desc.color;

	// pBoundsDesc must be initialized before
	if (pBoundsDesc == 0)
		throw ExTinyShape(Message::LEVEL_CRIT, "Shape::create(): Unable to create bounds description");
	
	this->pBoundsDesc = pBoundsDesc;

	pNxShapeDesc = rigidBody.scene.createNxShapeDesc(this->pBoundsDesc->pBoundsDesc); // throws
	if (pNxShapeDesc == 0)
		return false;
	pNxShapeDesc->density = NxReal(this->density);
	// pBounds created in container

	return true;
}

ShapeType Shape::getType() const {
	return type;
}

RGBA Shape::getColor() const {
	return color;
}

golem::Mat34 Shape::getLocalPose() const {
	return pBounds->getPose();
}

int Shape::getGroup() const {
	return (int)pBounds->getGroup();
}

void Shape::setGroup(int group) {
	rigidBody.pActor->setBoundsGroup(*pBounds, U32(group));
}

ShapeDescPtr Shape::getDesc() const {
	return pShapeDesc;
}

//------------------------------------------------------------------------------

PlaneShape::PlaneShape(RigidBody& rigidBody) : Shape(rigidBody) {
}
bool PlaneShape::create(const PlaneShapeDesc& desc) {
	BoundingPlane::Desc* pDesc = new BoundingPlane::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = desc.localPose;
	pDesc->normal = desc.normal;
	pDesc->distance = (Real)desc.distance;
	return Shape::create(desc, ShapeTypePlane, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

SphereShape::SphereShape(RigidBody& rigidBody) : Shape(rigidBody) {
}
bool SphereShape::create(const SphereShapeDesc& desc) {
	BoundingSphere::Desc* pDesc = new BoundingSphere::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = desc.localPose;
	pDesc->radius = (Real)desc.radius;
	return Shape::create(desc, ShapeTypeSphere, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

CylinderShape::CylinderShape(RigidBody& rigidBody) : Shape(rigidBody) {
}
bool CylinderShape::create(const CylinderShapeDesc& desc) {
	BoundingCylinder::Desc* pDesc = new BoundingCylinder::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = desc.localPose;
	pDesc->radius = (Real)desc.radius;
	pDesc->length = (Real)desc.length;
	return Shape::create(desc, ShapeTypeCylinder, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

BoxShape::BoxShape(RigidBody& rigidBody) : Shape(rigidBody) {
}
bool BoxShape::create(const BoxShapeDesc& desc) {
	BoundingBox::Desc* pDesc = new BoundingBox::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = desc.localPose;
	pDesc->dimensions = desc.dimensions;
	return Shape::create(desc, ShapeTypeBox, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

ConvexMeshShape::ConvexMeshShape(RigidBody& rigidBody) : Shape(rigidBody) {
}
bool ConvexMeshShape::create(const ConvexMeshShapeDesc& desc) {
	BoundingConvexMesh::Desc* pDesc = new BoundingConvexMesh::Desc();
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	pDesc->pose = desc.localPose;
	pDesc->vertices = desc.vertices;
	pDesc->bCook = true;
	return Shape::create(desc, ShapeTypeConvexMesh, shared_ptr<BoundsDesc>(new BoundsDesc(pBoundsDesc)));
}

//------------------------------------------------------------------------------

Actor::Actor(Tiny& tiny) : tiny(tiny), context(*tiny.pContext), scene(*tiny.pScene), pXMLContext(tiny.pXMLContext), owner(false) {
}

Actor::~Actor() {
}

RigidBody::RigidBody(Tiny& tiny) : Actor(tiny), pActor(0) {
}

RigidBody::~RigidBody() {
	shapeList.clear(); // release shapes before releasing the Actor
	if (pActor == 0 || !owner)
		return;
	scene.releaseObject(*pActor);
}

ShapePtr RigidBody::createShape(const golem::Bounds* pBounds) {
	ShapeDescPtr pShapeDesc;
	ShapePtr pShape;

	switch (pBounds->getType()) {
	case Bounds::TYPE_PLANE:
		{
			PlaneShapeDesc* pDesc = new PlaneShapeDesc;
			pShapeDesc.reset(pDesc);
			pDesc->normal = dynamic_cast<const BoundingPlane*>(pBounds)->getNormal();
			pDesc->distance = (double)dynamic_cast<const BoundingPlane*>(pBounds)->getDistance();
		}
		break;
	case Bounds::TYPE_SPHERE:
		{
			SphereShapeDesc* pDesc = new SphereShapeDesc;
			pShapeDesc.reset(pDesc);
			pDesc->radius = (double)dynamic_cast<const BoundingSphere*>(pBounds)->getRadius();
		}
		break;
	case Bounds::TYPE_CYLINDER:
		{
			CylinderShapeDesc* pDesc = new CylinderShapeDesc;
			pShapeDesc.reset(pDesc);
			pDesc->radius = (double)dynamic_cast<const BoundingCylinder*>(pBounds)->getRadius();
			pDesc->length = (double)dynamic_cast<const BoundingCylinder*>(pBounds)->getLength();
		}
		break;
	case Bounds::TYPE_BOX:
		{
			BoxShapeDesc* pDesc = new BoxShapeDesc;
			pShapeDesc.reset(pDesc);
			pDesc->dimensions = dynamic_cast<const BoundingBox*>(pBounds)->getDimensions();
		}
		break;
	case Bounds::TYPE_CONVEX_MESH:
		{
			ConvexMeshShapeDesc* pDesc = new ConvexMeshShapeDesc;
			pShapeDesc.reset(pDesc);
			const BoundingConvexMesh* pBoundingConvexMesh = dynamic_cast<const BoundingConvexMesh*>(pBounds);
			pDesc->vertices = pBoundingConvexMesh->getVertices();
		}
		break;
	}
	if (pShapeDesc == 0)
		return pShape;

	pShapeDesc->localPose = pBounds->getPose();
	pShapeDesc->group = (int)pBounds->getGroup();

	pShape = pShapeDesc->create(*this);// throws
	if (pShape == 0)
		return pShape;
	pShape->pShapeDesc = pShapeDesc;
	pShape->pBounds = pBounds;
	
	return pShape;
}

bool RigidBody::create(const RigidBodyDesc& desc) {
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
		ShapePtr pShape = (**i).create(*this); // throws
		if (pShape == 0 || pShape->pNxShapeDesc == 0)
			return false;
		pShape->pShapeDesc = *i;
		
		if (pShape->getType() == ShapeTypePlane)
			actorDesc.nxActorDesc.body = 0; // Actors with Plane Shapes cannot have a body
		actorDesc.nxActorDesc.shapes.push_back(pShape->pNxShapeDesc);
		
		shapeList.push_back(ShapeList::Pair(pShape.get(), pShape));
	}

	pActor = dynamic_cast<golem::Actor*>(scene.createObject(actorDesc)); // throws
	if (pActor == 0)
		return false;
	
	const golem::Actor::BoundsConstSeq& boundsSeq = pActor->getBoundsSeq();
	golem::Actor::BoundsConstSeq::const_iterator i = boundsSeq.begin();
	ShapeList::iterator j = shapeList.begin();
	while (i != boundsSeq.end() && j != shapeList.end())
		(**j++).pBounds = *i++; // order is preserved
	
	return true;
}

Shape* RigidBody::createShape(const ShapeDescPtr& pShapeDesc) {
	try {
		if (pShapeDesc == 0)
			throw ExTinyShapeCreate(Message::LEVEL_CRIT, "RigidBody::createShape(): empty description");

		ShapePtr pShape = pShapeDesc->create(*this); // throws
		if (pShape == 0)
			return 0;
		pShape->pShapeDesc = pShapeDesc;

		pShape->pBounds = pActor->createBounds(pShape->pBoundsDesc->pBoundsDesc); // throws
		if (pShape->pBounds == 0)
			return 0;
		pShape->owner = true;

		shapeList.push_back(ShapeList::Pair(pShape.get(), pShape));
		return pShape.get();
	}
	catch (const Message& ex) {
		throw ExTinyShapeCreate(Message::LEVEL_CRIT, "RigidBody::createShape(): %s", ex.str().c_str()); // translate
	}
	catch (const std::exception &ex) {
		throw ExTinyShapeCreate( Message::LEVEL_CRIT, "RigidBody::createShape(): C++ exception: %s", ex.what()); // translate
	}

	return 0;
}

void RigidBody::releaseShape(Shape* shape) {
	if (!shapeList.contains(shape))
		throw ExTinyShapeNotFound(Message::LEVEL_CRIT, "RigidBody::releaseShape(): Unable to find specified Shape");
	if (!shape->owner)
		throw ExTinyShapeNotRemovable(Message::LEVEL_CRIT, "RigidBody::releaseShape(): The specified Shape cannot be removed");
	shapeList.erase(shape);
}

ShapeSeq RigidBody::getShapes() const {
	return ShapeSeq(shapeList.begin(), shapeList.end());
}

golem::Mat34 RigidBody::getGlobalPose() const {
	return pActor->getPose();
}

void RigidBody::setGlobalPose(const golem::Mat34& pose) {
	pActor->setPose(pose);
}

void RigidBody::setGroup(int group) {
	pActor->setBoundsGroup(U32(group));
}

//------------------------------------------------------------------------------

Joint::Joint(Arm& arm) : RigidBody(arm.tiny), arm(arm) {
}

bool Joint::create(const JointDesc& desc) {
	ASSERT(desc.pJointActor)
	pActor = desc.pJointActor;
	kinematic = true;
	
	shapeList.clear();
	const golem::Actor::BoundsConstSeq& boundsSeq = pActor->getBoundsSeq();
	for (golem::Actor::BoundsConstSeq::const_iterator i = boundsSeq.begin(); i != boundsSeq.end(); i++) {
		ShapePtr pShape = RigidBody::createShape(*i); // throws
		if (pShape == 0)
			return false;
		shapeList.push_back(ShapeList::Pair(pShape.get(), pShape));
	}

	return true;
}

ActorPtr JointDesc::create(Tiny& tiny) const {
	// Joint cannot be created as a normal Actor, return 0
	return ActorPtr();
}

Arm::Arm(Tiny& tiny) : Actor(tiny), pPhysPlanner(0) {
}

Arm::~Arm() {
	jointList.clear(); // joints wrappers must be released before PhysPlanner!
	if (pPhysPlanner == 0 || !owner)
		return;
	scene.releaseObject(*pPhysPlanner);
}

bool Arm::create(const ArmDesc& desc) {
	golem::tiny::PhysPlannerDesc physPlannerDesc;
	physPlannerDesc.pArmDesc = golem::Arm::Desc::load(context, desc.path);
	physPlannerDesc.pArmDesc->globalPose = desc.globalPose;
	
	if (!Arm::create(physPlannerDesc))
		return false;

	return true;
}

bool Arm::create(const PhysPlannerDesc& desc) {
	// create controller
	pPhysPlanner = dynamic_cast<golem::PhysPlanner*>(scene.createObject(desc));// throws
	if (pPhysPlanner == 0)
		return false;

	// create joints
	jointList.clear();
	const golem::JointActor::Seq& jointActorSeq = pPhysPlanner->getJointActors();
	for (golem::JointActor::Seq::const_iterator i = jointActorSeq.begin(); i != jointActorSeq.end(); i++) {
		JointPtr pJoint;
		if (*i != 0) { // Joint may have no body
			JointDesc jointDesc;
			jointDesc.pJointActor = *i;
			pJoint = jointDesc.create(*this); // throws
			if (pJoint == 0)
				return false;
			jointList.push_back(JointList::Pair(pJoint.get(), pJoint));
		}
	}

	return true;
}

GenConfigspaceState Arm::recvGenConfigspaceState(double t) const {
	golem::GenConfigspaceState gcs;
	pPhysPlanner->getArm().lookupCommand(gcs, t);
	return make(gcs);
}

GenWorkspaceState Arm::recvGenWorkspaceState(double t) const {
	golem::GenConfigspaceState gcs;
	pPhysPlanner->getArm().lookupCommand(gcs, t);
	golem::GenWorkspaceState gws;
	pPhysPlanner->getArm().forwardTransform(gws.pos, gcs.pos);
	gws.pos.multiply(gws.pos, pPhysPlanner->getArm().getReferencePose());
	gws.vel.set(Vec3(REAL_ZERO), Vec3(REAL_ZERO)); // TODO
	gws.t = gcs.t;
	return make(gws);
}

void Arm::send(const GenConfigspaceStateSeq& trajectory, double timeWait) {
	const golem::Planner::Trajectory tmp(make(trajectory));
	if (!pPhysPlanner->getPlanner().send<golem::GenConfigspaceState>(tmp.begin(), tmp.end()))
		throw ExTinyArmSend(Message::LEVEL_CRIT, "Arm::sendTrajectory(): send trajectory error");

	golem::GenConfigspaceState gcs;
	pPhysPlanner->getArm().lookupCommand(gcs, golem::SEC_TM_REAL_INF);

	context.getTimer().sleep(Math::clamp(
		gcs.t - context.getTimer().elapsed()/* - pPhysPlanner->getArm().getTimeDeltaAsync()*/, pPhysPlanner->getArm().getTimeDeltaAsync()/*golem::SEC_TM_REAL_ZERO*/, golem::SecTmReal(timeWait)
	));
}

void Arm::stop() {
	if (!pPhysPlanner->getPlanner().stop())
		throw ExTinyArmStop(Message::LEVEL_CRIT, "Arm::stop(): arm stop error");
}

GenConfigspaceState Arm::findTarget(const GenConfigspaceState &begin, const GenWorkspaceState& end) {
	golem::GenConfigspaceState cend;
	if (!pPhysPlanner->getPlanner().findTarget(cend, make(begin), make(end)))
		throw ExTinyArmFindTarget(Message::LEVEL_CRIT, "Arm::findTarget(): find target error");

	return make(cend);
}

GenConfigspaceStateSeq Arm::findTrajectory(const GenConfigspaceState &begin, const GenConfigspaceState &end) {
	golem::Planner::Trajectory tmp;
	if (!pPhysPlanner->getPlanner().findTrajectory(tmp, tmp.begin(), make(begin), make(end)))
		throw ExTinyArmFindTrajectory(Message::LEVEL_CRIT, "Arm::findTrajectory(): find trajectory error");

	return make(tmp);
}

GenConfigspaceStateSeq Arm::findConfigspaceTrajectory(const GenConfigspaceState &begin, const GenConfigspaceState &end) {
	golem::Planner::Trajectory tmp;
	if (!pPhysPlanner->getPlanner().findConfigspaceTrajectory(tmp, tmp.begin(), make(begin), make(end)))
		throw ExTinyArmFindTrajectory(Message::LEVEL_CRIT, "Arm::findConfigspaceTrajectory(): find trajectory error");

	return make(tmp);
}

GenConfigspaceStateSeq Arm::findWorkspaceTrajectory(const GenConfigspaceState &begin, const GenWorkspaceState &end) {
	golem::Planner::Trajectory tmp;
	if (!pPhysPlanner->getPlanner().findWorkspaceTrajectory(tmp, tmp.begin(), make(begin), make(end)))
		throw ExTinyArmFindTrajectory(Message::LEVEL_CRIT, "Arm::findWorkspaceTrajectory(): find trajectory error");

	return make(tmp);
}

Mat34Seq Arm::getForwardTransform(const ConfigspaceCoord& cc) const {
	std::vector<golem::Mat34> seq(pPhysPlanner->getArm().getJoints().size());
	pPhysPlanner->getArm().forwardTransformEx(&seq.front(), make(cc));
	return seq;
}

Jacobian Arm::getJacobian(const ConfigspaceCoord& cc) const {
	golem::Jacobian jacobian;
	pPhysPlanner->getArm().jacobian(jacobian, make(cc));
	return make(jacobian);
}

JointSeq Arm::getJoints() const {
	return JointSeq(jointList.begin(), jointList.end());
}

int Arm::getArmGroup() const {
	return (int)pPhysPlanner->getArmBoundsGroup();
}

int Arm::getCollisionGroup() const {
	return (int)pPhysPlanner->getCollisionBoundsGroup();
}

void Arm::setCollisionGroup(int group) {
	pPhysPlanner->setCollisionBoundsGroup((U32)group);
}

golem::Mat34 Arm::getGlobalPose() const {
	return pPhysPlanner->getArm().getGlobalPose();
}

void Arm::setGlobalPose(const golem::Mat34& pose) {
	pPhysPlanner->getArm().setGlobalPose(pose);
}

golem::Mat34 Arm::getReferencePose() const {
	return pPhysPlanner->getArm().getReferencePose();
}

void Arm::setReferencePose(const golem::Mat34& pose) {
	pPhysPlanner->getArm().setReferencePose(pose);
}

double Arm::getTimeDelta() const {
	return (double)pPhysPlanner->getPlanner().getArm().getTimeDelta();
}

double Arm::getTimeDeltaAsync() const {
	return (double)pPhysPlanner->getPlanner().getArm().getTimeDeltaAsync();
}

//------------------------------------------------------------------------------

KatanaArm::KatanaArm(Tiny& tiny) : Arm(tiny) {
}

bool KatanaArm::create(const KatanaArmDesc& desc) {
	golem::tiny::PhysPlannerDesc physPlannerDesc;
	physPlannerDesc.pArmDesc = golem::Arm::Desc::load(context, desc.path);
	physPlannerDesc.pArmDesc->globalPose = desc.globalPose;

	golem::KatanaGripper::Desc* pDesc = dynamic_cast<golem::KatanaGripper::Desc*>(&*physPlannerDesc.pArmDesc);
	if (pDesc == NULL)
		throw ExTinyKatanaArm(Message::LEVEL_CRIT, "KatanaArm::create(): Unknown arm type");
	pDesc->bGripper = desc.bGripper;
	pDesc->sensorIndexSet = desc.sensorIndexSet;

	if (!Arm::create(physPlannerDesc))
		return false;

	pKatanaGripper = dynamic_cast<golem::KatanaGripper*>(&pPhysPlanner->getArm());
	ASSERT(pKatanaGripper) // pKatanaGripper==NULL should never happen
	return true;
}

KatanaSensorDataSet KatanaArm::gripperRecvSensorData(double timeOut) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent(Message::LEVEL_CRIT, "KatanaArm::gripperRecvSensorData(): gripper not present");

	golem::KatanaGripper::SensorDataSet data;
	if (!pKatanaGripper->gripperRecvSensorData(data, SecTmRealToMSecTmU32(timeOut)))
		throw ExTinyKatanaGripperIOError(Message::LEVEL_CRIT, "KatanaArm::gripperRecvSensorData(): gripper IO error");

	return make(data);
}

KatanaGripperEncoderData KatanaArm::gripperRecvEncoderData(double timeOut) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent(Message::LEVEL_CRIT, "KatanaArm::gripperRecvEncoderData(): gripper not present");

	golem::KatanaGripper::GripperEncoderData data;
	if (!pKatanaGripper->gripperRecvEncoderData(data, SecTmRealToMSecTmU32(timeOut)))
		throw ExTinyKatanaGripperIOError(Message::LEVEL_CRIT, "KatanaArm::gripperRecvEncoderData(): gripper IO error");

	return make(data);
}

void KatanaArm::gripperOpen(double timeOut) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent(Message::LEVEL_CRIT, "KatanaArm::gripperOpen(): gripper not present");

	if (!pKatanaGripper->gripperOpen(SecTmRealToMSecTmU32(timeOut)))
		throw ExTinyKatanaGripperIOError(Message::LEVEL_CRIT, "KatanaArm::gripperOpen(): gripper IO error");
}

void KatanaArm::gripperClose(const KatanaSensorDataSet& sensorThreshold, double timeOut) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent(Message::LEVEL_CRIT, "KatanaArm::gripperClose(): gripper not present");

	if (!pKatanaGripper->gripperClose(make(sensorThreshold), SecTmRealToMSecTmU32(timeOut)))
		throw ExTinyKatanaGripperIOError(Message::LEVEL_CRIT, "KatanaArm::gripperClose(): gripper IO error");
}

void KatanaArm::gripperFreeze(double timeOut) {
	if (!pKatanaGripper->hasGripper())
		throw ExTinyKatanaGripperNotPresent(Message::LEVEL_CRIT, "KatanaArm::gripperFreeze(): gripper not initialized");

	if (!pKatanaGripper->gripperFreeze(SecTmRealToMSecTmU32(timeOut)))
		throw ExTinyKatanaGripperIOError(Message::LEVEL_CRIT, "KatanaArm::gripperFreeze(): gripper IO error");
}

//------------------------------------------------------------------------------

Tiny::Tiny(int argc, char *argv[]) {
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
		if (pXMLContext == 0)
			throw ExTinyCreate(Message::LEVEL_CRIT, "Unknown configuration file: %s", cfg.c_str());

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
		sceneDesc.name = "Tiny interface";
		XMLData(sceneDesc, pXMLContext->getContextFirst("scene"));
		pScene = pUniverse->createScene(sceneDesc);

		// Launching Universe
		pUniverse->launch();
	}
	catch (const ExTiny& ex) {
		throw ex;
	}
	catch (const golem::Message& ex) {
		throw ExTinyCreate(Message::LEVEL_CRIT, "Tiny::Tiny(): %s", ex.str().c_str()); // translate
	}
	catch (const std::exception &ex) {
		throw ExTinyCreate(Message::LEVEL_CRIT, "Tiny::Tiny(): C++ exception: %s", ex.what()); // translate
	}
}

Tiny::~Tiny() {
	actorList.clear();
	pUniverse.release();
}

//------------------------------------------------------------------------------

double Tiny::getTime() const {
	return (double)pContext->getTimer().elapsed();
}

void Tiny::sleep(double duration) const {
	pContext->getTimer().sleep(SecTmReal(duration));
}

void Tiny::print(const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	pContext->getMessageStream()->write(Message::TIME_UNDEF, Message::THREAD_UNDEF, Message::LEVEL_UNDEF, Message::CODE_UNDEF, format, argptr);
	va_end(argptr);
}

bool Tiny::interrupted() const {
	return pUniverse->interrupted();
}

int Tiny::waitKey(double timeOut) {
	return pUniverse->waitKey(SecTmRealToMSecTmU32(timeOut));
}

const golem::XMLContext* Tiny::getXMLContext() const {
	return pXMLContext;
}
golem::XMLContext* Tiny::getXMLContext() {
	return pXMLContext;
}

const golem::Context* Tiny::getContext() const {
	return pContext.get();
}
golem::Context* Tiny::getContext() {
	return pContext.get();
}

Actor* Tiny::createActor(const ActorDescPtr& pActorDesc) {
	try {
		if (pActorDesc == 0)
			throw ExTinyActorCreate(Message::LEVEL_CRIT, "Tiny::createActor(): empty description");

		ActorPtr pActor = pActorDesc->create(*this);
		if (pActor == 0)
			return 0;
		
		pActor->owner = true;
		actorList.push_back(ActorList::Pair(pActor.get(), pActor));
		return pActor.get();
	}
	catch (const Message& ex) {
		throw ExTinyActorCreate(Message::LEVEL_CRIT, "Tiny::createActor(): %s", ex.str().c_str()); // translate
	}
	catch (const std::exception &ex) {
		throw ExTinyActorCreate(Message::LEVEL_CRIT, "Tiny::createActor(): C++ exception: %s", ex.what()); // translate
	}

	return 0;
}

void Tiny::releaseActor(Actor* actor) {
	if (!actorList.contains(actor))
		throw ExTinyActorNotFound(Message::LEVEL_CRIT, "Tiny::releaseActor(): Unable to find specified Actor");
	if (!actor->owner)
		throw ExTinyActorNotRemovable(Message::LEVEL_CRIT, "Tiny::releaseActor(): The specified Actor cannot be removed");

	actorList.erase(actor);
}

ActorSeq Tiny::getActors() const {
	return ActorSeq(actorList.begin(), actorList.end());
}

//------------------------------------------------------------------------------

void Tiny::bang() {
	pContext->getMessageStream()->write("Bang!");

	Rand rand(pContext->getRandSeed());
	golem::Creator creator(*pScene);
	golem::Actor::Desc *pActorDesc = creator.createTreeDesc(rand.nextUniform(Real(0.07), Real(0.10)));
	pActorDesc->nxActorDesc.globalPose.t.set(
		rand.nextUniform(NxReal(-0.3), NxReal(0.3)),
		rand.nextUniform(NxReal(-0.3), NxReal(0.3)),
		rand.nextUniform(NxReal(+0.3), NxReal(0.9))
	);
	pScene->createObject(*pActorDesc);
}

//------------------------------------------------------------------------------
