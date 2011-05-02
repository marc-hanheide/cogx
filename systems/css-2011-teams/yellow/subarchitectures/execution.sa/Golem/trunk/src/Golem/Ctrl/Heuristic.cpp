/** @file Heuristic.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Ctrl/Heuristic.h>
#include <Golem/Ctrl/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

#ifdef _HEURISTIC_PERFMON
U32 Heuristic::waypointCollisionCounter;
U32 Heuristic::pathCollisionCounter;

void Heuristic::resetLog() {
	waypointCollisionCounter = 0;
	pathCollisionCounter = 0;
}

void Heuristic::writeLog(Context &context, const char *str) {
	context.getMessageStream()->write(Message::LEVEL_DEBUG,
		"%s: WCollision %d, PCollision %d",
		str, waypointCollisionCounter, pathCollisionCounter
	);
}
#endif

//------------------------------------------------------------------------------

Heuristic::Heuristic(golem::Arm &arm) :
	arm(arm), context(arm.getContext()), numOfJoints(U32(arm.getJoints().size()))
{}

Heuristic::~Heuristic() {
}

bool Heuristic::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgHeuristicInvalidDesc(Message::LEVEL_CRIT, "Heuristic::create(): Invalid description");
	
	distAngularQuatDot = desc.distAngularQuatDot;

	distLinearFac = desc.distLinearFac;
	distWorkspaceFac = desc.distWorkspaceFac;
	distJointspaceFac = desc.distJointspaceFac;
	
	distLinearMax = desc.distLinearMax;
	distAngularMax = desc.distAngularMax;
	distJointcoordMax = desc.distJointcoordMax;
	distRestJointcoordFac = desc.distRestJointcoordFac;

	collisionDetection = desc.collisionDetection;
	colissionDistDelta = desc.colissionDistDelta;
	skinThickness = desc.skinThickness;

	armBounds.resize(arm.getJoints().size());
	armBoundsPoses.resize(arm.getJoints().size());

	for (U32 j = 0; j < numOfJoints; j++) {
		const Joint &joint = *arm.getJoints()[j];

		min[j] = joint.getMin().pos;
		max[j] = joint.getMax().pos;
		delta[j] = max[j] - min[j];
	}
	
	collisionBounds.clear();
	syncArmBoundsDesc();
	
	return true;
}

//------------------------------------------------------------------------------

void Heuristic::setPose(golem::Bounds::Seq& boundsSeq, const Mat34Seq& boundsPoses, const Mat34& pose) const {
	const U32 numOfBounds = (U32)std::min(boundsSeq.size(), boundsPoses.size());
	for (U32 i = 0; i < numOfBounds; i++)
		boundsSeq[i]->multiplyPose(pose, boundsPoses[i]);
}

bool Heuristic::intersect(const golem::Bounds::Seq& boundsSeq0, const golem::Bounds::Seq& boundsSeq1) const {
	for (golem::Bounds::Seq::const_iterator i = boundsSeq0.begin(); i != boundsSeq0.end(); i++)
		for (golem::Bounds::Seq::const_iterator j = boundsSeq1.begin(); j != boundsSeq1.end(); j++)
			if ((**i).intersect(**j))
				return true;

	return false;
}

bool Heuristic::collides(const Waypoint &w) const {
#ifdef _HEURISTIC_PERFMON
	waypointCollisionCounter++;
#endif
	
	if (!collisionDetection)
		return false;

	CriticalSectionWrapper csw(cs);

	// iterate through all joints starting with the tool
	for (I32 i = numOfJoints - 1; i >= 0; i--) {
		const golem::Joint &joint = *arm.getJoints()[i];
		golem::Bounds::Seq& jointBounds = armBounds[i];

		if (jointBounds.empty() || !joint.hasCollision())
			continue;

		// reset to the current joint pose
		setPose(jointBounds, armBoundsPoses[i], w.trn[i]);

		// check for collisions between the current joint and the environment (bounds)
		if (!collisionBounds.empty() && intersect(jointBounds, collisionBounds))
			return true;
		
		const I32 collisionRange = i - joint.getCollisionOffset();

		// check for collisions between the current joint and the joints within the collision range
		for (I32 j = 0; j < collisionRange; j++) {
			golem::Bounds::Seq &jointBoundsTmp = armBounds[j];
			if (jointBoundsTmp.empty())
				continue;
			
			setPose(jointBoundsTmp, armBoundsPoses[j],w.trn[j]);
			if (intersect(jointBounds, jointBoundsTmp))
				return true;
		}
	}
	
	return false;
}

bool Heuristic::collides(const Waypoint &w0, const Waypoint &w1) const {
#ifdef _HEURISTIC_PERFMON
	pathCollisionCounter++;
#endif

	if (!collisionDetection)
		return false;

	// check the end waypoint first
	if (collides(w1))
		return true;
	
	Waypoint w;
	w.trn.resize(numOfJoints);
	
	const Real dist = getDist(w0, w1);
	const U32 steps = (U32)Math::round(dist/colissionDistDelta);

	// the first waypoint does not collide by assumption
	// TODO binary collision detection
	for (U32 i = 1; i < steps; i++) {
		// lineary interpolate arm pose
		for (U32 j = 0; j < numOfJoints; j++)
			w.cpos[j] = w1.cpos[j] - (w1.cpos[j] - w0.cpos[j])*Real(i)/Real(steps);
		
		arm.forwardTransformEx(&w.trn.front(), w.cpos);

		if (collides(w))
			return true;
	}

	return false;
}

//------------------------------------------------------------------------------

void Heuristic::syncArmBoundsDesc() {
	CriticalSectionWrapper csw(cs);
	
	for (U32 j = 0; j < numOfJoints; j++) {
		const Joint &joint = *arm.getJoints()[j];
		Bounds::Desc::SeqPtr pBoundsDescSeq = joint.getBoundsDescSeq();
		if (pBoundsDescSeq->empty())
			continue;
	
		armBounds[j].clear();
		armBoundsPoses[j].clear();

		for (Bounds::Desc::Seq::const_iterator i = pBoundsDescSeq->begin(); i != pBoundsDescSeq->end(); i++) {
			Bounds::Ptr pBounds = (*i)->create();
			if (pBounds == NULL) {
				context.getMessageStream()->write(Message::LEVEL_ERROR, "Heuristic::syncArmBounds(): unable to create bounds");
				continue;
			}

			//context.getMessageStream()->write(Message::LEVEL_DEBUG, "Heuristic::syncArmBoundsDesc(): joint #%d, group #%08x, name: %s", j, pBounds->getGroup(), pBounds->getName());
			armBounds[j].push_back(pBounds);
			armBoundsPoses[j].push_back(pBounds->getPose());
		}
	}
}

void Heuristic::setCollisionBounds(const Bounds::SeqPtr &pCollisionBounds) {
	CriticalSectionWrapper csw(cs);

	class Expander {
	public:
		const Real skinThickness;

		Expander(Real skinThickness) : skinThickness(skinThickness) {
		}

		Bounds::Ptr expand(const Bounds &bounds) const {
			Bounds::Ptr pBounds;

			switch (bounds.getType()) {
			case Bounds::TYPE_PLANE:
				pBounds = expand(dynamic_cast<const BoundingPlane&>(bounds));
				break;
			case Bounds::TYPE_SPHERE:
				pBounds = expand(dynamic_cast<const BoundingSphere&>(bounds));
				break;
			case Bounds::TYPE_CYLINDER:
				pBounds = expand(dynamic_cast<const BoundingCylinder&>(bounds));
				break;
			case Bounds::TYPE_BOX:
				pBounds = expand(dynamic_cast<const BoundingBox&>(bounds));
				break;
			case Bounds::TYPE_CONVEX_MESH:
				pBounds = expand(dynamic_cast<const BoundingConvexMesh&>(bounds));
				break;
			default:
				break;
			}

			return pBounds;
		}

		Bounds::Ptr expand(const BoundingPlane &bounds) const {
			BoundingPlane::Desc desc;
			desc.normal = bounds.getNormal();
			desc.distance = bounds.getDistance();// + skinThickness;
			return desc.create();
		}

		Bounds::Ptr expand(const BoundingSphere &bounds) const {
			BoundingSphere::Desc desc;
			desc.pose = bounds.getPose();
			desc.radius = bounds.getRadius() + skinThickness;
			return desc.create();
		}

		Bounds::Ptr expand(const BoundingCylinder &bounds) const {
			BoundingCylinder::Desc desc;
			desc.pose = bounds.getPose();
			desc.length = bounds.getLength() + REAL_TWO*skinThickness;
			desc.radius = bounds.getRadius() + skinThickness;
			return desc.create();
		}

		Bounds::Ptr expand(const BoundingBox &bounds) const {
			BoundingBox::Desc desc;
			desc.pose = bounds.getPose();
			desc.dimensions.set(
				bounds.getDimensions().v1 + skinThickness,
				bounds.getDimensions().v2 + skinThickness,
				bounds.getDimensions().v3 + skinThickness
			);
			return desc.create();
		}

		Bounds::Ptr expand(const BoundingConvexMesh &bounds) const {
			BoundingConvexMesh::Desc desc;

			const U32 numOfVertices = (U32)bounds.getVertices().size();
			desc.vertices.resize(numOfVertices);

			Vec3 centroid(REAL_ZERO);
			for (U32 i = 0; i < numOfVertices; ++i)
				centroid.add(centroid, bounds.getVertices()[i]);
			centroid.multiply(REAL_ONE/numOfVertices, centroid);
			
			// expand along lines drawn from centroid to vertices
			for (U32 i = 0; i < numOfVertices; ++i) {
				Vec3 axis;
				axis.subtract(bounds.getVertices()[i], centroid);
				axis.normalise();
				desc.vertices[i].multiplyAdd(skinThickness, axis, bounds.getVertices()[i]);
			}
			
			const U32 numOfTriangles = (U32)bounds.getTriangles().size();

			if (numOfTriangles > 0) {
				desc.bCook = false;
				desc.triangles = bounds.getTriangles();
			}
			else
				desc.bCook = true;
			
			return desc.create();
		}
	};

	// Expand bounds
	Expander expander(skinThickness);

	this->collisionBounds.clear();
	for (Bounds::Seq::const_iterator i = pCollisionBounds->begin(); i != pCollisionBounds->end(); i++) {
		Bounds::Ptr pBounds = expander.expand(**i);
		if (pBounds == NULL) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Heuristic::setCollisionBounds(): unable to create %s", (*i)->getName());
			continue;
		}
		//context.getMessageStream()->write(Message::LEVEL_DEBUG, "Heuristic::setCollisionBounds(): group #%08x, name: %s", pBounds->getGroup(), pBounds->getName());

		this->collisionBounds.push_back(pBounds);
	}
}

//------------------------------------------------------------------------------

Real Heuristic::getRestJointspaceDist(const ConfigspaceCoord& cc) const {
	const ConfigspaceCoord restConfig = arm.getRestConfig();
	Real dist = REAL_ZERO;
	for (U32 i = 0; i < numOfJoints; i++)
		dist += distRestJointcoordFac[i]*Math::sqr(cc[i] - restConfig[i]);

	return Math::sqrt(dist);
}

Real Heuristic::getJointspaceDist(const ConfigspaceCoord& j0, const ConfigspaceCoord& j1) const {
	Real dist = REAL_ZERO;
	for (U32 i = 0; i < numOfJoints; i++) {
		Real d = j1[i] - j0[i];
		dist += d*d;
	}

	return Math::sqrt(dist)/numOfJoints;
}

Real Heuristic::getJointspaceMagnitude(const ConfigspaceCoord& cc) const {
	Real dist = REAL_ZERO;
	for (U32 i = 0; i < numOfJoints; i++) {
		Real d = cc[i];
		dist += d*d;
	}

	return Math::sqrt(dist)/numOfJoints;
}

Real Heuristic::getJointspaceBoundedDist(const ConfigspaceCoord& j0, const ConfigspaceCoord& j1) const {
	Real dist = REAL_ZERO;
	for (U32 i = 0; i < numOfJoints; i++) {
		Real d = j1[i] - j0[i];

		if (Math::abs(d) > distJointcoordMax[i])
			return Node::COST_INF;
		
		dist += d*d;
	}

	return Math::sqrt(dist)/numOfJoints;
}

Real Heuristic::getLinearDist(const Vec3& l0, const Vec3& l1) const {
	return l0.distance(l1);
}

Real Heuristic::getAngularDist(const Quat& o0, const Quat& o1) const {
	const Real d = o0.dot(o1);
	
	// since body displacement represented by rotation around axis a by angle b corresponds to rotation around -a by angle -b
	// use max of a quaternion product as below (or equivalently abs)
	// abs(quat0 * quat1): range: <0, 1>, 1 - identity, 0 - the largest distance
	//return REAL_ONE - Math::abs(d);
	
	// acos(max(quat0 * quat1)): range: <0, Pi>, 0 - identity, Pi - the largest distance
	//return (REAL_ONE/REAL_PI)*Math::acos(Math::abs(d));

	return distAngularQuatDot ? REAL_ONE - Math::abs(d) : (REAL_ONE/REAL_PI)*Math::acos(Math::abs(d));
}

Real Heuristic::getWorkspaceDist(const Waypoint& w0, const Waypoint& w1) const {
	// linear distance
	const Real p = Heuristic::getLinearDist(w0.wpos.p, w1.wpos.p);
	// angular distance
	const Real q = Heuristic::getAngularDist(w0.wposq, w1.wposq);
	
	return distLinearFac*distWorkspaceFac*p + (REAL_ONE - distWorkspaceFac)*q;
}

Real Heuristic::getWorkspaceBoundedDist(const Waypoint& w0, const Waypoint& w1) const {
	// linear distance
	const Real p = Heuristic::getLinearDist(w0.wpos.p, w1.wpos.p);
	if (p > distLinearMax)
		return Node::COST_INF;
	
	// angular distance
	const Real q = Heuristic::getAngularDist(w0.wposq, w1.wposq);
	if (q > distAngularMax)
		return Node::COST_INF;
	
	return distLinearFac*distWorkspaceFac*p + (REAL_ONE - distWorkspaceFac)*q;
}

Real Heuristic::getDist(const Waypoint& w0, const Waypoint& w1) const {
	const Real wDist = Heuristic::getWorkspaceDist(w0, w1);
	const Real jDist = Heuristic::getJointspaceDist(w0.cpos, w1.cpos);
	
	return distJointspaceFac*jDist + (REAL_ONE - distJointspaceFac)*wDist;
}

Real Heuristic::getBoundedDist(const Waypoint& w0, const Waypoint& w1) const {
	const Real wDist = Heuristic::getWorkspaceBoundedDist(w0, w1);
	if (wDist >= Node::COST_INF)
		return Node::COST_INF;

	const Real jDist = Heuristic::getJointspaceBoundedDist(w0.cpos, w1.cpos);
	if (jDist >= Node::COST_INF)
		return Node::COST_INF;

	return distJointspaceFac*jDist + (REAL_ONE - distJointspaceFac)*wDist;
}

//------------------------------------------------------------------------------
