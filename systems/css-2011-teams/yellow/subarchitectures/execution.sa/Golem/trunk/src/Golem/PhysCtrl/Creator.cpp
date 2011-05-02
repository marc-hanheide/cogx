/** @file Creator.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/PhysCtrl/Creator.h>
#include <iomanip>
#include <sstream>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Creator::Creator(Scene &scene) :
	scene(scene),
	context(scene.getContext()),
	rand(context.getRandSeed())
{
	Creator::setToDefault();
}

Creator::~Creator() {
}

//------------------------------------------------------------------------------

golem::Actor::Desc *Creator::setupActorDesc() {
	buffer.write(Actor::Desc());
	Actor::Desc *pActorDesc = (Actor::Desc*)buffer.get().back();
	
	buffer.write(NxBodyDesc());
	NxBodyDesc *pNxBodyDesc = (NxBodyDesc*)buffer.get().back();
	pNxBodyDesc->solverIterationCount = NxU32(solverIterationCount);
	pNxBodyDesc->sleepEnergyThreshold = NxReal(sleepEnergyThreshold);
	pActorDesc->nxActorDesc.body = pNxBodyDesc;
	pActorDesc->nxActorDesc.density = (NxReal)density;
	
	return pActorDesc;
}

void Creator::setupBoundsDesc(golem::Actor::Desc *pActorDesc, const BoundingPlane::Desc &desc) {
	NxShapeDesc *pNxShapeDesc = scene.createNxShapeDesc(desc.clone()); // throws
	pActorDesc->nxActorDesc.shapes.push_back(pNxShapeDesc);
}

void Creator::setupBoundsDesc(golem::Actor::Desc *pActorDesc, const Bounds::Desc &desc) {
	NxShapeDesc *pNxShapeDesc = scene.createNxShapeDesc(desc.clone()); // throws
	pNxShapeDesc->density = (NxReal)density;
	pActorDesc->nxActorDesc.shapes.push_back(pNxShapeDesc);
}

//------------------------------------------------------------------------------

Actor::Desc *Creator::createGroundPlaneDesc() {
	// Create a ground plane. Plain does not need a body
	buffer.write(Actor::Desc());
	Actor::Desc *pActorDesc = (Actor::Desc*)buffer.get().back();
	BoundingPlane::Desc boundingPlaneDesc;
	setupBoundsDesc(pActorDesc, boundingPlaneDesc);
	
	return pActorDesc;
}

Actor::Desc *Creator::createBoxDesc(Real x, Real y, Real z) {
	Actor::Desc *pActorDesc = setupActorDesc();

	BoundingBox::Desc boxDesc;
	boxDesc.dimensions.set(x, y, z);
	if (mode == MODE_GROUNDPLANE) {
		//boxDesc.pose.p.v3 = z;
	}

	setupBoundsDesc(pActorDesc, boxDesc);
	
	if (mode == MODE_GROUNDPLANE) {
		pActorDesc->nxActorDesc.globalPose.t.z = NxReal(z);
	}
	
	return pActorDesc;
}

Actor::Desc *Creator::createTreeDesc(Real radius, Real thickness) {
	Actor::Desc *pActorDesc = setupActorDesc();
		
	BoundingSphere::Desc sphereDesc;
	sphereDesc.radius = radius;
	setupBoundsDesc(pActorDesc, sphereDesc);
		
	BoundingBox::Desc boxDesc;
	const Real size = thickness*radius;
	boxDesc.dimensions.set(size, size, size);
	boxDesc.pose.p.set((Real)0.0, (Real)0.0, - radius - size);
	setupBoundsDesc(pActorDesc, boxDesc);

	if (mode == MODE_GROUNDPLANE) {
		pActorDesc->nxActorDesc.globalPose.t.z += NxReal(radius + REAL_TWO*size);
	}

	return pActorDesc;
}

Actor::Desc *Creator::createSimple2FlapDesc(Real width, Real height, Real length, Real thickness, Real angle) {
	return create2FlapDesc(width, width, Real(0.0), Real(0.0), height, length, thickness, angle, false);
}

golem::Actor::Desc *Creator::create2FlapDesc(Real width1, Real width2, Real shift1, Real shift2, Real height, Real length, Real thickness, Real angle, bool moveFrame) {
	Actor::Desc *pActorDesc = setupActorDesc();
	
	BoundingBox::Desc boxDesc;

	// Y-up flap
	boxDesc.dimensions.set(width1, length, thickness);
	if (moveFrame) {
		boxDesc.pose.p.set(shift1, length + shift2, thickness);
		boxDesc.pose.R.setId();
//		boxDesc.pose.R.rotX(angle);
	}
	else {
		boxDesc.pose.p.set(REAL_ZERO, length, thickness);
		boxDesc.pose.R.setId();
	}
	setupBoundsDesc(pActorDesc, boxDesc);
	
	// Z-up flap
	Real sin, cos;
	Math::sinCos(angle, sin, cos);
	boxDesc.dimensions.set(width2, height, thickness);
	if (moveFrame) {
		boxDesc.pose.p.set(REAL_ZERO, cos*height, sin*height + thickness);
//		boxDesc.pose.R.setId();
		boxDesc.pose.R.rotX(angle);
	}
	else {
		boxDesc.pose.p.set(shift1, cos*height + shift2, sin*height + thickness);
		boxDesc.pose.R.rotX(angle);
	}
	setupBoundsDesc(pActorDesc, boxDesc);

	return pActorDesc;
}

//------------------------------------------------------------------------------

golem::shared_ptr<golem::GenWorkspaceState::Seq> Creator::createP2PTrajectory(const golem::WorkspaceCoord &begin, const golem::WorkspaceCoord &end, const golem::Profile &trajectory, SecTmReal duration, SecTmReal delta) {
	ASSERT(delta > REAL_EPS)
	
	const Real lengthScale = trajectory.getLength();
	const Real durationScale = trajectory.getDuration();
	const U32 partitions = (U32)Math::round(duration/delta);
	
	golem::shared_ptr<golem::GenWorkspaceState::Seq> seq(new golem::GenWorkspaceState::Seq(partitions + 1));
	
	const Quat qbegin(begin.R), qend(end.R);

	Vec3 dir;
	dir.subtract(end.p, begin.p);
	const Real length = dir.normalise();

	golem::GenWorkspaceState *prev = &(*seq)[0];
	prev->t = SEC_TM_REAL_ZERO;
	prev->pos = begin;
	prev->vel.setZero();
	for (U32 i = 1; i <= partitions; i++) {
		golem::GenWorkspaceState *next = &(*seq)[i];

		const SecTmReal ti = SecTmReal(i)/SecTmReal(partitions);
		next->t = duration*ti;

		const Real distance = trajectory.getDistance(ti*durationScale)/lengthScale;
		next->pos.p.multiplyAdd(length*distance, dir, begin.p);
		Quat q;
		q.slerp(qbegin, qend, distance);
		next->pos.R.fromQuat(q);

		const Real velocity = trajectory.getVelocity(ti*durationScale)/lengthScale;
		next->vel.v.multiply(length*velocity/duration, dir);
		next->vel.w.setZero(); // TODO
		// e.g. find angular difference dR between two poses at time t-1 and t+1 (R - rotation matrix):
		// dR*R(t-1) = R(t+1); dR = R(t+1)*R(t-1)^-1; vel(t) = {axis(dR), angle(dR)/(2.0*delta)}
		
		prev = next;
	}
	
	return seq;
}

//------------------------------------------------------------------------------
