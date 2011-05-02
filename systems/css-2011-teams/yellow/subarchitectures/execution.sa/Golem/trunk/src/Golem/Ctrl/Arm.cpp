/** @file Arm.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Ctrl/Arm.h>
#include <Golem/Ctrl/Msg.h>
#include <Golem/Tools/Library.h>
#ifndef WIN32
#include <dlfcn.h>
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GenCoordTrj::GenCoordTrj() {
	setBegin(REAL_ZERO);
	setEnd(REAL_ZERO); // zero is required by simulator, use create() otherwise
}

GenCoordTrj::GenCoordTrj(Real t0, Real t1, const GenCoord& c0, const GenCoord& c1) {
	set(t0, t1, c0, c1);
}

void GenCoordTrj::set(Real t0, Real t1, const GenCoord& c0, const GenCoord& c1) {
	Polynomial4::set2dvdv(t0, t1, c0.pos, c0.vel, c1.pos, c1.vel);
}

GenCoord GenCoordTrj::get(Real t) const {
	const Real tc = Math::clamp(t, getBegin(), getEnd());
	GenCoord c;

	c.pos = getDistance(tc);
	c.vel = getVelocity(tc);
	c.acc = getAcceleration(tc);
	
	return c;
}

//------------------------------------------------------------------------------

Joint::Joint(Arm& arm) : arm(arm), context(arm.getContext()), index(0) {
}

bool Joint::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgJointInvalidDesc(Message::LEVEL_CRIT, "Joint::create(): Invalid description");
	
	index = Math::clamp((U32)desc.index, (U32)0, CONFIG_SPACE_DIM);
	name = desc.name;
	min = desc.min;
	max = desc.max;
	trn = desc.trn;
	trnInit = desc.trnInit;
	collision = desc.collision;
	collisionOffset = desc.collisionOffset;
	
	boundsDescSeq.clear();	
	for (Bounds::Desc::Seq::const_iterator i = desc.bounds.begin(); i != desc.bounds.end(); i++)
		if (*i != NULL)
			boundsDescSeq.push_back((*i)->clone());

	pCallback = NULL;

	return true;
}

bool Joint::addBoundsDesc(Bounds::Desc::Ptr pDesc) {
	{
		CriticalSectionWrapper csw(csData);
		boundsDescSeq.push_back(pDesc);
	}

	// TODO this should be atomic call
	if (pCallback != NULL)
		pCallback->syncJointBoundsDesc();

	return true;
}

bool Joint::removeBoundsDesc(const Bounds::Desc* pDesc) {
	Bounds::Desc::Seq::iterator pos = std::find(boundsDescSeq.begin(), boundsDescSeq.end(), pDesc);
	if (pos == boundsDescSeq.end()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Joint::removeBoundsDesc(): Unable to find bounds description"	);
		return false;
	}
	{
		CriticalSectionWrapper csw(csData);
		boundsDescSeq.erase(pos);
	}

	// TODO this should be atomic call
	if (pCallback != NULL)
		pCallback->syncJointBoundsDesc();

	return true;
}

Bounds::Desc::SeqPtr Joint::getBoundsDescSeq() const {
	CriticalSectionWrapper csw(csData);
	return Bounds::Desc::clone(boundsDescSeq.begin(), boundsDescSeq.end());
}

//------------------------------------------------------------------------------

// TODO: unloading libraries
Arm::Desc::Ptr Arm::Desc::load(Context& context, const std::string& path) {
	Arm::Desc::Ptr pArmDesc;

#ifdef WIN32
	const size_t pos = path.find_last_of("\\/") + 1;
	const std::string library = path.substr(0, pos) + "" + path.substr(pos) + ".dll";

	HINSTANCE handle = ::LoadLibrary(library.c_str());
	if (!handle)
		throw MsgArmOpenLib(Message::LEVEL_CRIT, "Arm::Desc::load(): unable to open library: %s", library.c_str());

	LoadArmDesc loadArmDesc = (LoadArmDesc)::GetProcAddress(handle, "loadArmDesc");
	if (!loadArmDesc)
		throw MsgArmLoadDesc(Message::LEVEL_CRIT, "Arm::Desc::load(): unable to obtain pointer to loadArmDesc()");

	loadArmDesc(&context, path, &pArmDesc);
#else
	const size_t pos = path.find_last_of("\\/") + 1;
	const std::string library = path.substr(0, pos) + "lib" + path.substr(pos) + ".so";

	void* handle = ::dlopen(library.c_str(), RTLD_LAZY);
	if (!handle)
		throw MsgArmOpenLib(Message::LEVEL_CRIT, "Arm::Desc::load(): unable to open library: %s", /*library.c_str()*/dlerror());

	LoadArmDesc loadArmDesc = (LoadArmDesc)::dlsym(handle, "loadArmDesc");
	if (!loadArmDesc)
		throw MsgArmLoadDesc(Message::LEVEL_CRIT, "Arm::Desc::load(): unable to obtain pointer to loadArmDesc()");

	loadArmDesc(&context, path, &pArmDesc);
#endif

	return pArmDesc;
}

//------------------------------------------------------------------------------

Arm::Arm(golem::Context& context) :	context(context) {
}

Arm::~Arm() {
}

bool Arm::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgArmInvalidDesc(Message::LEVEL_CRIT, "Arm::create(): Invalid description");

	name = desc.name;
	customKinematics = desc.customKinematics;
	
	jointList.clear();
	joints.clear();

	const U32 numOfJoints = (U32)desc.joints.size();
	for (U32 i = 0; i < numOfJoints; i++) {
		desc.joints[i]->index = i;

		Joint::Ptr pJoint = desc.joints[i]->create(*this); // throws
		jointList.push_back(pJoint);
		joints.push_back(pJoint.get());
	}

	globalPose = desc.globalPose;
	referencePose = desc.referencePose;
	restConfig = desc.restConfig;
	return true;
}

//------------------------------------------------------------------------------

bool Arm::interpolate(CriticalSection& cs, const Queue& queue, GenConfigspaceState& state, SecTmReal t) const {
	GenConfigspaceState states[2];
	
	const GenConfigspaceState* ptr;
	{
		CriticalSectionWrapper csw(cs);
		ptr = lookup(queue, states, states + 2, t, t);
	}
	if (ptr == states || ptr == states + 1) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Arm::interpolate(): queue query error");
		return false;
	}

	// clamp on limits
	state.t = Math::clamp(t, states[0].t, states[1].t);

	// interpolate
	const U32 numOfJoints = (U32)joints.size();
	for (U32 j = 0; j < numOfJoints; j++)
		state.set(j, GenCoordTrj(states[0].t, states[1].t, states[0].get(j), states[1].get(j)).get(state.t));

	return true;
}

//------------------------------------------------------------------------------

Mat34 Arm::getGlobalPose() const {
	CriticalSectionWrapper csw(csData);
	return globalPose;
}

void Arm::setGlobalPose(const Mat34 &globalPose) {
	CriticalSectionWrapper csw(csData);
	this->globalPose = globalPose;
}

Mat34 Arm::getReferencePose() const {
	CriticalSectionWrapper csw(csData);
	return referencePose;
}

void Arm::setReferencePose(const Mat34 &referencePose) {
	CriticalSectionWrapper csw(csData);
	this->referencePose = referencePose;
}		

ConfigspaceCoord Arm::getRestConfig() const {
//	CriticalSectionWrapper csw(csData);
	return restConfig;
}

void Arm::setRestConfig(const ConfigspaceCoord &restConfig) {
//	CriticalSectionWrapper csw(csData);
	this->restConfig = restConfig;
}

//------------------------------------------------------------------------------

// Generic froward kinematics generated from twist coordinates
//
// n		- number of joints
// cc(i)	- transformation generated by the twist of the i-th joint
// ccini(i)	- initial transformation generated by the twist of the i-th joint
// cctp		- optional transformation generated by the fixed/static tool frame pose 
//
// trn		= cc(0)*cc(1)* ... *cc(n-2)*cc(n-1)*ccini(n-1)*cctp
//
void Arm::forwardTransform(Mat34& trn, const ConfigspaceCoord& cc) const {
	Mat34 tmp;

	const I32 numOfJoints = (I32)joints.size();
	trn.fromTwist(joints[numOfJoints - 1]->getTrnInit().twist, joints[numOfJoints - 1]->getTrnInit().theta);
	for (I32 i = numOfJoints - 1; i >= 0; i--) {
		tmp.fromTwist(joints[i]->getTrn().twist, cc[i]);
		trn.multiply(tmp, trn);
	}

	trn.multiply(getGlobalPose(), trn);
}


// Generic (extended) forward transformation for all joints
//
// n		- number of joints
// cc(i)	- transformation generated by the twist of the i-th joint
// ccini(i)	- initial transformation generated by the twist of the i-th joint
// cctp		- optional transformation generated by the fixed/static tool frame pose 
//
// trn[0]	= cc(0)*ccini(0)
// trn[1]	= cc(0)*cc(1)*ccini(1)
// trn[2]	= cc(0)*cc(1)*cc(2)*ccini(2)
// ...
// trn[n-2]	= cc(0)*cc(1)* ... *cc(n-3)*cc(n-2)*ccini(n-2)
// trn[n-1]	= cc(0)*cc(1)* ... *cc(n-3)*cc(n-2)*cc(n-1)*ccini(n-1)*cctp
//
void Arm::forwardTransformEx(Mat34 trn [], const ConfigspaceCoord& cc) const {
	ASSERT(trn != NULL)
	const U32 numOfJoints = (U32)joints.size();
	const Mat34 globalPose = getGlobalPose();
	
	trn[0].fromTwist(joints[0]->getTrn().twist, cc[0]);	
	for (U32 i = 1; i < numOfJoints; i++) {
		trn[i].fromTwist(joints[i]->getTrn().twist, cc[i]);
		trn[i].multiply(trn[i - 1], trn[i]);
	}

	Mat34 tmp;
	for (U32 i = 0; i < numOfJoints; i++) {
		tmp.fromTwist(joints[i]->getTrnInit().twist, joints[i]->getTrnInit().theta);
		trn[i].multiply(trn[i], tmp);
		trn[i].multiply(globalPose, trn[i]);
	}
}

//------------------------------------------------------------------------------

// Generic spatial velocity generated from twist coordinates
void Arm::velocitySpatial(Twist& vs, const ConfigspaceCoord& cc, const ConfigspaceCoord& dcc) const {
	Mat34 tmp, trn;
	Twist ad;

	vs.multiply(dcc[0], joints[0]->getTrn().twist);

	const U32 numOfJoints = (U32)joints.size();
	trn.setId();
	for (U32 i = 1; i < numOfJoints; i++) {
		tmp.fromTwist(joints[i - 1]->getTrn().twist, cc[i - 1]);
		trn.multiply(trn, tmp);
		trn.adjointTransform(ad, joints[i]->getTrn().twist);

		vs.multiplyAdd(dcc[i], ad, vs);
	}
}

// Generic body velocity generated from twist coordinates
void Arm::velocityBody(Twist& vb, const ConfigspaceCoord& cc, const ConfigspaceCoord& dcc) const {
	Mat34 tmp, trn;
	Twist ad;

	const I32 numOfJoints = (I32)joints.size();
	trn.fromTwist(joints[numOfJoints - 1]->getTrnInit().twist, joints[numOfJoints - 1]->getTrnInit().theta);
	vb.setZero();
	
	for (I32 i = numOfJoints - 1; i >= 0; i--) {
		tmp.fromTwist(joints[i]->getTrn().twist, cc[i]);
		trn.multiply(tmp, trn);
		trn.adjointInverseTransform(ad, joints[i]->getTrn().twist);

		vb.multiplyAdd(dcc[i], ad, vb);
	}
}

// End-effector velocity
void Arm::velocity(Twist& v, const ConfigspaceCoord& cc, const ConfigspaceCoord& dcc) const {
	Arm::velocitySpatial(v, cc, dcc);
}

void Arm::velocityFromJacobian(Twist& v, const Jacobian& jac, const ConfigspaceCoord& dcc) const {
	const U32 numJoints = (U32)joints.size();
	
	v.setZero();
	for (U32 i = 0; i < numJoints; i++)
		v.multiplyAdd(dcc[i], jac[i], v);
}

void Arm::velocityFromSpatial(Twist& v, const Twist& vs, const Mat34& trn) const {
	v.v.cross(vs.w, trn.p);
	v.v.add(vs.v, v.v);
	v.w = vs.w;
}

void Arm::velocityFromBody(Twist& v, const Twist& vb, const Mat34& trn) const {
	trn.R.multiply(v.v, vb.v);
	trn.R.multiply(v.w, vb.w);
}

// Generic spatial jacobian generated from twist coordinates
void Arm::jacobianSpatial(Jacobian& jac, const ConfigspaceCoord& cc) const {
	Mat34 tmp, trn;

	const U32 numOfJoints = (U32)joints.size();
	trn.setId();

	jac[0] = joints[0]->getTrn().twist;
	for (U32 i = 1; i < numOfJoints; i++) {
		tmp.fromTwist(joints[i - 1]->getTrn().twist, cc[i - 1]);
		trn.multiply(trn, tmp);
		trn.adjointTransform(jac[i], joints[i]->getTrn().twist);
	}
}

// Generic body jacobian generated from twist coordinates
void Arm::jacobianBody(Jacobian& jac, const ConfigspaceCoord& cc) const {
	Mat34 tmp, trn;

	const I32 numOfJoints = (I32)joints.size();
	trn.fromTwist(joints[numOfJoints - 1]->getTrnInit().twist, joints[numOfJoints - 1]->getTrnInit().theta);
	
	for (I32 i = numOfJoints - 1; i >= 0; i--) {
		tmp.fromTwist(joints[i]->getTrn().twist, cc[i]);
		trn.multiply(tmp, trn);
		trn.adjointInverseTransform(jac[i], joints[i]->getTrn().twist);
	}
}

// Generic jacobian generated from twist coordinates
void Arm::jacobian(Jacobian& jac, const ConfigspaceCoord& cc) const {
	Arm::jacobianSpatial(jac, cc);
}

void Arm::jacobianFromSpatial(Jacobian& jac, const Jacobian& jacs, const Mat34& trn) const {
	const U32 numOfJoints = (U32)joints.size();
	const Vec3 p = trn.p;
	
	for (U32 i = 0; i < numOfJoints; i++) {
		const Twist s = jacs[i];
		
		jac[i].set(
			s.v.v1 + p.v3*s.w.v2 - p.v2*s.w.v3,
			s.v.v2 - p.v3*s.w.v1 + p.v1*s.w.v3,
			s.v.v3 + p.v2*s.w.v1 - p.v1*s.w.v2,
			s.w.v1, s.w.v2, s.w.v3
		);
	}
}

void Arm::jacobianFromBody(Jacobian& jac, const Jacobian& jacb, const Mat34& trn) const {
	const U32 numOfJoints = (U32)joints.size();
	const Mat33 R = trn.R;
	
	for (U32 i = 0; i < numOfJoints; i++) {
		const Twist b = jacb[i];
		
		jac[i].set(
			R.m11*b.v.v1 + R.m12*b.v.v2 + R.m13*b.v.v3,
			R.m21*b.v.v1 + R.m22*b.v.v2 + R.m23*b.v.v3,
			R.m31*b.v.v1 + R.m32*b.v.v2 + R.m33*b.v.v3,
			R.m11*b.w.v1 + R.m12*b.w.v2 + R.m13*b.w.v3,
			R.m21*b.w.v1 + R.m22*b.w.v2 + R.m23*b.w.v3,
			R.m31*b.w.v1 + R.m32*b.w.v2 + R.m33*b.w.v3
		);
	}
}

//------------------------------------------------------------------------------
