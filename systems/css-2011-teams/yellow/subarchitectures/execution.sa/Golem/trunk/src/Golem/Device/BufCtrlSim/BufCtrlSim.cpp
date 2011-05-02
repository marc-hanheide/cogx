/** @file BufCtrlSim.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Device/BufCtrlSim/BufCtrlSim.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

BufCtrlSimJoint::BufCtrlSimJoint(Arm& arm) : BufCtrlJoint(arm) {}

bool BufCtrlSimJoint::create(const Desc& desc) {
	if (!Joint::create(desc))
		return false;
	
	readPtr = writePtr = 0;
	bStart = false;

	current.pos = trn.theta;
	current.vel = REAL_ZERO;
	current.acc = REAL_ZERO;

	return true;
}

bool BufCtrlSimJoint::sysSync() {
	GenCoordTrj& trjRead = trj[readPtr];
	bool bReset = false;

	if (bStart) {
		writePtr = Math::cycle_add((int)BufCtrlArm::SYS_BUF_LEN, writePtr, 1);
		bStart = false;
		bReset = true;
	}
	else if (trjRead.getEnd() > REAL_ZERO && trjRead.getEnd() < (Real)timer.elapsed()) {
		trjRead.setEnd(REAL_ZERO);
		readPtr = Math::cycle_add((int)BufCtrlArm::SYS_BUF_LEN, readPtr, 1);
		bReset = true;
	}
	
	if (trj[writePtr].getEnd() > REAL_ZERO)
		return false;

	if (bReset)
		timer.reset();
	
	return true;
}

bool BufCtrlSimJoint::sysRecv(GenCoord& curr) {
	GenCoordTrj& trjRead = trj[readPtr];

	if (trjRead.getEnd() > REAL_ZERO) {
		curr = trjRead.get((Real)timer.elapsed());
		// TODO provide real body movement effects
	}
	else
		curr = current;
	
	return true;
}

bool BufCtrlSimJoint::sysSend(const GenCoord& prev, const GenCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt) {
	trj[writePtr].set(REAL_ZERO, (Real)dt, prev, next);
	current = next;
	return true;
}

//------------------------------------------------------------------------------

BufCtrlSimArm::BufCtrlSimArm(golem::Context& context) : BufCtrlArm(context) {
}

BufCtrlSimArm::~BufCtrlSimArm() {
}

bool BufCtrlSimArm::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgBufCtrlSimArmInvalidDesc(Message::LEVEL_CRIT, "BufCtrlSimArm::create(): Invalid description");
	
	// all joints must inherit BufCtrlSimJoint
	for (Joint::Seq::const_iterator i = joints.begin(); i < joints.end(); ++i) {
		if (dynamic_cast<const BufCtrlSimJoint*>(*i) == NULL)
			throw MsgBufCtrlSimArmInvalidJointType(Message::LEVEL_CRIT, "BufCtrlSimArm::create(): invalid Joint type");
	}

	// must be initialized *before* arm creation (used in calibration)!
	deltaSync = desc.deltaSync;
	deltaRecv = desc.deltaRecv;
	deltaSend = desc.deltaSend;

	// create the arm
	BufCtrlArm::create(desc); // throws

	return true;
}

bool BufCtrlSimArm::sysRecv(GenConfigspaceCoord& curr) {
	bool bRes = BufCtrlArm::sysRecv(curr);
	timer.sleep(deltaRecv);
	return bRes;
}

bool BufCtrlSimArm::sysSend(const GenConfigspaceCoord& prev, const GenConfigspaceCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt) {
	bool bRes = BufCtrlArm::sysSend(prev, next, bSendPrev, bSendNext, dt);
	
	timer.sleep(deltaSend);
	
	for (Joint::Seq::const_iterator i = joints.begin(); i < joints.end(); ++i)
		dynamic_cast<BufCtrlSimJoint*>(*i)->bStart = true;
	
	return bRes;
}

bool BufCtrlSimArm::sysSync() {
	bool bRes = true;
	
	for (Joint::Seq::const_iterator i = joints.begin(); i < joints.end(); ++i)
		if (!dynamic_cast<BufCtrlSimJoint*>(*i)->sysSync())
			bRes = false;
	
	timer.sleep(deltaSync);
	
	return bRes;
}

//------------------------------------------------------------------------------
