/** @file Katana450.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Device/Katana450/Katana450.h>
#include <Golem/Device/Katana450/AxNI_base.h>
#include <Golem/Device/Katana450/Data.h>
#include <Golem/Ctrl/LoadArmDesc.h>
//#include <stdio.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadArmDesc(void* pContext, const std::string& path, void* pArmDesc) {
	loadArmDesc<golem::Katana450Arm::Desc>((Context*)pContext, path, (Arm::Desc::Ptr*)pArmDesc);
}

//------------------------------------------------------------------------------

const Real Katana450Joint::trj4ScaleFac[4] = {
	(Real)(pow(TRJ_TIME_QUANT, 0) * (1 << 0)),
	(Real)(pow(TRJ_TIME_QUANT, 1) * (1 << 6)),
	(Real)(pow(TRJ_TIME_QUANT, 2) * (1 << 10)),
	(Real)(pow(TRJ_TIME_QUANT, 3) * (1 << 15)),
};

//------------------------------------------------------------------------------

Katana450Joint::Katana450Joint(Arm& arm) : KatanaJoint(&arm), BufCtrlJoint(arm), pAxNI_base(NULL) {
}

Katana450Joint::~Katana450Joint() {
}

/** Creates Katana Joint */
bool Katana450Joint::create(const Desc& desc) {
	KatanaJoint::create(desc); // throws
	BufCtrlJoint::create(desc); // throws

	pKatana450Arm = dynamic_cast<Katana450Arm*>(&arm);
	if (pKatana450Arm == NULL)
		throw MsgKatana450JointInvalidArmType(Message::LEVEL_CRIT, "Katana450Joint::create() Invalid arm type");

	pAxNI_base = pKatana450Arm->pAxNI_base.get();

	calibration = desc.calibration;
	calibrationControl = desc.calibrationControl;
	calibrationCollision = desc.calibrationCollision;
	operationControl = desc.operationControl;
	operationCollision = desc.operationCollision;
	polynomial = desc.polynomial;

	// activate
	pAxNI_base->activate(U8(getIndex() + 1));
	// calibration
	pAxNI_base->calibrate(U8(getIndex() + 1), calibration, calibrationControl, calibrationCollision);
	// PID position controller parameters
	pAxNI_base->setControllerParameter(U8(getIndex() + 1), operationControl);
	// setup collision detection
	pAxNI_base->setCollisionDetection(U8(getIndex() + 1), false, operationCollision);
	// Polynomial settings
	pAxNI_base->setPolyScale(U8(getIndex() + 1), polynomial);
	pAxNI_base->moveBufferFlush(U8(getIndex() + 1));
	PerfTimer::sleep(AxNI_base::CANopenCycleTime);
	moveBufferSize = pAxNI_base->getMoveBufferSize(U8(getIndex() + 1));
	if (moveBufferSize < 2)
		throw MsgKatana450JointMoveBuffer(Message::LEVEL_CRIT, "Katana450Joint::create() Invalid move buffer size %i", moveBufferSize);

	// overwrite min, max
	min.pos = this->getKNIMinPos();
	max.pos = this->getKNIMaxPos();

	return true;
}

void Katana450Joint::release() {
	if (pAxNI_base) {
		while (pAxNI_base->getAxisFSMstate(U8(getIndex() + 1)) == AXIS_STATE_MOVING_SPLINE)
			PerfTimer::sleep(AxNI_base::CANopenCycleTime);
		pAxNI_base->release(U8(getIndex() + 1));
	}
	pAxNI_base = NULL;
}

//------------------------------------------------------------------------------

bool Katana450Joint::sysSync() {
	bool bRet;
	try {
		// Simulate buffer of length 2
		// Katana 450 doesnt report currently executed buffer
		bRet = pAxNI_base->getMoveBufferSize(U8(getIndex() + 1)) >= moveBufferSize;
	} catch (const std::exception& ex) {
		bRet = false;
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Katana450Joint::sysSync() [joint %d]: IO error (%s)", getIndex(), ex.what());
	}

	return bRet;
}

bool Katana450Joint::sysRecv(GenCoord& curr) {
	I32 encPos = 0;
	//I32 encVel = 0;

	try {
		encPos = (I32)pAxNI_base->getPosition(U8(getIndex() + 1));
		//encVel = (I32)pAxNI_base->getSpeed(U8(getIndex() + 1));
	} catch (const std::exception& ex) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Katana450Joint::sysRecv() [joint %d]: IO error (%s)", getIndex(), ex.what());
		return false;
	}

	curr.pos = posFromEnc(encPos);
	curr.vel = REAL_ZERO; // velFromEnc(encVel);
	curr.acc = REAL_ZERO; // not used

	return true;
}

bool Katana450Joint::sysSend(const GenCoord& prev, const GenCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt) {
	// Setup trajectory (dt > 0)
	GenCoord x0((posToEnc(prev.pos) - polynomial.offset)/polynomial.slope, velToEnc(prev.vel)/polynomial.slope, REAL_ZERO);
	GenCoord x1((posToEnc(next.pos) - polynomial.offset)/polynomial.slope, velToEnc(next.vel)/polynomial.slope, REAL_ZERO);
	GenCoordTrj trj(REAL_ZERO, Real(dt), x0, x1);

	// The trajectory end-point (the last position of a single movement)
	const I32 target = (I32)((posToEnc(next.pos) - polynomial.offset)/polynomial.slope);
	// Time duration of a single window, rounding
	const U16 duration = (U16)Math::round(SecTmReal(dt)/TRJ_TIME_QUANT);
	// Rescale coefficients
	I16 coeffs[4];
	for (U32 i = 0; i < 4; i++)
		coeffs[i] = (I16)Math::round(Math::clamp(Math::round(trj4ScaleFac[i]*trj.getCoeffs()[i]), (Real)numeric_const<I16>::MIN, (Real)numeric_const<I16>::MAX));

	try {
		pAxNI_base->moveBufferStore(U8(getIndex() + 1), target, duration, polynomial.tolerance, coeffs, bSendNext, true);
		PerfTimer::sleep(AxNI_base::CANopenCycleTime); // more conservative
		//PerfTimer::sleep(AxNI_base::CANopenInhibitTime); // faster
	} catch (const std::exception& ex) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Katana450Joint::sysSend() [joint %d]: IO error (%s)", getIndex(), ex.what());
		return false;
	}

	return true;
}

bool Katana450Joint::sysStart() {
	try {
		pAxNI_base->setAxisFSMcommand(U8(getIndex() + 1), AXIS_CMD_MOVE_SPLINE, true);
	} catch (std::exception& ex) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Katana450Joint::sysStart() [joint %d]: Setting move spline error (%s)", getIndex(), ex.what()	);
		return false;
	}
	
	return true;
}

//------------------------------------------------------------------------------

Katana450Arm::Katana450Arm(golem::Context& context) : KatanaArm(&context), KatanaGripper(&context), BufCtrlArm(context) {
	bCalibrationInit = false;
}

Katana450Arm::~Katana450Arm() {
	// stop control thread before releasing AxNI stuff
	BufCtrlArm::release();

	for (Joint::Seq::const_iterator i = joints.begin(); i < joints.end(); ++i) {
		Katana450Joint* pKatana450Joint = dynamic_cast<Katana450Joint*>(*i);
		if (pKatana450Joint != NULL)
			pKatana450Joint->release();
	}
}

bool Katana450Arm::create(const Desc& desc) {
	KatanaArm::create(desc); // throws
	KatanaGripper::create(desc); // throws

	canDevice = desc.canDevice;
	bMeasurements = desc.bMeasurements;
	syncJointIndex = desc.syncJointIndex;

	// TODO: all this should go into something like a Katana450Gripper class
	gripperMaxCommTime = desc.gripperMaxCommTime;
	gripperEncoderData.open = desc.gripperOpenEncVal;
	gripperEncoderData.closed = desc.gripperClosedEncVal;
	gripperAngleOffset = desc.gripperAngleOffset*REAL_2_PI/360.0;
	gripperAngleRange = desc.gripperAngleRange*REAL_2_PI/360.0;
	gripperEncoderOffset = desc.gripperEncoderOffset;
	gripperEncodersPerCycle = desc.gripperEncodersPerCycle;
	gripperRotationDirection = desc.gripperRotationDirection;
	gripperEncoderPositionAfter = desc.gripperEncoderPositionAfter;
	gripperOffset = desc.gripperOffset;
	gripperGain = desc.gripperGain;
	gripperCalibration = desc.gripperCalibration;
	gripperCalibrationControl = desc.gripperCalibrationControl;
	gripperCalibrationCollision = desc.gripperCalibrationCollision;
	gripperOperationControl = desc.gripperOperationControl;
	gripperOperationCollision = desc.gripperOperationCollision;

	// start
	pAxNI_base.reset(new AxNI_base);
	pAxNI_base->start(canDevice);
	
	// all joints must inherit Katana450Joint
	for (Joint::Seq::const_iterator i = joints.begin(); i < joints.end(); ++i) {
		Katana450Joint* pKatana450Joint = dynamic_cast<Katana450Joint*>(*i);
		if (pKatana450Joint == NULL)
			throw MsgKatana450ArmInvalidJointType(Message::LEVEL_CRIT, "Katana450Arm::create(): invalid Joint type");
	}

	if (bGripper) {
		evGripper.set(false);
		gripperStatus = OPEN;
		gripperStallCounter = 0;
		bGripperResult = false;
	}

#ifdef GOLEM_DEVICE_KATANA_DEBUG_
	return Arm::create(desc);
#else
	return BufCtrlArm::create(desc);
#endif
}

//------------------------------------------------------------------------------
// Controller

bool Katana450Arm::sysRecv(GenConfigspaceCoord& curr) {
	if (bMeasurements)
		return BufCtrlArm::sysRecv(curr);

	if (isCalibrating()) {
		if (!bCalibrationInit && !BufCtrlArm::sysRecv(current))
			return false;
		
		bCalibrationInit = true;
		curr = current;
		return true;
	}
	
	bCalibrationInit = false;
	
	GenConfigspaceState states[2];
	GenCoordTrj trj;
	SecTmReal t = context.getTimer().elapsed();
	if (!Arm::lookupCommand(states, states + 2, t, t))
		return false;
		
	t = Math::clamp(t, states[0].t, states[1].t);
	
	for (U32 i = 0; i < joints.size(); i++) {
		trj.set(REAL_ZERO, Real(states[1].t - states[0].t), states[0].get(i), states[1].get(i));
		curr.set(i, trj.get(Real(t - states[0].t)));
	}
	
	return true;
}

bool Katana450Arm::sysSend(const GenConfigspaceCoord& prev, const GenConfigspaceCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt) {
	if (!BufCtrlArm::sysSend(prev, next, bSendPrev, bSendNext, dt))
		return false;

	if (!bSendPrev) {
		for (Joint::Seq::const_iterator i = joints.begin(); i < joints.end(); ++i) {
			Katana450Joint* pKatana450Joint = dynamic_cast<Katana450Joint*>(*i);
			ASSERT(pKatana450Joint)
			pKatana450Joint->sysStart();
		}
	}

	PerfTimer::sleep(AxNI_base::CANopenCycleTime);

	return true;
}

bool Katana450Arm::sysSync() {
	bool bRet = true;
	
	for (std::set<U32>::const_iterator i = syncJointIndex.begin(); i != syncJointIndex.end(); i++) {
		Katana450Joint* pKatana450Joint = dynamic_cast<Katana450Joint*>(joints[*i]);
		ASSERT(pKatana450Joint)
		bRet = bRet && pKatana450Joint->sysSync(); // detects false positive only, to keep timing constant always iterate all joints
	}
	
	PerfTimer::sleep(AxNI_base::CANopenInhibitTime);
	
	return bRet;
}

//------------------------------------------------------------------------------
// Gripper

void Katana450Arm::userCalibrate() {
	if (bGripper)
		gripperCalibrate();
}

void Katana450Arm::userComm() {
	if (bGripper)
		gripperComm();
}

void Katana450Arm::gripperCalibrate() {
	U8 gripperNode = U8(joints.size() + 1);

	{
		// NOTE: not sure if critical section is needed here, but can't hurt
		CriticalSectionWrapper csw(csGripper);

		try {
			// activate
			pAxNI_base->activate(gripperNode);
			// calibration
			pAxNI_base->calibrate(gripperNode, gripperCalibration, gripperCalibrationControl, gripperCalibrationCollision);
			// PID position controller parameters
			pAxNI_base->setControllerParameter(gripperNode, gripperOperationControl);
			// setup collision detection
			pAxNI_base->setCollisionDetection(gripperNode, false, gripperOperationCollision);
			// and read current encoder value
			gripperEncoderData.current = pAxNI_base->getPosition(gripperNode);
		} catch (std::exception& e) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Katana450Arm::gripperCalibrate(): IO error (%s)", e.what()	);
		}
	}
}

void Katana450Arm::gripperComm() {
	// While system is calibrating timings, return a fixed delay, which is
	// large enough to cover actual maximum delays in gripperComm(). The
	// actual delays in gripperComm() are varying and hard to time.
	if (isCalibrating())
	{
		PerfTimer::sleep(gripperMaxCommTime);
		return;
	}

	U8 gripperNode = U8(joints.size() + 1);

	{
		CriticalSectionWrapper csw(csGripper);

		try {

			PerfTimer timer;
			if(gripperStatus & (OPEN | CLOSE | FREEZE)) {

				// getPosition() takes around 0.000004 s (2.4 GHz dual core)
				gripperEncoderData.current = pAxNI_base->getPosition(gripperNode);

				// getSpeed() takes around 0.000004 s (2.4 GHz dual core)
				I16 speed = pAxNI_base->getSpeed(gripperNode);

				U32 target = gripperStatus & OPEN ? gripperEncoderData.open :
						gripperStatus & CLOSE ? gripperEncoderData.closed : gripperEncoderData.current;

				// TODO: where do magic values speed = 10, accel = 1, tolerance = 50 come from?
				// try immediate = true first, if does not work use false and maybe need to hack moveP2P()
				// result: immediate = true does not work, false seems to work fine.
				//
				// moveP2P(immediate = false) takes at most 0.012939 s (2.4 GHz dual core)
				pAxNI_base->moveP2P(gripperNode, target, 10, 1, 50, false);

				if (gripperStatus & FREEZE) {
					gripperStatus &= ~FREEZE;
					bGripperResult = true;
					evGripper.set(true);
				}
				else if (gripperStatus & (OPEN | CLOSE)) {
					// if reached target
					if (Math::abs(I32(target - gripperEncoderData.current)) < GRIP_TARGET_ENC_TOLERANCE) {
						gripperStatus &= ~(OPEN | CLOSE);
						bGripperResult = true;
						evGripper.set(true);
					}
					// if not reached, but no further movement for some time (i.e. for some number of
					// cycles) -> fingers are blocked by grasped object, stop
					else if (Math::abs(speed) == 0) {
						gripperStallCounter++;
						if (gripperStallCounter >= GRIP_STALL_CYCLES) {
							// hold() takes at most 0.010187 s (2.4 GHz dual core)
							pAxNI_base->hold(gripperNode);
							gripperStatus &= ~(OPEN | CLOSE);
							bGripperResult = false;
							evGripper.set(true);
						}
					}
				}
			}
		} catch (std::exception& e) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Katana450Arm::gripperComm(): IO error (%s)", e.what()	);
		}
	}
}

//------------------------------------------------------------------------------

bool Katana450Arm::gripperRecvSensorData(SensorDataSet& sensorData, MSecTmU32 timeWait) {
	// TODO
	return false;
}

bool Katana450Arm::gripperRecvEncoderData(GripperEncoderData& encoderData, MSecTmU32 timeWait) {
	if (!bGripper || isCalibrating())
		return false;

	{
		CriticalSectionWrapper csw(csGripper);
		encoderData = gripperEncoderData;
	}

	return true;
}

bool Katana450Arm::gripperOpen(MSecTmU32 timeWait) {
	if (!bGripper || isCalibrating())
		return false;

	{
		CriticalSectionWrapper csw(csGripper);
		evGripper.set(false);
		gripperStatus &= ~(OPEN | CLOSE | FREEZE);
		gripperStatus |= OPEN;
		gripperStallCounter = 0;
		bGripperResult = false;
	}

	return timeWait <= 0 || evGripper.wait(timeWait) && bGripperResult;
}

bool Katana450Arm::gripperClose(const SensorDataSet& sensorThreshold, MSecTmU32 timeWait) {
	if (!bGripper || isCalibrating())
		return false;

	{
		CriticalSectionWrapper csw(csGripper);
		evGripper.set(false);
		gripperStatus &= ~(OPEN | CLOSE | FREEZE);
		gripperStatus |= CLOSE;
		gripperStallCounter = 0;
		bGripperResult = false;
		// TODO: At present we can't read Katana450 sensors, so sensorThreshold is not used.
		// Instead we use motor stall to detect when we grasped something.
	}

	return timeWait <= 0 || evGripper.wait(timeWait) && bGripperResult;
}

bool Katana450Arm::gripperFreeze(MSecTmU32 timeWait) {
	if (!bGripper || isCalibrating())
		return false;

	{
		CriticalSectionWrapper csw(csGripper);
		evGripper.set(false);
		gripperStatus &= ~(OPEN | CLOSE | FREEZE);
		gripperStatus |= FREEZE;
		bGripperResult = false;
	}

	return timeWait <= 0 || evGripper.wait(timeWait) && bGripperResult;
}

//------------------------------------------------------------------------------

void Katana450Arm::forwardTransform(Mat34& trn, const ConfigspaceCoord& cc) {
	if (customKinematics)
		KatanaArm::forwardTransform(trn, cc);
	else
		Arm::forwardTransform(trn, cc);
}

void Katana450Arm::velocitySpatial(Twist& v, const ConfigspaceCoord& cc, const ConfigspaceCoord& dcc) {
	if (customKinematics)
		KatanaArm::velocitySpatial(v, cc, dcc);
	else
		Arm::velocitySpatial(v, cc, dcc);
}

void Katana450Arm::jacobianSpatial(Jacobian& jac, const ConfigspaceCoord& cc) {
	if (customKinematics)
		KatanaArm::jacobianSpatial(jac, cc);
	else
		Arm::jacobianSpatial(jac, cc);
}

//------------------------------------------------------------------------------
