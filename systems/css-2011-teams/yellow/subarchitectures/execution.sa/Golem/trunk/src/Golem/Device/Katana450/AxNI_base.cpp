/** @file AxNI_base.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Device/Katana450/AxNI_base.h>
#include <Golem/Device/Katana450/canopen_master.h>
#include <Golem/Device/Katana450/enum_nmt.h>
#include <Golem/Math/Math.h>
#include <Golem/Tools/Message.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const U32 AxNI_base::CANpollingTime = 1000;// [us]
const SecTmReal AxNI_base::CANopenCycleTime = 0.010;// [s]
const SecTmReal AxNI_base::CANopenInhibitTime = 0.0025;// [s]
const SecTmReal AxNI_base::CANopenHeartBeatPeriod = 0.1;// [s]

AxNI_base::AxNI_base() : pCANopen(new CANopenMaster) {
}

AxNI_base::~AxNI_base() {
	stop();
}

//------------------------------------------------------------------------------

void AxNI_base::wait4state(U8 node, EAxisFsmState newState, SecTmReal timeout) {
	PerfTimer timer;
	const SecTmReal t = timer.elapsed() + timeout;
	while (t > timer.elapsed()) {
		EAxisFsmState state = getAxisFSMstate(node);
		if (state == newState)
			return;
		PerfTimer::sleep(CANopenInhibitTime);
	}
	throw Message("AxNI_base::wait4state() [Node %d]: waiting for state %d failed", (U32)node, (U32)newState);
}

void AxNI_base::wait2stop(U8 node) {
	while (Math::abs(getSpeed(node)) > 0)
		PerfTimer::sleep(0.1);
}

//------------------------------------------------------------------------------

void AxNI_base::start(const std::string& canDevice) {
	pCANopen->CanBusSetDevice(canDevice);
	if (!pCANopen->CanBusStart(CANpollingTime))
		throw Message("AxNI_base::start(): Device %s could not open CAN bus interface", canDevice.c_str());
	PerfTimer::sleep(SecTmReal(2.0)*CANopenHeartBeatPeriod);
}

void AxNI_base::activate(U8 node) {
	if (!pCANopen->NMTisAlive(node))
		throw Message("AxNI_base::activate() [Node %d]: is not alive", (U32)node);
	if (!(pCANopen->NMTgetState(node) == NMT_State_Operational)) {
		pCANopen->NMTsendCmd(NMT_Cmd_Operational, node);
		PerfTimer::sleep(SecTmReal(2.0)*CANopenHeartBeatPeriod);
	}
	if (pCANopen->NMTgetState(node) != NMT_State_Operational)
		throw Message("AxNI_base::activate() [Node %d]: can not be set into operational state.", (U32)node);
}

void AxNI_base::stop() {
	pCANopen->CanBusStop();
}

void AxNI_base::hold(U8 node) {
	setAxisFSMcommand(node, AXIS_CMD_HOLD);
	wait4state(node, AXIS_STATE_HOLDING, CANopenHeartBeatPeriod);
}

void AxNI_base::release(U8 node) {
	setAxisFSMcommand(node, AXIS_CMD_RELEASE);
	wait4state(node, AXIS_STATE_RELEASED, CANopenHeartBeatPeriod);
}

//------------------------------------------------------------------------------

void AxNI_base::setAxisFSMcommand(U8 node, EAxisFsmCommand fsmCommand, bool immediate) {
	pCANopen->PDOwriteVal_ui8(CANopen_i8(fsmCommand), RX_DATA_AXIS_COMMAND, node);
	pCANopen->PDOsend(RX_DATA_AXIS_COMMAND, node);
	if (!immediate)
		PerfTimer::sleep(CANopenCycleTime);
}

EAxisFsmState AxNI_base::getAxisFSMstate(U8 node) {
	CANopen_ui8 state = pCANopen->PDOreadVal_ui8(TX_DATA_AXIS_STATE, node);
	return EAxisFsmState(state);
}

//------------------------------------------------------------------------------

void AxNI_base::setEncoder(U8 node, U32 target) {
	pCANopen->PDOwriteVal_ui32(target, RX_DATA_ENCODER_POS, node);
	pCANopen->PDOsend(RPDO_ENCODER_SETUP, node);
	PerfTimer::sleep(CANopenInhibitTime);
	setAxisFSMcommand(node, AXIS_CMD_SET_ENCODER);
	wait4state(node, AXIS_STATE_SET_ENCODER, CANopenHeartBeatPeriod);
	hold(node);
}

void AxNI_base::setCollisionDetection(U8 node, bool active, const CollisionParameter& colParam) {
	pCANopen->PDOwriteVal_ui8(CANopen_ui8(active), RX_DATA_COLLISION_ACTIVE, node);
	pCANopen->PDOwriteVal_ui16(colParam.limitPos, RX_DATA_COLLISION_LIMIT_ERR_POS, node);
	pCANopen->PDOwriteVal_ui16(colParam.limitSpeed, RX_DATA_COLLISION_LIMIT_ERR_SPEED, node);
	pCANopen->PDOsend(RPDO_COLLISION_DETECTION, node);
	PerfTimer::sleep(CANopenInhibitTime);
}

void AxNI_base::setControllerParameter(U8 node, const ControllerParameter& ctrlParam) {
	pCANopen->PDOwriteVal_ui16(ctrlParam.kPPos, RX_DATA_CONTROLLER_KP_POS, node);
	pCANopen->PDOwriteVal_ui16(ctrlParam.kIPos, RX_DATA_CONTROLLER_KI_POS, node);
	pCANopen->PDOwriteVal_ui16(ctrlParam.kDPos, RX_DATA_CONTROLLER_KD_POS, node);
	pCANopen->PDOwriteVal_ui16(ctrlParam.kILegacy , RX_DATA_CONTROLLER_KI_LEGACY, node);
	pCANopen->PDOsend(RPDO_CONTROL_PARAMETERS_1, node);
	pCANopen->PDOwriteVal_ui16(ctrlParam.kCurrent , RX_DATA_CONTROLLER_KP_CURRENT, node);
	pCANopen->PDOwriteVal_ui16(ctrlParam.kICurrent , RX_DATA_CONTROLLER_KI_CURRENT, node);
	pCANopen->PDOwriteVal_ui16(ctrlParam.maxDrive , RX_DATA_MAX_DRIVE, node);
	pCANopen->PDOwriteVal_ui16(ctrlParam.maxCurrent , RX_DATA_MAX_CURRENT, node);
	pCANopen->PDOsend(RPDO_CONTROL_PARAMETERS_2, node);
	pCANopen->PDOwriteVal_ui8(CANopen_i8(ctrlParam.type), RX_DATA_CONTROLLER_TYPE, node);
	pCANopen->PDOwriteVal_ui16(ctrlParam.ke, RX_DATA_CONTROLLER_KE, node);
	pCANopen->PDOwriteVal_ui16(ctrlParam.ra, RX_DATA_CONTROLLER_RA, node);
	pCANopen->PDOwriteVal_i16(ctrlParam.offsetE1, RX_DATA_CONTROLLER_OFFSET_E1, node);
	pCANopen->PDOsend(RPDO_CONTROL_PARAMETERS_3, node);
	PerfTimer::sleep(CANopenInhibitTime);
}

void AxNI_base::calibrate(U8 node, const CalibrationParameter& calParam, const ControllerParameter& ctrlParam, const CollisionParameter& collParam) {
	// Calibrates an axis to its home position.
	const U32 targetInf = calParam.encOffset < calParam.encPosAfter ? calParam.encOffset + calParam.encPerCycle : calParam.encOffset - calParam.encPerCycle;
	const I32 gap = I32(calParam.encPosAfter) - I32(calParam.encOffset);
	const U32 targetFreeRange = calParam.encOffset + 2*gap;
	const U32 targetFreeRangeMid = calParam.encOffset + gap;
	const SecTmReal sleep = SecTmReal(0.2);

	hold(node);
	// Set a non dangerous Drive
	setControllerParameter(node, ctrlParam);
	// disable Collision Detection
	setCollisionDetection(node, false, collParam);
	// Set Encoder to the high end of its Range
	setEncoder(node, targetInf);
	// Move to a block and hold if speed == 0
	moveP2P(node, calParam.encOffset, calParam.speed, calParam.accel, calParam.rangeTolerance);
	PerfTimer::sleep(sleep);
	wait2stop(node);
	hold(node);
	// Set virtual 0-Point (prevent overflow of Uint32 encoder)
	setEncoder(node, calParam.encOffset);
	// Move encOffset Encoder back and check for free Range
	moveP2Pwait(node, targetFreeRange, calParam.speed, calParam.accel, calParam.rangeTolerance);
	PerfTimer::sleep(sleep);
	moveP2P(node, targetFreeRangeMid, calParam.speed, calParam.accel, calParam.rangeTolerance);
	PerfTimer::sleep(sleep);
	wait2stop(node);
	hold(node);
	PerfTimer::sleep(sleep);
	// Check for free range
	if (Math::abs(I32(getPosition(node)) - I32(targetFreeRangeMid)) > calParam.rangeTolerance)
		throw Message("AxNI_base::calibrate() [Node %d]: deviation is too big", (U32)node);
	// if Range is free move again to block and set again Virtual 0-Point
	setEncoder(node, targetInf);
	moveP2P(node, calParam.encOffset, calParam.speed, calParam.accel, calParam.stopTolerance);
	PerfTimer::sleep(sleep);
	wait2stop(node);
	hold(node);
	setEncoder(node, calParam.encOffset);
	moveP2Pwait(node, calParam.encPosAfter, calParam.speed, calParam.accel, calParam.rangeTolerance);
	hold(node);
}

//------------------------------------------------------------------------------

U32 AxNI_base::getPosition(U8 node) {
	return pCANopen->PDOreadVal_ui32(TX_DATA_POSITION, node);
}

I16 AxNI_base::getSpeed(U8 node) {
	return pCANopen->PDOreadVal_i16(TX_DATA_SPEED, node);
}

I16 AxNI_base::getDrive(U8 node) {
	return pCANopen->PDOreadVal_i16(TX_DATA_DRIVE, node);
}

I16 AxNI_base::getCurrent(U8 node) {
	return pCANopen->PDOreadVal_i16(TX_DATA_CURRENT, node);
}

I16 AxNI_base::getCurrentBufferData(U8 node) {
	return pCANopen->PDOreadVal_i16(TX_DATA_ANALYSIS_DATA, node);
}

I16 AxNI_base::getVoltage(U8 node) {
	return pCANopen->PDOreadVal_i16(TX_DATA_VOLTAGE, node);
}

std::string AxNI_base::getVersion(U8 node) {
	std::string version = pCANopen->getAxisVersionString(node);
	PerfTimer::sleep(CANopenCycleTime);
	return version;
}

//------------------------------------------------------------------------------

void AxNI_base::moveP2P(U8 node, U32 target, U16 speed, U8 accel, U8 tolerance, bool immediate) {
	pCANopen->PDOwriteVal_ui32(target, RX_DATA_P2P_TARGET_POS, node);
	pCANopen->PDOwriteVal_ui16(speed, RX_DATA_P2P_MAX_SPEED, node);
	pCANopen->PDOwriteVal_ui8(accel, RX_DATA_P2P_MAX_ACCEL, node);
	pCANopen->PDOwriteVal_ui8(tolerance, RX_DATA_P2P_TOLERANCE, node);
	pCANopen->PDOsend(RPDO_P2P_SETUP, node);

	if (!immediate) {
		PerfTimer::sleep(CANopenInhibitTime);
		setAxisFSMcommand(node, AXIS_CMD_MOVE_P2P);
		wait4state(node, AXIS_STATE_MOVING_P2P, CANopenHeartBeatPeriod);
	}
}

void AxNI_base::moveP2Pwait(U8 node, U32 target, U16 speed, U8 accel, U8 tolerance) {
	const U32 position = getPosition(node);
	const SecTmReal timeout = (Math::abs(I32(target) - I32(position)) / SecTmReal(speed)) * (1.0 / 400) * 12 + 1;
	moveP2P(node, target, speed, accel, tolerance);
	wait4state(node, AXIS_STATE_HOLDING, timeout);
}

//------------------------------------------------------------------------------

void AxNI_base::setPolyScale(U8 node, const PolynomialParameter& polyParam) {
	//This sets the parameters for the internal translation between legacy scaling parameters. 
	//These are axis specific. The robot will move off spline if set to values that do not 
	//correspond to the axe's encoder-per-cycle/gear ratio.
	//The respective setpoints will internally be calculated using pos(axis) = slope*pos(protocol) + offset
	//@param offset: the b argument in y=ax+b
	//@param slope: the a argument in y=ax+b
	//@param node the axis concerned
	pCANopen->PDOwriteVal_i32(polyParam.offset, RX_DATA_KNI_RNI_ENC_OFFSET, node);
	pCANopen->PDOwriteVal_i16(polyParam.slope, RX_DATA_KNI_RNI_ENC_SLOPE, node);
	pCANopen->PDOsend(RPDO_KNI_RNI_ENC_FACTORS, node);
	PerfTimer::sleep(CANopenInhibitTime);
}

U8 AxNI_base::getMoveBufferSize(U8 node) {
	return pCANopen->PDOreadVal_ui8(TX_DATA_MOVE_BUFFER_SIZE, node);
}

U8 AxNI_base::getMoveBufferCounter(U8 node) {
	return pCANopen->PDOreadVal_ui8(TX_DATA_MOVE_BUFFER_COUNTER, node);
}

void AxNI_base::moveBufferFlush(U8 node) {
	pCANopen->PDOwriteVal_ui8(MOVE_BUFFER_CMD_FLUSH, RX_DATA_MOVE_BUFFER_COMMAND, node);
	pCANopen->PDOsend(RX_DATA_MOVE_BUFFER_COMMAND, node);
	PerfTimer::sleep(CANopenCycleTime);
}

bool AxNI_base::moveBufferIsReady(U8 node) {
	return getMoveBufferSize(node) > 0;
}

void AxNI_base::moveBufferStore(U8 node, I32 target, U16 duration, U8 tolerance, const I16* coeffs, bool next, bool immediate) {
	U8 counter;
	if (!immediate) {
		counter= getMoveBufferCounter(node);
	}

	pCANopen->PDOwriteVal_i32(target, RX_DATA_POLY_TARGET_POS, node); // must be signed integer!!!!
	pCANopen->PDOwriteVal_ui16(duration, RX_DATA_POLY_TIME, node);
	pCANopen->PDOwriteVal_ui8(CANopen_i8(next), RX_DATA_POLY_NEXT, node);
	pCANopen->PDOwriteVal_ui8(tolerance, RX_DATA_POLY_TOLERANCE, node);
	pCANopen->PDOsend(RPDO_POLY_PARAMETERS_1, node);
	pCANopen->PDOwriteVal_i16(coeffs[0], RX_DATA_POLY_PARAM_P0, node);
	pCANopen->PDOwriteVal_i16(coeffs[1], RX_DATA_POLY_PARAM_P1, node);
	pCANopen->PDOwriteVal_i16(coeffs[2], RX_DATA_POLY_PARAM_P2, node);
	pCANopen->PDOwriteVal_i16(coeffs[3], RX_DATA_POLY_PARAM_P3, node);
	pCANopen->PDOsend(RPDO_POLY_PARAMETERS_2, node);
	//PerfTimer::sleep(CANopenInhibitTime);
	pCANopen->PDOwriteVal_ui8(MOVE_BUFFER_CMD_STORE, RX_DATA_MOVE_BUFFER_COMMAND, node);
	pCANopen->PDOsend(RPDO_MOVE_BUFFER_FSM_COMMAND, node);

	if (!immediate) {
		PerfTimer timer;
		const SecTmReal t = timer.elapsed() + AxNI_base::CANopenHeartBeatPeriod;
		while (t > timer.elapsed()) {
			if (getMoveBufferCounter(node)%255 == counter + 1)
				return;
			PerfTimer::sleep(CANopenInhibitTime);
		}
		throw Message("AxNI_base::moveBufferStore() [Node %d]: move buffer error", (U32)node);
	}
}

//------------------------------------------------------------------------------
