/** @file Katana300.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Device/Katana300/Katana300.h>
#include <Golem/Device/Katana300/Data.h>
#include <Golem/Ctrl/LoadArmDesc.h>
#include <Golem/Device/Katana300/cdlCOM.h>
#include <Golem/Device/Katana300/cplSerial.h>
#include <Golem/Device/Katana300/kmlBase.h>
#include <Golem/Device/Katana300/kmlExt.h>

//------------------------------------------------------------------------------

/** Controller messages */
#define GOLEM_DEVICE_KATANA_VERBOSE_

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadArmDesc(void* pContext, const std::string& path, void* pArmDesc) {
	loadArmDesc<golem::Katana300SerialArm::Desc>((golem::Context*)pContext, path, (Arm::Desc::Ptr*)pArmDesc);
}

//------------------------------------------------------------------------------

/** Big-Endian conversion */
static inline U16 BigEndian(U16 x) {
	return (x >> 8) | (x << 8);
}

//------------------------------------------------------------------------------

const Real Katana300Joint::MCU_TM_PERIOD = (Real)1.638e-3;
const Real Katana300Joint::VEL_ENC_UNIT = 6.0;

Katana300Joint::Katana300Joint(Arm& arm) : KatanaJoint(&arm), BufCtrlJoint(arm), pKatana(NULL), pProtocol(NULL), pMotBase(NULL) {
}

/** Creates Katana Joint */
bool Katana300Joint::create(const Desc& desc) {
	KatanaJoint::create(desc); // throws
	BufCtrlJoint::create(desc); // throws

	pKatana300Arm = dynamic_cast<Katana300Arm*>(&arm);
	if (pKatana300Arm == NULL)
		throw MsgKatana300JointInvalidArmType(Message::LEVEL_CRIT, "Katana300Joint::create() Invalid arm type");

	pKatana = pKatana300Arm->pKatana;
	pProtocol = pKatana300Arm->pProtocol;
	pMotBase = &pKatana->GetBase()->GetMOT()->arr[getIndex()];

	for (U32 i = 0; i < NUM_FILTER_COEFFS; i++)
		coeffs[i] = desc.coeffs[i];
	if (desc.activateFIRFilter)
		(void)setFIRFilter(coeffs);

	limit = desc.limit;
	if (desc.activateSlopeSaturation)
		(void)setSlopeSaturation(limit);

	// overwrite min, max
	min.pos = this->getKNIMinPos();
	max.pos = this->getKNIMaxPos();

	return true;
}

//------------------------------------------------------------------------------

bool Katana300Joint::sysSync() {
	ASSERT(pProtocol != NULL)

	try {
		sendBuf[0] = 'I';	// new firmware required!
		sendBuf[1] = (U8)(getIndex() + 1);
		
		U8 size = 0;

		pProtocol->comm(sendBuf, recvBuf, &size);
	} catch (std::exception& e) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Katana300Joint::sysSync() [joint %d]: IO error (%s)", getIndex(), e.what());
		return false;
	}
	
	return recvBuf[2] != IO_SYNC_FLAG;
}

//------------------------------------------------------------------------------
// IJointBase

bool Katana300Joint::sysRecv(GenCoord& curr) {
	ASSERT(pMotBase != NULL)

	try {
		pMotBase->recvPVP();
	} catch (std::exception& e) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Katana300Joint::sysRecv() [joint %d]: IO error (%s)", getIndex(), e.what());
		return false;
	}

	curr.pos = posFromEnc(pMotBase->GetPVP()->pos);
	curr.vel = REAL_ZERO; // velFromEnc(pMotBase->GetPVP()->vel);
	curr.acc = REAL_ZERO; // not used

	return true;
}

//------------------------------------------------------------------------------

bool Katana300Joint::setFIRFilter(const U8 *coeffs) {
	for (U32 i = 0; i < NUM_FILTER_COEFFS; i++)
		this->coeffs[i] = coeffs[i];

	try {
		sendBuf[0] = 'S';	// new firmware required!
		sendBuf[1] = (U8)(getIndex() + 1);
		sendBuf[2] = 8;
		sendBuf[3] = coeffs[0];
		sendBuf[4] = coeffs[1];
		sendBuf[5] = coeffs[2];

		U8 size = 0;

		pProtocol->comm(sendBuf, recvBuf, &size);

		sendBuf[0] = 'S';	// new firmware required!
		sendBuf[1] = (U8)(getIndex() + 1);
		sendBuf[2] = 9;
		sendBuf[3] = coeffs[3];
		sendBuf[4] = coeffs[4];
		sendBuf[5] = 0;

		size = 0;

		pProtocol->comm(sendBuf, recvBuf, &size);
	} catch (std::exception& e) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Katana300Joint::setFIRFilter() [joint %d]: IO error (%s)", getIndex(), e.what());
		return false;
	}

	return true;
}

const U8 *Katana300Joint::getFIRFilter() const {
	return coeffs;
}

bool Katana300Joint::setSlopeSaturation(U8 limit) {
	ASSERT(pProtocol != NULL)

	try {
		sendBuf[0] = 'S';	// new firmware required!
		sendBuf[1] = (U8)(getIndex() + 1);
		sendBuf[2] = 10;
		sendBuf[3] = limit;
		sendBuf[4] = 0;
		sendBuf[5] = 0;

		U8 size = 0;
		
		pProtocol->comm(sendBuf, recvBuf, &size);
	} catch (std::exception& e) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Katana300Joint::setSlopeSaturation() [joint %d]: IO error (%s)", getIndex(), e.what()	);
		return false;
	}

	return true;
}

U8 Katana300Joint::getSlopeSaturation() const {
	return limit;
}

//------------------------------------------------------------------------------

Katana300StdJoint::Katana300StdJoint(Arm& arm) : Katana300Joint(arm) {
}

bool Katana300StdJoint::sysSend(const GenCoord& prev, const GenCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt) {
	ASSERT(pProtocol != NULL)

	short* buf;

	try {
		sendBuf[0] = 'C';
		sendBuf[1] = (U8)(getIndex() + 1);
		sendBuf[2] = MCF_ON;
		
		buf = (short*)&sendBuf[3*sizeof(U8)];
		// The target point
		buf[0] = BigEndian((short)posToEnc(next.pos));

		U8 size = 0;
		
		pProtocol->comm(sendBuf, recvBuf, &size);
	} catch (std::exception& e) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Katana300StdJoint::sysSend() [joint %d]: IO error (%s)", getIndex(), e.what()	);
		return false;
	}

	return true;
}

//------------------------------------------------------------------------------

const Real Katana300TrjJoint::trj4ScaleFac[4] = {
	(Real)(pow(TRJ_TIME_QUANT, 0) * (1 << 0)),
	(Real)(pow(TRJ_TIME_QUANT, 1) * (1 << 6)),
	(Real)(pow(TRJ_TIME_QUANT, 2) * (1 << 10)),
	(Real)(pow(TRJ_TIME_QUANT, 3) * (1 << 15)),
};

Katana300TrjJoint::Katana300TrjJoint(Arm& arm) : Katana300Joint(arm) {
}

bool Katana300TrjJoint::sysSend(const GenCoord& prev, const GenCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt) {
	ASSERT(pProtocol != NULL)

	short* buf;
	
	GenCoord x0(posToEnc(prev.pos), velToEnc(prev.vel), REAL_ZERO);
	GenCoord x1(posToEnc(next.pos), velToEnc(next.vel), REAL_ZERO);
	
	// set new trajectory (dt > 0)
	GenCoordTrj trj(REAL_ZERO, Real(dt), x0, x1);
	
	sendBuf[0] = 'G';	// new firmware required!
	sendBuf[1] = (U8)(getIndex() + 1);
	
	buf = (short*)&sendBuf[2*sizeof(U8)];
	// The trajectory end-point (the last position of a single movement)
	buf[0] = BigEndian((short)Math::round(x1.pos));
	// Time duration of a single window, rounding
	buf[1] = BigEndian((short)Math::round(SecTmReal(dt)/TRJ_TIME_QUANT));
	
	// Rescale coefficients and update send buffer
	buf = (short*)&sendBuf[2*sizeof(U8) + 2*sizeof(short)];
	for (int i = 0; i < 4; i++)
		buf[i] = BigEndian((short)Math::clamp(Math::round(trj4ScaleFac[i] * trj.getCoeffs()[i]), (Real)numeric_const<I16>::MIN, (Real)numeric_const<I16>::MAX));

	U8 size = 0;
	try {
		pProtocol->comm(sendBuf, recvBuf, &size);
	} catch (std::exception& e) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Katana300TrjJoint::sysSend() [joint %d]: IO error (%s)", getIndex(), e.what()	);
		return false;
	}

	return true;
}

//------------------------------------------------------------------------------

Katana300Arm::Katana300Arm(golem::Context& context) : KatanaArm(&context), KatanaGripper(&context), BufCtrlArm(context)  {
	pKatana = NULL;
	pProtocol = NULL;
	bCalibrationInit = false;
	gripperIsPresent = false;
}

Katana300Arm::~Katana300Arm() {
}

bool Katana300Arm::create(CKatana* pKatana, CCplBase* pProtocol, const Desc& desc) {
	if (pKatana == NULL || pProtocol == NULL)
		throw MsgKatana300ArmNullKNIPointers(Message::LEVEL_CRIT, "Katana300Arm::create(): Null pointers to CKatana and CCplBase");

	this->pKatana = pKatana;
	this->pProtocol = pProtocol;
	
	KatanaArm::create(desc); // throws
	KatanaGripper::create(desc); // throws

	// all joints must inherit Katana300Joint
	for (Joint::Seq::const_iterator i = joints.begin(); i < joints.end(); ++i) {
		if (dynamic_cast<const Katana300Joint*>(*i) == NULL)
			throw MsgKatana300ArmInvalidJointType(Message::LEVEL_CRIT, "Katana300Arm::create(): invalid Joint type");
	}

	bMeasurements = desc.bMeasurements;
	syncJointIndex = desc.syncJointIndex;

	if (bGripper) {
		pKatana->getGripperParameters(gripperIsPresent, gripperEncoderData.open, gripperEncoderData.closed);
		
		if (gripperIsPresent) {
			gripperDur = SEC_TM_REAL_ZERO;
			evGripper.set(false);
			bGripperRecvResult = false;

			evGripperRecv.set(false);
			gripperStatus = OPEN;
			bGripperResult = false;
		}
	}

	katDeltaOffs = desc.katDeltaOffs;

	BufCtrlArm::create(desc); // throws

	return true;
}

//------------------------------------------------------------------------------
// Controller

bool Katana300Arm::sysRecv(GenConfigspaceCoord& curr) {
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

bool Katana300Arm::sysSend(const GenConfigspaceCoord& prev, const GenConfigspaceCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt) {
	ASSERT(pProtocol != NULL)

	bool bRet = true;
	
	for (U32 i = 0; i < joints.size(); i++) {
#ifdef GOLEM_DEVICE_KATANA_VERBOSE_
		const SecTmReal t0 = context.getTimer().elapsed();
#endif
		Katana300Joint* pKatana300Joint = dynamic_cast<Katana300Joint*>(joints[i]);
		ASSERT(pKatana300Joint)
		if (!pKatana300Joint->sysSend(prev.get(i), next.get(i), bSendPrev, bSendNext, dt))
			bRet = false;

#ifdef GOLEM_DEVICE_KATANA_VERBOSE_
		const SecTmReal t1 = context.getTimer().elapsed();
		if (isCalibrating()) {
			msrDeltaJointSend.update(t1 - t0);
		}
		if (isCalibrated()) {
			if ((t1 - t0) > msrDeltaJointSend.getAvr() + katDeltaOffs)
				context.getMessageStream()->write(Message::LEVEL_WARNING, "Katana300Arm::sysSend(): [joint %d]: timeout = %f", i, (t1 - t0));
		}
#endif
	}

#ifdef GOLEM_DEVICE_KATANA_VERBOSE_
	const SecTmReal t2 = context.getTimer().elapsed();
#endif
	
	sendBuf[0] = (U8)128 + 'G';	// new firmware required!
	sendBuf[1] = 1;
	sendBuf[2] = 0; // = 1 for exact movement
	
	U8 size = 0;
	try {
		pProtocol->comm(sendBuf, recvBuf, &size);
	} catch (std::exception& e) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Katana300Arm::send(): IO error (%s)", e.what());
		bRet = false;
	}

#ifdef GOLEM_DEVICE_KATANA_VERBOSE_
	const SecTmReal t3 = context.getTimer().elapsed();
	if (isCalibrating()) {
		msrDeltaSendSync.update(t3 - t2);
	}
	if (isCalibrated()) {
		if ((t3 - t2) > msrDeltaSendSync.getAvr() + katDeltaOffs)
			context.getMessageStream()->write(Message::LEVEL_WARNING, "Katana300Arm::sysSend(): sync timeout = %f", (t3 - t2));
	}
#endif
	
	return bRet;
}

bool Katana300Arm::sysSync() {
	// assuming that all trajectories are of the same duration, its sufficient
	// to wait for a 'sync flag' from a single motor
	bool bRet = true;
	for (std::set<U32>::const_iterator i = syncJointIndex.begin(); i != syncJointIndex.end(); i++) {
		Katana300Joint* pKatana300Joint = dynamic_cast<Katana300Joint*>(joints[*i]);
		ASSERT(pKatana300Joint)
		bRet = bRet && pKatana300Joint->sysSync(); // detects false positive only
	}
	return bRet;
}

//------------------------------------------------------------------------------

void Katana300Arm::forwardTransform(Mat34& trn, const ConfigspaceCoord& cc) {
	if (customKinematics)
		KatanaArm::forwardTransform(trn, cc);
	else
		Arm::forwardTransform(trn, cc);
}

void Katana300Arm::velocitySpatial(Twist& v, const ConfigspaceCoord& cc, const ConfigspaceCoord& dcc) {
	if (customKinematics)
		KatanaArm::velocitySpatial(v, cc, dcc);
	else
		Arm::velocitySpatial(v, cc, dcc);
}

void Katana300Arm::jacobianSpatial(Jacobian& jac, const ConfigspaceCoord& cc) {
	if (customKinematics)
		KatanaArm::jacobianSpatial(jac, cc);
	else
		Arm::jacobianSpatial(jac, cc);
}


//------------------------------------------------------------------------------

void Katana300Arm::userCalibrate() {
	if (/*bGripper && */gripperIsPresent)
		gripperCalibrate();

	msrDeltaSendSync.reset();
	msrDeltaJointSend.reset();
}

void Katana300Arm::userComm() {
	if (/*bGripper && */gripperIsPresent)
		gripperComm();
}

//------------------------------------------------------------------------------

void Katana300Arm::gripperCalibrate() {
}

void Katana300Arm::gripperComm() {
	SecTmReal t0, t1;

	t0 = context.getTimer().elapsed();
	
	{
		CriticalSectionWrapper csw(csGripper);

		try {
			CMotBase &base = pKatana->GetBase()->GetMOT()->arr[pKatana->getNumberOfMotors() - 1];

			if (gripperStatus & (OPEN | CLOSE | FREEZE) && !(gripperStatus & READ)) {
				if (gripperStatus & FREEZE) {
					base.recvPVP();
					gripperEncoderData.current = base.GetPVP()->pos;
				}

				TMotTPS tps;
				tps.mcfTPS = gripperStatus & FREEZE ? MCF_FREEZE : MCF_ON;
				tps.tarpos = gripperStatus & OPEN ? gripperEncoderData.open :
					gripperStatus & CLOSE ? gripperEncoderData.closed : gripperEncoderData.current;

				base.sendTPS(&tps);

				if (gripperStatus & FREEZE) {
					gripperStatus &= ~(OPEN | CLOSE | FREEZE);
					bGripperResult = true;
					evGripper.set(true);
				}
				else
					gripperStatus |= READ;
			} else if (gripperStatus & (RECV | READ)) {
				if (gripperStatus & OPEN) {
					base.recvPVP();
					gripperEncoderData.current = base.GetPVP()->pos;

					if (Math::abs(I32(gripperEncoderData.open - gripperEncoderData.current)) < GRIP_ENC_TOLERANCE) {
						gripperStatus &= ~(OPEN | READ);
						bGripperResult = true;
						evGripper.set(true);
					}
				}
				else {
					CSctBase &sctBase = pKatana->GetBase()->GetSCT()->arr[0];
					const TSctDAT* pSctDAT	= sctBase.GetDAT();

					sctBase.recvDAT();

					size_t items = 0;
					for (SensorIndexSet::const_iterator i = sensorIndexSet.begin(); i != sensorIndexSet.end(); i++)
						if (*i < pSctDAT->cnt) {
							if (sensorData.size() < ++items)
								sensorData.resize(items);
							sensorData[items - 1] = SensorData(*i, pSctDAT->arr[*i]);
						}

					sensorData.resize(items);
					
					gripperStatus &= ~RECV;
					bGripperRecvResult = true;
					evGripperRecv.set(true);

					if (gripperStatus & CLOSE) {
						// TODO more advanced check
						for (SensorDataSet::const_iterator i = sensorThreshold.begin(); i != sensorThreshold.end(); i++)
							if (i->index < pSctDAT->cnt && pSctDAT->arr[i->index] > i->value) {
								gripperStatus &= ~(CLOSE | READ);
								gripperStatus |= FREEZE;
								break;
							}
					}
				}
			}
		} catch (std::exception& e) {
			//gripperStatus = NOOP;
			//evGripper.set(true);
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Katana300Arm::gripperComm(): IO error (%s)", e.what()	);
		}
	}

	t1 = context.getTimer().elapsed();
	
	if (isCalibrating()) {
		gripperDur = std::max(gripperDur, t1 - t0);
		PerfTimer::sleep(std::max(gripperDur - (t1 - t0), SEC_TM_REAL_ZERO));
	}
}

//------------------------------------------------------------------------------

bool Katana300Arm::gripperRecvSensorData(SensorDataSet& sensorData, MSecTmU32 timeWait) {
	if (!gripperIsPresent || isCalibrating())
		return false;

	{
		CriticalSectionWrapper csw(csGripper);
		evGripperRecv.set(false);
		gripperStatus |= RECV;
		bGripperRecvResult = false;
	}
	
	if (timeWait <= 0 || evGripperRecv.wait(timeWait) && bGripperRecvResult)
	{
		CriticalSectionWrapper csw(csGripper);
		sensorData = this->sensorData;
		return true;
	}

	return false;
}

bool Katana300Arm::gripperRecvEncoderData(GripperEncoderData& encoderData, MSecTmU32 timeWait) {
	if (!gripperIsPresent || isCalibrating())
		return false;

	{
		CriticalSectionWrapper csw(csGripper);
		encoderData = gripperEncoderData;
	}
	
	return true;
}

bool Katana300Arm::gripperOpen(MSecTmU32 timeWait) {
	if (!gripperIsPresent || isCalibrating())
		return false;

	{
		CriticalSectionWrapper csw(csGripper);
		evGripper.set(false);
		gripperStatus &= ~(OPEN | CLOSE | FREEZE | READ);
		gripperStatus |= OPEN;
		bGripperResult = false;
	}

	return timeWait <= 0 || evGripper.wait(timeWait) && bGripperResult;
}

bool Katana300Arm::gripperClose(const SensorDataSet& sensorThreshold, MSecTmU32 timeWait) {
	if (!gripperIsPresent || isCalibrating())
		return false;

	{
		CriticalSectionWrapper csw(csGripper);
		evGripper.set(false);
		gripperStatus &= ~(OPEN | CLOSE | FREEZE | READ);
		gripperStatus |= CLOSE;
		bGripperResult = false;
		this->sensorThreshold = sensorThreshold;
	}

	return timeWait <= 0 || evGripper.wait(timeWait) && bGripperResult;
}

bool Katana300Arm::gripperFreeze(MSecTmU32 timeWait) {
	if (!gripperIsPresent || isCalibrating())
		return false;

	{
		CriticalSectionWrapper csw(csGripper);
		evGripper.set(false);
		gripperStatus &= ~(OPEN | CLOSE | FREEZE | READ);
		gripperStatus |= FREEZE;
		bGripperResult = false;
	}

	return timeWait <= 0 || evGripper.wait(timeWait) && bGripperResult;
}

//------------------------------------------------------------------------------

Katana300SerialArm::Katana300SerialArm(golem::Context& context) : Katana300Arm(context), katana(NULL), protocol(NULL), device(NULL) {
}

Katana300SerialArm::~Katana300SerialArm() {
	// stop control thread before releasing KNI stuff
	BufCtrlArm::release();

	if (katana != NULL)
		delete katana;
	if (protocol != NULL)
		delete protocol;
	if (device != NULL)
		delete device;
}

bool Katana300SerialArm::create(const Desc& desc) {
	cfgPath = desc.cfgPath;
	serialDesc = desc.serialDesc;

	TCdlCOMDesc ccd = {
		(int)serialDesc.commPort,
		(int)serialDesc.baudRate,
		(int)serialDesc.dataBit,
		(int)serialDesc.parityBit,
		(int)serialDesc.stopBit,
		(int)serialDesc.readTimeout,
		(int)serialDesc.writeTimeout
	};
	
	try {
		device = new CCdlCOM(ccd);
	} catch (std::exception& ex) {
		throw MsgKatana300ArmCommPort(Message::LEVEL_CRIT, "Katana300SerialArm::create(): Unable to open communication port: %s", ex.what());
	}

	try {
		protocol = new CCplSerialCRC();
		protocol->init(device);
	} catch (std::exception& ex) {
		throw MsgKatana300ArmProtocolInit(Message::LEVEL_CRIT, "Katana300SerialArm::create(): Unable to initialise protocol: %s", ex.what());
	}
	
	try {
		katana = new CKatana();
		katana->create(cfgPath.c_str(), protocol);
	} catch (std::exception& ex) {
		throw MsgKatana300ArmCKatanaCreate(Message::LEVEL_CRIT, "Katana300SerialArm::create(): Unable to create CKatana: %s", ex.what());
	}

	try {
		katana->calibrate();
	} catch (std::exception& ex) {
		throw MsgKatana300ArmKNICalibration(Message::LEVEL_CRIT, "Katana300SerialArm::create(): KNI calibration failure: %s", ex.what());
	}

	Katana300Arm::create(katana, protocol, desc); // throws

	return true;
}

//------------------------------------------------------------------------------
