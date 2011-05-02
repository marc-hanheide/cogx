/** @file BufCtrl.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Device/BufCtrl/BufCtrl.h>

//------------------------------------------------------------------------------

/** Controller debug */
//#define GOLEM_DEVICE_BUFCTRL_BUFCTRL_DBG_

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

BufCtrlJoint::BufCtrlJoint(Arm& arm) : Joint(arm) {
}

//------------------------------------------------------------------------------

BufCtrlArm::BufCtrlArm(golem::Context& context) : Arm(context) {
	bTerminate = true;
	bStart = false;
	bCalibrating = false;
	bCalibrated = false;
	bSyncRequest = bCycleRequest = false;
}

BufCtrlArm::~BufCtrlArm() {
	release();
}

//------------------------------------------------------------------------------

void BufCtrlArm::initBuffers(const GenConfigspaceCoord& state, SecTmReal t, SecTmReal dt) {
	// Init state queue, 2 elements
	qState.clear();
	qState.push_back(GenConfigspaceState(state, t));	// previous
	qState.push_back(GenConfigspaceState(state, t + dt));	// next

	// Init sent queue, 2 elements
	qSent.clear();
	qSent.push_back(GenConfigspaceState(state, t));	// previous
	qSent.push_back(GenConfigspaceState(state, t + dt));	// next

	// Init control queue, 1 element
	qCommand.clear();
	qCommand.push_back(GenConfigspaceState(state, t + dt));	// awaiting
}

bool BufCtrlArm::create(const Desc& desc) {
	Arm::create(desc); // throws

	// all joints must inherit BufCtrlJoint
	for (Joint::Seq::const_iterator i = joints.begin(); i < joints.end(); ++i) {
		if (dynamic_cast<const BufCtrlJoint*>(*i) == NULL)
			throw MsgBufCtrlArmInvalidJointType(Message::LEVEL_CRIT, "BufCtrlArm::create(): invalid Joint type");
	}

	if (bStart) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "BufCtrlArm::create(): Controller has already been created");
		return true;
	}

	threadTimeOut = desc.threadTimeOut;

	evReady.set(true);

	qState.reserve(desc.qStateSize);
	qSent.reserve(desc.qSentSize);
	qCommand.reserve(desc.qCommandSize);
	
	bTerminate = false;
	if (!thread.start(this))
		throw MsgBufCtrlArmThreadLaunch(Message::LEVEL_CRIT, "BufCtrlArm::create(): Unable to launch Controller thread");

	if (!thread.setPriority(desc.threadPriority))
		context.getMessageStream()->write(Message::LEVEL_ERROR, "BufCtrlArm::create(): Unable to change Controller thread priority");
	
	bStart = true;

	if (!calibrate(desc))
		throw MsgBufCtrlArmCalibration(Message::LEVEL_CRIT, "BufCtrlArm::create(): Calibration failure");
	
	return true;
}


void BufCtrlArm::release() {
	bTerminate = true;
	if (!thread.join(threadTimeOut))
		context.getMessageStream()->write(Message::LEVEL_ERROR, "BufCtrlArm::release(): Unable to stop working thread");
}

bool BufCtrlArm::calibrate(const Desc& desc) {
	// Enter calibration mode
	bCalibrating = true;
	bCalibrated = false;
	
	// user calibration
	userCalibrate();
	
	// Request synchronisation and wait for confirmation
	// The main control loop in the working thread become blocked
	bSyncRequest = true;
	if (!evSyncLock.wait(threadTimeOut)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "BufCtrlArm::calibrate(): evSyncLock timeout (1)");
		return false;
	}
	
	// Receive the current output state
	GenConfigspaceCoord curr;
	if (!sysRecv(curr)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "BufCtrlArm::calibrate(): receive error");
		return false;
	}
	
	// Reset all measurements
	msrDeltaSync.reset();
	msrDeltaRecv.reset();
	msrDeltaProc.reset();
	msrDeltaSend.reset();

	timeQuant = desc.timeQuant;
	deltaOffs = desc.deltaOffs;
	skewOffs = desc.skewOffs;
	tmDelta = desc.cycleDur;
	initBuffers(curr, context.getTimer().elapsed() - tmDelta, tmDelta);

	GenConfigspaceState next;
	(GenConfigspaceCoord&)next = curr;	// requires a suitable copying operator
	next.t = context.getTimer().elapsed() + (SYS_BUF_LEN + 1)*desc.cycleDur;
	// GenConfigspaceState series of data without changing the system state:
	for (U32 i = 0; i <= desc.cycleNum; i++, next.t += desc.cycleDur)
		qCommand.push_back(next);
	
	// Unlock the control loop
	evSyncUnlock.set(true);
	
	// Wait until the system has reached the target state
	if (!cycleWait(next.t, (MSecTmU32)(1000.0*desc.cycleDur*(desc.cycleNum+2)) + threadTimeOut)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "BufCtrlArm::calibrate(): cycleWait timeout");
		return false;
	}
	
	// Lock the control loop
	bSyncRequest = true;
	if (!evSyncLock.wait(threadTimeOut)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "BufCtrlArm::calibrate(): evSyncLock timeout (2)");
		return false;
	}
	
	// The minimal trajectory duration time is a multiple of timeQuant
	tmDelta = Math::ceil(msrDeltaSync.getAvr() + msrDeltaRecv.getAvr() + msrDeltaProc.getAvr() + 
		msrDeltaSend.getAvr() + deltaOffs, timeQuant);
	
	// Init all buffers
	initBuffers(curr, context.getTimer().elapsed() - tmDelta, tmDelta);
	
	context.getMessageStream()->write(Message::LEVEL_DEBUG,
		"BufCtrlArm::calibrate(): delta_offs=%f, skew_offs=%f, cycle=%f, sync=%f, recv=%f, proc=%f, send=%f",
		deltaOffs, skewOffs, tmDelta, msrDeltaSync.getAvr(), msrDeltaRecv.getAvr(), msrDeltaProc.getAvr(), msrDeltaSend.getAvr()
	);

	// Leave calibration mode
	bCalibrating = false;
	bCalibrated = true;
	
	// Unlock the control loop
	evSyncUnlock.set(true);

	return true;
}

bool BufCtrlArm::isCalibrating() const {
	return bCalibrating;
}

bool BufCtrlArm::isCalibrated() const {
	return bCalibrated;
}

bool BufCtrlArm::cycleWait(SecTmReal t, MSecTmU32 timeWait) {
	tmCycle = t;
	bCycleRequest = true;
	evCycle.set(false);
	
	if (!evCycle.wait(timeWait)) {
		//context.getMessageStream()->write(Message::LEVEL_NOTICE, "BufCtrlArm::cycleWait(): evCycle timeout");
		return false;
	}
	
	bCycleRequest = false;
	return true;
}

//------------------------------------------------------------------------------
// Arm (protected members)
// Basic implementation only collects/dispatches data from/to joints

bool BufCtrlArm::sysRecv(GenConfigspaceCoord& state) {
	bool bRet = true;
	
	for (U32 i = 0; i < joints.size(); i++) {
		GenCoord c;
		if (!dynamic_cast<BufCtrlJoint*>(joints[i])->sysRecv(c))
			bRet = false;
		state.set(i, c);
	}
	
	return bRet;
}

bool BufCtrlArm::sysSend(const GenConfigspaceCoord& prev, const GenConfigspaceCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt) {
	bool bRet = true;
	
	for (U32 i = 0; i < joints.size(); i++) {
		if (!dynamic_cast<BufCtrlJoint*>(joints[i])->sysSend(prev.get(i), next.get(i), bSendPrev, bSendNext, dt))
			bRet = false;
	}

	return bRet;
}

// default implementation does not do anything
void BufCtrlArm::userCalibrate() {
}

// default implementation does not do anything
void BufCtrlArm::userComm() {
}

//------------------------------------------------------------------------------
// Main control interface

// Default implementation of the function returns measured states
bool BufCtrlArm::recv(GenConfigspaceState& curr, MSecTmU32 timeWait) {
	if (timeWait > 0 && !cycleWait(SEC_TM_REAL_MIN, timeWait)) {
		context.getMessageStream()->write(Message::LEVEL_WARNING, "BufCtrlArm::recv(): timeout");
		return false;
	}

	CriticalSectionWrapper csw(csState);	// enter critical section
	curr = qState.back(); // fetch the latest reading
	return true;
}

bool BufCtrlArm::send(const GenConfigspaceState& command, MSecTmU32 timeWait) {
	if (!evReady.wait(timeWait)) {
		context.getMessageStream()->write(Message::LEVEL_WARNING, "BufCtrlArm::send(): timeout");
		return false;
	}

	CriticalSectionWrapper csw(csCommand);	// enter critical section

	Queue::iterator first = qCommand.begin() + 1;
	if (first != qCommand.end()) {
		// FIND last SUCH THAT last->t <= command.t AND (last+1)->t > command.t OR last+1 == qCommand.end())
		Queue::iterator last = std::upper_bound(first, qCommand.end(), command.t);
		if (first != last)
			--last;
		
		// If the consecutive commands are too close, drop the one before last as well
		// i.e. do not increment last
		if (last->t + tmDelta - skewOffs < command.t)
			++last;

		// If 'command' refers to a moment in time before the last queued element,
		// re-plan the sequence of states (reactive behaviour) - clear up the queue
		// but always leave the first element (qCommandCurrentPtr)
		qCommand.erase(last, qCommand.end());
	}

	qCommand.push_back(command);
	if (qCommand.full())
		evReady.set(false);

	return true;
}

SecTmReal BufCtrlArm::getTimeQuant() const {
	return timeQuant;
}

SecTmReal BufCtrlArm::getTimeDelta() const {
	return tmDelta + skewOffs;
}

SecTmReal BufCtrlArm::getTimeDeltaAsync() const {
	return (SYS_BUF_LEN + 1)*BufCtrlArm::getTimeDelta();
}

//------------------------------------------------------------------------------
// Runnable

void BufCtrlArm::run() {
	bool bSendCurr = false;
	SecTmReal tCurr = SEC_TM_REAL_ZERO, dtCurr = SEC_TM_REAL_ZERO, tPred = SEC_TM_REAL_ZERO;
	
	while (!bTerminate) {
		const SecTmReal t0 = context.getTimer().elapsed();

		// Wait for buffer ready to accept new data
		if (!sysSync())
			continue;
		
		const SecTmReal t1 = context.getTimer().elapsed();
		
		// Receive data
		GenConfigspaceState state;
		(void)sysRecv((GenConfigspaceCoord&)state);
		// User IO function
		userComm();

		const SecTmReal t2 = context.getTimer().elapsed();
		
		{
			CriticalSectionWrapper csw(csState);	// enter critical section
			// Process received data
			state.t = t1; 	// approx
			qState.push_back(state);
		}
		
		GenConfigspaceState prev, next;
		bool bSendPrev, bSendNext;

		{
			CriticalSectionWrapper csw(csCommand);	// enter critical section
			// Variables controlling send queue
			const SecTmReal tPrev = tCurr;
			tCurr = t0;
			bSendPrev = bSendCurr;
			bSendCurr = qCommand.size() > 1; // check if there is data to send
			bSendNext = qCommand.size() > 2;
			
			// correction <- prediction (prev cycle)
			if (bSendPrev) {
				qSent.back().t = tCurr + dtCurr;
			}
			
			// Process data to send
			if (bSendCurr) {
				prev = *qCommand.begin();
				next = *(qCommand.begin() + 1);
				
				const SecTmReal dtPrev = dtCurr;
				SecTmReal dtSkew;

				if (bSendPrev) {
					dtSkew = prev.t - (tCurr + dtCurr);
					tPred = tCurr + dtPrev;
				}
				else {
					dtSkew = std::min(SEC_TM_REAL_ZERO, (next.t - tmDelta) - std::max(tPred + dtPrev, tCurr + tmDelta));
					tPred = std::max(SEC_TM_REAL_ZERO, tPred + dtPrev - (tCurr + tmDelta)) + t2 + msrDeltaRecv.getAvr() + msrDeltaSend.getAvr();
					// Report a possible timer skew (Controller does not allow for sending so short trajectories)
					if (isCalibrated() && dtSkew + tmDelta/SecTmReal(2.0) < SEC_TM_REAL_ZERO) {
						context.getMessageStream()->write(Message::LEVEL_WARNING, "BufCtrlArm::run(): timer skew detected = %f", dtSkew);
					}
				}
				
				dtCurr = Math::round(std::max(next.t - tPred, tmDelta), timeQuant);
				
#ifdef GOLEM_DEVICE_BUFCTRL_BUFCTRL_DBG_
				context.getMessageStream()->write(Message::LEVEL_DEBUG,
					"SENDING t=%f, dt=%f, tPred=%f, tCurr=%f, dtCurr=%f, dtSkew=%f",
					next.t, next.t - prev.t, tPred, tCurr, dtCurr, dtSkew
				);
#endif

				// prediction -> correction (next cycle)
				next.t = tPred + dtCurr;

				qSent.push_back(next);
				qCommand.pop_front();
				qCommand.front().t = next.t; // qCommand.front() == qSent.back()

				evReady.set(true);	// after removing the front item, the command queue is ready for new commands
			}
		
		}// leave critical section
		
		evCycle.set(bCycleRequest && tmCycle < t0);
		
		const SecTmReal t3 = context.getTimer().elapsed();
		
		// Send data
		if (bSendCurr)
			(void)sysSend(prev, next, bSendPrev, bSendNext, dtCurr);
		
		const SecTmReal t4 = context.getTimer().elapsed();
		
		if (isCalibrating() && bSendCurr) {
			msrDeltaSync.update(t1 - t0);
			msrDeltaRecv.update(t2 - t1);
			msrDeltaProc.update(t3 - t2);
			msrDeltaSend.update(t4 - t3);
		}

		if (isCalibrated()) {
			//if ((t1 - t0) > msrDeltaSync.getAvr() + deltaOffs) {
			//	context.getMessageStream()->write(Message::LEVEL_WARNING, "BufCtrlArm::run(): sysSync() timeout = %f", (t1 - t0)	);
			//}
			if ((t2 - t1) > msrDeltaRecv.getAvr() + deltaOffs) {
				context.getMessageStream()->write(Message::LEVEL_WARNING, "BufCtrlArm::run(): sysRecv()/userComm() timeout = %f", (t2 - t1));
			}
			if ((t3 - t2) > msrDeltaProc.getAvr() + deltaOffs) {
				context.getMessageStream()->write(Message::LEVEL_WARNING, "BufCtrlArm::run(): critical section timeout = %f", (t3 - t2));
			}
			if ((t4 - t3) > msrDeltaSend.getAvr() + deltaOffs) {
				context.getMessageStream()->write(Message::LEVEL_WARNING, "BufCtrlArm::run(): sysSend() timeout = %f", (t4 - t3));
			}
		}
		
		if (bSyncRequest) {
			bSyncRequest = false;
			evSyncLock.set(true);
			if (!evSyncUnlock.wait(threadTimeOut)) {
				context.getMessageStream()->write(Message::LEVEL_ERROR, "BufCtrlArm::run(): evSyncUnlock timeout");
				//bTerminate = true;
			}
			evSyncUnlock.set(false);
		}
	}
	
	// Wake up waiting threads
	evReady.set(true);
	evCycle.set(true);
}

//------------------------------------------------------------------------------
