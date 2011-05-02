/** @file BufCtrl.h
 * 
 * Interface for controlling open chain manipulators using trajectories.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEVICE_BUFCTRL_BUFCTRL_H_
#define _GOLEM_DEVICE_BUFCTRL_BUFCTRL_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Arm.h>
#include <Golem/Tools/Library.h>
#include <Golem/Ctrl/Msg.h>
#include <Golem/Math/Function.h>
#include <vector>
//#include <Golem/Tools/Debug.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR MsgBufCtrlJoint : public MsgJoint {};
class GOLEM_LIBRARY_DECLDIR MsgBufCtrlArm : public MsgArm {};
class GOLEM_LIBRARY_DECLDIR MsgBufCtrlArmThreadLaunch : public MsgBufCtrlArm {MESSAGE_BODY(MsgBufCtrlArmThreadLaunch)};
class GOLEM_LIBRARY_DECLDIR MsgBufCtrlArmInvalidJointType : public MsgBufCtrlArm {MESSAGE_BODY(MsgBufCtrlArmInvalidJointType)};
class GOLEM_LIBRARY_DECLDIR MsgBufCtrlArmCalibration : public MsgBufCtrlArm {MESSAGE_BODY(MsgBufCtrlArmCalibration)};

//------------------------------------------------------------------------------

class BufCtrlArm;

/** Joint of BufCtrlArm manipulator.
 */
class GOLEM_LIBRARY_DECLDIR BufCtrlJoint: public Joint {
	friend class BufCtrlArm;

public:
	/** BufCtrlJoint description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Joint::Desc {
	public:
		Desc() {
			Desc::setToDefault();
		}
		virtual void setToDefault() {
			Joint::Desc::setToDefault();
		}
		virtual bool isValid() const {
			if (!Joint::Desc::isValid())
				return false;
			return true;
		}
	};

protected:
	virtual bool sysRecv(GenCoord& curr) = 0;
	virtual bool sysSend(const GenCoord& prev, const GenCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt) = 0;
	
	BufCtrlJoint(Arm& arm);
};

//------------------------------------------------------------------------------

/** Controller of a motor system which can be modeled as a 2-message-length control buffer.
 */
class GOLEM_LIBRARY_DECLDIR BufCtrlArm: public Arm, protected Runnable {
public:
	/** System internal buffer length */
	static const U32 SYS_BUF_LEN = 2;
	/** Measurement */
	typedef golem::Measurement<SecTmReal> Measurement;

	/** Controller description
	 */
	class GOLEM_LIBRARY_DECLDIR Desc : public Arm::Desc {
	public:
		/** Working thread priority */
		Thread::Priority threadPriority;
		/** Inter-thread signalling time out */
		MSecTmU32 threadTimeOut;
		
		/** State buffer size */
		size_t qStateSize;
		/** Sent buffer size */
		size_t qSentSize;
		/** Command buffer size */
		size_t qCommandSize;
		/** number of calibration cycles */
		U32 cycleNum;
		/** calibration cycle duration */
		SecTmReal cycleDur;
		/** Models the resolution of internal timers of a system - the smallest allowable time increment. */
		SecTmReal timeQuant;
		/** cycle offset */
		SecTmReal deltaOffs;
		/** skew correction offset */
		SecTmReal skewOffs;

		Desc() {
			Desc::setToDefault();
		}

		virtual void setToDefault() {
			Arm::Desc::setToDefault();

			threadPriority = Thread::HIGHEST;
			threadTimeOut = 5000; //[msec]

			qStateSize = 5000;
			qSentSize = 5000;
			qCommandSize = 5000;
			cycleNum = 20;
			cycleDur = (SecTmReal)0.300; // [sec]
			timeQuant = (SecTmReal)0.01; // [sec]
			deltaOffs = (SecTmReal)0.02; // [sec]
			skewOffs = (SecTmReal)0.01; // [sec]
		}

		virtual bool isValid() const {
			if (!Arm::Desc::isValid())
				return false;

			if (threadTimeOut <= 0)
				return false;
			
			if (qStateSize < 2 || qSentSize < 2 || qCommandSize < 2)
				return false;
			if (timeQuant <= (SecTmReal)0.0)
				return false;
			if (cycleNum <= 0 || cycleDur <= (SecTmReal)0.0 || deltaOffs <= (SecTmReal)0.0 || skewOffs <= (SecTmReal)0.0)
				return false;

			return true;
		}
	};

private:
	volatile bool bTerminate;
	Thread thread;
	
	/** Inter-thread signalling time out */
	MSecTmU32 threadTimeOut;
	
	// Calibration variables
	/** Controller initialisation state. */
	bool bStart;
	/** Calibration state. */
	bool bCalibrating;
	/** Calibration state. */
	bool bCalibrated;
	/** I/O timings. */
	Measurement msrDeltaSync, msrDeltaRecv, msrDeltaProc, msrDeltaSend;
	/** Time delta - I/O cycle duration. */
	SecTmReal tmDelta;
	/** Time quant - I/O cycle duration "granularity". */
	SecTmReal timeQuant;
	/** cycle offset */
	SecTmReal deltaOffs;
	/** skew correction offset */
	SecTmReal skewOffs;

	// Working thread variables
	Event evSyncLock, evSyncUnlock;
	Event evReady, evCycle;
	SecTmReal tmCycle;

	volatile bool bSyncRequest, bCycleRequest;
	
	/** Initilises buffer queues */
	void initBuffers(const GenConfigspaceCoord& state, SecTmReal t, SecTmReal dt);

	/** Calibration stuff */
	bool calibrate(const Desc& desc);
	
	/** Waits for a new reading */
	bool cycleWait(SecTmReal t, MSecTmU32 timeWait = MSEC_TM_U32_INF);

	/** Communication function working in a separate thread. 
	 * 
	 * The function dispatches messages (states) from/to queues to/from 
	 * system.
	 */
	virtual void run();
	
protected:
	/** Indicates if the controller undergoes calibration process.
	*
	*	Typically this function is used by overrides BufCtrlArm#sysRecv(),
	*	BufCtrlArm#sysSend(), BufCtrlArm#sysSync() or BufCtrlArm#userComm(), to check
	*	whether the control loop is in calibration mode.
	*
	*	@return				<code>true</code> the controller is just being calibrated;
	* 						<code>false</code> otherwise
	*/
	bool isCalibrating() const;
	
	/** Indicates if the controller is calibratied.
	*
	*	@return				<code>true</code> the controller is calibrated;
	* 						<code>false</code> otherwise
	*/
	bool isCalibrated() const;
	
	/** Models the output of a system.
	 * 
	 * This is the IO function which receives a single data packet, 
	 * describing the current state of a system.
	 * Controller assumes that <code>curr</code> is the state a system 
	 * has reached just before calling this function.
	 * 
	 * @param state			current state of a system	
	 * @return				<code>true</code> no IO errors;
	 * 						<code>false</code> otherwise
	 * @see	Implementation for a generic robotic arm 
	 * 		<code>Arm#recv(GenConfigspaceCoord& curr)</code>
	 */
	virtual bool sysRecv(GenConfigspaceCoord& state);

	/** Models the input of a system.
	 * 
	 * This is the IO function which sends two data packets, which specify the 
	 * previous target state and the next target state of a system, 
	 * together with the desired time distance between them.
	 * Although BufCtrlArm does not assumes how any potential system-specific 
	 * implementation of this function will interpret the function description,
	 * optimally a system will try to achieve <code>next</code> in 
	 * <code>dt + offset</code> seconds just after leaving this function.
	 * If the system internal buffer is empty <code>offset</code> equals zero,
	 * otherwise <code>offset</code> is equal to the previously sent distance
	 * <code>dt</code>.
	 * 
	 * @param prev			pointer to the previous state
	 * @param next			pointer to the next state
	 * @param dt			time distance between the previous and the next state	
	 * @return				<code>true</code> no IO errors;
	 * 						<code>false</code> otherwise
	 * @see	Implementation for a generic robotic arm 
	 * 		<code>Arm#send(const GenConfigspaceCoord& prev, const GenConfigspaceCoord& next, SecTmReal dt)</code>
	 * @see	System internal buffer <code>BufCtrlArm#sync()</code>
	 */
	virtual bool sysSend(const GenConfigspaceCoord& prev, const GenConfigspaceCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt);

	
	/** Models the internal buffer of a system.
	 * 
	 * The method signalises if a system is ready to accept new target state.
	 * It is assumed that a system has an internal buffer of length 2.
	 * This is an optimal length for all physical systems with limited 
	 * but reliable bandwidth of a communication channel between BufCtrlArm and 
	 * a modelled system.
	 * 
	 * @return				<code>true</code> if the buffer is ready to send
	 * 						a new target state; <code>false</code> otherwise
	 * @see	Implementation for Katana arm 
	 * 		<code>Katana300Arm#sync()</code>
	 * @see	Implementation for Katana simulator 
	 * 		<code>SimArm#sync()</code>
	 */
	virtual bool sysSync() = 0;

	/** User calibration function. */
	virtual void userCalibrate();
	
	/** User communication function. */
	virtual void userComm();
	
	/** Creates/initialises BufCtrlArm form the description
	 * @param desc			BufCtrlArm description
	 * @return				<code>true</code> no errors;
	 * 						<code>false</code> otherwise
	*/
	bool create(const Desc& desc);
	
	/** Shuts down control thread and releases related resources */
	void release();
	
	/** Constructs Controller */
	BufCtrlArm(golem::Context& context);
	
	/** Each derived class should have virtual destructor releasing resources
	*	to avoid calling virtual functions of non-existing objects
	*/
	virtual ~BufCtrlArm();

public:
	/** Receives the latest state of the motor system (blocking call).
	 */
	virtual bool recv(GenConfigspaceState& state, MSecTmU32 timeWait = MSEC_TM_U32_INF);
	
	/** Sends motor command (blocking call).
	 */
	virtual bool send(const GenConfigspaceState& command, MSecTmU32 timeWait = MSEC_TM_U32_INF);

	/** Time quant is a minimum time increment. */
	virtual SecTmReal getTimeQuant() const;

	/** Returns the minimum effective time period between consecutive motor commands. */
	virtual SecTmReal getTimeDelta() const;

	/** Returns the minimum effective time period between consecutive asynchronous motor commands. */
	virtual SecTmReal getTimeDeltaAsync() const;
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEVICE_BUFCTRL_BUFCTRL_H_*/
