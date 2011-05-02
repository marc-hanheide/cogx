/** @file BufCtrlSim.h
 * 
 * Interface for controlling open chain manipulators using trajectories.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEVICE_BUFCTRLSIM_BUFCTRLSIM_H_
#define _GOLEM_DEVICE_BUFCTRLSIM_BUFCTRLSIM_H_

//------------------------------------------------------------------------------

#include <Golem/Device/BufCtrl/BufCtrl.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR MsgBufCtrlSimJoint : public MsgBufCtrlJoint {};
class GOLEM_LIBRARY_DECLDIR MsgBufCtrlSimArm : public MsgBufCtrlArm {};
class GOLEM_LIBRARY_DECLDIR MsgBufCtrlSimArmInvalidDesc : public MsgBufCtrlSimArm {MESSAGE_BODY(MsgBufCtrlSimArmInvalidDesc)};
class GOLEM_LIBRARY_DECLDIR MsgBufCtrlSimArmInvalidJointType : public MsgBufCtrlSimArm {MESSAGE_BODY(MsgBufCtrlSimArmInvalidJointType)};

//------------------------------------------------------------------------------

class BufCtrlSimArm;

/** Joint of BufCtrlSimArm manipulator.
 */
class GOLEM_LIBRARY_DECLDIR BufCtrlSimJoint: public BufCtrlJoint {
	friend class BufCtrlSimArm;

public:
	/** BufCtrlSimJoint description */
	class GOLEM_LIBRARY_DECLDIR Desc : public BufCtrlJoint::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(BufCtrlSimJoint, Joint::Ptr, Arm&)
	public:
	};

protected:
	PerfTimer timer;
	GenCoordTrj trj[BufCtrlArm::SYS_BUF_LEN];
	int readPtr, writePtr;
	volatile bool bStart;
	GenCoord current;
	
	bool sysSync();

	// IJointBase
	virtual bool sysRecv(GenCoord& curr);
	virtual bool sysSend(const GenCoord& prev, const GenCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt);
	
	/** Creates BufCtrlSimJoint from the description. */
	bool create(const Desc& desc);
	
	BufCtrlSimJoint(Arm& arm);
};

//------------------------------------------------------------------------------

/** Simulator of a robotic manipulator which uses BufCtrlArm controller.
 */
class GOLEM_LIBRARY_DECLDIR BufCtrlSimArm: public BufCtrlArm {
public:
	/** BufCtrlSimArm description
	 */
	class GOLEM_LIBRARY_DECLDIR Desc : public BufCtrlArm::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(BufCtrlSimArm, Arm::Ptr, Context&)

		/** sync() time duration */
		SecTmReal deltaSync;
		/** recv() time duration */
		SecTmReal deltaRecv;
		/** send() time duration */
		SecTmReal deltaSend;

		Desc() {
			setToDefault();
		}
		
		virtual void setToDefault() {
			BufCtrlArm::Desc::setToDefault();

			deltaSync = (SecTmReal)0.005;
			deltaRecv = (SecTmReal)0.005;
			deltaSend = (SecTmReal)0.040;
		}
		
		virtual bool isValid() const {
			if (!BufCtrlArm::Desc::isValid())
				return false;

			if (deltaSync < (SecTmReal)0.0 || deltaRecv < (SecTmReal)0.0 || deltaSend < (SecTmReal)0.0)
				return false;

			return true;
		}
	};

protected:
	/** sync() time duration */
	SecTmReal deltaSync;
	/** recv() time duration */
	SecTmReal deltaRecv;
	/** send() time duration */
	SecTmReal deltaSend;
	
	PerfTimer timer;

	// IArmBase
	virtual bool sysRecv(GenConfigspaceCoord& curr);
	virtual bool sysSend(const GenConfigspaceCoord& prev, const GenConfigspaceCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt);
	virtual bool sysSync();
	
	bool create(const Desc& desc);
	
	/** Arm initialisation */
	BufCtrlSimArm(golem::Context& context);

	/** Each derived class should have virtual destructor releasing resources
	*	to avoid calling virtual functions of non-existing objects
	*/
	virtual ~BufCtrlSimArm();
	
public:
	/** sync() time duration */
	SecTmReal getDeltaSync() const {
		return deltaSync;
	}

	/** recv() time duration */
	SecTmReal getDeltaRecv() const {
		return deltaRecv;
	}

	/** send() time duration */
	SecTmReal getDeltaSend() const {
		return deltaSend;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEVICE_BUFCTRLSIM_BUFCTRLSIM_H_*/
