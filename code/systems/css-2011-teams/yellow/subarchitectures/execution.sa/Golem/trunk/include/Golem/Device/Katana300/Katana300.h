/** @file Katana300.h
 * 
 * Katana 300 controller.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEVICE_KATANA300_KATANA300_H_
#define _GOLEM_DEVICE_KATANA300_KATANA300_H_

//------------------------------------------------------------------------------

#include <Golem/Device/Katana/Katana.h>
#include <Golem/Device/BufCtrl/BufCtrl.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadArmDesc(void* pContext, const std::string& path, void* pArmDesc);
};

class CKatana;
class CCplBase;
class CCdlBase;
class CMotBase;

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR MsgKatana300Joint : public MsgBufCtrlJoint {};
class GOLEM_LIBRARY_DECLDIR MsgKatana300JointInvalidArmType : public MsgKatana300Joint {MESSAGE_BODY(MsgKatana300JointInvalidArmType)};

class GOLEM_LIBRARY_DECLDIR MsgKatana300Arm : public MsgBufCtrlArm {};
class GOLEM_LIBRARY_DECLDIR MsgKatana300ArmInvalidDesc : public MsgKatana300Arm {MESSAGE_BODY(MsgKatana300ArmInvalidDesc)};
class GOLEM_LIBRARY_DECLDIR MsgKatana300ArmNullKNIPointers : public MsgKatana300Arm {MESSAGE_BODY(MsgKatana300ArmNullKNIPointers)};
class GOLEM_LIBRARY_DECLDIR MsgKatana300ArmInvalidJointType : public MsgKatana300Arm {MESSAGE_BODY(MsgKatana300ArmInvalidJointType)};
class GOLEM_LIBRARY_DECLDIR MsgKatana300ArmCommPort : public MsgKatana300Arm {MESSAGE_BODY(MsgKatana300ArmCommPort)};
class GOLEM_LIBRARY_DECLDIR MsgKatana300ArmProtocolInit : public MsgKatana300Arm {MESSAGE_BODY(MsgKatana300ArmProtocolInit)};
class GOLEM_LIBRARY_DECLDIR MsgKatana300ArmCKatanaCreate : public MsgKatana300Arm {MESSAGE_BODY(MsgKatana300ArmCKatanaCreate)};
class GOLEM_LIBRARY_DECLDIR MsgKatana300ArmKNICalibration : public MsgKatana300Arm {MESSAGE_BODY(MsgKatana300ArmKNICalibration)};

//------------------------------------------------------------------------------

class Katana300Arm;

class GOLEM_LIBRARY_DECLDIR Katana300Joint: public BufCtrlJoint, public KatanaJoint {
	friend class Katana300Arm;

public:
	/** MCU internal time unit [sec] */
	static const Real MCU_TM_PERIOD;
	/** Katana internal constants */
	static const Real VEL_ENC_UNIT;
	/** FIR-Filter coefficients */
	static const U32 NUM_FILTER_COEFFS = 5; // number of FIR-Filter coefficients
	/** Synchronisation flag */
	static const U32 IO_SYNC_FLAG = 152;	// see wait() function
	/** Synchronisation flag */
	static const U32 IO_CRASH_FLAG = 40;	// 
	/** IO communication buffer */
	static const U32 IO_BUF_SIZE = 256;

	/** Katana Joint description */
	class GOLEM_LIBRARY_DECLDIR Desc : public BufCtrlJoint::Desc, public KatanaJoint::Desc {
	public:
		/** Activate FIR-Filter */
		bool activateFIRFilter;
		/** FIR-Filter coefficients */
		U8 coeffs[NUM_FILTER_COEFFS];
		/** Activate slope saturation */
		bool activateSlopeSaturation;
		/** Slope saturation limit */
		U8 limit;
		
		Desc() {
			Desc::setToDefault();
		}

		virtual void setToDefault() {
			BufCtrlJoint::Desc::setToDefault();
			KatanaJoint::Desc::setToDefault();

			activateFIRFilter = true;
			coeffs[0] = 5;
			coeffs[1] = 3;
			coeffs[2] = 2;
			coeffs[3] = 1;
			coeffs[4] = 1;
			activateSlopeSaturation = true;
			limit = 255;
		}

		virtual bool isValid() const {
			if (!BufCtrlJoint::Desc::isValid() || !KatanaJoint::Desc::isValid())
				return false;

			return true;
		}
	};

protected:
	/** FIR-Filter coefficients */
	U8 coeffs[NUM_FILTER_COEFFS];
	/** Slope saturation limit */
	U8 limit;
	
	// Katana specific pointers initialised during construction
	Katana300Arm *pKatana300Arm;
	CKatana* pKatana;
	CCplBase* pProtocol;
	CMotBase* pMotBase;
	
	// IO buffers
	U8 sendBuf[IO_BUF_SIZE];
	U8 recvBuf[IO_BUF_SIZE];

	virtual bool sysSync();
	virtual bool sysSend(const GenCoord& prev, const GenCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt) = 0;
	
	// IJointBase, partial implementation
	virtual bool sysRecv(GenCoord& curr);

	/** Creates Katana Joint from the description. */
	bool create(const Desc& desc);
	
	Katana300Joint(Arm& arm);

public:
	/** Sets FIR-Filter coefficients */
	bool setFIRFilter(const U8 *coeffs);
	/** Returns FIR-Filter coefficients */
	const U8 *getFIRFilter() const;
	
	/** Slope saturation limit */
	bool setSlopeSaturation(U8 limit);
	/** Returns FIR-Filter coefficients */
	U8 getSlopeSaturation() const;
};

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR Katana300StdJoint: public Katana300Joint {
	friend class Katana300Arm;

public:
	/** Katana300StdJoint description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Katana300Joint::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Katana300StdJoint, Joint::Ptr, Arm&)
	};

protected:
	virtual bool sysSend(const GenCoord& prev, const GenCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt);

	Katana300StdJoint(Arm& arm);
};

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR Katana300TrjJoint: public Katana300Joint {
	friend class Katana300Arm;

public:
	/** Katana300TrjJoint description */
	class GOLEM_LIBRARY_DECLDIR Desc : public Katana300Joint::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Katana300TrjJoint, Joint::Ptr, Arm&)
	};

private:
	// Scaling description
	static const Real trj4ScaleFac[4];

protected:
	virtual bool sysSend(const GenCoord& prev, const GenCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt);

	Katana300TrjJoint(Arm& arm);
};

//------------------------------------------------------------------------------

class Katana300Joint;

class GOLEM_LIBRARY_DECLDIR Katana300Arm : public BufCtrlArm, public KatanaArm, public KatanaGripper {
	friend class Katana300Joint;

public:
	typedef shared_ptr<Katana300Arm> Ptr;

	/** Katana300Arm description */
	class GOLEM_LIBRARY_DECLDIR Desc : public BufCtrlArm::Desc, public KatanaArm::Desc, public KatanaGripper::Desc {
	public:
		/** Katana measurements */
		bool bMeasurements;
		/** Joint index set for synchronization function  */
		std::set<U32> syncJointIndex;

		/** Katana communication time offset */
		SecTmReal katDeltaOffs;
		
		Desc() {
			setToDefault();
		}

		/** Creates Katana300Arm given the description object and pointers to KNI objects. 
		* @param context	golem::Context object
		* @return			pointer to the Katana300Arm, if no errors have occured;
		*					<code>NULL</code> otherwise 
		*/
		Katana300Arm::Ptr create(golem::Context& context, CKatana* pKatana, CCplBase* pProtocol) const {
			Katana300Arm::Ptr pKatana300Arm(new Katana300Arm(context));

			if (!pKatana300Arm->create(pKatana, pProtocol, *this))
				pKatana300Arm.release();

			return pKatana300Arm;
		}
		
		virtual void setToDefault() {
			BufCtrlArm::Desc::setToDefault();
			KatanaArm::Desc::setToDefault();
			KatanaGripper::Desc::setToDefault();

			name = "Katana 300 (6M180) controller";
			
			joints.clear();
			for (U32 i = 0; i < NUM_JOINTS; ++i)
				joints.push_back(Joint::Desc::Ptr(new Katana300TrjJoint::Desc));

//			restPosition[0] = REAL_PI;

			bMeasurements = false;

			referencePose.p.v2 += L3;
		
			katDeltaOffs = (SecTmReal)0.02; // [sec]
			deltaOffs = (SecTmReal)0.07;//0.02; // [sec]
			skewOffs = (SecTmReal)0.02;//0.01; // [sec]

			syncJointIndex.insert(1); // Birmingham troubles with #0
		}

		virtual bool isValid() const {
			if (!BufCtrlArm::Desc::isValid() || !KatanaArm::Desc::isValid() || !KatanaGripper::Desc::isValid())
				return false;

			if (katDeltaOffs <= (SecTmReal)0.0)
				return false;
			if (syncJointIndex.empty() || *syncJointIndex.rbegin() >= joints.size())
				return false;

			return true;
		}
	};
	
protected:
	/** Gripper position encoder tolerance */
	static const U32 GRIP_ENC_TOLERANCE = 100;
	/** IO communication buffer */
	static const U32 IO_BUF_SIZE = 256;

	enum GripperStatus {
		NOOP = 0x0,
		OPEN = 0x1,
		CLOSE = 0x2,
		FREEZE = 0x4,
		RECV = 0x8,
		READ = 0x10,
	};

	/** Katana measurements */
	bool bMeasurements;
	/** Joint index set for synchronization function  */
	std::set<U32> syncJointIndex;
	
	// Simulator specific pointers initialised during construction
	CKatana* pKatana;
	CCplBase* pProtocol;

	bool bCalibrationInit;
	GenConfigspaceCoord current;

	bool gripperIsPresent;
	CriticalSection csGripper;
	volatile int gripperStatus;
	Event evGripper, evGripperRecv;
	volatile bool bGripperResult, bGripperRecvResult;
	SensorDataSet sensorData, sensorThreshold;
	GripperEncoderData gripperEncoderData;
	SecTmReal gripperDur;

	// IO buffers
	U8 sendBuf[IO_BUF_SIZE];
	U8 recvBuf[IO_BUF_SIZE];
	
	/** I/O timings. */
	BufCtrlArm::Measurement msrDeltaSendSync, msrDeltaJointSend;
	/** Katana communication time offset */
	SecTmReal katDeltaOffs;
	
	// Arm
	virtual void userCalibrate();
	virtual void userComm();
	
	void gripperCalibrate();
	void gripperComm();

	virtual bool sysRecv(GenConfigspaceCoord& curr);
	virtual bool sysSend(const GenConfigspaceCoord& prev, const GenConfigspaceCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt);
	virtual bool sysSync();
	
	bool create(CKatana* pKatana, CCplBase* pProtocol, const Desc& desc);
	
	Katana300Arm(golem::Context& context);

public:
	/** Each derived class should have virtual destructor releasing resources
	*	to avoid calling virtual functions of non-existing objects
	*/
	virtual ~Katana300Arm();
	
	// Arm kinematics
	virtual void forwardTransform(Mat34& trn, const ConfigspaceCoord& cc);
	virtual void velocitySpatial(Twist& v, const ConfigspaceCoord& cc, const ConfigspaceCoord& dcc);
	virtual void jacobianSpatial(Jacobian& jac, const ConfigspaceCoord& cc);

	/** Receives gripper sensor values */
	virtual bool gripperRecvSensorData(SensorDataSet& sensorData, MSecTmU32 timeWait = MSEC_TM_U32_INF);
	
	/** Receives gripper encoder values */
	virtual bool gripperRecvEncoderData(GripperEncoderData& encoderData, MSecTmU32 timeWait = MSEC_TM_U32_INF);
	
	/** Opens the gripper */
	virtual bool gripperOpen(MSecTmU32 timeWait = MSEC_TM_U32_INF);

	/** Closes the gripper, stops if only a signal from any sensor is above the given threshold */
	virtual bool gripperClose(const SensorDataSet& sensorThreshold, MSecTmU32 timeWait = MSEC_TM_U32_INF);
	
	/** Freezes the gripper */
	virtual bool gripperFreeze(MSecTmU32 timeWait = MSEC_TM_U32_INF);

	/** Katana measurements */
	bool hasMeasurements() const {
		return bMeasurements;
	}
};

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR Katana300SerialArm: public Katana300Arm {
public:
	/** RS232 serial protocol settings */
	class GOLEM_LIBRARY_DECLDIR SerialDesc {
	public:
		/** communication port */
		U32 commPort;
		/** baud rate */
		U32 baudRate;
		/** data bit */
		U32 dataBit;
		/** parity bit */
		U32 parityBit;
		/** stop bit */
		U32 stopBit;
		/** read total timeout */
		U32 readTimeout;
		/** write total timeout */
		U32 writeTimeout;
		
		SerialDesc() {
			setToDefault();
		}
		
		void setToDefault() {
			commPort = 0;
			baudRate = 57600;
			dataBit = 8;
			parityBit = 'N';
			stopBit = 1;
			readTimeout = 500;
			writeTimeout = 0;
		}
		
		bool isValid() const {
			if (readTimeout <= 0 && writeTimeout <= 0)
				return false;

			return true;
		}
	};
	
	/** Katana Serial Arm description
	 */
	class GOLEM_LIBRARY_DECLDIR Desc : public Katana300Arm::Desc {
	public:
		/** configuration file */
		std::string cfgPath;
		/** RS232 serial protocol settings */
		SerialDesc serialDesc;

		Desc() {
			setToDefault();
		}
		
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Katana300SerialArm, Arm::Ptr, Context&)
		
		virtual void setToDefault() {
			Katana300Arm::Desc::setToDefault();

			cfgPath = "";
			serialDesc.setToDefault();
		}
		
		virtual bool isValid() const {
			if (!Katana300Arm::Desc::isValid())
				return false;
			if (cfgPath.empty())
				return false;
			if (!serialDesc.isValid())
				return false;

			return true;
		}
	};

protected:
	CKatana* katana;
	CCplBase* protocol;
	CCdlBase* device;

	/** configuration file */
	std::string cfgPath;
	/** RS232 serial protocol settings */
	SerialDesc serialDesc;

	// Initialisation
	bool create(const Desc& desc);

	Katana300SerialArm(golem::Context& context);

	/** Each derived class should have virtual destructor releasing resources
	*	to avoid calling virtual functions of non-existing objects
	*/
	virtual ~Katana300SerialArm();
	
public:
	/** Returns configuration file */
	const std::string& getCfgPath() const {
		return cfgPath;
	}

	/** Returns RS232 serial protocol settings */
	const SerialDesc& getSerialDesc() const {
		return serialDesc;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEVICE_KATANA300_KATANA300_H_*/
