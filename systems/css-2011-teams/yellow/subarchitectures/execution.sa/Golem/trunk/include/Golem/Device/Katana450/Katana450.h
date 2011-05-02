/** @file Katana450.h
 * 
 * Katana 450 controller.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEVICE_KATANA450_KATANA450_H_
#define _GOLEM_DEVICE_KATANA450_KATANA450_H_

//------------------------------------------------------------------------------

#include <Golem/Device/Katana/Katana.h>
#include <Golem/Device/BufCtrl/BufCtrl.h>
#include <Golem/Device/Katana450/AxNI_base.h>
#include <memory>

//------------------------------------------------------------------------------

/** Debugging */
//#define GOLEM_DEVICE_KATANA_DEBUG_

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadArmDesc(void* pContext, const std::string& path, void* pArmDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR MsgKatana450Joint : public MsgBufCtrlJoint {};
class GOLEM_LIBRARY_DECLDIR MsgKatana450JointInvalidArmType : public MsgKatana450Joint {MESSAGE_BODY(MsgKatana450JointInvalidArmType)};
class GOLEM_LIBRARY_DECLDIR MsgKatana450JointMoveBuffer : public MsgKatana450Joint {MESSAGE_BODY(MsgKatana450JointMoveBuffer)};

class GOLEM_LIBRARY_DECLDIR MsgKatana450Arm : public MsgBufCtrlArm {};
class GOLEM_LIBRARY_DECLDIR MsgKatana450ArmInvalidJointType : public MsgKatana450Arm {MESSAGE_BODY(MsgKatana450ArmInvalidJointType)};

//------------------------------------------------------------------------------

class Katana450Arm;

class GOLEM_LIBRARY_DECLDIR Katana450Joint: public BufCtrlJoint, public KatanaJoint {
	friend class Katana450Arm;

public:
	/** Katana Joint description */
	class GOLEM_LIBRARY_DECLDIR Desc : public BufCtrlJoint::Desc, public KatanaJoint::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Katana450Joint, Joint::Ptr, Arm&)

	public:
		/** AxNI calibration settings */
		AxNI_base::CalibrationParameter calibration;
		/** AxNI calibration controller settings */
		AxNI_base::ControllerParameter calibrationControl;
		/** AxNI calibration collision detection settings */
		AxNI_base::CollisionParameter calibrationCollision;
		/** AxNI normal-operation controller settings */
		AxNI_base::ControllerParameter operationControl;
		/** AxNI normal-operation collision detection settings */
		AxNI_base::CollisionParameter operationCollision;
		/** AxNI polynomial settings */
		AxNI_base::PolynomialParameter polynomial;

		Desc() {
			Desc::setToDefault();
		}

		virtual void setToDefault() {
			BufCtrlJoint::Desc::setToDefault();
			KatanaJoint::Desc::setToDefault();

			calibration.setToDefault();
			calibrationControl.setToDefault();
			calibrationCollision.setToDefault();
			operationControl.setToDefault();
			operationCollision.setToDefault();
			polynomial.setToDefault();
		}

		virtual bool isValid() const {
			if (!BufCtrlJoint::Desc::isValid() || !KatanaJoint::Desc::isValid())
				return false;
			if (encoderOffset < 0 || encoderPositionAfter < 0 || encodersPerCycle < 0)
				return false;
			if (!calibration.isValid() || !calibrationControl.isValid() || !calibrationCollision.isValid() || !operationControl.isValid() || !operationCollision.isValid() || !polynomial.isValid())
				return false;

			return true;
		}
	};

private:
	U8 moveBufferSize;

protected:
	// Scaling description
	static const Real trj4ScaleFac[4];

	/** AxNI calibration settings */
	AxNI_base::CalibrationParameter calibration;
	/** AxNI calibration controller settings */
	AxNI_base::ControllerParameter calibrationControl;
	/** AxNI calibration collision detection settings */
	AxNI_base::CollisionParameter calibrationCollision;
	/** AxNI normal-operation controller settings */
	AxNI_base::ControllerParameter operationControl;
	/** AxNI normal-operation collision detection settings */
	AxNI_base::CollisionParameter operationCollision;
	/** AxNI polynomial settings */
	AxNI_base::PolynomialParameter polynomial;

#ifdef GOLEM_DEVICE_KATANA_DEBUG_
public:
#endif
	// Katana specific pointers initialised during construction
	Katana450Arm *pKatana450Arm;
	AxNI_base* pAxNI_base;
	
	virtual bool sysSync();
	virtual bool sysSend(const GenCoord& prev, const GenCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt);
	virtual bool sysStart();
	virtual bool sysRecv(GenCoord& curr);

	/** Creates Katana Joint from the description. */
	bool create(const Desc& desc);

	/** Cleaning up */
	void release();

	Katana450Joint(Arm& arm);
	virtual ~Katana450Joint();

public:
};

//------------------------------------------------------------------------------

class Katana450Joint;

class GOLEM_LIBRARY_DECLDIR Katana450Arm : public BufCtrlArm, public KatanaArm, public KatanaGripper {
	friend class Katana450Joint;

public:
	/** Katana450Arm description */
	class GOLEM_LIBRARY_DECLDIR Desc : public BufCtrlArm::Desc, public KatanaArm::Desc, public KatanaGripper::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Katana450Arm, Arm::Ptr, Context&)
		
		/** Can bus device driver path */
		std::string canDevice;
		/** Katana measurements */
		bool bMeasurements;
		/** Joint index set for synchronization function  */
		std::set<U32> syncJointIndex;
		
		// TODO: all this should go into something like a Katana450Gripper class
		/** Maximum amount of time spent in gripperComm(). The actual times are variable
		 * and difficult to measure. So we assume a max time, which must be large enough
		 * to cover actual times. Otherwise you will get:
		 * ... level=WARNING] BufCtrlArm::run(): sysRecv()/userComm() timeout = 0.115030 */
		Real gripperMaxCommTime;
		/** Encoder value for gripper open state. */
		I32 gripperOpenEncVal;
		/** Encoder value for gripper closed state. */
		I32 gripperClosedEncVal;
		/** KNI angleOffset [deg] */
		Real gripperAngleOffset;
		/** KNI angleRange [deg] */
		Real gripperAngleRange;
		/** KNI encoderOffset */
		I32 gripperEncoderOffset;
		/** KNI encodersPerCycle */
		I32 gripperEncodersPerCycle;
		/** KNI rotationDirection {+1=DIR_POSITIVE, -1=DIR_NEGATIVE} */
		I32 gripperRotationDirection;
		/** KNI encoderPositionAfter */
		I32 gripperEncoderPositionAfter;
		/** Rescaling offset [rad] */
		Real gripperOffset;
		/** Rescaling gain */
		Real gripperGain;
		/** AxNI calibration settings */
		AxNI_base::CalibrationParameter gripperCalibration;
		/** AxNI calibration controller settings */
		AxNI_base::ControllerParameter gripperCalibrationControl;
		/** AxNI calibration collision detection settings */
		AxNI_base::CollisionParameter gripperCalibrationCollision;
		/** AxNI normal-operation controller settings */
		AxNI_base::ControllerParameter gripperOperationControl;
		/** AxNI normal-operation collision detection settings */
		AxNI_base::CollisionParameter gripperOperationCollision;

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			BufCtrlArm::Desc::setToDefault();
			KatanaArm::Desc::setToDefault();
			KatanaGripper::Desc::setToDefault();

			name = "Katana 450 (6M180) controller";
			canDevice = "/dev/pcanusb0";
			bMeasurements = false;
			syncJointIndex.insert(NUM_JOINTS - 1); // last joint
			
			joints.clear();
			for (U32 i = 0; i < NUM_JOINTS; ++i)
				joints.push_back(Joint::Desc::Ptr(new Katana450Joint::Desc));

			gripperMaxCommTime = Real(0.02);
			gripperAngleOffset = Real(0.0);
			gripperAngleRange = Real(360.0);
			gripperEncoderOffset = 10000;
			gripperEncodersPerCycle = 100000;
			gripperRotationDirection = 1;
			gripperEncoderPositionAfter = 11000;
			gripperOffset = Real(0.0);
			gripperGain = 1.0;
			gripperCalibration.setToDefault();
			gripperCalibrationControl.setToDefault();
			gripperCalibrationCollision.setToDefault();
			gripperOperationControl.setToDefault();
			gripperOperationCollision.setToDefault();
		}

		virtual bool isValid() const {
			if (!BufCtrlArm::Desc::isValid() || !KatanaArm::Desc::isValid() || !KatanaGripper::Desc::isValid())
				return false;
			if (Math::isZero(gripperAngleRange))
				return false;
			if (gripperEncoderOffset == gripperEncoderPositionAfter || gripperEncodersPerCycle <= 0 || !(gripperRotationDirection == 1 || gripperRotationDirection == -1))
				return false;
			if (Math::isZero(gripperGain))
				return false;
			if (gripperEncoderOffset < 0 || gripperEncoderPositionAfter < 0 || gripperEncodersPerCycle < 0)
				return false;
			if (!gripperCalibration.isValid() || !gripperCalibrationControl.isValid() || !gripperCalibrationCollision.isValid() || !gripperOperationControl.isValid() || !gripperOperationCollision.isValid())
				return false;

			return true;
		}
	};
	
protected:
	/** Gripper position encoder tolerance for reaching target position */
	static const U32 GRIP_TARGET_ENC_TOLERANCE = 100;
	/** number of gripperComm() cycles for detecting motor stall */
	static const U32 GRIP_STALL_CYCLES = 10;
	enum GripperStatus {
		NOOP = 0x0,
		OPEN = 0x1,
		CLOSE = 0x2,
		FREEZE = 0x4
	};

	/** Can bus device driver path */
	std::string canDevice;
	/** Katana measurements */
	bool bMeasurements;
	/** Joint index set for synchronization function  */
	std::set<U32> syncJointIndex;

	// TODO: all this should go into something like a Katana450Gripper class
	CriticalSection csGripper;
	/** Maximum amount of time spent in gripperComm(). The actual times are variable
	 * and difficult to measure. So we assume a max time, which must be large enough
	 * to cover actual times. Otherwise you will get:
	 * ... level=WARNING] BufCtrlArm::run(): sysRecv()/userComm() timeout = 0.115030 */
	Real gripperMaxCommTime;
	GripperEncoderData gripperEncoderData;
	/** remember how many cycles the motor stalled */
  I32 gripperStallCounter;
	volatile int gripperStatus;
	Event evGripper;
	volatile bool bGripperResult;
	/** KNI angleOffset [deg] */
	Real gripperAngleOffset;
	/** KNI angleRange [deg] */
	Real gripperAngleRange;
	/** KNI encoderOffset */
	I32 gripperEncoderOffset;
	/** KNI encodersPerCycle */
	I32 gripperEncodersPerCycle;
	/** KNI rotationDirection {+1=DIR_POSITIVE, -1=DIR_NEGATIVE} */
	I32 gripperRotationDirection;
	/** KNI encoderPositionAfter */
	I32 gripperEncoderPositionAfter;
	/** Rescaling offset [rad] */
	Real gripperOffset;
	/** Rescaling gain */
	Real gripperGain;
	/** AxNI calibration settings */
	AxNI_base::CalibrationParameter gripperCalibration;
	/** AxNI calibration controller settings */
	AxNI_base::ControllerParameter gripperCalibrationControl;
	/** AxNI calibration collision detection settings */
	AxNI_base::CollisionParameter gripperCalibrationCollision;
	/** AxNI normal-operation controller settings */
	AxNI_base::ControllerParameter gripperOperationControl;
	/** AxNI normal-operation collision detection settings */
	AxNI_base::CollisionParameter gripperOperationCollision;

	/** AxNI device */
	std::auto_ptr<AxNI_base> pAxNI_base;
	bool bCalibrationInit;
	GenConfigspaceCoord current;

	virtual void userCalibrate();
	virtual void userComm();
	
	void gripperCalibrate();
	void gripperComm();

	virtual bool sysRecv(GenConfigspaceCoord& curr);
	virtual bool sysSend(const GenConfigspaceCoord& prev, const GenConfigspaceCoord& next, bool bSendPrev, bool bSendNext, SecTmReal dt);
	virtual bool sysSync();

	/** Creates Katana */
	bool create(const Desc& desc);
	
	Katana450Arm(golem::Context& context);

public:
	virtual ~Katana450Arm();
	
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
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEVICE_KATANA450_KATANA450_H_*/
