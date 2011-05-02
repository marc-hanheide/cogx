/** @file Katana300Sim.h
 * 
 * Katana 300 (6M180) simulator.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEVICE_KATANA300SIM_KATANA300SIM_H_
#define _GOLEM_DEVICE_KATANA300SIM_KATANA300SIM_H_

//------------------------------------------------------------------------------

#include <Golem/Device/Katana/Katana.h>
#include <Golem/Device/BufCtrlSim/BufCtrlSim.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadArmDesc(void* pContext, const std::string& path, void* pArmDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class Katana300SimArm;

class GOLEM_LIBRARY_DECLDIR Katana300SimJoint: public BufCtrlSimJoint, public KatanaJoint {
	friend class Katana300SimArm;

public:
	/** Katana Joint description */
	class GOLEM_LIBRARY_DECLDIR Desc : public BufCtrlSimJoint::Desc, public KatanaJoint::Desc {
	public:
		Desc() {
			Desc::setToDefault();
		}

		virtual void setToDefault() {
			BufCtrlSimJoint::Desc::setToDefault();
			KatanaJoint::Desc::setToDefault();
		}

		virtual bool isValid() const {
			if (!BufCtrlSimJoint::Desc::isValid() || !KatanaJoint::Desc::isValid())
				return false;

			return true;
		}
	};

protected:
	/** Creates Katana Joint from the description. */
	bool create(const Desc& desc);
	
	Katana300SimJoint(Arm& arm);

public:
};

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR Katana300SimArm : public BufCtrlSimArm, public KatanaArm, public KatanaGripper {
	friend class Katana300SimJoint;

public:
	/** Katana300SimArm description
	 */
	class GOLEM_LIBRARY_DECLDIR Desc : public BufCtrlSimArm::Desc, public KatanaArm::Desc, public KatanaGripper::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Katana300SimArm, Arm::Ptr, Context&)

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			BufCtrlSimArm::Desc::setToDefault();
			KatanaArm::Desc::setToDefault();

			name = "Katana 300 (6M180) Simulator";
			
			joints.clear();
			for (U32 i = 0; i < NUM_JOINTS; ++i)
				joints.push_back(Joint::Desc::Ptr(new Katana300SimJoint::Desc));

			//restPosition[0] = REAL_PI;
			
			timeQuant = (SecTmReal)0.01;
			deltaSync = (SecTmReal)0.005;
			deltaRecv = (SecTmReal)0.0;
			deltaSend = (SecTmReal)0.075;

			referencePose.p.v2 += L3;
		}

		virtual bool isValid() const {
			if (!BufCtrlSimArm::Desc::isValid() || !KatanaArm::Desc::isValid())
				return false;
			
			return true;
		}
	};

protected:
	/** Arm initialisation */
	bool create(const Desc& desc);

	Katana300SimArm(golem::Context& context);

	/** */
	virtual ~Katana300SimArm();
	
public:
	// Inverse and forward transforms
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

#endif /*_GOLEM_DEVICE_KATANA300SIM_KATANA300SIM_H_*/
