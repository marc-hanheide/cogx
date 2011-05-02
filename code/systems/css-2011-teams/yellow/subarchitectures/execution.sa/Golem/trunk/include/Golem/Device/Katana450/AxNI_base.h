/** @file AxNI_base.h
 * 
 * Katana 450 (6M180) controller.
 * C++ equivalent of the Python version of AxNI_base by Neuronics.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEVICE_KATANA450_AxNI_base_H_
#define _GOLEM_DEVICE_KATANA450_AxNI_base_H_

//------------------------------------------------------------------------------

#include <Golem/Sys/Timer.h>
#include <Golem/Device/Katana450/axis_config.h>
#include <memory>
#include <string>

//------------------------------------------------------------------------------

class CANopenMaster;

namespace golem {

//------------------------------------------------------------------------------

class AxNI_base {
public:
	static const U32 CANpollingTime;//[us]
	static const SecTmReal CANopenCycleTime;//[s]
	static const SecTmReal CANopenInhibitTime;//[s]
	static const SecTmReal CANopenHeartBeatPeriod;//[s]

	//These are the standard values if the position controller is selected:
	//--------------------------------------------------------------------
	//	kPPos	kIPos	kDPos	kILegacy	kCurrent	kICurrent	maxDrive	maxCurrent	type	ke	ra	offsetE1	node
	//	2700	840		810		0			78			2200		32000		1800		type	0	0	0			1
	//	3500	2460	1370	0			62			1200		32000		1200		type	0	0	0			2
	//	2400	1120	1000	0			23			1200		32000		550			type	0	0	0			3
	//	1500	1000	500		0			31			2000		32000		900			type	0	0	0			4
	//	1500	1170	310		0			23			2200		32000		700			type	0	0	0			5
	//	1500	1170	310		0			78			2200		32000		500			type	0	0	0			6
	//
	//These are the standard values if the current controller is selected:
	//--------------------------------------------------------------------
	//kPos kIPos kSpeed kISpeed kCurrent kICurrent maxDrive maxCurrent type ke ra offset node
	//[ 100, 0 , 800, 50 , 0, 0, pwm(127), 0, type, 0, 0, 0, 1 ]
	//[ 100, 0, 600, 200, 0, 0, pwm(100), 0, type, 0, 0, 0, 2 ]
	//[ 100, 0, 500, 200, 0, 0, pwm(80), 0, type, 0, 0, 0 , 3 ]
	//[ 200, 0, 600, 250, 0, 0, pwm(100), 0, type, 0, 0, 0, 4 ]
	//[ 400, 0, 350, 180, 0 , 0, pwm(127), 0, type, 0, 0, 0, 5 ]
	//[ 200, 0, 500, 150, 0, 0, pwm(127), 0, type, 0, 0 , 0, 6 ]

	class ControllerParameter {
	public:
		/** proportional parameter for the position controller */
		U16 kPPos;
		/** integral parameter for the position controller */
		U16 kIPos;
		/** differential parameter for the position controller */
		U16 kDPos;
		/** integral parameter for the legacy speed controller, set to 0 */
		U16 kILegacy;
		/** proportional parameter for the current controller */
		U16 kCurrent;
		/** integral parameter for the current controller */
		U16 kICurrent;
		/** the highest drive allowed */
		U16 maxDrive;
		/** the highest current allowed */
		U16 maxCurrent;
		/** according to the EControllerType enum */
		EControllerType type;
		/** currently not supported */
		U16 ke;
		/** currently not supported */
		U16 ra;
		/** currently not supported */
		I16 offsetE1;

		ControllerParameter() {
			setToDefault();
		}
		void setToDefault() {
			kPPos = 200;
			kIPos = 250;
			kDPos = 50;
			kILegacy = 0;
			kCurrent = 500;
			kICurrent = 600;
			maxDrive = 10000;
			maxCurrent = 1500;
			type = CONTROLLER_TYPE_DRIVE_V2;
			ke = 230;
			ra = 120;
			offsetE1 = 0;
		}
		bool isValid() const {
			return true;
		}
	};

	class CollisionParameter {
	public:
		U16 limitPos;
		U16 limitSpeed;

		CollisionParameter() {
			setToDefault();
		}
		void setToDefault() {
			limitPos = 80;
			limitSpeed = 80;
		}
		bool isValid() const {
			return true;
		}
	};

	class CalibrationParameter {
	public:
		U32 encOffset;
		U32 encPosAfter;
		U32 encPerCycle;
		U16 speed;
		U8 accel;
		U8 stopTolerance;
		U8 rangeTolerance;

		CalibrationParameter() {
			setToDefault();
		}
		void setToDefault() {
			encOffset = 0;
			encPosAfter = 10000;
			encPerCycle = 20000;
			speed = 10;
			accel = 1;
			stopTolerance = 1;
			rangeTolerance = 50;
		}
		bool isValid() const {
			return true;
		}
	};

	class PolynomialParameter {
	public:
		I32 offset;
		I16 slope;
		U8 tolerance;

		PolynomialParameter() {
			setToDefault();
		}
		void setToDefault() {
			offset = 0;
			slope = 1;
			tolerance = 10;
		}
		bool isValid() const {
			if (slope < 1)
				return false;
			return true;
		}
	};

private:
	std::auto_ptr<CANopenMaster> pCANopen;

	void wait4state(U8 node, EAxisFsmState newState, SecTmReal timeout);
	void wait2stop(U8 node);

public:
	AxNI_base();
	~AxNI_base();

	void start(const std::string& canDevice);
	void stop();
	void activate(U8 node);
	void hold(U8 node);
	void release(U8 node);

	void setAxisFSMcommand(U8 node, EAxisFsmCommand fsmCommand, bool immediate = false);
	EAxisFsmState getAxisFSMstate(U8 node);

	void setEncoder(U8 node, U32 target);
	void setCollisionDetection(U8 node, bool active, const CollisionParameter& colParam);
	void setControllerParameter(U8 node, const ControllerParameter& ctrlParam);
	void calibrate(U8 node, const CalibrationParameter& calParam, const ControllerParameter& ctrlParam, const CollisionParameter& collParam);
	
	U32 getPosition(U8 node);
	I16 getSpeed(U8 node);
	I16 getDrive(U8 node);
	I16 getCurrent(U8 node);
	I16 getCurrentBufferData(U8 node);
	I16 getVoltage(U8 node);
	std::string getVersion(U8 node);
	
	void moveP2P(U8 node, U32 target, U16 speed, U8 accel, U8 tolerance, bool immediate = false);
	void moveP2Pwait(U8 node, U32 target, U16 speed, U8 accel, U8 tolerance);

	void setPolyScale(U8 node, const PolynomialParameter& polyParam);
	U8 getMoveBufferSize(U8 node);
	U8 getMoveBufferCounter(U8 node);
	void moveBufferFlush(U8 node);
	bool moveBufferIsReady(U8 node);
	void moveBufferStore(U8 node, I32 target, U16 duration, U8 tolerance, const I16* coeffs, bool next, bool immediate = false);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEVICE_KATANA450_AxNI_base_H_*/
