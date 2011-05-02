/** @file Robot.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_ROBOT_H_
#define _GOLEM_DEMO_ROBOT_H_

//------------------------------------------------------------------------------

#include <Golem/Demo/Common/Finger.h>
#include <Golem/Demo/Common/FTSensor.h>
#include <Golem/Demo/Common/PointTracker.h>
#include <Golem/Demo/Common/Msg.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class Robot : public golem::Embodiment {
public:
	/** Object description */
	class Desc : public golem::Embodiment::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Robot, Object::Ptr, Scene&)

	public:
		/** Finger description */
		Finger::Desc fingerDesc;
		/** Control signal gain */
		Wrench ctrlGain;
		
		/** Virtual FT sensor description prototype */
		golem::VirtualFTSensor::Desc virtualFTSensorDesc;
		/** Virtual object tracker description prototype */
		golem::VirtualPointTracker::Desc virtualTrackerDesc;		
		
		/** Constructs Collision description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			Embodiment::Desc::setToDefault();
			fingerDesc.setToDefault();
			
			// Control signal gain setup
			ctrlGain.set(Real(2.0e-2), Real(2.0e-1), Real(2.0e-2), Real(2.0e-1), Real(2.0e-1), Real(2.0e-1));
		
			virtualFTSensorDesc.setToDefault();
			virtualTrackerDesc.setToDefault();
			virtualTrackerDesc.planeBoundsDesc.dimensions.set(Real(0.4), Real(0.4), Real(0.002));
			virtualTrackerDesc.planeBoundsDesc.pose.p.set(Real(0.0), Real(0.25), Real(0.0));
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!Embodiment::Desc::isValid())
				return false;
			
			if (!fingerDesc.isValid() || !ctrlGain.isFinite())
				return false;
			
			if (!virtualFTSensorDesc.isValid() || !virtualTrackerDesc.isValid())
				return false;
			
			return true;
		}
	};

protected:
	/** Finger */
	golem::Finger *pFinger;

	/** Finger FT sensor */
	golem::FTSensor *pFingerFTSensor;
	/** Finger tracker */
	golem::PointTracker *pFingerTracker;

	/** Control signal gain */
	Wrench ctrlGain;
	
	/** Virtual FT sensor description prototype */
	golem::VirtualFTSensor::Desc virtualFTSensorDesc;
	/** Virtual object tracker description prototype */
	golem::VirtualPointTracker::Desc virtualTrackerDesc;
	
	/** Generators of random numbers. */
	Rand rand;
	
	int button, state, x, y, dx, dy, dz;

	void mouseHandler(int button, int state, int x, int y);

	void motionHandler(int x, int y);

	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);
	
	/** (Pre)processing function called BEFORE every physics simulation step and before randering. */
	virtual void preprocess(SecTmReal elapsedTime);

	/** Creates object from description. */
	bool create(const Robot::Desc& desc) {
		if (!Embodiment::create(desc))
			return false;
		
		pFinger = dynamic_cast<golem::Finger*>(createChannel(desc.fingerDesc));
		if (pFinger == NULL)
			return false;

		ctrlGain = desc.ctrlGain;

		virtualFTSensorDesc = desc.virtualFTSensorDesc;
		virtualTrackerDesc = desc.virtualTrackerDesc;

		golem::VirtualFTSensor::Desc fingerFTSensorDesc = virtualFTSensorDesc;
		fingerFTSensorDesc.joint = &pFinger->getFingerJoint();
		pFingerFTSensor = dynamic_cast<golem::FTSensor*>(createChannel(fingerFTSensorDesc));
		if (pFingerFTSensor == NULL)
			return false;

		golem::VirtualPointTracker::Desc fingerTrackerDesc = virtualTrackerDesc;
		fingerTrackerDesc.actor = &pFinger->getFingerActor();
		pFingerTracker = dynamic_cast<golem::PointTracker*>(createChannel(fingerTrackerDesc));
		if (pFingerTracker == NULL)
			return false;
		
		return true;
	}
	
	/** Objects can be constructed only in the Scene context. */
	Robot(golem::Scene &scene);

public:
	golem::VirtualPointTracker* createVirtualPointTracker(golem::Actor *actor);
	
	/** Finger */
	const golem::Finger *getFinger() const {
		return pFinger;
	}
	golem::Finger *getFinger() {
		return pFinger;
	}
	
	/** Finger FT sensor */
	const golem::FTSensor *getFingerFTSensor() const {
		return pFingerFTSensor;
	}
	golem::FTSensor *getFingerFTSensor() {
		return pFingerFTSensor;
	}
	
	/** Finger tracker */
	const golem::PointTracker *getFingerTrackerr() const {
		return pFingerTracker;
	}
	golem::PointTracker *getFingerTracker() {
		return pFingerTracker;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_ROBOT_H_*/
