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

#include "Scenario.h"
#include "Predictor.h"
#include <Golem/Demo/Common/Finger.h>
#include <Golem/Demo/Common/FTSensor.h>
#include <Golem/Demo/Common/PointTracker.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class Robot : public golem::Embodiment, public Performer {
public:	
	/** Object description */
	class Desc : public golem::Embodiment::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Robot, Object::Ptr, Scene&)

	public:
		/** Finger description */
		Finger::Desc fingerDesc;
		
		/** Virtual FT sensor description prototype */
		golem::VirtualFTSensor::Desc virtualFTSensorDesc;
		/** Nano 17 FT sensor description prototype */
		golem::Nano17FTSensor::Desc nano17FTSensorDesc;
		
		/** Virtual object tracker description prototype */
		golem::VirtualPointTracker::Desc virtualTrackerDesc;
		/** Vision object tracker description prototype */
		golem::VisionPointTracker::Desc visionTrackerDesc;
		
		/** Predictor */
		Predictor::Desc predictorDesc;

		/** Constructs Collision description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			Embodiment::Desc::setToDefault();
			fingerDesc.setToDefault();
			fingerDesc.fingerCtrlDesc.pReacPlannerDesc->setToDefault(); // overwrite WrenchTransmitter

			// Initial transmitter pose influence the finger pose
			fingerDesc.fingerCtrlDesc.pReacPlannerDesc->pTransmitterDesc->initSignal.type = golem::Transmitter::Signal::TYPE_WORKSPACE;
			fingerDesc.fingerCtrlDesc.pReacPlannerDesc->pTransmitterDesc->initSignal.gws.pos.R.rotX(-REAL_PI_2);
			fingerDesc.fingerCtrlDesc.pReacPlannerDesc->pTransmitterDesc->initSignal.gws.pos.p.set(Real(0.0), Real(0.25), Real(0.075));
		
			// Disable signal synchronization
			fingerDesc.fingerCtrlDesc.pReacPlannerDesc->signalSync = false;
			
			// turn off planner and collision detection
			fingerDesc.fingerCtrlDesc.pPlannerDesc->pHeuristicDesc->collisionDetection = false;
			
			// turn off renderables
			fingerDesc.fingerCtrlDesc.optimisedPathExShow = false;
			fingerDesc.fingerCtrlDesc.poseShow = false;
			fingerDesc.fingerCtrlDesc.pathShow = false;
			fingerDesc.fingerRendererShow = false;
			
			virtualFTSensorDesc.setToDefault();
			nano17FTSensorDesc.setToDefault();

			virtualTrackerDesc.setToDefault();
			virtualTrackerDesc.planeBoundsDesc.dimensions.set(Real(0.4), Real(0.4), Real(0.002));
			virtualTrackerDesc.planeBoundsDesc.pose.p.set(Real(0.0), Real(0.25), Real(0.0));
			visionTrackerDesc.setToDefault();

			predictorDesc.setToDefault();
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!Embodiment::Desc::isValid())
				return false;
			
			if (!fingerDesc.isValid())
				return false;

			if (!virtualFTSensorDesc.isValid() || !nano17FTSensorDesc.isValid())
				return false;
			if (!virtualTrackerDesc.isValid() || !visionTrackerDesc.isValid())
				return false;
			
			if (!predictorDesc.isValid())
				return false;
			
			return true;
		}
	};

protected:
	/** Creator */
	Creator creator;

	/** Finger */
	Finger *pFinger;

	/** Action */
	golem::shared_ptr<golem::GenWorkspaceState::Seq> pAction;
	
	/** Finger FT sensor */
	golem::FTSensor *pFingerFTSensor;
	/** Finger tracker */
	golem::PointTracker *pFingerTracker;
	/** Playground tracker */
	golem::PointTracker *pPlaygroundTracker;
	/** Polyflap tracker */
	golem::PointTracker *pPolyflapTracker;

	/** Virtual FT sensor description prototype */
	golem::VirtualFTSensor::Desc virtualFTSensorDesc;
	/** Nano 17 FT sensor description prototype */
	golem::Nano17FTSensor::Desc nano17FTSensorDesc;
	
	/** Virtual object tracker description prototype */
	golem::VirtualPointTracker::Desc virtualTrackerDesc;
	/** Vision object tracker description prototype */
	golem::VisionPointTracker::Desc visionTrackerDesc;
	
	/** Predictor */
	Predictor* pPredictor;

	SecTmReal tmBegin;
	Event evAction, evPrediction, evCleanup;
	
	/** (Pre)processing function called BEFORE every physics simulation step and before randering. */
	virtual void preprocess(SecTmReal elapsedTime);

	/** Creates Predictor from description. */
	bool create(const Robot::Desc& desc) {
		Embodiment::create(desc); // throws
		
		pFinger = dynamic_cast<golem::Finger*>(createChannel(desc.fingerDesc));
		if (pFinger == NULL)
			throw Message(Message::LEVEL_CRIT, "Robot::create(): Unable to create Finger");

		pAction.reset();

		virtualFTSensorDesc = desc.virtualFTSensorDesc;
		nano17FTSensorDesc = desc.nano17FTSensorDesc;
		
		virtualTrackerDesc = desc.virtualTrackerDesc;
		visionTrackerDesc = desc.visionTrackerDesc;

		// Finger FT sensor
		golem::VirtualFTSensor::Desc fingerFTSensorDesc = virtualFTSensorDesc;
		fingerFTSensorDesc.joint = &pFinger->getFingerJoint();
		pFingerFTSensor = dynamic_cast<golem::FTSensor*>(createChannel(fingerFTSensorDesc));
		if (pFingerFTSensor == NULL)
			throw Message(Message::LEVEL_CRIT, "Robot::create(): Unable to create FingerFTSensor");

		// Finger tracker
		golem::VirtualPointTracker::Desc fingerTrackerDesc = virtualTrackerDesc;
		fingerTrackerDesc.actor = &pFinger->getFingerActor();
		pFingerTracker = dynamic_cast<golem::PointTracker*>(createChannel(fingerTrackerDesc));
		if (pFingerTracker == NULL)
			throw Message(Message::LEVEL_CRIT, "Robot::create(): Unable to create FingerTracker");
		
		// Playground tracker
		golem::VirtualPointTracker::Desc playgroundTrackerDesc = virtualTrackerDesc;
		playgroundTrackerDesc.actor = dynamic_cast<golem::Actor*>(scene.createObject(*creator.createGroundPlaneDesc()));
		pPlaygroundTracker = dynamic_cast<golem::PointTracker*>(createChannel(playgroundTrackerDesc));
		if (pPlaygroundTracker == NULL)
			throw Message(Message::LEVEL_CRIT, "Robot::create(): Unable to create PlaygroundTracker");

		pPolyflapTracker = NULL;

		pPredictor = dynamic_cast<Predictor*>(scene.createObject(desc.predictorDesc));
		if (pPredictor == NULL)
			throw Message(Message::LEVEL_CRIT, "Robot::create(): Unable to create Predictor");

		evAction.set(false);
		evPrediction.set(false);
		evCleanup.set(false);
		tmBegin = SEC_TM_REAL_ZERO;

		return true;
	}
	
	/** Objects can be constructed only in the Scene context. */
	Robot(golem::Scene &scene);

	/** Initialises the arm */
	void init(const Scenario &scenario, U32 mode);

	/** Runs the predictor */
	void run(const Scenario &scenario, U32 mode);

public:
	/** Runs scenario */
	virtual void run(const Scenario &scenario);

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
	const golem::PointTracker *getFingerTracker() const {
		return pFingerTracker;
	}
	golem::PointTracker *getFingerTracker() {
		return pFingerTracker;
	}

	/** Playground tracker */
	const golem::PointTracker *getPlaygroundTracker() const {
		return pPlaygroundTracker;
	}
	golem::PointTracker *getPlaygroundTracker() {
		return pPlaygroundTracker;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_ROBOT_H_*/
