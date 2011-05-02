/** @file Robot.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include "Prediction/Robot.h"

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Robot::Robot(golem::Scene &scene) : Embodiment(scene), creator(scene) {
}

void Robot::preprocess(SecTmReal elapsedTime) {
	if (!evAction.wait(0))
		return;

	const SecTmReal tmCurrent = context.getTimer().elapsed();
	if (tmBegin <= SEC_TM_REAL_ZERO) {
		tmBegin = tmCurrent;
		evPrediction.set(true);
//context.getMessageStream()->write(Message::LEVEL_DEBUG, "Robot::preprocess(): begin"));
	}
	const SecTmReal tmDuration = pAction->back().t;
	if (tmBegin + tmDuration <= tmCurrent) {
		tmBegin = SEC_TM_REAL_ZERO;
		evAction.set(false);
		evCleanup.set(true);
	}

	// TODO use interpolation
	const U32 items = (U32)pAction->size();
	const U32 index = Math::clamp((U32)Math::round((tmCurrent - tmBegin)*SecTmReal(items - 1)/tmDuration), (U32)0, items - 1);
	
	// Send action
	const GenWorkspaceState *data = &(*pAction)[index];
	pFinger->write(*data, tmCurrent);
}

void Robot::run(const Scenario &scenario, U32 mode) {	
	const bool bOffline = (mode & Scenario::MODE_OFFLINE) > 0;	
	const bool bRecording = mode == Scenario::MODE_OFFLINE;
	const bool bPlayback = bOffline && !bRecording;
	
	// create polyflap reacPlanner
	bool bVirtual = scenario.pPolyflap != NULL;
	Actor *pActor = NULL;
	
	ReacPlanner &reacPlanner = pFinger->getFingerCtrl().getReacPlanner();
	
	// suspend controller
	reacPlanner.stop();
	// wait for completion of the action
	reacPlanner.waitForEnd();
		
	if (!bPlayback) {
		// HACK
		GenConfigspaceCoord rng = reacPlanner.getGlobalRange();
		rng.pos[0] = Real(0.1)*REAL_PI;
		reacPlanner.setGlobalRange(rng);

		// set new init pose
		reacPlanner.send(scenario.pAction->front());
		// wait for completion of the action
		reacPlanner.waitForEnd();
		
		// Action
		pAction = scenario.pAction;
		
		// Create polyflap reacPlanner
		if (bVirtual) {
			{
				CriticalSectionWrapper csw(getScene().getUniverse().getCSPhysX()); // Access to PhysX
				
				// Set simulation parameters
				NxMaterial* defaultMaterial = getNxScene()->getMaterialFromIndex(0); 
				defaultMaterial->setRestitution((NxReal)scenario.restitution);
				defaultMaterial->setStaticFriction((NxReal)scenario.staticFriction);
				defaultMaterial->setDynamicFriction((NxReal)scenario.dynamicFriction);
			}

			scenario.pPolyflap->appearance = pFinger->getFingerActor().getAppearance();
			pActor = dynamic_cast<Actor*>(scene.createObject(*scenario.pPolyflap));
			if (pActor == NULL) {
				context.getMessageStream()->write(Message::LEVEL_ERROR,
					"Robot::run(): unable to create actor"
				);
				return;
			}
			
			// Virtual reacPlanner
			VirtualPointTracker::Desc polyflapTrackerDesc = virtualTrackerDesc;
			polyflapTrackerDesc.actor = pActor;
			pPolyflapTracker = dynamic_cast<PointTracker*>(createChannel(polyflapTrackerDesc));
			if (pPolyflapTracker == NULL) {
				scene.releaseObject(*polyflapTrackerDesc.actor);
				context.getMessageStream()->write(Message::LEVEL_ERROR,
					"Robot::run(): unable to create virtual reacPlanner"
				);
				return;
			}
		}
		else {
			// Vision reacPlanner
			VisionPointTracker::Desc polyflapTrackerDesc = visionTrackerDesc;
			pPolyflapTracker = dynamic_cast<PointTracker*>(createChannel(polyflapTrackerDesc));
			if (pPolyflapTracker == NULL) {
				context.getMessageStream()->write(Message::LEVEL_ERROR,
					"Robot::run(): unable to create vision reacPlanner"
				);
				return;
			}
		}
	}
	
	// re-initialise other trackers
	pFingerTracker->setPointsDensity(pFingerTracker->getPointsDensity());
	pPlaygroundTracker->setPointsDensity(pPlaygroundTracker->getPointsDensity());
	
	// Setup predictor
	pPredictor->setRunTime(scenario.tmRunTime);
	pPredictor->setArm(&reacPlanner.getArm());
	pPredictor->setAction(scenario.pAction);
	pPredictor->setGenerators(scenario.generators);
	pPredictor->setFingerFTSensor(pFingerFTSensor);
	pPredictor->setFingerTracker(pFingerTracker);
	pPredictor->setPlaygroundTracker(pPlaygroundTracker);
	pPredictor->setPolyflapTracker(pPolyflapTracker);

	// Initialise predictor
	if (pPredictor->init(mode)) {
		PerfTimer::sleep(scenario.tmOffsetBegin);
		
		if (!bPlayback) {
			evAction.set(true);
			if (evPrediction.wait()) {
				// Run predictor
				pPredictor->run(mode);
				
				if (!evCleanup.wait()) {
					context.getMessageStream()->write(Message::LEVEL_ERROR,
						"Robot::run(): Event cleanup timeout"
					);
				}
				evCleanup.set(false);
			}
			else {
				context.getMessageStream()->write(Message::LEVEL_ERROR,
					"Robot::run(): Event prediction timeout"
				);		
			}
			evPrediction.set(false);
		}
		else {
			// Run predictor
			pPredictor->run(mode);
			
		}

		PerfTimer::sleep(scenario.tmOffsetEnd);
		// Clean up predictor
		pPredictor->cleanup();
	}
	else {
		context.getMessageStream()->write(Message::LEVEL_ERROR,
			"Robot::run(): unable to initialise predictor"
		);
	}

	if (!bPlayback) {
		// release polyflap reacPlanner
		if (bVirtual) {
			releaseChannel(*pPolyflapTracker);			
			scene.releaseObject(*pActor);
		}
		else {
			// Vision reacPlanner
			releaseChannel(*pPolyflapTracker);
		}
	}
}


void Robot::run(const Scenario &scenario) {
	if (scenario.mode & Scenario::MODE_OFFLINE) {
		context.getMessageStream()->write(Message::LEVEL_INFO,
			"Recording..."
		);
		run(scenario, Scenario::MODE_OFFLINE);
	}
	if (scenario.mode & Scenario::MODE_FILTERING) {
		context.getMessageStream()->write(Message::LEVEL_INFO,
			"Filtering..."
		);
		run(scenario, scenario.mode ^ Scenario::MODE_SIMULATION);
	}
	if (scenario.mode & Scenario::MODE_SIMULATION) {
		context.getMessageStream()->write(Message::LEVEL_INFO,
			"Simulating..."
		);
		run(scenario, scenario.mode ^ Scenario::MODE_FILTERING);
	}
}

//------------------------------------------------------------------------------
