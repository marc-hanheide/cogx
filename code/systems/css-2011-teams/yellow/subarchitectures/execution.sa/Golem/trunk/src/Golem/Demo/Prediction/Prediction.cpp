/** @file Predictor.cpp
 * 
 * Program demonstrates simplified-physics prediction.
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the Katana arm simulator
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/XMLData.h>
#include <Golem/Phys/Application.h>
#include <Golem/Demo/Common/Tools.h>
#include "Prediction/Robot.h"
#include <iostream>
#include <sstream>

//#define _CRT_SECURE_NO_WARNINGS 1

using namespace golem;

//------------------------------------------------------------------------------

/** Virtual scenario setup */
void setupVirtualSenarios(Experiment &experiment) {
	// Setup action
	WorkspaceCoord begin;
	begin.p.set(Real(0.0), Real(0.15), Real(0.1));
	begin.R.rotX(-REAL_PI_2);// rotate finger down around X axis
	
	WorkspaceCoord end;
	end.p.set(Real(0.0), Real(0.4), Real(0.1));
	end.R.rotX(-REAL_PI_2);// rotate finger down around X axis

	Polynomial4 trajectory;
	trajectory.create(Polynomial4::Desc());

	const SecTmReal duration = SecTmReal(20.0);
	const SecTmReal delta = SecTmReal(0.02);

	golem::shared_ptr<golem::GenWorkspaceState::Seq> pAction = experiment.createP2PTrajectory(begin, end, trajectory, duration, delta);
	
	// Setup scenarios
	Scenario *pScenario;
	experiment.scenarios.clear();
	experiment.randomRuns = true;
	experiment.numOfRuns = 100;

	U32 scenarioID = 0;
	
	const U32 variations = 10;
	const Real linVar = Real(0.01);
	const Real angVar = Real(0.02)*REAL_PI;
	
	// Scenario "Symmetric polyflap"
	for (U32 i = 0; i < variations; i++) {
		experiment.scenarios.push_back(Scenario());
		pScenario = &experiment.scenarios.back();
		
		// Run time
		pScenario->tmRunTime = duration;

		// Action
		pScenario->pAction = pAction;

		// Polyflap
		const Real size = Real(0.075);
		pScenario->pPolyflap = experiment.createSimple2FlapDesc(size, size, size);
		
		const Real x = Real(0.0) + experiment.rand.nextUniform(-linVar, +linVar);
		const Real y = Real(0.25);// + experiment.rand.nextUniform(-linVar, +linVar);
		const Real z = Real(0.0);
		pScenario->pPolyflap->nxActorDesc.globalPose.t.set(NxReal(x), NxReal(y), NxReal(z));

		const Real angle = REAL_PI + experiment.rand.nextUniform(-angVar, +angVar);
		pScenario->pPolyflap->nxActorDesc.globalPose.M.rotZ(NxReal(angle));

		//generators
		Mat33 R;
		
		Generator genSpin;
		genSpin.weight = Real(0.5);
		genSpin.dirLinVar = Real(0.2);
		genSpin.dirAngVar = Real(0.001);
		genSpin.anchor.set(Real(0.0), y, Real(0.0));
		genSpin.axis.set(Real(0.0), Real(0.0), Real(1.0));
		R.fromAngleAxis(angle, genSpin.axis);
		R.multiply(genSpin.dir, Vec3(Real(0.0), -Real(1.0), Real(0.0)));
		pScenario->generators.push_back(genSpin);

		Generator genFlip;
		genFlip.weight = Real(0.5);
		genFlip.dirLinVar = Real(0.001);
		genFlip.dirAngVar = Real(0.1);
		genFlip.anchor.set(Real(0.0), y, Real(0.0));
		R.fromAngleAxis(angle, Vec3(Real(0.0), Real(0.0), Real(1.0)));
		R.multiply(genFlip.axis, Vec3(Real(1.0), Real(0.0), Real(0.0)));
		R.multiply(genFlip.dir, Vec3(Real(0.0), -Real(1.0), Real(0.0)));
		pScenario->generators.push_back(genFlip);
		
		// Friction coefficients
		pScenario->staticFriction = 0.2;
		pScenario->dynamicFriction = 0.03;
		
		std::stringstream str;
		str << "Symmetric polyflap {behaviour: forward movement}, {pose: front}, {friction: 0.2, 0.03} #" << ++scenarioID;
		pScenario->name = str.rdbuf()->str();
	}
	
	// Scenario "Symmetric polyflap"
	for (U32 i = 0; i < variations; i++) {
		experiment.scenarios.push_back(Scenario());
		pScenario = &experiment.scenarios.back();
		
		// Run time
		pScenario->tmRunTime = duration;
		
		// Action
		pScenario->pAction = pAction;

		// Polyflap
		const Real size = Real(0.064);
		pScenario->pPolyflap = experiment.createSimple2FlapDesc(size, size, size);
		
		const Real x = Real(0.0) + experiment.rand.nextUniform(-linVar, +linVar);
		const Real y = Real(0.25);// + experiment.rand.nextUniform(-linVar, +linVar);
		const Real z = Real(0.0);
		pScenario->pPolyflap->nxActorDesc.globalPose.t.set(NxReal(x), NxReal(y), NxReal(z));

		const Real angle = REAL_PI + experiment.rand.nextUniform(-angVar, +angVar);
		pScenario->pPolyflap->nxActorDesc.globalPose.M.rotZ(NxReal(angle));

		//generators
		Mat33 R;
		
		Generator genSpin;
		genSpin.weight = Real(0.5);
		genSpin.dirLinVar = Real(0.2);
		genSpin.dirAngVar = Real(0.001);
		genSpin.anchor.set(Real(0.0), y, Real(0.0));
		genSpin.axis.set(Real(0.0), Real(0.0), Real(1.0));
		R.fromAngleAxis(angle, genSpin.axis);
		R.multiply(genSpin.dir, Vec3(Real(0.0), -Real(1.0), Real(0.0)));
		pScenario->generators.push_back(genSpin);

		Generator genFlip;
		genFlip.weight = Real(0.5);
		genFlip.dirLinVar = Real(0.001);
		genFlip.dirAngVar = Real(0.1);
		genFlip.anchor.set(Real(0.0), y, Real(0.0));
		R.fromAngleAxis(angle, Vec3(Real(0.0), Real(0.0), Real(1.0)));
		R.multiply(genFlip.axis, Vec3(Real(1.0), Real(0.0), Real(0.0)));
		R.multiply(genFlip.dir, Vec3(Real(0.0), -Real(1.0), Real(0.0)));
		pScenario->generators.push_back(genFlip);
		
		// Friction coefficients
		pScenario->staticFriction = 0.2;
		pScenario->dynamicFriction = 0.03;
		
		std::stringstream str;
		str << "Symmetric polyflap {behaviour: tip over}, {pose: front}, {friction: 0.2, 0.03} #" << ++scenarioID;
		pScenario->name = str.rdbuf()->str();
	}
	
	// Scenario "Symmetric polyflap"
	//for (U32 i = 0; i < variations; i++) {
	//	experiment.scenarios.push_back(Scenario());
	//	pScenario = &experiment.scenarios.back();
	//	
	//	pScenario->pAction = pAction;
	//	pScenario->pPolyflap = experiment.createSimple2FlapDesc(Real(0.07), Real(0.07), Real(0.07));
	//	pScenario->pPolyflap->nxActorDesc.globalPose.t.set(
	//		NxReal(0.0) + NxReal(experiment.rand.nextUniform(-linVar, +linVar)),
	//		NxReal(0.25),// + NxReal(experiment.rand.nextUniform(-linVar, +linVar)),
	//		NxReal(0.0)
	//	);
	//	pScenario->pPolyflap->nxActorDesc.globalPose.M.rotZ(
	//		(NxReal)NxReal(experiment.rand.nextUniform(-angVar, +angVar))
	//	);
	//	pScenario->staticFriction = 0.3;
	//	pScenario->dynamicFriction = 0.03;
	//	
	//	std::stringstream str;
	//	str << "Symmetric polyflap {pose: back}, {friction: 0.3, 0.03} #" << ++scenarioID;
	//	pScenario->name = str.rdbuf()->str();
	//}
}

/** Real scenario setup */
void setupRealSenarios(Experiment &experiment) {
	// Setup action
	WorkspaceCoord begin;
	begin.p.set(Real(0.0), Real(0.15), Real(0.1));
	begin.R.rotX(-REAL_PI_2);// rotate finger down around X axis
	
	WorkspaceCoord end;
	end.p.set(Real(0.0), Real(0.4), Real(0.1));
	end.R.rotX(-REAL_PI_2);// rotate finger down around X axis

	Polynomial4 trajectory;
	trajectory.create(Polynomial4::Desc());

	const SecTmReal duration = SecTmReal(20.0);
	const SecTmReal delta = SecTmReal(0.02);

	golem::shared_ptr<golem::GenWorkspaceState::Seq> pAction = experiment.createP2PTrajectory(begin, end, trajectory, duration, delta);
	
	// Setup scenarios
	Scenario *pScenario;
	experiment.scenarios.clear();
	experiment.randomRuns = false;
	experiment.numOfRuns = 1;
	
	// Scenario "Symmetric polyflap"
	experiment.scenarios.push_back(Scenario());
	pScenario = &experiment.scenarios.back();
	pScenario->name = "Generic object";
	pScenario->tmRunTime = duration;
	pScenario->pAction = pAction;
	pScenario->pPolyflap = NULL; // from vision system
}

//------------------------------------------------------------------------------

/** MyApplication */
class MyApplication : public Application {
protected:
	/** Runs MyApplication */
	virtual void run(int argc, char *argv[]) {
		printf("Use the arrow keys to move the camera.\n");
		printf("Use the mouse and mouse left button to rotate the camera.\n");
		printf("Use the mouse, mouse right button and wheel to move the arm.\n");
		printf("Press p to pause simulations.\n");
		printf("Press pgup/pgdn/space to switch between simulations.\n");
		printf("Press v to show Actors reference frames.\n");
		printf("Use z, x, c to change randering mode.\n");
		printf("Use F1-F12 to display program specific debug information.\n");
		printf("Press esc to exit.\n");
	
		//-----------------------------------------------------------------------------

		// Create experiment
		Experiment experiment(*scene());

		// Get arm driver name
		std::string driver;
		XMLData("driver", driver, xmlcontext()->getContextFirst("arm"));
		// Load driver and setup planner
		Robot::Desc robotDesc;
		robotDesc.fingerDesc.fingerCtrlDesc.pArmDesc = Arm::Desc::load(*context(), driver);

		// setup scenarios
		setupVirtualSenarios(experiment);
		//setupRealSenarios(experiment);

		// Create Robot embodiment
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Initialising finger...");
		Robot *pRobot = dynamic_cast<Robot*>(scene()->createObject(robotDesc));
		if (pRobot == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create finger");
		
		// get reactive planner
		ReacPlanner& reacPlanner = pRobot->getFinger()->getFingerCtrl().getReacPlanner();
		
		// Run experiment
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Running experiment...");
		experiment.run(*pRobot);
		
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Good bye!");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
