/** @file Finger.cpp
 * 
 * Program demonstrates a robotic finger which motion is controlled by mouse.
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
#include <Golem/PhysCtrl/Creator.h>
#include <Golem/Demo/Common/Tools.h>
#include "Finger/Robot.h"
#include <iostream>

using namespace golem;

//------------------------------------------------------------------------------

void setupObjects(Scene &scene, Robot &robotFinger) {
	// Creator
	Creator creator(scene);
	Actor::Desc *pActorDesc;
	
	// Create ground plane.
	pActorDesc = creator.createGroundPlaneDesc();
	//scene.createObject(*pActorDesc);
	robotFinger.createVirtualPointTracker(dynamic_cast<Actor*>(scene.createObject(*pActorDesc)));
	
	//// box
	//pActorDesc = creator.createBoxDesc(Real(0.5), Real(0.5), Real(0.5));
	//pActorDesc->nxActorDesc.globalPose.t.set(NxReal(-0.4), NxReal(0.75), NxReal(0.5));
	////scene.createObject(*pActorDesc);
	//robotFinger.createVirtualPointTracker(dynamic_cast<Actor*>(scene.createObject(*pActorDesc)));

	// polyflap
	//pActorDesc = creator.createSimple2FlapDesc(Real(0.07), Real(0.07), Real(0.07));
	//pActorDesc->nxActorDesc.globalPose.t.set(NxReal(0.0), NxReal(0.33), NxReal(0.0));
	//pActorDesc->nxActorDesc.globalPose.M.rotZ((NxReal)MATH_PI);
	////scene.createObject(*pActorDesc);
	//robotFinger.createVirtualPointTracker(dynamic_cast<Actor*>(scene.createObject(*pActorDesc)));
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

		// Get arm driver name
		std::string driver;
		XMLData("driver", driver, xmlcontext()->getContextFirst("arm"));
		// Load driver
		Robot::Desc robotDesc;
		robotDesc.fingerDesc.fingerCtrlDesc.pArmDesc = Arm::Desc::load(*context(), driver);

		// Setup pointer to Universe in the WrenchTransmitter
		dynamic_cast<WrenchTransmitter::Desc*>(robotDesc.fingerDesc.fingerCtrlDesc.pReacPlannerDesc->pTransmitterDesc.get())->pUniverse = universe();
		
		// Create Robot embodiment
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Initialising finger...");
		Robot *pRobot = dynamic_cast<Robot*>(scene()->createObject(robotDesc));
		if (pRobot == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create finger");
		
		// get reactive planner
		ReacPlanner& reacPlanner = pRobot->getFinger()->getFingerCtrl().getReacPlanner();
		
		// Display arm information
		armInfo(reacPlanner.getArm());

		// Setup objects
		setupObjects(*scene(), *pRobot);

		// set velocity and acceleration limits
		reacPlanner.getPlanner().setVelocity(Real(1.0));
		reacPlanner.getPlanner().setAcceleration(Real(0.5));
		
		// go to the initial pose
		reacPlanner.reset();

		// Move joints to a random pose
		while (!universe()->interrupted()) {
			// the main thread has not much to do in this example
			PerfTimer::sleep(0.1);
		}

		context()->getMessageStream()->write(Message::LEVEL_INFO, "Good bye!");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
