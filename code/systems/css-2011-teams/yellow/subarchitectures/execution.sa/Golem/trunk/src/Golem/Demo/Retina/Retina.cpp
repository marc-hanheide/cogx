/** @file Retina.cpp
 * 
 * Program demonstrates 3D artificial retina.
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
#include <Golem/Phys/Data.h>
#include <Golem/PhysCtrl/Creator.h>
#include <Golem/Demo/Common/Tools.h>
#include <Golem/Demo/Common/Data.h>
#include "Retina/Robot.h"

using namespace golem;

//------------------------------------------------------------------------------

void setupObjects(Scene* scene, Robot* robot) {
	// Creator
	Creator creator(*scene);
	Actor::Desc *pActorDesc;
	RetinaObject::Desc retinaObjectDesc;
	
	// Create ground plane.
	pActorDesc = creator.createGroundPlaneDesc();
	retinaObjectDesc.actor = dynamic_cast<Actor*>(scene->createObject(*pActorDesc));
	retinaObjectDesc.rigidBodyRendererDesc.bodyColour = RGBA::YELLOW;
	robot->createRetinaObject(retinaObjectDesc);
	
	//// box
	pActorDesc = creator.createBoxDesc(Real(0.05), Real(0.05), Real(0.05));
	pActorDesc->nxActorDesc.globalPose.t.set(NxReal(-0.06), NxReal(0.3), NxReal(0.051));
	retinaObjectDesc.actor = dynamic_cast<Actor*>(scene->createObject(*pActorDesc));
	retinaObjectDesc.rigidBodyRendererDesc.bodyColour = RGBA::RED;
	robot->createRetinaObject(retinaObjectDesc);

	// polyflap
	//pActorDesc = creator.createSimple2FlapDesc(Real(0.07), Real(0.07), Real(0.07));
	//pActorDesc->nxActorDesc.globalPose.t.set(NxReal(0.0), NxReal(0.33), NxReal(0.0));
	//pActorDesc->nxActorDesc.globalPose.M.rotZ((NxReal)MATH_PI);
	//retinaObjectDesc.actor = dynamic_cast<Actor*>(scene->createObject(*pActorDesc));
	//retinaObjectDesc.rigidBodyRendererDesc.bodyColour = RGBA::CYAN;
	//robot->createRetinaObject(retinaObjectDesc);
}

//------------------------------------------------------------------------------

void setupRobot(Robot::Desc &desc, XMLContext* xmlcontext, golem::Context* context, golem::Universe* universe) {
	dynamic_cast<WrenchTransmitter::Desc*>(desc.fingerDesc.fingerCtrlDesc.pReacPlannerDesc->pTransmitterDesc.get())->pUniverse = universe;
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

		// Get arm driver name
		std::string driver;
		XMLData("driver", driver, xmlcontext()->getContextFirst("arm"));
		// Load driver
		Robot::Desc robotDesc;
		robotDesc.fingerDesc.fingerCtrlDesc.pArmDesc = Arm::Desc::load(*context(), driver);
		// Retina
		XMLData(robotDesc.retinaDesc, xmlcontext()->getContextFirst("retina"));
		setupRobot(robotDesc, xmlcontext(), context(), universe());
		
		// Create Robot embodiment
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Initialising retina...");
		Robot *robot = dynamic_cast<Robot*>(scene()->createObject(robotDesc));
		if (robot == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create arm controller");
		
		Image3::Idx3 dim = robot->getRetina()->getReceptiveFieldNum();
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Retina dimensions: %d x %d x %d", dim.n1, dim.n2, dim.n3);

		// get reactive planner
		ReacPlanner& reacPlanner = robot->getFinger()->getFingerCtrl().getReacPlanner();
		
		// Display arm information
		armInfo(reacPlanner.getArm());

		// Setup objects
		setupObjects(scene(), robot);

		// set velocity and acceleration limits
		reacPlanner.getPlanner().setVelocity(Real(1.0));
		reacPlanner.getPlanner().setAcceleration(Real(1.0));
		
		// go to the initial pose
		reacPlanner.reset();

		U32 n = 0;
		PerfTimer t0;
		
		// Recognition vortex arrays store retina image
		golem::RetinaOut out;
		while (!universe()->interrupted()) {
			PerfTimer t1;
			
			// just compute retinas image
			SecTmReal timeStamp;
			if (!robot->read(out, timeStamp))
				continue;
			
			printf("%.3f (%.3f) FPS  \r", SecTmReal(1.0)/t1.elapsed(), SecTmReal(++n)/t0.elapsed());
		}

		context()->getMessageStream()->write(Message::LEVEL_INFO, "Good bye!           ");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
