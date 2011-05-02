/** @file Scenes.cpp
 * 
 * Demonstration program which creates multiple arms in multiple scenes.
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm (it's dangerous - no collision detection!)
 * - the second simply runs a specified number of arm simulators
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Phys/Application.h>
#include <Golem/Phys/Data.h>
#include <Golem/PhysCtrl/PhysArm.h>
#include <Golem/PhysCtrl/Creator.h>
#include <Golem/Demo/Common/Tools.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Device/Katana300Sim/Katana300Sim.h>
#include <Golem/Device/Katana300Sim/Data.h>
#include <Golem/Device/SixAxisSim/SixAxisSim.h>
#include <Golem/Device/SixAxisSim/Data.h>
#include <iostream>
#include <sstream>

using namespace golem;

//------------------------------------------------------------------------------

void setupObjects(Scene &scene) {
	static Rand rand(scene.getContext().getRandSeed());

	// Creator
	Creator creator(scene);
	Actor::Desc *pActorDesc;
	
	// Create ground plane.
	pActorDesc = creator.createGroundPlaneDesc();
	scene.createObject(*pActorDesc);
	
	// add some objects!
	const U32 numOfObject = 10;
	for (U32 i = 0; i < numOfObject; i++) {
		pActorDesc = creator.createTreeDesc(rand.nextUniform(Real(0.07), Real(0.10)));
		pActorDesc->nxActorDesc.globalPose.t.set(
			(NxReal)rand.nextUniform(Real(-0.3), Real(0.3)),
			(NxReal)rand.nextUniform(Real(-0.3), Real(0.3)),
			(NxReal)rand.nextUniform(Real(+0.3), Real(0.9))
		);
		scene.createObject(*pActorDesc);
	}
}

//------------------------------------------------------------------------------

/** MyApplication */
class MyApplication : public Application {
protected:
	/** Runs MyApplication */
	virtual void run(int argc, char *argv[]) {
		printf("Use the arrow keys to move the camera.\n");
		printf("Use the mouse to rotate the camera.\n");
		printf("Press p to pause simulations.\n");
		printf("Press pgup/pgdn/space to switch between simulations.\n");
		printf("Press v to show Actors reference frames.\n");
		printf("Use z, x, c to change randering mode.\n");
		printf("Use F1-F12 to display program specific debug information.\n");
		printf("\tF1 to display/hide the current destination pose.\n");
		printf("\tF2 to display/hide the current trajectory.\n");
		printf("Press esc to exit.\n");
	
		// Do not display LEVEL_DEBUG messages (only with level at least LEVEL_INFO)
		//context()->getLogger()->setMsgFilter(MessageFilter::Ptr(new LevelFilter<Message>(Message::LEVEL_INFO)));

		//-----------------------------------------------------------------------------

		U32 numOfSim = 2;
		XMLData("num_scenes", numOfSim, xmlcontext()->getContextFirst("universe"));
		// Number of arm controllers
		const U32 numOfArms = numOfSim*(numOfSim + 1)/2;
		// Arm controllers
		golem::shared_ptr< PhysArm*, golem::arr_cnt<PhysArm*> > arms(new PhysArm* [numOfArms]);
	
		// Create simulations
		for (U32 i = 0; i < numOfSim; i++) {			
			// Create scene
			Scene::Desc sceneDesc;
			XMLData(sceneDesc, xmlcontext()->getContextFirst("scene"));
			std::stringstream name;
			name << "Scene #" << i + 1;
			sceneDesc.name = name.str();
			Scene *pScene = i == 0 ? scene() : universe()->createScene(sceneDesc); // use already created scene
			
			// Scene objects setup
			setupObjects(*pScene);

			for (U32 j = 0; j <= i ; j++) {
				U32 n = i*(i + 1)/2 + j;

				// Create arm controller description
				PhysArm::Desc physArmDesc;
				if (n%2) {
					physArmDesc.pArmDesc = Arm::Desc::load(*context(), "GolemDeviceSixAxisSim");
				}
				else {
					physArmDesc.pArmDesc = Arm::Desc::load(*context(), "GolemDeviceKatana300Sim");
				}

				physArmDesc.pArmDesc->globalPose.p.set(Real(-1.0 * j), Real(-1.0 * j), Real(0.0));

				// Create PhysArm controller
				context()->getMessageStream()->write(Message::LEVEL_INFO, "Initialising arm controller #%d...", n + 1);
				arms[n] = dynamic_cast<PhysArm*>(pScene->createObject(physArmDesc));
				
				// Display arm information
				armInfo(arms[n]->getArm());
			}
		}

		// Move joints to a random pose
		while (!universe()->interrupted()) {
			for (U32 i = 0; i < numOfArms; i++)
				armRandMove(arms[i]->getArm(), Real(0.5), Real(0.5));
			
			// Wait for some time to give the arm a chance to move
			PerfTimer::sleep(2.0);

			//for (U32 i = 0; i < numOfArms; i++)
			//	armStop(arms[i]->getArm());
			//PerfTimer::sleep(1.0);
		}

		context()->getMessageStream()->write(Message::LEVEL_INFO, "Good bye!");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
