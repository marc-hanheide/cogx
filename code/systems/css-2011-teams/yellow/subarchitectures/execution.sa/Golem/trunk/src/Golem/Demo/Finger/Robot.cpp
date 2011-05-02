/** @file Robot.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/PhysCtrl/Creator.h>
#include "Finger/Robot.h"

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Robot::Robot(Scene &scene) : Embodiment(scene), rand(scene.getContext().getRandSeed()) {
	dx = dy = dz = 0;
}

void Robot::mouseHandler(int button, int state, int x, int y) {
	dx = 0;
	dy = 0;
	dz += button == 3 ? 1 : button == 4 ? -1 : 0;
	this->button = button;
	this->state = state;
	this->x = x;
	this->y = y;
}

void Robot::motionHandler(int x, int y) {
	if (button == 2) {
		dx += this->x - x;
		dy += this->y - y;
		this->x = x;
		this->y = y;
	}
}

void Robot::keyboardHandler(unsigned char key, int x, int y) {
	static bool bLoF = true;
	static VirtualPointTracker *pTracker = NULL;
	NxReal rot = numeric_const<NxReal>::PI;
	
	switch (key) {
	case 's':
		pFinger->getFingerCtrl().getReacPlanner().stop();
		break;
	case 'r':
		pFinger->getFingerCtrl().getReacPlanner().reset();
		break;
	case 'j':
		bLoF = !bLoF;
		if (bLoF)
			context.getMessageStream()->write("Low friction");
		else
			context.getMessageStream()->write("High friction");
		break;
	case 'o':
		rot = rand.nextUniform(-numeric_const<NxReal>::PI, numeric_const<NxReal>::PI);
		if (pTracker == NULL)
			context.getMessageStream()->write("Random orientation");

	case 't':
		if (pTracker == NULL) {
			{
				CriticalSectionWrapper csw(scene.getUniverse().getCSPhysX()); // Access to PhysX

				if (bLoF) {
					NxMaterial* defaultMaterial = getNxScene()->getMaterialFromIndex(0); 
					defaultMaterial->setStaticFriction((NxReal)0.1);
				}
				else {
					NxMaterial* defaultMaterial = getNxScene()->getMaterialFromIndex(0); 
					defaultMaterial->setStaticFriction((NxReal)0.4);
				}
			}

			Creator creator(scene);
			Actor::Desc *pActorDesc;
			pActorDesc = creator.createBoxDesc(Real(0.07), Real(0.02), Real(0.05));
			//pActorDesc = creator.createSimple2FlapDesc(Real(0.07), Real(0.07), Real(0.07));
			pActorDesc->nxActorDesc.globalPose.t.set(NxReal(0.0), NxReal(0.33), NxReal(0.0));
			pActorDesc->nxActorDesc.globalPose.M.rotZ(rot);
			pTracker = createVirtualPointTracker(dynamic_cast<Actor*>(scene.createObject(*pActorDesc)));
		}
		else {
			Actor *pActor = pTracker->getActor();
			releaseChannel(*pTracker);
			scene.releaseObject(*pActor);
			pTracker = NULL;
		}
		
		break;
	}
}

void Robot::preprocess(SecTmReal elapsedTime) {
	if (dx !=0 || dy != 0 || dz != 0) {
		//printf("(%d, %d, %d)\n", dx, dy, dz);
		
		Wrench ctrlSig((Real)dx, (Real)dz, -(Real)dy, (Real)0.0, (Real)0.0, (Real)0.0);
		ctrlSig.arrayMultiply(ctrlSig, ctrlGain);		
		pFinger->write(&ctrlSig, getContext().getTimer().elapsed());

		dx = dy = dz = 0;
	}
}


VirtualPointTracker* Robot::createVirtualPointTracker(Actor *actor) {
	VirtualPointTracker::Desc virtualTrackerDesc = this->virtualTrackerDesc;
	virtualTrackerDesc.actor = actor;
	return dynamic_cast<VirtualPointTracker*>(createChannel(virtualTrackerDesc));
}

//------------------------------------------------------------------------------
