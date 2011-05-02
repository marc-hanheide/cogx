/** @file Robot.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include "Retina/Robot.h"
#include <Golem/PhysCtrl/Creator.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Robot::Robot(Scene &scene) : Embodiment(scene) {
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
	switch (key) {
	case 's':
		pFinger->getFingerCtrl().getReacPlanner().stop();
		break;
	case 'r':
		pFinger->getFingerCtrl().getReacPlanner().reset();
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

//------------------------------------------------------------------------------

RetinaObject* Robot::createRetinaObject(const RetinaObject::Desc& desc) {
	RetinaObject* pRetinaObject = dynamic_cast<RetinaObject*>(createChannel(desc));
	if (pRetinaObject == NULL)
		throw Message(Message::LEVEL_CRIT, "Robot::createRetinaObject(): Unable to create retina object");

	retinaObjectList.insert(pRetinaObject);
	return pRetinaObject;
}

void Robot::releaseRetinaObject(RetinaObject& retinaObject) {
	RetinaObjectList::iterator i = retinaObjectList.find(&retinaObject);
	if (i != retinaObjectList.end()) {
		retinaObjectList.erase(i);
		releaseChannel(retinaObject);
	}
}

//------------------------------------------------------------------------------

bool Robot::read(golem::RetinaOut &out, SecTmReal &timeStamp, MSecTmU32 timeOut) {
	golem::RetinaInp inp;
	
	for (RetinaObjectList::const_iterator i = retinaObjectList.begin(); i != retinaObjectList.end(); i++) {
		inp.objects.push_back(RigidBodyData());
		(*i)->read(inp.objects.back(), timeStamp);
	}

	return pRetina->process(out, inp, timeOut);
}

//------------------------------------------------------------------------------
