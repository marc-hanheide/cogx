/** @file RigidBodyTracker.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Demo/Common/RigidBodyTracker.h>
#include <Golem/Demo/Common/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

bool RigidBodyRenderer::create(const Desc &desc) {
	if (!desc.isValid())
		return false;

	// reserve memory buffer
	reset();

	return true;
}

//------------------------------------------------------------------------------

RigidBodyTracker::RigidBodyTracker(Embodiment &embodiment) :
	Base(embodiment)
{
	pRigidBodyRenderer.reset(new RigidBodyRenderer());
}

RigidBodyTracker::~RigidBodyTracker() {
}

bool RigidBodyTracker::create(const RigidBodyTracker::Desc& desc) {
	Base::create(desc); // throws

	rigidBodyRendererDesc = desc.rigidBodyRendererDesc;
	rigidBodyRendererDesc.pRigidBodyData = &rigidBodyData;
	rigidBodyShow = desc.rigidBodyShow;
	
	timeStamp = SEC_TM_REAL_ZERO;
	if (rigidBodyData.pBoundsSeq != NULL)
		rigidBodyData.pBoundsSeq->clear();
	evWait.set(false);
	
	return true;
}

bool RigidBodyTracker::read(RigidBodyData &rigidBodyData, SecTmReal &timeStamp, MSecTmU32 timeOut) {
	if (!evWait.wait(timeOut))
		return false;

	CriticalSectionWrapper csw(cs);
	
	timeStamp = this->timeStamp;
	rigidBodyData = this->rigidBodyData;

	evWait.set(false);
	return true;
}

void RigidBodyTracker::postprocess(SecTmReal elapsedTime) {
	CriticalSectionWrapper csw(cs);
	
	if (!get(rigidBodyData, timeStamp))
		return;
	evWait.set(true);

	if (rigidBodyShow)
		pRigidBodyRenderer->create(rigidBodyRendererDesc);
}

void RigidBodyTracker::render() {
	// postprocess() and render() are always called from the same thread 
	CriticalSectionWrapper csw(cs);
	
	if (rigidBodyShow)
		pRigidBodyRenderer->render();
}

void RigidBodyTracker::keyboardHandler(unsigned char key, int x, int y) {
	if (key == 5) {// F5
		CriticalSectionWrapper csw(cs);
		
		rigidBodyShow = !rigidBodyShow;
		if (rigidBodyShow)
			pRigidBodyRenderer->create(rigidBodyRendererDesc);
	}
}

//------------------------------------------------------------------------------

VirtualRigidBodyTracker::VirtualRigidBodyTracker(Embodiment &embodiment) : RigidBodyTracker(embodiment) {
	actor = NULL;
}

VirtualRigidBodyTracker::~VirtualRigidBodyTracker() {
}

bool VirtualRigidBodyTracker::create(const VirtualRigidBodyTracker::Desc& desc) {
	RigidBodyTracker::create(desc); // throws

	actor = desc.actor;
	actorColour = desc.actorColour;
	return true;
}

bool VirtualRigidBodyTracker::get(RigidBodyData &rigidBodyData, SecTmReal &timeStamp) {
	if (actor == NULL)
		return true;
	
	// object bounds
	rigidBodyData.pBoundsSeq = actor->getGlobalBoundsSeq();
	// Object colour
	rigidBodyData.colour = actorColour ? actor->getAppearance().solidColour : rigidBodyRendererDesc.bodyColour;
	
	// current time stamp
	timeStamp = context.getTimer().elapsed();

	return true;
}

void VirtualRigidBodyTracker::setActor(Actor *actor) {
	CriticalSectionWrapper csw(cs);
	this->actor = actor;
}

//------------------------------------------------------------------------------

VisionRigidBodyTracker::VisionRigidBodyTracker(Embodiment &embodiment) : RigidBodyTracker(embodiment) {
}

VisionRigidBodyTracker::~VisionRigidBodyTracker() {
}

bool VisionRigidBodyTracker::create(const VisionRigidBodyTracker::Desc& desc) {
	RigidBodyTracker::create(desc); // throws

	// TODO initialise vision device
	
	return true;
}

bool VisionRigidBodyTracker::get(RigidBodyData &rigidBodyData, SecTmReal &timeStamp) {
	// TODO read points poses
	return false;
	
	//return true;
}

//------------------------------------------------------------------------------
