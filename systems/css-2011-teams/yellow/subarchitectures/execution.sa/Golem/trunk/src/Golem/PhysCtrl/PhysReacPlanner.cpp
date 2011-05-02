/** @file PhysReacPlanner.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/PhysCtrl/PhysReacPlanner.h>
#include <Golem/PhysCtrl/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

ReacPlannerRenderer::ReacPlannerRenderer(PhysReacPlanner &physReacPlanner) :
	physReacPlanner(physReacPlanner)
{}

bool ReacPlannerRenderer::create(const Desc &desc) {
	if (!desc.isValid())
		return false;

	// reserve memory buffer
	reset();
	if (desc.desiredStateShow)
		reserveAxesInc(2);

	if (desc.desiredStateShow) {
		Transmitter::Signal signal;
		(void)physReacPlanner.getReacPlanner().getTransmitter().get(signal);

		const Mat34 referencePose = physReacPlanner.getArm().getReferencePose();

		if (signal.type == Transmitter::Signal::TYPE_JOINTSPACE) {
			Mat34 pose;
			physReacPlanner.getArm().forwardTransform(pose, signal.gjs.pos);
			pose.multiply(pose, referencePose); // reference pose
			addAxes(pose, desc.desiredPoseSize);

			// TODO velocity
		}
		else {
			addAxes(signal.gws.pos, desc.desiredPoseSize);

			const Real theta = signal.gws.vel.w.normalise();
			Mat34 pose;
			pose.fromTwist(signal.gws.vel, theta);
			pose.multiply(signal.gws.pos, pose);
			
			addAxes(pose, desc.desiredPoseSize);
		}
	}

	return true;
}

//------------------------------------------------------------------------------

PhysReacPlanner::PhysReacPlanner(Scene &scene) : PhysPlanner(scene) {
	pReacPlannerRenderer.reset(new ReacPlannerRenderer(*this));
}

PhysReacPlanner::~PhysReacPlanner() {
}

//------------------------------------------------------------------------------

bool PhysReacPlanner::create(const Desc& desc) {
	PhysPlanner::create(desc); // throws
	
	reacPlannerRendererDesc = desc.reacPlannerRendererDesc;
	reacPlannerShow = desc.reacPlannerShow;
	
	pReacPlanner = desc.pReacPlannerDesc->create(*pPlanner); // throws

	return true;
}

void PhysReacPlanner::release() {
	pReacPlanner.release();
	PhysPlanner::release();
}

//------------------------------------------------------------------------------

void PhysReacPlanner::keyboardHandler(unsigned char key, int x, int y) {
	PhysPlanner::keyboardHandler(key, x ,y);

	switch (key) {
	case 9:// F9
		reacPlannerShow = !reacPlannerShow;
		break;
	}
}

void PhysReacPlanner::postprocess(SecTmReal elapsedTime) {
	PhysPlanner::postprocess(elapsedTime);

	// PhysReacPlanner renderer
	if (reacPlannerShow && !pReacPlannerRenderer->create(reacPlannerRendererDesc))
		context.getMessageStream()->write(Message::LEVEL_WARNING,"PhysReacPlanner::postprocess(): Unable to create reactive planner renderer");
}

void PhysReacPlanner::render() {
	PhysPlanner::render();

	if (reacPlannerShow)
		pReacPlannerRenderer->render();
}

//------------------------------------------------------------------------------
