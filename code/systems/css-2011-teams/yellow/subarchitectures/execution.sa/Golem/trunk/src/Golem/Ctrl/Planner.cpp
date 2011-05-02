/** @file Planner.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Ctrl/Planner.h>
#include <Golem/Ctrl/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Planner::Planner(golem::Arm &arm) :
	arm(arm), context(arm.getContext()),
	rand(context.getRandSeed())
{}

Planner::~Planner() {
}

bool Planner::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgPlannerInvalidDesc(Message::LEVEL_CRIT, "Planner::create(): Invalid description");

	velocity = desc.velocity;
	acceleration = desc.acceleration;
	pProfile = desc.pProfileDesc->create(); // throws
	pHeuristic = desc.pHeuristicDesc->create(arm); // throws
	pCallback = NULL;
	return true;
}

//------------------------------------------------------------------------------

bool Planner::stop() {
	// The soonest possible trajectory end
	const SecTmReal tmAsync = arm.getContext().getTimer().elapsed() + arm.getTimeDeltaAsync();
	GenConfigspaceState s[2];
	
	if (!arm.lookupCommand(s, s + 2, tmAsync, tmAsync))
		return false;
	
	// find the target pose
	GenConfigspaceState end;
	// as soon as possible i.e.:
	end.t = std::min(s[0].t + arm.getTimeDelta(), tmAsync);
	// compute trajectory for each joint
	for (U32 j = 0; j < arm.getJoints().size(); j++) {
		golem::GenCoordTrj trj;
		// boundary conditions are: at t0 {pos, vel, acc}; at t1 {pos=?, vel=0, acc=?}
		trj.set2dvav(s[0].t, end.t, s[0].pos[j], s[0].vel[j], s[0].acc[j], Real(0.0));
		end.set(j, trj.get(end.t));
	}
	
	if (!arm.send(end))
		return false;

	return true;
}

//------------------------------------------------------------------------------

