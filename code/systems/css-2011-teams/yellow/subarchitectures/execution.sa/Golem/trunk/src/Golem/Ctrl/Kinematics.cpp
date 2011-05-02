/** @file Kinematics.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Ctrl/Kinematics.h>
#include <Golem/Ctrl/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Kinematics::Kinematics(golem::Heuristic &heuristic) :
	heuristic(heuristic),
	arm(heuristic.getArm()),
	context(arm.getContext()),
	rand(context.getRandSeed())
{
}

Kinematics::~Kinematics() {
}

bool Kinematics::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgKinematicsInvalidDesc(Message::LEVEL_CRIT, "Kinematics::create(): Invalid description"	);
	
	setMin(desc.min);
	setMax(desc.max);
	collisionDetection = desc.collisionDetection;

	return true;
}

void Kinematics::setMin(const GenConfigspaceCoord &min) {
	// number of joints
	const U32 numOfJoints = U32(arm.getJoints().size());
	
	for (U32 i = 0; i < numOfJoints; i++) {
		const Joint &joint = *arm.getJoints()[i];

		this->min.pos[i] = Math::clamp(min.pos[i], joint.getMin().pos, joint.getMax().pos);
		if (this->max.pos[i] < this->min.pos[i] + REAL_EPS)
			this->max.pos[i] = this->min.pos[i] + REAL_EPS;

		this->min.vel[i] = Math::clamp(min.vel[i], joint.getMin().vel, joint.getMax().vel);
		if (this->max.vel[i] < this->min.vel[i] + REAL_EPS)
			this->max.vel[i] = this->min.vel[i] + REAL_EPS;
		
		this->min.acc[i] = Math::clamp(min.acc[i], joint.getMin().acc, joint.getMax().acc);
		this->max.acc[i] = std::max(this->max.acc[i], this->min.acc[i]);
		if (this->max.acc[i] < this->min.acc[i] + REAL_EPS)
			this->max.acc[i] = this->min.acc[i] + REAL_EPS;
	}
}

void Kinematics::setMax(const GenConfigspaceCoord &max) {
	// number of joints
	const U32 numOfJoints = U32(arm.getJoints().size());
	
	for (U32 i = 0; i < numOfJoints; i++) {
		const Joint &joint = *arm.getJoints()[i];

		this->max.pos[i] = Math::clamp(max.pos[i], joint.getMin().pos, joint.getMax().pos);
		if (this->min.pos[i] > this->max.pos[i] - REAL_EPS)
			this->min.pos[i] = this->max.pos[i] - REAL_EPS;

		this->max.vel[i] = Math::clamp(max.vel[i], joint.getMin().vel, joint.getMax().vel);
		if (this->min.vel[i] > this->max.vel[i] - REAL_EPS)
			this->min.vel[i] = this->max.vel[i] - REAL_EPS;
		
		this->max.acc[i] = Math::clamp(max.acc[i], joint.getMin().acc, joint.getMax().acc);
		if (this->min.acc[i] > this->max.acc[i] - REAL_EPS)
			this->min.acc[i] = this->max.acc[i] - REAL_EPS;
	}
}

//------------------------------------------------------------------------------
