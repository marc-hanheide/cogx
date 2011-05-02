/** @file Msg.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_MSG_H_
#define _GOLEM_CTRL_MSG_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Message.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class MsgProfile : public Message {};
class MsgProfileInvalidDesc : public MsgProfile {MESSAGE_BODY(MsgProfileInvalidDesc)};

class MsgArm : public Message {};
class MsgArmInvalidDesc : public MsgArm {MESSAGE_BODY(MsgArmInvalidDesc)};
class MsgArmOpenLib : public MsgArm {MESSAGE_BODY(MsgArmOpenLib)};
class MsgArmLoadDesc : public MsgArm {MESSAGE_BODY(MsgArmLoadDesc)};
class MsgArmInvalidConfig : public MsgArm {MESSAGE_BODY(MsgArmInvalidConfig)};

class MsgJoint : public MsgArm {};
class MsgJointInvalidDesc : public MsgJoint {MESSAGE_BODY(MsgJointInvalidDesc)};

class MsgHeuristic : public MsgArm {};
class MsgHeuristicInvalidDesc : public MsgHeuristic {MESSAGE_BODY(MsgHeuristicInvalidDesc)};

class MsgKinematics : public MsgHeuristic {};
class MsgKinematicsInvalidDesc : public MsgKinematics {MESSAGE_BODY(MsgKinematicsInvalidDesc)};

class MsgTransmitter : public MsgArm {};
class MsgTransmitterInvalidDesc : public MsgTransmitter {MESSAGE_BODY(MsgTransmitterInvalidDesc)};

class MsgPlanner : public MsgArm {};
class MsgPlannerInvalidDesc : public MsgPlanner {MESSAGE_BODY(MsgPlannerInvalidDesc)};

class MsgPathFinder : public MsgPlanner {};
class MsgPathFinderInvalidDesc : public MsgPathFinder {MESSAGE_BODY(MsgPathFinderInvalidDesc)};
class MsgPathFinderUnknownKinematics : public MsgPathFinder {MESSAGE_BODY(MsgPathFinderUnknownKinematics)};

class MsgGraphPlanner : public MsgPlanner {};
class MsgGraphPlannerUnknownKinematics : public MsgGraphPlanner {MESSAGE_BODY(MsgGraphPlannerUnknownKinematics)};

class MsgReacPlanner : public MsgPlanner {};
class MsgReacPlannerInvalidDesc : public MsgReacPlanner {MESSAGE_BODY(MsgReacPlannerInvalidDesc)};
class MsgReacPlannerUnknownKinematics : public MsgPathFinder {MESSAGE_BODY(MsgReacPlannerUnknownKinematics)};
class MsgReacPlannerThreadLaunch : public MsgReacPlanner {MESSAGE_BODY(MsgReacPlannerThreadLaunch)};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_MSG_H_*/
