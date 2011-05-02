/** @file Msg.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_PHYSCTRL_MSG_H_
#define _GOLEM_PHYSCTRL_MSG_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Msg.h>
#include <Golem/Phys/Msg.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class MsgPhysArm : public MsgActor {};
class MsgPhysArmBoundsGroupCreate : public MsgPhysArm {MESSAGE_BODY(MsgPhysArmBoundsGroupCreate)};
class MsgPhysArmShapeDescCreate : public MsgPhysArm {MESSAGE_BODY(MsgPhysArmShapeDescCreate)};
class MsgPhysArmJointActor : public MsgPhysArm {MESSAGE_BODY(MsgPhysArmJointActor)};

class MsgPhysPlanner : public MsgPhysArm {};
class MsgPhysPlannerUnknownPlanner : public MsgPhysPlanner {MESSAGE_BODY(MsgPhysPlannerUnknownPlanner)};

class MsgPhysReacPlanner : public MsgPhysPlanner {};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYSCTRL_MSG_H_*/
