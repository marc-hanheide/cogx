/** @file Data.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Device/Katana300Sim/Data.h>
#include <Golem/Device/BufCtrlSim/Data.h>
#include <Golem/Device/Katana/Data.h>
#include <Golem/Tools/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(Katana300SimJoint::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	// do not repeat for BufCtrlSimJoint::Desc, but run for KatanaJoint::Desc ()
	XMLData((KatanaJoint::Desc&)val, context, create);
}

void golem::XMLData(Katana300SimArm::Desc &val, XMLContext* context, bool create) {
	XMLData((BufCtrlSimArm::Desc &)val, context, create);
	XMLData((KatanaArm::Desc &)val, context, create);
	XMLDataPtr<Katana300SimJoint::Desc>(val.joints.begin(), val.joints.end(), context, "joint", create); // repeat for Katana300SimJoint::Desc
}

//------------------------------------------------------------------------------
