/** @file Data.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Device/Katana300/Data.h>
#include <Golem/Device/Katana/Data.h>
#include <Golem/Device/BufCtrl/Data.h>
#include <Golem/Tools/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(Katana300Joint::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	// do not repeat for BufCtrlJoint::Desc, but run for KatanaJoint::Desc ()
	XMLData((KatanaJoint::Desc&)val, context, create);
}

void golem::XMLData(Katana300SerialArm::Desc &val, XMLContext* context, bool create) {
	XMLData((BufCtrlArm::Desc &)val, context, create);
	XMLData((KatanaArm::Desc &)val, context, create);
	XMLData((KatanaGripper::Desc &)val, context, create);
	XMLDataPtr<Katana300Joint::Desc>(val.joints.begin(), val.joints.end(), context, "joint", create); // repeat for Katana300Joint::Desc
	XMLData("cfg_path", val.cfgPath, context, create);
	XMLData("com_port", val.serialDesc.commPort, context, create);
}

//------------------------------------------------------------------------------
