/** @file Data.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Device/BufCtrlSim/Data.h>
#include <Golem/Device/BufCtrl/Data.h>
#include <Golem/Tools/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(BufCtrlSimJoint::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	// do not repeat for BufCtrlJoint::Desc
}

void golem::XMLData(BufCtrlSimArm::Desc &val, XMLContext* context, bool create) {
	XMLData((BufCtrlArm::Desc &)val, context, create);
	XMLDataPtr<BufCtrlSimJoint::Desc>(val.joints.begin(), val.joints.end(), context, "joint", create); // repeat for BufCtrlSimJoint::Desc
	golem::XMLData("delta_sync", val.deltaSync, context->getContextFirst("calibration"), create);
	golem::XMLData("delta_recv", val.deltaRecv, context->getContextFirst("calibration"), create);
	golem::XMLData("delta_send", val.deltaSend, context->getContextFirst("calibration"), create);
}

//------------------------------------------------------------------------------
