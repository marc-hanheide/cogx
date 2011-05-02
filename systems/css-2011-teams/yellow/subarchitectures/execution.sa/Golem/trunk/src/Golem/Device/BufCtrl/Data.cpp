/** @file Data.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Device/BufCtrl/Data.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Ctrl/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(BufCtrlJoint::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	// do not repeat for Joint::Desc
}

void golem::XMLData(BufCtrlArm::Desc &val, XMLContext* context, bool create) {
	XMLData((Arm::Desc &)val, context, create);
	XMLDataPtr<BufCtrlJoint::Desc>(val.joints.begin(), val.joints.end(), context, "joint", create); // repeat for BufCtrlJoint::Desc
	golem::XMLData("cycle_num", val.cycleNum, context->getContextFirst("calibration"), create);
	golem::XMLData("cycle_dur", val.cycleDur, context->getContextFirst("calibration"), create);
	golem::XMLData("time_quant", val.timeQuant, context->getContextFirst("calibration"), create);
	golem::XMLData("delta_offs", val.deltaOffs, context->getContextFirst("calibration"), create);
	golem::XMLData("skew_offs", val.skewOffs, context->getContextFirst("calibration"), create);
}


//------------------------------------------------------------------------------
