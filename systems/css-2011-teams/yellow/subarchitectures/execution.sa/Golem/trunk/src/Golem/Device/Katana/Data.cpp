/** @file Data.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Device/Katana/Data.h>
#include <Golem/Tools/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(KatanaJoint::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	golem::XMLData("angleOffset", val.angleOffset, context->getContextFirst("KNI"), create);
	golem::XMLData("angleRange", val.angleRange, context->getContextFirst("KNI"), create);
	golem::XMLData("encoderOffset", val.encoderOffset, context->getContextFirst("KNI"), create);
	golem::XMLData("encodersPerCycle", val.encodersPerCycle, context->getContextFirst("KNI"), create);
	golem::XMLData("rotationDirection", val.rotationDirection, context->getContextFirst("KNI"), create);
	golem::XMLData("encoderPositionAfter", val.encoderPositionAfter, context->getContextFirst("KNI"), create);
	golem::XMLData("offset", val.offset, context->getContextFirst("KNI"), create);
	golem::XMLData("gain", val.gain, context->getContextFirst("KNI"), create);
}

void golem::XMLData(KatanaArm::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	golem::XMLData("L0", val.L0, context->getContextFirst("links"), create);
	golem::XMLData("L1", val.L1, context->getContextFirst("links"), create);
	golem::XMLData("L2", val.L2, context->getContextFirst("links"), create);
	golem::XMLData("L3", val.L3, context->getContextFirst("links"), create);
}

void golem::XMLData(KatanaGripper::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData("gripper", val.bGripper, context, create);
}

//------------------------------------------------------------------------------
