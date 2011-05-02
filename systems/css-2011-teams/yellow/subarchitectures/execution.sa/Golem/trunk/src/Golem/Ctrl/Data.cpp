/** @file Data.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Ctrl/Data.h>
#include <Golem/Tools/Data.h>
#include <Golem/Tools/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(GenCoord &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData("pos", val.pos, context, create);
	XMLData("vel", val.vel, context, create);
	XMLData("acc", val.acc, context, create);
}

void golem::XMLData(ConfigspaceCoord &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData("c1", val.c[0], context, create);
	XMLData("c2", val.c[1], context, create);
	XMLData("c3", val.c[2], context, create);
	XMLData("c4", val.c[3], context, create);
	XMLData("c5", val.c[4], context, create);
	XMLData("c6", val.c[5], context, create);
}

void golem::XMLData(ExpCoord &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData(val.twist, context, create);
	XMLData("th", val.theta, context, create);
}

void golem::XMLData(Joint::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData("name", val.name, context, create);
	XMLData("collision", val.collision, context, create);
	XMLData("collision_offs", val.collisionOffset, context, create);
	XMLData(val.min, context->getContextFirst("min"), create);
	XMLData(val.max, context->getContextFirst("max"), create);
	XMLData(val.trn, context->getContextFirst("trn"), create);
	XMLData(val.trnInit, context->getContextFirst("trn_init"), create);
	try {
		XMLData(val.bounds, context, "bounds", create);
	}
	catch (const MsgXMLParserNameNotFound&) {
	}
}

void golem::XMLData(Arm::Desc &val, XMLContext* context, bool create) {
	XMLData("name", val.name, context, create);
	XMLDataPtr<Joint::Desc>(val.joints.begin(), val.joints.end(), context, "joint", create);
	XMLData(val.globalPose, context->getContextFirst("global_pose"), create);
	XMLData(val.referencePose, context->getContextFirst("reference_pose"), create);
	XMLData(val.restConfig, context->getContextFirst("rest_config"), create);
	XMLData("custom_kinematics", val.customKinematics, context, create);
}

//------------------------------------------------------------------------------
