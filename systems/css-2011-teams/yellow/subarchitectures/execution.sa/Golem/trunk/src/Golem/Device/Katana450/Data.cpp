/** @file Data.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Device/Katana450/Data.h>
#include <Golem/Device/BufCtrl/Data.h>
#include <Golem/Device/Katana/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

template <> void golem::XMLGetAttribute(const std::string &attr, EControllerType &val, const XMLContext* context) {
	ASSERT(context)

	std::string type;
	XMLGetAttribute(attr, type, context);
	if (!type.compare("drive_v1"))
		val = CONTROLLER_TYPE_DRIVE_V1;
	else if (!type.compare("drive_v2"))
		val = CONTROLLER_TYPE_DRIVE_V2;
	else if (!type.compare("current_measured"))
		val = CONTROLLER_TYPE_CURRENT_MEASURED;
	else if (!type.compare("none"))
		val = CONTROLLER_TYPE_NONE;
	else
		throw MsgXMLParser("XMLGetAttribute(): Unknown type '%s' at: %s", type.c_str(), context->getName().c_str());
}

template <> void golem::XMLSetAttribute(const std::string &attr, const EControllerType &val, XMLContext* context) {
	ASSERT(context)
	// TODO
}

namespace golem {

void XMLData(AxNI_base::CalibrationParameter &val, XMLContext* context, bool create) {
	U32 tmp;
	
	XMLData("speed", val.speed, context, create);
	
	tmp = 0;
	XMLData("accel", tmp, context, create);
	val.accel = (U8)tmp;
	
	tmp = 0;
	XMLData("stopTolerance", tmp, context, create);
	val.stopTolerance = (U8)tmp;
	
	tmp = 0;
	XMLData("rangeTolerance", tmp, context, create);
	val.rangeTolerance = (U8)tmp;
}

void XMLData(AxNI_base::ControllerParameter &val, XMLContext* context, bool create) {
	XMLData("kPPos", val.kPPos, context, create);
	XMLData("kIPos", val.kIPos, context, create);
	XMLData("kDPos", val.kDPos, context, create);
	XMLData("kILegacy", val.kILegacy, context, create);
	XMLData("kCurrent", val.kCurrent, context, create);
	XMLData("kICurrent", val.kICurrent, context, create);
	XMLData("maxDrive", val.maxDrive, context, create);
	XMLData("maxCurrent", val.maxCurrent, context, create);
	XMLData("type", val.type, context, create);
	XMLData("ke", val.ke, context, create);
	XMLData("ra", val.ra, context, create);
	XMLData("offsetE1", val.offsetE1, context, create);
}

void XMLData(AxNI_base::CollisionParameter &val, XMLContext* context, bool create) {
	XMLData("limitPos", val.limitPos, context, create);
	XMLData("limitSpeed", val.limitSpeed, context, create);
}

void XMLData(AxNI_base::PolynomialParameter &val, XMLContext* context, bool create) {
	U32 tmp;
	
	XMLData("offset", val.offset, context, create);
	XMLData("slope", val.slope, context, create);
	
	tmp = 0;
	XMLData("tolerance", tmp, context, create);
	val.tolerance = (U8)tmp;
}

};

void golem::XMLData(Katana450Joint::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	// do not repeat for BufCtrlJoint::Desc, but run for KatanaJoint::Desc ()
	XMLData((KatanaJoint::Desc&)val, context, create);

	XMLData(val.calibration, context->getContextFirst("AxNI calibration"), create);
	val.calibration.encOffset = (U32)val.encoderOffset; // from KNI
	val.calibration.encPosAfter = (U32)val.encoderPositionAfter; // from KNI
	val.calibration.encPerCycle = (U32)val.encodersPerCycle; // from KNI
	XMLData(val.calibrationControl, context->getContextFirst("AxNI calibration"), create);
	XMLData(val.calibrationCollision, context->getContextFirst("AxNI calibration"), create);
	XMLData(val.operationControl, context->getContextFirst("AxNI operation"), create);
	XMLData(val.operationCollision, context->getContextFirst("AxNI operation"), create);
	XMLData(val.polynomial, context->getContextFirst("AxNI polynomial"), create);
}

void golem::XMLData(Katana450Arm::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)
	XMLData((BufCtrlArm::Desc &)val, context, create);
	XMLData((KatanaArm::Desc &)val, context, create);
	XMLDataPtr<Katana450Joint::Desc>(val.joints.begin(), val.joints.end(), context, "joint", create); // repeat for Katana450Joint::Desc

	XMLData("can_device", val.canDevice, context, create);

	XMLData("gripper", val.bGripper, context, create);
	XMLData("maxCommTime", val.gripperMaxCommTime, context->getContextFirst("gripper"), create);
	XMLData("encoder_open", val.gripperOpenEncVal, context->getContextFirst("gripper graspRange"), create);
	XMLData("encoder_closed", val.gripperClosedEncVal, context->getContextFirst("gripper graspRange"), create);
	XMLData("angleOffset", val.gripperAngleOffset, context->getContextFirst("gripper KNI"), create);
	XMLData("angleRange", val.gripperAngleRange, context->getContextFirst("gripper KNI"), create);
	XMLData("encoderOffset", val.gripperEncoderOffset, context->getContextFirst("gripper KNI"), create);
	XMLData("encodersPerCycle", val.gripperEncodersPerCycle, context->getContextFirst("gripper KNI"), create);
	XMLData("rotationDirection", val.gripperRotationDirection, context->getContextFirst("gripper KNI"), create);
	XMLData("encoderPositionAfter", val.gripperEncoderPositionAfter, context->getContextFirst("gripper KNI"), create);
	XMLData("offset", val.gripperOffset, context->getContextFirst("gripper KNI"), create);
	XMLData("gain", val.gripperGain, context->getContextFirst("gripper KNI"), create);
	XMLData(val.gripperCalibration, context->getContextFirst("gripper AxNI calibration"), create);
	val.gripperCalibration.encOffset = (U32)val.gripperEncoderOffset; // from KNI
	val.gripperCalibration.encPosAfter = (U32)val.gripperEncoderPositionAfter; // from KNI
	val.gripperCalibration.encPerCycle = (U32)val.gripperEncodersPerCycle; // from KNI
	XMLData(val.gripperCalibrationControl, context->getContextFirst("gripper AxNI calibration"), create);
	XMLData(val.gripperCalibrationCollision, context->getContextFirst("gripper AxNI calibration"), create);
	XMLData(val.gripperOperationControl, context->getContextFirst("gripper AxNI operation"), create);
	XMLData(val.gripperOperationCollision, context->getContextFirst("gripper AxNI operation"), create);
}

//------------------------------------------------------------------------------
