/** @file Katana300Sim.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Device/Katana300Sim/Katana300Sim.h>
#include <Golem/Device/Katana300Sim/Data.h>
#include <Golem/Ctrl/LoadArmDesc.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void loadArmDesc(void* pContext, const std::string& path, void* pArmDesc) {
	loadArmDesc<golem::Katana300SimArm::Desc>((Context*)pContext, path, (Arm::Desc::Ptr*)pArmDesc);
}

//------------------------------------------------------------------------------

Katana300SimJoint::Katana300SimJoint(Arm& arm) : KatanaJoint(&arm), BufCtrlSimJoint(arm) {
}

bool Katana300SimJoint::create(const Desc& desc) {
	KatanaJoint::create(desc); // throws
	BufCtrlSimJoint::create(desc); // throws

	// overwrite min, max
	min.pos = this->getKNIMinPos();
	max.pos = this->getKNIMaxPos();

	return true;
}

//------------------------------------------------------------------------------

Katana300SimArm::Katana300SimArm(golem::Context& context) : KatanaArm(&context), KatanaGripper(&context), BufCtrlSimArm(context) {
}

Katana300SimArm::~Katana300SimArm() {
}

bool Katana300SimArm::create(const Desc& desc) {
	KatanaArm::create(desc); // throws
	KatanaGripper::create(desc); // throws
	BufCtrlSimArm::create(desc); // throws

	return true;
}

//------------------------------------------------------------------------------

void Katana300SimArm::forwardTransform(Mat34& trn, const ConfigspaceCoord& cc) {
	if (customKinematics)
		KatanaArm::forwardTransform(trn, cc);
	else
		Arm::forwardTransform(trn, cc);
}

void Katana300SimArm::velocitySpatial(Twist& v, const ConfigspaceCoord& cc, const ConfigspaceCoord& dcc) {
	if (customKinematics)
		KatanaArm::velocitySpatial(v, cc, dcc);
	else
		Arm::velocitySpatial(v, cc, dcc);
}

void Katana300SimArm::jacobianSpatial(Jacobian& jac, const ConfigspaceCoord& cc) {
	if (customKinematics)
		KatanaArm::jacobianSpatial(jac, cc);
	else
		Arm::jacobianSpatial(jac, cc);
}

//------------------------------------------------------------------------------

bool Katana300SimArm::gripperRecvSensorData(SensorDataSet& sensorData, MSecTmU32 timeWait) {
	return true;
}
	
bool Katana300SimArm::gripperRecvEncoderData(GripperEncoderData& encoderData, MSecTmU32 timeWait) {
	return true;
}
	
bool Katana300SimArm::gripperOpen(MSecTmU32 timeWait) {
	return true;
}

bool Katana300SimArm::gripperClose(const SensorDataSet& sensorThreshold, MSecTmU32 timeWait) {
	return true;
}
	
bool Katana300SimArm::gripperFreeze(MSecTmU32 timeWait) {
	return true;
}

//------------------------------------------------------------------------------
