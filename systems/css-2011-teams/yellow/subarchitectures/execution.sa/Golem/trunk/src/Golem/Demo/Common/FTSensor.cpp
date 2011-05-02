/** @file FTSensor.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Demo/Common/FTSensor.h>
#include <Golem/Demo/Common/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

FTSensorRenderer::FTSensorRenderer(FTSensor &ftSensor) :
	ftSensor(ftSensor)
{}

bool FTSensorRenderer::create(const Desc &desc) {
	if (!desc.isValid())
		return false;

	// reserve memory buffer
	reset();
	if (desc.forceShow)
		reserveLines(1);
	if (desc.torqueShow)
		reserveAxesInc(1);

	return true;
}

//------------------------------------------------------------------------------

FTSensor::FTSensor(Embodiment &embodiment) : Base(embodiment) {
	pFTSensorRenderer.reset(new FTSensorRenderer(*this));
}

FTSensor::~FTSensor() {
}

bool FTSensor::create(const FTSensor::Desc& desc) {
	Base::create(desc); // throws

	wrenchGain = desc.wrenchGain;
	ftSensorRendererDesc = desc.ftSensorRendererDesc;
	ftSensorShow = desc.ftSensorShow;
	
	timeStamp = SEC_TM_REAL_ZERO;
	evWait.set(false);
	
	return true;
}

bool FTSensor::read(Wrench &wrench, SecTmReal &timeStamp, MSecTmU32 timeOut) {
	if (timeOut > 0 && !evWait.wait(timeOut))
		return false;

	CriticalSectionWrapper csw(cs);

	if (timeStamp <= SEC_TM_REAL_ZERO)
		return false;

	timeStamp = this->timeStamp;
	wrench = this->wrench;
	
	evWait.set(false);
	return true;
}

void FTSensor::postprocess(SecTmReal elapsedTime) {
	CriticalSectionWrapper csw(cs);
	
	if (!get(wrench, timeStamp))
		return;

	wrench.arrayMultiply(wrenchGain, wrench);
	
	if (ftSensorShow)
		pFTSensorRenderer->create(ftSensorRendererDesc);
	
	evWait.set(true);
}

void FTSensor::render() {
	CriticalSectionWrapper csw(cs);
	
	// postprocess() and render() are always called from the same thread 
	if (ftSensorShow)
		pFTSensorRenderer->render();
}

void FTSensor::keyboardHandler(unsigned char key, int x, int y) {
	if (key == 4) {// F4
		CriticalSectionWrapper csw(cs);
	
		ftSensorShow = !ftSensorShow;
		if (ftSensorShow)
			pFTSensorRenderer->create(ftSensorRendererDesc);
	}
}

void FTSensor::setWrenchGain(const Wrench &wrenchGain) {
	CriticalSectionWrapper csw(cs);
	this->wrenchGain = wrenchGain;
}

//------------------------------------------------------------------------------

VirtualFTSensor::VirtualFTSensor(Embodiment &embodiment) : FTSensor(embodiment) {
}

VirtualFTSensor::~VirtualFTSensor() {
}

bool VirtualFTSensor::create(const VirtualFTSensor::Desc& desc) {
	FTSensor::create(desc); // throws

	joint = desc.joint;

	for (U32 i = 0; i < 6; i++) {
		fastFilter[i].create(desc.fastFilterDesc); // throws
		slowFilter[i].create(desc.slowFilterDesc); // throws
	}
	
	return true;
}

void VirtualFTSensor::resetsFilters() {
	for (U32 i = 0; i < 6; i++) {
		fastFilter[i].reset();
		slowFilter[i].reset();
	}
}

bool VirtualFTSensor::get(Wrench &wrench, SecTmReal &timeStamp) {
	if (joint == NULL)
		return false;
	
	// measure joint actors' relative linear and angular displacements as an approx. of force and torque 
	
	NxActor *pNxActor1, *pNxActor2;
	joint->getActors(&pNxActor1, &pNxActor2);
	
	const NxMat34 nxPose1 = pNxActor1->getGlobalPose();
	Mat34 pose1;
	nxPose1.M.getRowMajor(&pose1.R.m11);
	nxPose1.t.get(&pose1.p.v1);
	
	const NxMat34 nxPose2 = pNxActor2->getGlobalPose();
	Mat34 pose2;
	nxPose2.M.getRowMajor(&pose2.R.m11);
	nxPose2.t.get(&pose2.p.v1);

	Mat33 base;
	base.setTransposed(pose1.R);
	
	Vec3 dp;
	dp.subtract(pose2.p, pose1.p);
	base.multiply(dp, dp);
	for (U32 i = 0; i < 3; i++) {
		fastFilter[i].input(dp[i]);
		slowFilter[i].input(dp[i]);
		wrench.getV()[i] = fastFilter[i].output() - slowFilter[i].output();
	}
	
	Mat33 R;
	R.multiply(base, pose2.R);
	
	Vec3 x(REAL_ONE, REAL_ZERO, REAL_ZERO), z(REAL_ZERO, REAL_ZERO, REAL_ONE);
	R.multiply(x, x);
	R.multiply(z, z);
	const Real dxx = Vec3(REAL_ONE, REAL_ZERO, REAL_ZERO).dot(x);
	const Real dxy = Vec3(REAL_ZERO, REAL_ONE, REAL_ZERO).dot(x);
	const Real dzx = Vec3(REAL_ONE, REAL_ZERO, REAL_ZERO).dot(z);
	const Real dzy = Vec3(REAL_ZERO, REAL_ONE, REAL_ZERO).dot(z);
	const Real dzz = Vec3(REAL_ZERO, REAL_ZERO, REAL_ONE).dot(z);

	Vec3 dq;
	// approx. rotation about X axis (right hand rotation)
	dq.v1 = Math::atan2(dzz, dzy);
	// approx. rotation about Y axis (right hand rotation)
	dq.v2 = Math::atan2(-dzz, dzx);
	// approx. rotation about Z axis (right hand rotation)
	dq.v3 = Math::atan2(dxy, dxx);

	for (U32 i = 0; i < 3; i++) {
		fastFilter[i + 3].input(dq[i]);
		slowFilter[i + 3].input(dq[i]);
		wrench.getW()[i] = fastFilter[i + 3].output() - slowFilter[i + 3].output();
	}
	
	//context.getMessageStream()->write(Message::LEVEL_DEBUG,
	//	"FTSensor: {(%f, %f, %f), (%f, %f, %f)}",
	//	wrench.getV().v1, wrench.getV().v2, wrench.getV().v3, wrench.getW().v1, wrench.getW().v2, wrench.getW().v3
	//));
	
	return true;
}

void VirtualFTSensor::setJoint(NxJoint *joint) {
	CriticalSectionWrapper csw(cs);
	this->joint = joint;
	resetsFilters();
}

//------------------------------------------------------------------------------

Nano17FTSensor::Nano17FTSensor(Embodiment &embodiment) : FTSensor(embodiment) {
}

Nano17FTSensor::~Nano17FTSensor() {
}

bool Nano17FTSensor::create(const Nano17FTSensor::Desc& desc) {
	FTSensor::create(desc); // throws

	// TODO initialise FT sensor device
	
	return true;
}

bool Nano17FTSensor::get(Wrench &wrench, SecTmReal &timeStamp) {
	// TODO read wrench
	
	return true;
}

//------------------------------------------------------------------------------
