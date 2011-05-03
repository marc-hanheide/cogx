/** @file TinyGrasp.cpp
 * 
 * Program demonstrating Tiny Golem interface
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Phys/Universe.h>
#include <Golem/Tiny/Tiny.h>
#include <Golem/Demo/TinyGrasp/TinyGrasp.h>
#include <Golem/Math/Rand.h>
#include <Golem/Math/Quat.h>
#include <Golem/Phys/Data.h>
#include <Golem/Tools/Data.h>
#include <Golem/Tools/XMLData.h>
#include <iostream>

using namespace golem;
using namespace golem::tiny;

//------------------------------------------------------------------------------

PoseError::PoseError(Real lin, Real ang) : lin(lin), ang(ang) {
}

PoseError::PoseError(const Mat34& a, const Mat34& b) {
	lin = getLinearDist(a.p, b.p);
	ang = getAngularDist(Quat(a.R), Quat(b.R));
}

Real PoseError::getLinearDist(const Vec3& v0, const Vec3& v1) {
	return v0.distance(v1);
}

Real PoseError::getAngularDist(const Quat& q0, const Quat& q1) {
	const Real d = q0.dot(q1);
	return REAL_ONE - ::fabs(d);
}

//------------------------------------------------------------------------------

namespace golem {
namespace tiny {

class Debug: public Object {
private:
	CriticalSection cs;
	DebugRenderer debugRenderer;

	Debug(Scene &scene) : Object(scene) {
	}
	bool create(const Object::Desc& desc) {
		return Object::create(desc);
	}
	void render() {
		CriticalSectionWrapper csw(cs);
		debugRenderer.render();
	}

public:
	/** Object description */
	class Desc : public Object::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Debug, Object::Ptr, Scene&)
	};

	void render(const DebugRenderer* debugRenderer = NULL) {
		CriticalSectionWrapper csw(cs);
		if (debugRenderer)
			this->debugRenderer = *debugRenderer;
		else
			this->debugRenderer.reset();
	}
};

};
};

TinyEx::TinyEx(int argc, char *argv[]) : Tiny(argc, argv) {
	Debug::Desc desc; 
	debug = dynamic_cast<Debug*>(pScene->createObject(desc));
}
void TinyEx::render(const DebugRenderer* debugRenderer) {
	debug->render(debugRenderer);
}

XMLContext* TinyEx::getXMLContext() {
	return pXMLContext;
}

//------------------------------------------------------------------------------

TinyGrasp::TinyGrasp(const char* driver) :
	objectGrasped(NULL),
	object(NULL)
{
	int argc = 1;
	char* argv [] = {"TinyGrasp"};
	tiny.reset(new TinyEx(argc, argv));

	// XML data
	XMLContext* context = tiny->getXMLContext()->getContextFirst("robot");	
	XMLData(armPose, context->getContextFirst("arm pose"));
	XMLData("planner_tries", plannerTries, context->getContextFirst("arm"));
	XMLData("approach_offset", approachOffset, context->getContextFirst("arm grasping"));
	XMLData("grasp_offset", graspOffset, context->getContextFirst("arm grasping"));
	XMLData("error_lin", graspError.lin, context->getContextFirst("arm grasping"));
	XMLData("error_ang", graspError.lin, context->getContextFirst("arm grasping"));
	XMLData("sensor_threshold", sensorThreshold, context->getContextFirst("arm grasping"));

	// create arm
	KatanaArmDesc* pArmDesc = new KatanaArmDesc; // specialised Katana 300/450 description
	//ArmDesc* pArmDesc = new ArmDesc; // generic description
	pArmDesc->globalPose = armPose;
	pArmDesc->path = driver; // specify driver path
	pArmDesc->bGripper = true;
	tiny->print("Creating the arm...");
	arm = (KatanaArm*)tiny->createActor(ActorDescPtr(pArmDesc));
	//arm = dynamic_cast<KatanaArm*>(tiny->createActor(ActorDescPtr(pArmDesc)));
	//if (arm == NULL)
	//	throw ExTiny("TinyGrasp::TinyGrasp(): Katana arm driver required!");

	// get sensor data assuming no object is in the gripper
	zero = arm->gripperRecvSensorData(numeric_const<double>::INF);
	
	// robot base
	RigidBodyDesc* pBaseDesc = new RigidBodyDesc;
	pBaseDesc->kinematic = true;
	XMLData(pBaseDesc->globalPose, context->getContextFirst("base pose"));
	BoxShapeDesc* pBaseShapeDesc = new BoxShapeDesc;
	XMLData(pBaseShapeDesc->dimensions, context->getContextFirst("base dimensions"));
	pBaseDesc->shapes.push_back(ShapeDescPtr(pBaseShapeDesc));
	tiny->createActor(ActorDescPtr(pBaseDesc));
	
	// robot beam
	RigidBodyDesc* pBeamDesc = new RigidBodyDesc;
	pBeamDesc->kinematic = true;
	XMLData(pBeamDesc->globalPose, context->getContextFirst("beam pose"));
	BoxShapeDesc* pBeamShapeDesc = new BoxShapeDesc;
	XMLData(pBeamShapeDesc->dimensions, context->getContextFirst("beam dimensions"));
	pBeamDesc->shapes.push_back(ShapeDescPtr(pBeamShapeDesc));
	tiny->createActor(ActorDescPtr(pBeamDesc));

	// attached a finger to the end-effector (the last joint)
	Joint* effector = arm->getJoints().back();
	// get the end-effector reference armPose
	Mat34 referencePose = arm->getReferencePose();
	// construct a finger from a box and a sphere in local end-effector coordinates (arm at reference configuration stretched along Y-axis)
	Real fingerLength = 0.08;
	XMLData("length", fingerLength, context->getContextFirst("arm gripper"));
	Real fingerDiam = 0.015;
	XMLData("diam", fingerDiam, context->getContextFirst("arm gripper"));
	Real fingerGap = 0.08;
	XMLData("gap", fingerGap, context->getContextFirst("arm gripper"));
	
	BoxShapeDesc* pFingerLeftShapeDesc = new BoxShapeDesc;
	pFingerLeftShapeDesc->dimensions.set(Real(fingerDiam/2.0), Real(fingerLength/2.0), Real(fingerDiam/2.0));
	pFingerLeftShapeDesc->localPose = referencePose;
	pFingerLeftShapeDesc->localPose.p.v1 += Real(fingerGap/2.0);
	pFingerLeftShapeDesc->localPose.p.v2 += Real(fingerLength/2.0);
	(void)effector->createShape(ShapeDescPtr(pFingerLeftShapeDesc));
	BoxShapeDesc* pFingerRightShapeDesc = new BoxShapeDesc;
	pFingerRightShapeDesc->dimensions.set(Real(fingerDiam/2.0), Real(fingerLength/2.0), Real(fingerDiam/2.0));
	pFingerRightShapeDesc->localPose = referencePose;
	pFingerRightShapeDesc->localPose.p.v1 -= Real(fingerGap/2.0);
	pFingerRightShapeDesc->localPose.p.v2 += Real(fingerLength/2.0);
	(void)effector->createShape(ShapeDescPtr(pFingerRightShapeDesc));
}

//------------------------------------------------------------------------------

RigidBody* TinyGrasp::createObject(const Mat34& pose, const Vec3& dimensions) {
	RigidBodyDesc* pObjectDesc = new RigidBodyDesc;
	pObjectDesc->kinematic = true;
	pObjectDesc->globalPose = pose;
	BoxShapeDesc* pObjectShapeDesc = new BoxShapeDesc;
	pObjectShapeDesc->dimensions = dimensions;
	pObjectDesc->shapes.push_back(ShapeDescPtr(pObjectShapeDesc));

	RigidBody* object = dynamic_cast<RigidBody*>(tiny->createActor(ActorDescPtr(pObjectDesc)));
	if (object == NULL)
		throw ExTiny("TinyGrasp::createObject(): Oops");
	return object;
}

void TinyGrasp::removeObject(tiny::RigidBody*& object) {
	if (object) {
		tiny->releaseActor(object);
		object = NULL;
	}
}

void TinyGrasp::attachObject() {
	if (objectGrasped)
		throw ExTiny("TinyGrasp::attachObject(): Object has been already attached");
	if (object == NULL)
		throw ExTiny("TinyGrasp::attachObject(): No object to attach");

	BoxShapeDesc* pObjectShapeDesc = dynamic_cast<BoxShapeDesc*>(object->getShapes().back()->getDesc().get());
	if (pObjectShapeDesc == NULL)
		throw ExTinyShape("TinyGrasp::attachObject(): Must be box shape");
	BoxShapeDesc* pBoxShapeDesc = new BoxShapeDesc(*pObjectShapeDesc);

	// objectPose = toolPose * localPose => localPose = toolPose^-1 * objectPose
	Mat34 pose = arm->getForwardTransform(arm->recvGenConfigspaceState(tiny->getTime()).pos).back();
	pBoxShapeDesc->localPose.setInverseRT(pose);
	pBoxShapeDesc->localPose.multiply(pBoxShapeDesc->localPose, object->getGlobalPose());

	Joint* effector = arm->getJoints().back();
	objectGrasped = effector->createShape(ShapeDescPtr(pBoxShapeDesc));

	// HACK: move object pose to infinity
	pose.setId();
	pose.p.z = 1e2; // not too high
	pose.p.x = 1e2; // not too high
	object->setGlobalPose(pose);
}

void TinyGrasp::releaseObject() {
	if (objectGrasped == NULL || object == NULL)
		throw ExTiny("TinyGrasp::releaseObject(): Object has not been attached");

	// objectPose = toolPose * localPose
	Mat34 pose = arm->getForwardTransform(arm->recvGenConfigspaceState(tiny->getTime()).pos).back();
	pose.multiply(pose, objectGrasped->getLocalPose());

	// remove shape
	Joint* effector = arm->getJoints().back();
	effector->releaseShape(objectGrasped);
	objectGrasped = NULL;

	// restore object pose
	object->setGlobalPose(pose);
}

//------------------------------------------------------------------------------

Mat34 TinyGrasp::read() {
	return TinyGrasp::getToolPose();
}

Mat34 TinyGrasp::moveTry(const Mat34& pose) {
	// compute movement end/target in joint configuration space
	cbegin = arm->recvGenConfigspaceState(tiny->getTime());
	end.pos = pose;
	end.t = cbegin.t + arm->getTimeDeltaAsync() + 1.0; // movement will last no shorter than 1 sec
	cend = arm->findTarget(cbegin, end);
	
	// compute actual pose, note that the target is always computed with respect to the reference pose in the tool frame (the last joint)
	Mat34 actual;
	actual.multiply(arm->getForwardTransform(cend.pos).back(), arm->getReferencePose());
	return actual;
}

void TinyGrasp::moveExec(Real duration) {
	// compute trajectory using path planning with collision detection
	GenConfigspaceStateSeq trajectory;
	
	for (U32 i = 0;; ++i) {
		try {		
			tiny->print("TinyGrasp::moveExec(): %u...", i + 1);
			cend.t = cbegin.t + duration;
			trajectory = arm->findTrajectory(cbegin, cend);
		}
		catch (ExTiny& ex) {
			if (i < plannerTries) continue;
			throw ex;
		}
		break;
	}

	// move the arm and wait until it stops
	tiny->print("TinyGrasp::moveExec(): OK");
	arm->send(trajectory, numeric_const<double>::INF);
}

//------------------------------------------------------------------------------

void TinyGrasp::setGraspObject(RigidBody* object) {
	if (object == NULL)
		throw ExTiny("TinyGrasp::setGraspObject(): Null pointer");
	this->object = object;
}

GraspPose::Seq TinyGrasp::getGraspPoses() const {
	if (object == NULL)
		throw ExTiny("TinyGrasp::getGraspPoses(): No objects to process");

	BoxShapeDesc* pObjectShapeDesc = dynamic_cast<BoxShapeDesc*>(object->getShapes().back()->getDesc().get());
	if (pObjectShapeDesc == NULL)
		throw ExTinyShape("TinyGrasp::getGraspPoses(): Must be box shape");

	const Mat34 pose = object->getGlobalPose();
	const Vec3 dimensions = pObjectShapeDesc->dimensions;

	GraspPose g[3][2];
	Real min = numeric_const<Real>::MAX;
	for (U32 i = 0; i < 3; ++i) {
		if (min > dimensions[i]) {
			min = dimensions[i];
			
			for (U32 j = 0; j < 2; ++j) {
				// object approach axis index
				const U32 k = (i + j + 1)%3;

				// local poses: 0 - approach from positive to negative, 1 - from negative to positive
				GraspPose* gp = g[j];
				Mat33 rot[2];
				switch (k) {
				case 0:	// X
					gp[0].approach.R.rotZ(+REAL_PI_2);
					gp[1].approach.R.rotZ(-REAL_PI_2);
					break;
				case 1:	// Y
					gp[0].approach.R.rotX(REAL_PI);
					gp[1].approach.R.setId();
					break;
				case 2:	// Z
					rot[0].rotX(-REAL_PI_2);
					rot[1].rotY(-REAL_PI_2);
					gp[0].approach.R.multiply(rot[0], rot[1]);
					rot[0].rotX(+REAL_PI_2);
					rot[1].rotY(+REAL_PI_2);
					gp[1].approach.R.multiply(rot[0], rot[1]);
					break;
				}

				gp[0].approach.p.setZero();
				gp[0].grasp = gp[0].approach;
				gp[0].approach.p[k] = +(dimensions[k] + approachOffset);
				gp[0].grasp.p[k] = +(dimensions[k] + graspOffset);
				gp[0].approach.multiply(pose, gp[0].approach);
				gp[0].grasp.multiply(pose, gp[0].grasp);

				gp[1].approach.p.setZero();
				gp[1].grasp = gp[1].approach;
				gp[1].approach.p[k] = -(dimensions[k] + approachOffset);
				gp[1].grasp.p[k] = -(dimensions[k] + graspOffset);
				gp[1].approach.multiply(pose, gp[1].approach);
				gp[1].grasp.multiply(pose, gp[1].grasp);
			}
		}
	}

	GraspPose::Seq poses;
	poses.push_back(g[0][0]);
	poses.push_back(g[0][1]);
	poses.push_back(g[1][0]);
	poses.push_back(g[1][1]);

	DebugRenderer debugRenderer;
	debugRenderer.addAxes(g[0][0].approach, Vec3(0.05));
	debugRenderer.addAxes(g[0][0].grasp, Vec3(0.05));
	debugRenderer.addAxes(g[0][1].approach, Vec3(0.05));
	debugRenderer.addAxes(g[0][1].grasp, Vec3(0.05));
	debugRenderer.addAxes(g[1][0].approach, Vec3(0.05));
	debugRenderer.addAxes(g[1][0].grasp, Vec3(0.05));
	debugRenderer.addAxes(g[1][1].approach, Vec3(0.05));
	debugRenderer.addAxes(g[1][1].grasp, Vec3(0.05));
	//debugRenderer.addAxes(g[2][0].approach, Vec3(0.05));
	//debugRenderer.addAxes(g[2][0].grasp, Vec3(0.05));
	//debugRenderer.addAxes(g[2][1].approach, Vec3(0.05));
	//debugRenderer.addAxes(g[2][1].grasp, Vec3(0.05));
	tiny->render(&debugRenderer);

	return poses;
}

std::pair<GraspPose, GraspPose::Seq::const_iterator> TinyGrasp::graspTry(const GraspPose::Seq& poses) {	
	GraspPose::Seq::const_iterator index = poses.begin();
	PoseError errorMin(moveTry(index->grasp), index->grasp);

	for (GraspPose::Seq::const_iterator i = ++poses.begin(); i != poses.end(); ++i) {
		PoseError error(moveTry(i->grasp), i->grasp);
		if (error.ang < errorMin.ang && error.lin < graspError.lin || error.lin < errorMin.lin && error.ang < graspError.ang) {
			errorMin = error;
			index = i;
		}
	}

	return std::pair<GraspPose, GraspPose::Seq::const_iterator>(graspTry(*index), index);
}

GraspPose TinyGrasp::graspTry(const GraspPose& pose) {
	gend = pose;

	GraspPose actual;
	actual.grasp = moveTry(gend.grasp);
	actual.approach = moveTry(gend.approach);

	return actual;
}

void TinyGrasp::graspExec(Real duration) {
	Mat34 actual;

	arm->gripperOpen(numeric_const<double>::INF);

	actual = moveTry(gend.approach);
	PoseError approachError(actual, gend.approach);
	tiny->print("TinyGrasp::graspExec(): approach error = (%f, %f)", approachError.lin, approachError.ang);
	moveExec(duration);

	actual = moveTry(gend.grasp);
	PoseError graspError(actual, gend.grasp);
	tiny->print("TinyGrasp::graspExec(): grasp error = (%f, %f)", graspError.lin, graspError.ang);
	moveExec();

	Mat34 pose = read();
	gapproach = toBody(pose, diff(pose, gend.approach));

	KatanaSensorDataSet threshold = zero;
	for (KatanaSensorDataSet::iterator i = threshold.begin(); i != threshold.end(); ++i)
		i->value += sensorThreshold;
	arm->gripperClose(threshold, numeric_const<double>::INF);

	attachObject();

	tiny->render();
}

void TinyGrasp::graspRelease() {
	arm->gripperOpen(numeric_const<double>::INF);

	releaseObject();
	
	Mat34 pose = read();
	pose.multiply(fromBody(pose, gapproach), pose);

	DebugRenderer debugRenderer;
	debugRenderer.addAxes(pose, Vec3(0.1));
	tiny->render(&debugRenderer);

	(void)moveTry(pose);
	moveExec();

	tiny->render();
}

//------------------------------------------------------------------------------

Mat34 TinyGrasp::getToolPose() {
	return arm->recvGenWorkspaceState(tiny->getTime()).pos;
}

Mat34 TinyGrasp::diff(const Mat34& a, const Mat34& b) {
	Mat34 ab;
	ab.setInverseRT(a); // a.R*(a.R)^T = Id, det(a.R) = 1 (rotation matrix)
	ab.multiply(b, ab);
	return ab;
}

Mat34 TinyGrasp::toBody(const Mat34& A, const Mat34& G) {
	Mat34 H;
	H.setInverseRT(A);
	H.multiply(H, G);
	H.multiply(H, A);
	return H;
}

Mat34 TinyGrasp::fromBody(const Mat34& A, const Mat34& H) {
	Mat34 G;
	G.setInverseRT(A);
	G.multiply(H, G);
	G.multiply(A, G);
	return G;
}

//------------------------------------------------------------------------------
