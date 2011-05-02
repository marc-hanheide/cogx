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

	void render(const DebugRenderer& debugRenderer) {
		CriticalSectionWrapper csw(cs);
		this->debugRenderer = debugRenderer;
	}
};

};
};

TinyEx::TinyEx(int argc, char *argv[]) : Tiny(argc, argv) {
	Debug::Desc desc; 
	debug = dynamic_cast<Debug*>(pScene->createObject(desc));
}
void TinyEx::render(const DebugRenderer& debugRenderer) {
	debug->render(debugRenderer);
}

XMLContext* TinyEx::getXMLContext() {
	return pXMLContext;
}

//------------------------------------------------------------------------------

TinyGrasp::TinyGrasp(const char* driver) :
	objectGrasped(NULL),
	object(NULL),
	obstacle(NULL)
{
	int argc = 1;
	char* argv [] = {"TinyGrasp"};
	tiny.reset(new TinyEx(argc, argv));

	// XML data
	XMLContext* context = tiny->getXMLContext()->getContextFirst("robot");	
	XMLData(armPose, context->getContextFirst("arm global_pose"));
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
	arm = dynamic_cast<KatanaArm*>(tiny->createActor(ActorDescPtr(pArmDesc)));
	if (arm == NULL)
		throw ExTiny("TinyGrasp::TinyGrasp(): Katana arm driver required!");

	// get sensor data assuming no object is in the gripper
	zero = arm->gripperRecvSensorData(100.0);

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

Mat34 TinyGrasp::getToolPose() {
	return arm->recvGenWorkspaceState(tiny->getTime()).pos;
}

void TinyGrasp::setObject(ActorDescPtr desc, RigidBody** object) {
	if (*object) {
		tiny->releaseActor(*object);
		*object = NULL;
	}
	if (desc != NULL) {
		RigidBodyDesc* pRigidBodyDesc = dynamic_cast<RigidBodyDesc*>(desc.get());
		if (pRigidBodyDesc == NULL)
			throw ExTiny("TinyGrasp::setObject(): Rigid body description required!");
		pRigidBodyDesc->kinematic = true;
		*object = dynamic_cast<RigidBody*>(tiny->createActor(desc));
	}
}

void TinyGrasp::setObject(const Mat34& pose, const Vec3& dimensions, RigidBody** object) {
	RigidBodyDesc* pObjectDesc = new RigidBodyDesc;
	pObjectDesc->kinematic = true;
	pObjectDesc->globalPose = pose;
	BoxShapeDesc* pObjectShapeDesc = new BoxShapeDesc;
	pObjectShapeDesc->dimensions = dimensions;
	pObjectDesc->shapes.push_back(ShapeDescPtr(pObjectShapeDesc));
	setObject(ActorDescPtr(pObjectDesc), object);
}

void TinyGrasp::setObstacle(const Mat34& pose, const Vec3& dimensions) {
	setObject(pose, dimensions, &obstacle);
}

void TinyGrasp::setObject(const Mat34& pose, const Vec3& dimensions) {
	setObject(pose, dimensions, &object);
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
	Mat34 toolPose = arm->getForwardTransform(arm->recvGenConfigspaceState(tiny->getTime()).pos).back();
	pBoxShapeDesc->localPose.setInverseRT(toolPose);
	pBoxShapeDesc->localPose.multiply(pBoxShapeDesc->localPose, object->getGlobalPose());

	Joint* effector = arm->getJoints().back();
	objectGrasped = effector->createShape(ShapeDescPtr(pBoxShapeDesc));

	tiny->releaseActor(object);
	object = NULL;
}

void TinyGrasp::releaseObject() {
	if (objectGrasped == NULL)
		throw ExTiny("TinyGrasp::releaseObject(): Object has not been attached");
	if (object)
		throw ExTiny("TinyGrasp::releaseObject(): Too many objects");

	// objectPose = toolPose * localPose
	Mat34 globalPose = arm->getForwardTransform(arm->recvGenConfigspaceState(tiny->getTime()).pos).back();
	globalPose.multiply(globalPose, objectGrasped->getLocalPose());
	BoxShapeDesc* pBoxShapeDesc = dynamic_cast<BoxShapeDesc*>(objectGrasped->getDesc().get());
	if (pBoxShapeDesc == NULL)
		throw ExTinyShape("TinyGrasp::releaseObject(): Must be box shape");
	setObject(globalPose, pBoxShapeDesc->dimensions);

	Joint* effector = arm->getJoints().back();
	effector->releaseShape(objectGrasped);
	objectGrasped = NULL;
}

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

void TinyGrasp::moveExec() {
	// compute trajectory using path planning with collision detection
	GenConfigspaceStateSeq trajectory;
	
	for (U32 i = 0;; ++i) {
		try {		
			tiny->print("TinyGrasp::moveExec(): %u...", i + 1); 
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

GraspPose::Seq TinyGrasp::getGraspPoses() const {
	if (object == NULL)
		throw ExTiny("TinyGrasp::getGraspPoses(): No object");

	BoxShapeDesc* pObjectShapeDesc = dynamic_cast<BoxShapeDesc*>(object->getShapes().back()->getDesc().get());
	if (pObjectShapeDesc == NULL)
		throw ExTinyShape("TinyGrasp::getGraspPoses(): Must be box shape");

	const Mat34 pose = object->getGlobalPose();
	const Vec3 dimensions = pObjectShapeDesc->dimensions;

	GraspPose g[2][2];
	Real min = numeric_const<Real>::MAX;
	for (U32 i = 0; i < 3; ++i) {
		if (min > dimensions[i]) {
			min = dimensions[i];
			
			for (U32 j = 0; j < 2; ++j) {
				const U32 k = (i + j + 1)%3;

				// rotation
				Mat33 rot[2];
				switch (k) {
				case 0:	// X, Y
					rot[0].rotZ(+REAL_PI_2);
					rot[1].rotZ(-REAL_PI_2);
					break;
				case 1:	// Y 
					rot[0].rotX(REAL_PI);
					rot[1].setId();
					break;
				case 2:	// Z, Y
					rot[0].rotX(-REAL_PI_2);
					rot[1].rotX(+REAL_PI_2);
					break;
				}

				// translation
				Vec3 p[2];

				p[0].setZero();
				p[0][k] = +(dimensions[k] + approachOffset);
				pose.R.multiply(p[0], p[0]);
				p[1].setZero();
				p[1][k] = -(dimensions[k] + approachOffset);
				pose.R.multiply(p[1], p[1]);

				g[j][0].approach.R.multiply(rot[0], pose.R);
				g[j][0].approach.p.add(p[0], pose.p);
				g[j][1].approach.R.multiply(rot[1], pose.R);
				g[j][1].approach.p.add(p[1], pose.p);

				p[0].setZero();
				p[0][k] = +(dimensions[k] + graspOffset);
				pose.R.multiply(p[0], p[0]);
				p[1].setZero();
				p[1][k] = -(dimensions[k] + graspOffset);
				pose.R.multiply(p[1], p[1]);

				g[j][0].grasp.R.multiply(rot[0], pose.R);
				g[j][0].grasp.p.add(p[0], pose.p);
				g[j][1].grasp.R.multiply(rot[1], pose.R);
				g[j][1].grasp.p.add(p[1], pose.p);
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
	tiny->render(debugRenderer);

	return poses;
}

GraspPose TinyGrasp::graspTry(const GraspPose::Seq& poses, GraspPose::Seq::const_iterator& index) {	
	index = poses.begin();
	PoseError errorMin(moveTry(index->grasp), index->grasp);

	for (GraspPose::Seq::const_iterator i = ++poses.begin(); i != poses.end(); ++i) {
		PoseError error(moveTry(i->grasp), i->grasp);
		if (error.ang < errorMin.ang && error.lin < graspError.lin || error.lin < errorMin.lin && error.ang < graspError.ang) {
			errorMin = error;
			index = i;
		}
	}

	return graspTry(*index);
}

GraspPose TinyGrasp::graspTry(const GraspPose& pose) {
	gend = pose;

	GraspPose actual;
	actual.grasp = moveTry(gend.grasp);
	actual.approach = moveTry(gend.approach);

	return actual;
}

void TinyGrasp::graspExec() {
	Mat34 actual;

	arm->gripperOpen(100.0);

	actual = moveTry(gend.approach);
	PoseError approachError(actual, gend.approach);
	tiny->print("TinyGrasp::graspExec(): approach error = (%f, %f)", approachError.lin, approachError.ang);
	moveExec();

	actual = moveTry(gend.grasp);
	PoseError graspError(actual, gend.grasp);
	tiny->print("TinyGrasp::graspExec(): grasp error = (%f, %f)", graspError.lin, graspError.ang);
	moveExec();

	Mat34 pose = read();
	grelease = toBody(pose, diff(pose, gend.approach));

	KatanaSensorDataSet threshold = zero;
	for (KatanaSensorDataSet::iterator i = zero.begin(); i != zero.end(); ++i) {
		tiny->print("sensor = (%i, %i)", i->index, i->value);
		i->value += 100;//sensorThreshold;
	}
	arm->gripperClose(threshold, 100.0);

	attachObject();
}

void TinyGrasp::graspRelease() {
	arm->gripperOpen(100.0);

	releaseObject();
	
	Mat34 pose = read();
	pose.multiply(fromBody(pose, grelease), pose);

	DebugRenderer debugRenderer;
	debugRenderer.addAxes(pose, Vec3(0.1));
	tiny->render(debugRenderer);

	(void)moveTry(pose);
	moveExec();

	debugRenderer.reset();
	tiny->render(debugRenderer);
}

//------------------------------------------------------------------------------

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
