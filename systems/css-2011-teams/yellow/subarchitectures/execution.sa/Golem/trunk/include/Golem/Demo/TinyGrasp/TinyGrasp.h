/** @file TinyGrasp.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_TINY_GRASP_H_
#define _GOLEM_DEMO_TINY_GRASP_H_

#include <Golem/Tiny/Tiny.h>
#include <Golem/Demo/TinyGrasp/TinyGrasp.h>
#include <Golem/Math/Quat.h>

//------------------------------------------------------------------------------

namespace golem {

class DebugRenderer;

namespace tiny {

//------------------------------------------------------------------------------

class PoseError {
public:
	Real lin;
	Real ang;

	PoseError(Real lin = REAL_ZERO, Real ang = REAL_ZERO);
	PoseError(const Mat34& a, const Mat34& b);

private:
	Real getLinearDist(const Vec3& v0, const Vec3& v1);
	Real getAngularDist(const Quat& q0, const Quat& q1);	
};

//------------------------------------------------------------------------------

class GraspPose {
public:
	typedef std::vector<GraspPose> Seq;

	Mat34 approach;
	Mat34 grasp;
};

//------------------------------------------------------------------------------

class Debug;

class TinyEx: public Tiny {
private:
	Debug* debug;

public:
	TinyEx(int argc, char *argv[]);
	void render(const DebugRenderer& debugRenderer);
	XMLContext* getXMLContext();
};

//------------------------------------------------------------------------------

class TinyGrasp {
public:
	TinyGrasp(const char* driver);

	void setObject(const Mat34& pose, const Vec3& dimensions);
	void setObstacle(const Mat34& pose, const Vec3& dimensions);
	
	Mat34 read();

	Mat34 moveTry(const Mat34& pose);
	void moveExec();

	GraspPose::Seq getGraspPoses() const;
	GraspPose graspTry(const GraspPose::Seq& poses, GraspPose::Seq::const_iterator& index);
	GraspPose graspTry(const GraspPose& pose);
	void graspExec();
	void graspRelease();

protected:
	shared_ptr<TinyEx> tiny;

	void setObject(ActorDescPtr desc, RigidBody** object);
	void setObject(const Mat34& pose, const Vec3& dimensions, RigidBody** object);

	Mat34 getToolPose();

	void attachObject();
	void releaseObject();

	/** Relative transformation: ab*a = b => ab = b*a^-1 */
	static Mat34 diff(const Mat34& a, const Mat34& b);
	/** Inertial frame G to body frame H given reference pose A: H = A^-1 G A */
	static Mat34 toBody(const Mat34& A, const Mat34& G);
	/** Inertial frame G from body frame H given reference pose A: G = A H A^-1 */
	static Mat34 fromBody(const Mat34& A, const Mat34& H);

private:
	GenConfigspaceState cbegin, cend;
	GenWorkspaceState end;
	GraspPose gend;
	Mat34 grelease;

	Mat34 armPose;
	KatanaArm* arm;
	Shape* objectGrasped;
	RigidBody* object;
	RigidBody* obstacle;

	U32 plannerTries;
	Real approachOffset;
	Real graspOffset;
	PoseError graspError;
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

#endif /*_GOLEM_DEMO_TINY_GRASP_H_*/
