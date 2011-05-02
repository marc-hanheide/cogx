/** @file Tiny.h
 * 
 * Golem "Tiny" interface
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_TINY_TINY_H_
#define _GOLEM_TINY_TINY_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/System.h>
#include <Golem/Defs/Pointers.h>
#include <Golem/Defs/Desc.h>
#include <Golem/Math/Mat34.h>
#include <Golem/Math/Collection.h>
#include <Golem/Tools/Message.h>
#include <string>
#include <vector>

//------------------------------------------------------------------------------

class NxShapeDesc;

namespace golem {
class Context;
class XMLContext;
class XMLParser;
class Bounds;
class BoundingPlane;
class BoundingSphere;
class BoundingCylinder;
class BoundingBox;
class BoundingConvexMesh;
class Actor;
class Scene;
class Universe;
class JointActor;
class KatanaGripper;
class PhysPlanner;

namespace tiny {
class BoundsDesc;
class PhysPlannerDesc;
};
};

//------------------------------------------------------------------------------

namespace golem {
namespace tiny {

//------------------------------------------------------------------------------

/** Golem Tiny exceptions */
class ExTiny : virtual public Message {MESSAGE_BODY(ExTiny)};

class ExTinyCreate : public ExTiny {MESSAGE_BODY(ExTinyCreate)};

class ExTinyActor : public ExTiny {MESSAGE_BODY(ExTinyActor)};
class ExTinyActorCreate : public ExTinyActor {MESSAGE_BODY(ExTinyActorCreate)};
class ExTinyActorNotFound : public ExTinyActor {MESSAGE_BODY(ExTinyActorNotFound)};
class ExTinyActorNotRemovable : public ExTinyActor {MESSAGE_BODY(ExTinyActorNotRemovable)};

class ExTinyShape : public ExTiny {MESSAGE_BODY(ExTinyShape)};
class ExTinyShapeCreate : public ExTinyShape {MESSAGE_BODY(ExTinyShapeCreate)};
class ExTinyShapeNotFound : public ExTinyShape {MESSAGE_BODY(ExTinyShapeNotFound)};
class ExTinyShapeNotRemovable : public ExTinyShape {MESSAGE_BODY(ExTinyShapeNotRemovable)};

class ExTinyArm : public ExTiny {MESSAGE_BODY(ExTinyArm)};
class ExTinyArmSend : public ExTinyArm {MESSAGE_BODY(ExTinyArmSend)};
class ExTinyArmStop : public ExTinyArm {MESSAGE_BODY(ExTinyArmStop)};
class ExTinyArmFindTarget : public ExTinyArm {MESSAGE_BODY(ExTinyArmFindTarget)};
class ExTinyArmFindTrajectory : public ExTinyArm {MESSAGE_BODY(ExTinyArmFindTrajectory)};

class ExTinyKatanaArm : public ExTinyArm {MESSAGE_BODY(ExTinyKatanaArm)};
class ExTinyKatanaGripperNotPresent : public ExTinyKatanaArm {MESSAGE_BODY(ExTinyKatanaGripperNotPresent)};
class ExTinyKatanaGripperIOError : public ExTinyKatanaArm {MESSAGE_BODY(ExTinyKatanaGripperIOError)};

//------------------------------------------------------------------------------

class Shape;
class ShapeDesc;
class PlaneShapeDesc;
class SphereShapeDesc;
class CylinderShapeDesc;
class BoxShapeDesc;
class ConvexMeshShapeDesc;
class Actor;
class ActorDesc;
class RigidBody;
class RigidBodyDesc;
class Joint;
class JointDesc;
class Arm;
class ArmDesc;
class SimArm;
class SimArmDesc;
class SixAxisSimArm;
class SixAxisSimArmDesc;
class KatanaSimArm;
class KatanaSimArmDesc;
class KatanaArm;
class KatanaArmDesc;
class Tiny;

//------------------------------------------------------------------------------

typedef std::vector<char> CharSeq;
typedef std::vector<short> ShortSeq;
typedef std::vector<int> IntSeq;
typedef std::vector<double> DoubleSeq;
typedef std::vector<Vec3> Vec3Seq;
typedef std::vector<Mat33> Mat33Seq;
typedef std::vector<Mat34> Mat34Seq;
typedef std::vector<Twist> TwistSeq;

//------------------------------------------------------------------------------

/** Configuration space dimensions (equals golem::CONFIG_SPACE_DIM) */
const int CONFIG_SPACE_DIM = 6;

/** Workspace coordinates */
typedef Mat34 WorkspaceCoord;
typedef std::vector<WorkspaceCoord> WorkspaceCoordSeq;

/** Workspace velocity */
typedef Twist WorkspaceVel;
typedef std::vector<WorkspaceVel> WorkspaceVelSeq;

/** Manipulator Jacobian (in twist coordinates). */
class Jacobian {
public:
	Twist j[CONFIG_SPACE_DIM];
};

/** Generalized workspace coordinates */
class GenWorkspaceCoord {
public:
	GenWorkspaceCoord() {
		setToDefault();
	}
	void setToDefault() {
		pos.setId();
		vel.setZero();
	}
	/** pose */
	WorkspaceCoord pos;
	/** velocity */
	WorkspaceVel vel;
};
typedef std::vector<GenWorkspaceCoord> GenWorkspaceCoordSeq;

/** Generalized workspace state */
class GenWorkspaceState : public GenWorkspaceCoord {
public:
	GenWorkspaceState() {
		setToDefault();
	}
	void setToDefault() {
		GenWorkspaceCoord::setToDefault();
		t = 0.;
	}
	/** time */
	double t;
};
typedef std::vector<GenWorkspaceState> GenWorkspaceStateSeq;

/** Configuration space coordinates */
class ConfigspaceCoord {
public:
	ConfigspaceCoord() {
		setToDefault();
	}
	void setToDefault() {
		std::fill(c, c + CONFIG_SPACE_DIM, 0.);
	}
	/** coordinates */
	double c[CONFIG_SPACE_DIM];
};
typedef std::vector<ConfigspaceCoord> ConfigspaceCoordSeq;

/** Generalized configuration space coordinates */
class GenConfigspaceCoord {
public:
	GenConfigspaceCoord() {
		setToDefault();
	}
	void setToDefault() {
		pos.setToDefault();
		vel.setToDefault();
	}
	/** position */
	ConfigspaceCoord pos;
	/** velocity */
	ConfigspaceCoord vel;
};
typedef std::vector<GenConfigspaceCoord> GenConfigspaceCoordSeq;

/** Generalized configuration space state */
class GenConfigspaceState : public GenConfigspaceCoord {
public:
	GenConfigspaceState() {
		setToDefault();
	}
	void setToDefault() {
		GenConfigspaceCoord::setToDefault();
		t = 0.;
	}
	/** time */
	double t;
};
typedef std::vector<GenConfigspaceState> GenConfigspaceStateSeq;

//----------------------------------------------------------------------------

/** Shape types. */
enum ShapeType {
	/** plane */
	ShapeTypePlane,
	/** sphere */
	ShapeTypeSphere,
	/** cylinder */
	ShapeTypeCylinder,
	/** capsule */
	ShapeTypeCapsule,
	/** parallelepiped */
	ShapeTypeBox,
	/** generic triangle mesh */
	ShapeTypeTriangleMesh,
	/** convex triangle mesh */
	ShapeTypeConvexMesh,
};

/** RGBA Color */
class RGBA {
public:
	RGBA() {
		setToDefault();
	}
	RGBA(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {
	}
	void setToDefault() {
		r = 1.;
		g = 1.;
		b = 1.;
		a = 1.;
	}
	float r;
	float g;
	float b;
	float a;
};

typedef shared_ptr<Shape> ShapePtr;
typedef std::vector<Shape*> ShapeSeq;

typedef shared_ptr<ShapeDesc> ShapeDescPtr;
typedef std::vector<ShapeDescPtr> ShapeDescSeq;

/** Shape interface */
class Shape {
private:
	friend class RigidBody;
	NxShapeDesc* pNxShapeDesc;
	ShapeType type;
	double density;
	RGBA color;

protected:
	RigidBody& rigidBody;
	shared_ptr<BoundsDesc> pBoundsDesc;
	ShapeDescPtr pShapeDesc;
	const Bounds* pBounds;
	bool owner;

	bool create(const ShapeDesc& desc, ShapeType type, const shared_ptr<BoundsDesc>& pBoundsDesc);
	Shape(RigidBody& rigidBody);
	
public:
	virtual ~Shape();
	/** Shape type */
	ShapeType getType() const;
	/** Shape color */
	RGBA getColor() const;
	/** Local pose */
	Mat34 getLocalPose() const;
	/** Returns shape group */
	int getGroup() const;
	/** Sets shape group */
	void setGroup(int group);
	/** Description */
	ShapeDescPtr getDesc() const;
};

/** Shape description */
class ShapeDesc {
protected:
	friend class RigidBody;

	virtual ShapePtr create(RigidBody& rigidBody) const = 0;
	ShapeDesc() {
		setToDefault();
	}

public:
	virtual ~ShapeDesc() {}
	void setToDefault() {
		localPose.setId();
		density = 1.;
		group = 1<<0;
		color.setToDefault();
	}
	/** Local pose */
	Mat34 localPose;
	/** Density */
	double density;
	/** Shape group */
	int group;
	/** Color */
	RGBA color;
};

/** Plane shape */
class PlaneShape : public Shape {
protected:
	friend class PlaneShapeDesc;
	const BoundingPlane* pBoundingPlane;

	bool create(const PlaneShapeDesc& desc);
	PlaneShape(RigidBody& rigidBody);
public:
};

/** Plane shape description */
class PlaneShapeDesc : public ShapeDesc {
protected:
	CREATE_FROM_OBJECT_DESC1(PlaneShape, ShapePtr, RigidBody&)
	virtual ~PlaneShapeDesc() {}
public:
	PlaneShapeDesc() {
		setToDefault();
	}
	void setToDefault() {
		ShapeDesc::setToDefault();
		normal.set((Real)0.0, (Real)0.0, (Real)1.0);
		distance = 0.;
	}
	/** The plane normal vector. */
	Vec3 normal;
	/** The distance from the origin. */
	double distance;
};

/** Sphere shape */
class SphereShape : public Shape {
protected:
	friend class SphereShapeDesc;
	const BoundingSphere* pBoundingSphere;
	
	bool create(const SphereShapeDesc& desc);
	SphereShape(RigidBody& rigidBody);
public:
};

/** Sphere shape description */
class SphereShapeDesc : public ShapeDesc {
protected:
	CREATE_FROM_OBJECT_DESC1(SphereShape, ShapePtr, RigidBody&)
	virtual ~SphereShapeDesc() {}
public:
	SphereShapeDesc() {
		setToDefault();
	}
	void setToDefault() {
		ShapeDesc::setToDefault();
		radius = 0.1;
	}
	/** The radius of the sphere. */
	double radius;
};

/** Cylinder shape */
class CylinderShape : public Shape {
protected:
	friend class CylinderShapeDesc;
	const BoundingCylinder* pBoundingCylinder;
	
	bool create(const CylinderShapeDesc& desc);
	CylinderShape(RigidBody& rigidBody);
public:
};

/** Cylinder shape description */
class CylinderShapeDesc : public ShapeDesc {
protected:
	CREATE_FROM_OBJECT_DESC1(CylinderShape, ShapePtr, RigidBody&)
	virtual ~CylinderShapeDesc() {}
public:
	CylinderShapeDesc() {
		setToDefault();
	}
	void setToDefault() {
		ShapeDesc::setToDefault();
		radius = 0.1;
		length = 0.1;
	}
	/** The radius of the cylinder. */
	double radius;
	/** The length of the cylinder. */
	double length;
};

/** Box shape */
class BoxShape : public Shape {
protected:
	friend class BoxShapeDesc;
	const BoundingBox* pBoundingBox;
	
	bool create(const BoxShapeDesc& desc);
	BoxShape(RigidBody& rigidBody);
public:
};

/** Box shape description */
class BoxShapeDesc : public ShapeDesc {
protected:
	CREATE_FROM_OBJECT_DESC1(BoxShape, ShapePtr, RigidBody&)
	virtual ~BoxShapeDesc() {}
public:
	BoxShapeDesc() {
		setToDefault();
	}
	void setToDefault() {
		ShapeDesc::setToDefault();
		dimensions.set((Real)0.1, (Real)0.1, (Real)0.1);
	}
	/** The dimensions are the radius of the shape, meaning 1/2 extents in each dimension. */
	Vec3 dimensions;
};

/** ConvexMesh shape */
class ConvexMeshShape : public Shape {
protected:
	friend class ConvexMeshShapeDesc;
	const BoundingConvexMesh* pBoundingConvexMesh;
	
	bool create(const ConvexMeshShapeDesc& desc);
	ConvexMeshShape(RigidBody& rigidBody);
public:
};

/** ConvexMesh shape description */
class ConvexMeshShapeDesc : public ShapeDesc {
protected:
	CREATE_FROM_OBJECT_DESC1(ConvexMeshShape, ShapePtr, RigidBody&)
	virtual ~ConvexMeshShapeDesc() {}
public:
	ConvexMeshShapeDesc() {
		setToDefault();
	}
	void setToDefault() {
		ShapeDesc::setToDefault();
		vertices.clear();
	}
	/** Vertices of the mesh to be transformed into convex hull */
	Vec3Seq vertices;
};

//----------------------------------------------------------------------------

typedef shared_ptr<Actor> ActorPtr;
typedef std::vector<Actor*> ActorSeq;

typedef shared_ptr<ActorDesc> ActorDescPtr;
typedef std::vector<ActorDescPtr> ActorDescSeq;

/** Actor interface */
class Actor {
protected:
	friend class ActorDesc;
	friend class Tiny;
	Tiny& tiny;
	Context& context;
	golem::Scene& scene;
	XMLContext* pXMLContext;
	bool owner;

	Actor(Tiny& tiny);

public:
	virtual ~Actor();
	/** Returns global pose */
	virtual Mat34 getGlobalPose() const = 0;
	/** Sets global pose */
	virtual void setGlobalPose(const Mat34& pose) = 0;
};

/** Actor description */
class ActorDesc {
protected:
	friend class Tiny;
	virtual ActorPtr create(Tiny& tiny) const = 0;
	ActorDesc() {
		setToDefault();
	}
public:
	virtual ~ActorDesc() {}
	void setToDefault() {
		globalPose.setId();
	}
	/** Global pose */
	Mat34 globalPose;
};


/** Rigid boody interface */
class RigidBody : public Actor {
protected:
	friend class RigidBodyDesc;
	friend class Shape;
	typedef golem::PrivateList<Shape*, ShapePtr> ShapeList;

	ShapeList shapeList;
	golem::Actor *pActor;
	bool kinematic;
	
	ShapePtr createShape(const Bounds* pBounds);
	bool create(const RigidBodyDesc& desc);
	RigidBody(Tiny& tiny);
	virtual ~RigidBody();

public:
	/** Create a shape from description */
	Shape* createShape(const ShapeDescPtr& pShapeDesc);
	/** Releases a given shape */
	void releaseShape(Shape* shape);
	/** Returns shapes */
	ShapeSeq getShapes() const;

	/** Returns global pose */
	virtual Mat34 getGlobalPose() const;
	/** Sets global pose */
	virtual void setGlobalPose(const Mat34& pose);
	
	/** Sets group for all shapes */
	void setGroup(int group);
};

/** Rigid boody description */
class RigidBodyDesc : public ActorDesc {
protected:
	CREATE_FROM_OBJECT_DESC1(RigidBody, ActorPtr, Tiny&)
	virtual ~RigidBodyDesc() {}
public:
	RigidBodyDesc() {
		setToDefault();
	}
	void setToDefault() {
		ActorDesc::setToDefault();
		shapes.clear();
		kinematic = false;
	}
	/** Shape descriptions */
	ShapeDescSeq shapes;
	/** Kinematic body not controlled by physics simulator */
	bool kinematic;
};

//----------------------------------------------------------------------------

typedef shared_ptr<Joint> JointPtr;
typedef std::vector<Joint*> JointSeq;

typedef shared_ptr<JointDesc> JointDescPtr;
typedef std::vector<JointDescPtr> JointDescSeq;

/** Joint interface */
class Joint : public RigidBody {
protected:
	friend class JointDesc;
	Arm& arm;

	bool create(const JointDesc& desc);
	Joint(Arm& arm);
public:
	// TODO empty for now
};

/** Joint description */
class JointDesc : public RigidBodyDesc {
protected:
	friend class Arm;
	friend class Joint;
	golem::JointActor* pJointActor;
	
	virtual ActorPtr create(Tiny& tiny) const;
	CREATE_FROM_OBJECT_DESC1(Joint, JointPtr, Arm&)
	virtual ~JointDesc() {}

public:
	JointDesc() {
		setToDefault();
	}
	void setToDefault() {
		RigidBodyDesc::setToDefault();
		pJointActor = 0;
	}
};

/** Arm interface */
class Arm : public Actor {
protected:
	friend class ArmDesc;
	friend class Joint;
	typedef golem::PrivateList<Joint*, JointPtr> JointList;

	golem::PhysPlanner *pPhysPlanner;
	JointList jointList;

	bool create(const ArmDesc& desc);
	bool create(const PhysPlannerDesc& desc);
	Arm(Tiny& tiny);
	virtual ~Arm();

public:
	/** Returns position at time t in configuration space coordinates (non-blocking call) */
	GenConfigspaceState recvGenConfigspaceState(double t) const;
	/** Returns position at time t in workspace coordinates (non-blocking call) */
	GenWorkspaceState recvGenWorkspaceState(double t) const;
	
	/** Sends trajectory and waits for movement completion: if timeWait=0 returns when movement starts, if timeWait=INF returns when movement finishes (blocking call). */
	void send(const GenConfigspaceStateSeq& trajectory, double timeWait);	
	/** Stops the arm. */
	void stop();

	/** Finds (optimal) trajectory target in the obstacle-free configuration space (blocking call). */
	GenConfigspaceState findTarget(const GenConfigspaceState &begin, const GenWorkspaceState& end);
	/** Finds obstacle-free (optimal) trajectory in the configuration space from begin to end (blocking call). */
	GenConfigspaceStateSeq findTrajectory(const GenConfigspaceState &begin, const GenConfigspaceState &end);
	/** Finds obstacle-free straight line trajectory in the configuration space from begin towards end (blocking call). */
	GenConfigspaceStateSeq findConfigspaceTrajectory(const GenConfigspaceState &begin, const GenConfigspaceState &end);
	/** Finds obstacle-free straight line trajectory in the workspace from begin towards end (blocking call). */
	GenConfigspaceStateSeq findWorkspaceTrajectory(const GenConfigspaceState &begin, const GenWorkspaceState &end);
	
	/** Forward transformation - SE(3) transformations for each joint (joint frame poses -> base frame pose) */
	Mat34Seq getForwardTransform(const ConfigspaceCoord& cc) const;
	/** Manipulator (spatial) Jacobian */
	Jacobian getJacobian(const ConfigspaceCoord& cc) const;

	/** Returns group of the shapes of the arm */
	int getArmGroup() const;
	/** Returns group of the shapes which can collide with the arm */
	int getCollisionGroup() const;
	/** Sets group of the shapes which can collide with the arm */
	void setCollisionGroup(int group);

	/** Arm joints */
	JointSeq getJoints() const;
	
	/** Returns global pose */
	virtual Mat34 getGlobalPose() const;
	/** Sets global pose */
	virtual void setGlobalPose(const Mat34& pose);

	/** Returns reference pose */
	Mat34 getReferencePose() const;
	/** Sets reference pose */
	void setReferencePose(const Mat34& pose);
	
	/** Arm controller time delta */
	double getTimeDelta() const;
	/** Arm controller asynchronous time delta */
	double getTimeDeltaAsync() const;
};

/** Arm description */
class ArmDesc : public ActorDesc {
protected:
	CREATE_FROM_OBJECT_DESC1(Arm, ActorPtr, Tiny&)
	virtual ~ArmDesc() {}

public:
	ArmDesc() {
		setToDefault();
	}
	void setToDefault() {
		ActorDesc::setToDefault();
		path = "GolemDeviceKatana300Sim"; // Katana 300 simulator
	}
	/** Driver path: directory + name (without OS specific prefix and suffix) */
	std::string path;
};

//----------------------------------------------------------------------------

/** Katana 300/450 sensor data */
class KatanaSensorData {
public:
	/** Sensor index */
	int index;
	/** Sensor value */
	int value;
};
typedef std::vector<KatanaSensorData> KatanaSensorDataSet;

/** Katana 300/450 gripper encoder data */
class KatanaGripperEncoderData {
public:
	/** Open gripper encoder value */
	int open;
	/** Closed gripper encoder value */
	int closed;
	/** Current gripper encoder value */
	int current;
};

/** Katana 300/450 arm interface */
class KatanaArm : public Arm {
protected:
	friend class KatanaArmDesc;

	golem::KatanaGripper* pKatanaGripper;

	bool create(const KatanaArmDesc& desc);
	KatanaArm(Tiny& tiny);

public:
	/** Receives gripper sensor values */
	KatanaSensorDataSet gripperRecvSensorData(double timeOut);
	/** Receives gripper encoder values */
	KatanaGripperEncoderData gripperRecvEncoderData(double timeOut);
	/** Opens the gripper */
	void gripperOpen(double timeOut);
	/** Closes the gripper, stops if only a signal from any sensor is above the given threshold */
	void gripperClose(const KatanaSensorDataSet& sensorThreshold, double timeOut);
	/** Freezes the gripper */
	void gripperFreeze(double timeOut);
};

/** Katana 300/450 arm description (creates KatanaArm interface) */
class KatanaArmDesc : public ArmDesc {
protected:
	CREATE_FROM_OBJECT_DESC1(KatanaArm, ActorPtr, Tiny&)
	virtual ~KatanaArmDesc() {}

public:
	KatanaArmDesc() {
		setToDefault();
	}
	void setToDefault() {
		ArmDesc::setToDefault();
		path = "GolemDeviceKatana300"; // Katana 300
		bGripper = false;
		sensorIndexSet.clear();
		// Katana Finger Type S03.02, force sensors
		sensorIndexSet.push_back(7);	// Right finger, Front
		sensorIndexSet.push_back(15);	// Left finger, Front
		sensorIndexSet.push_back(6);	// Right finger, Rear
		sensorIndexSet.push_back(14);	// Left finger, Rear
	}
	/** Katana gripper */
	bool bGripper;
	/** Katana sensors */
	IntSeq sensorIndexSet;
};

//------------------------------------------------------------------------------

/** Golem Tiny - a "tiny" version of Golem framework
*/
class Tiny {
protected:
	friend class Actor;
	typedef golem::PrivateList<Actor*, ActorPtr> ActorList;

	golem::shared_ptr<XMLParser> pParser;
	XMLContext* pXMLContext;
	golem::shared_ptr<Context> pContext;
	golem::shared_ptr<golem::Universe> pUniverse;
	golem::Scene* pScene;
	ActorList actorList;

public:
	/** Constructs Golem Tiny */
	Tiny(int argc, char *argv[]);
	/** Releases Golem Tiny objects */
	~Tiny();
	
	/** Current local time */
	double getTime() const;
	/** Sleeps for a given time */
	void sleep(double duration) const;
	/** Prints a message */
	void print(const char* format, ...);
	/** Interrupted */
	bool interrupted() const;
	/** Read a key, wait no longer than timeOut */
	int waitKey(double timeOut = numeric_const<double>::MAX);

	/** XML context (default XML configuration file) */
	const golem::XMLContext* getXMLContext() const;
	golem::XMLContext* getXMLContext();
	/** Golem context */
	const golem::Context* getContext() const;
	golem::Context* getContext();
	
	/** Create an Actor from description */
	Actor* createActor(const ActorDescPtr& pActorDesc);
	/** Releases a given Actor */
	void releaseActor(Actor* actor);
	/** Returns Actors */
	ActorSeq getActors() const;
	
	/** Bang! */
	void bang();
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

#endif /*_GOLEM_TINY_TINY_H_*/
