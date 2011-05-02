/** @file TinyIce.ice
 * 
 * Golem TinyIce interface in ZeroC Ice.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#ifndef TINYICE_ICE
#define TINYICE_ICE

module golem {
module tinyice {

//----------------------------------------------------------------------------

/** Tiny exceptions */

exception ExTiny { string what; };

exception ExTinyCreate extends ExTiny {};

exception ExTinyActor extends ExTiny {};
exception ExTinyActorCreate extends ExTinyActor {};
exception ExTinyActorNotFound extends ExTinyActor {};
exception ExTinyActorNotRemovable extends ExTinyActor {};

exception ExTinyShape extends ExTiny {};
exception ExTinyShapeCreate extends ExTinyShape {};
exception ExTinyShapeNotFound extends ExTinyShape {};
exception ExTinyShapeNotRemovable extends ExTinyShape {};

exception ExTinyArm extends ExTiny {};
exception ExTinyArmSend extends ExTinyArm {};
exception ExTinyArmStop extends ExTinyArm {};
exception ExTinyArmFindTarget extends ExTinyArm {};
exception ExTinyArmFindTrajectory extends ExTinyArm {};

exception  ExTinyKatanaArm extends ExTinyArm {};
exception  ExTinyKatanaGripperNotPresent extends ExTinyKatanaArm {};
exception  ExTinyKatanaGripperIOError extends ExTinyKatanaArm {};

//----------------------------------------------------------------------------

sequence<int> IntSeq;
sequence<double> DoubleSeq;

/** 3 Vector (translation or point) */
struct Vec3 {
	double v1;
	double v2;
	double v3;
};
sequence<Vec3> Vec3Seq;

/** 3x3 Matrix (rigid body rotation) */
struct Mat33 {
	double m11;
	double m12;
	double m13;

	double m21;
	double m22;
	double m23;

	double m31;
	double m32;
	double m33;
};
sequence<Mat33> Mat33Seq;

/** 3x4 Matrix (rigid body transformation)  */
struct Mat34 {
	/** rotation matrix	*/
	Mat33 R;
	/** translation	*/
	Vec3 p;
};
sequence<Mat34> Mat34Seq;

/** Twist */
struct Twist {
	/** linear component */
	Vec3 v;
	/** angular component (rotation axis = w/|w| and rotation speed = |w|) */
	Vec3 w;
};
sequence<Twist> TwistSeq;

/** Manipulator Jacobian (in twist coordinates). */
struct Jacobian {
	/** coordinates */
	TwistSeq j;
};
sequence<Jacobian> JacobianSeq;

//----------------------------------------------------------------------------

/** Generalized workspace coordinates */
struct GenWorkspaceCoord {
	/** pose */
	Mat34 pos;
	/** velocity */
	Twist vel;
};
sequence<GenWorkspaceCoord> GenWorkspaceCoordSeq;

/** Generalized workspace state */
struct GenWorkspaceState {
	/** pose */
	Mat34 pos;
	/** velocity */
	Twist vel;
	/** time */
	double t;
};
sequence<GenWorkspaceState> GenWorkspaceStateSeq;


/** Configuration space coordinates */
struct ConfigspaceCoord {
	/** coordinates */
	DoubleSeq c;
};
sequence<ConfigspaceCoord> ConfigspaceCoordSeq;

/** Generalized configuration space coordinates */
struct GenConfigspaceCoord {
	/** position */
	ConfigspaceCoord pos;
	/** velocity */
	ConfigspaceCoord vel;
};
sequence<GenConfigspaceCoord> GenConfigspaceCoordSeq;

/** Generalized configuration space state */
struct GenConfigspaceState {
	/** position */
	ConfigspaceCoord pos;
	/** velocity */
	ConfigspaceCoord vel;
	/** time */
	double t;
};
sequence<GenConfigspaceState> GenConfigspaceStateSeq;

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
struct RGBA {
	float r;
	float g;
	float b;
	float a;
};

/** Shape description */
class ShapeDesc {
	/** Local pose */
	Mat34 localPose;
	/** Density */
	double density;
	/** Shape group */
	int group;
	/** Color */
	RGBA color;
};
sequence<ShapeDesc> ShapeDescSeq;

/** Shape interface */
interface Shape {
	/** Shape type */
	["cpp:const"] idempotent ShapeType getType();
	/** Shape color */
	["cpp:const"] idempotent RGBA getColor();
	/** Local pose */
	["cpp:const"] idempotent Mat34 getLocalPose();
	/** Returns collision group */
	["cpp:const"] idempotent int getGroup();
	/** Sets collision group */
	void setGroup(int group);
	/** Description */
	["cpp:const"] idempotent ShapeDesc getDesc();
};
sequence<Shape*> ShapeSeq;


/** Plane shape description */
class PlaneShapeDesc extends ShapeDesc {
	/** The plane normal vector. */
	Vec3 normal;
	/** The distance from the origin. */
	double distance;
};

/** Plane shape interface */
interface PlaneShape extends Shape {
	// TODO interface
};


/** Sphere shape description */
class SphereShapeDesc extends ShapeDesc {
	/** The radius of the sphere. */
	double radius;
};

/** Sphere shape interface */
interface SphereShape extends Shape {
	// TODO interface
};


/** Cylinder shape description */
class CylinderShapeDesc extends ShapeDesc {
	/** The radius of the cylinder. */
	double radius;
	/** The length of the cylinder. */
	double length;
};

/** Cylinder shape interface */
interface CylinderShape extends Shape {
	// TODO interface
};


/** Box shape description */
class BoxShapeDesc extends ShapeDesc {
	/** The dimensions are the radius of the shape, meaning 1/2 extents in each dimension. */
	Vec3 dimensions;
};

/** Box shape interface */
interface BoxShape extends Shape {
	// TODO interface
};


/** ConvexMesh shape description */
class ConvexMeshShapeDesc extends ShapeDesc {
	/** Vertices of the mesh to be transformed into convex hull */
	Vec3Seq vertices;
};

/** ConvexMesh shape interface */
interface ConvexMeshShape extends Shape {
	// TODO interface
};

//----------------------------------------------------------------------------

/** Actor description */
class ActorDesc {
	/** Global pose */
	Mat34 globalPose;
};

/** Actor interface */
interface Actor {
	/** Returns global pose */
	["cpp:const"] idempotent Mat34 getGlobalPose();
	/** Sets global pose */
	void setGlobalPose(Mat34 pose);
};
sequence<Actor*> ActorSeq;

//----------------------------------------------------------------------------

/** Rigid boody description */
class RigidBodyDesc extends ActorDesc {
	/** Shape descriptions */
	ShapeDescSeq shapes;
	/** Kinematic body not controlled by physics simulator */
	bool kinematic;
};

/** Rigid boody interface */
interface RigidBody extends Actor {
	/** Create a shape from description */
	Shape* createShape(ShapeDesc desc) throws ExTiny;
	/** Releases a given shape */
	void releaseShape(Shape* pShape) throws ExTiny;
	/** Returns shapes */
	["cpp:const"] idempotent ShapeSeq getShapes();
	
	/** Sets collision group for all shapes */
	void setGroup(int group);
};
sequence<RigidBody*> RigidBodySeq;

//----------------------------------------------------------------------------

/** Joint description */
class JointDesc extends RigidBodyDesc {
	// TODO empty for now
};

/** Joint interface */
interface Joint extends RigidBody {
	// TODO interface
};
sequence<Joint*> JointSeq;


/** Arm description */
class ArmDesc extends ActorDesc {
	/** Driver path: directory + name (without OS specific prefix and suffix) */
	string path;
};

/** Arm interface */
interface Arm extends Actor {
	/** Returns position at time t in configuration space coordinates */
	GenConfigspaceState recvGenConfigspaceState(double t) throws ExTinyArm;
	/** Returns position at time t in workspace coordinates */
	GenWorkspaceState recvGenWorkspaceState(double t) throws ExTinyArm;
	
	/** Sends trajectory and waits for movement completion: if timeWait=0 returns when movement starts, if timeWait=INF returns when movement finishes (blocking call). */
	void send(GenConfigspaceStateSeq trajectory, double timeWait) throws ExTinyArm;
	/** Stops the arm. */
	void stop() throws ExTinyArm;

	/** Finds (optimal) trajectory target in the obstacle-free configuration space (blocking call). */
	GenConfigspaceState findTarget(GenConfigspaceState begin, GenWorkspaceState end) throws ExTinyArm;
	/** Finds obstacle-free (optimal) trajectory in the configuration space from begin to end (blocking call). */
	GenConfigspaceStateSeq findTrajectory(GenConfigspaceState begin, GenConfigspaceState end) throws ExTinyArm;
	/** Finds obstacle-free straight line trajectory in the configuration space from begin towards end (blocking call). */
	GenConfigspaceStateSeq findConfigspaceTrajectory(GenConfigspaceState begin, GenConfigspaceState end) throws ExTinyArm;
	/** Finds obstacle-free straight line trajectory in the workspace from begin towards end (blocking call). */
	GenConfigspaceStateSeq findWorkspaceTrajectory(GenConfigspaceState begin, GenWorkspaceState end) throws ExTinyArm;

	/** Forward transformation - SE(3) transformations for each joint (joint frame poses -> base frame pose) */
	["cpp:const"] idempotent Mat34Seq getForwardTransform(ConfigspaceCoord cc);
	/** Manipulator (spatial) Jacobian */
	["cpp:const"] idempotent Jacobian getJacobian(ConfigspaceCoord cc);

	/** Returns group of the shapes of the arm */
	["cpp:const"] idempotent int getArmGroup();
	/** Returns group of the shapes which can collide with the arm */
	["cpp:const"] idempotent int getCollisionGroup();
	/** Sets group of the shapes which can collide with the arm */
	void setCollisionGroup(int collisionGroup);

	/** Arm joints */
	["cpp:const"] idempotent JointSeq getJoints();
	
	/** Returns reference pose */
	["cpp:const"] idempotent Mat34 getReferencePose();
	/** Sets reference pose */
	void setReferencePose(Mat34 pose);
	
	/** Arm controller time delta */
	["cpp:const"] idempotent double getTimeDelta();
	/** Arm controller asynchronous time delta */
	["cpp:const"] idempotent double getTimeDeltaAsync();
};

//----------------------------------------------------------------------------

/** Katana 300/450 sensor data */
struct KatanaSensorData {
	/** Sensor index */
	int index;
	/** Sensor value */
	int value;
};
sequence<KatanaSensorData> KatanaSensorDataSet;

/** Katana 300/450 gripper encoder data */
struct KatanaGripperEncoderData {
	/** Open gripper encoder value */
	int open;
	/** Closed gripper encoder value */
	int closed;
	/** Current gripper encoder value */
	int current;
};

/** Katana 300/450 arm description (creates KatanaArm interface) */
class KatanaArmDesc extends ArmDesc {
	/** Katana gripper */
	bool bGripper;
	/** Katana sensors */
	IntSeq sensorIndexSet;
};

/** Katana 300/450 arm interface */
interface KatanaArm extends Arm {
	/** Receives gripper sensor values */
	KatanaSensorDataSet gripperRecvSensorData(double timeOut) throws ExTinyKatanaArm;
	/** Receives gripper encoder values */
	KatanaGripperEncoderData gripperRecvEncoderData(double timeOut) throws ExTinyKatanaArm;
	/** Opens the gripper */
	void gripperOpen(double timeOut) throws ExTinyKatanaArm;
	/** Closes the gripper, stops if only a signal from any sensor is above the given threshold */
	void gripperClose(KatanaSensorDataSet sensorThreshold, double timeOut) throws ExTinyKatanaArm;
	/** Freezes the gripper */
	void gripperFreeze(double timeOut) throws ExTinyKatanaArm;
};

//----------------------------------------------------------------------------

interface Tiny {
	/** Current local time */
	["cpp:const"] double getTime();
	/** Sleeps for a given time */
	["cpp:const"] void sleep(double duration);
	
	/** Returns true if universe has been interrupted */
	bool interrupted();
	
	/** Create an Actor from description */
	Actor* createActor(ActorDesc desc) throws ExTiny;
	/** Releases a given Actor */
	void releaseActor(Actor* pActor) throws ExTiny;
	/** Returns Actors */
	["cpp:const"] ActorSeq getActors();
	
	/** Bang! */
	void bang();
};

//----------------------------------------------------------------------------

};
};

#endif /*TINYICE_ICE*/
