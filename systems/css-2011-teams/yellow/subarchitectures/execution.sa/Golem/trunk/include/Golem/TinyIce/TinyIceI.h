/** @file TinyIceI.h
 * 
 * Implementation of Golem Tiny Ice server (header file).
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_TINYICE_TINYICEI_H_
#define _GOLEM_TINYICE_TINYICEI_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/System.h>
#include <Golem/Defs/Pointers.h>
#include <Golem/Math/Collection.h>
#include <Ice/Ice.h>
#include <TinyIce/TinyIce.hh>

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
class KatanaArm;
class PhysPlanner;

namespace tinyice {
class BoundsDesc;
class PhysPlannerDesc;
};
};

//------------------------------------------------------------------------------

namespace golem {
namespace tinyice {

//------------------------------------------------------------------------------

class ShapeI;
class PlaneShapeI;
class SphereShapeI;
class CylinderShapeI;
class BoxShapeI;
class ConvexMeshShapeI;
class ActorI;
class RigidBodyI;
class JointI;
class ArmI;
class SimArmI;
class SixAxisSimArmI;
class KatanaSimArmI;
class KatanaArmI;
class TinyI;

typedef ::IceInternal::Handle<ShapeI> ShapeIPtr;
typedef ::IceInternal::Handle<JointI> JointIPtr;
typedef ::IceInternal::Handle<ActorI> ActorIPtr;

//------------------------------------------------------------------------------

class ShapeI : virtual public Shape {
private:
	friend class RigidBodyI;

	Ice::ObjectAdapterPtr& adapter;
	Ice::Identity id;
	
	NxShapeDesc* pNxShapeDesc;
	ShapeType type;
	double density;
	RGBA color;

protected:
	RigidBodyI& rigidBody;
	shared_ptr<BoundsDesc> pBoundsDesc;
	ShapeDescPtr pShapeDesc;
	const golem::Bounds* pBounds;
	bool owner;

	bool create(const ShapeDesc& desc, ShapeType type, const shared_ptr<BoundsDesc>& pBoundsDesc);
	ShapeI(RigidBodyI &rigidBodyI);

public:
	virtual ~ShapeI();
	
	virtual ShapePrx activate();
	
	virtual void deactivate();
    
	virtual ::golem::tinyice::ShapeType getType(const ::Ice::Current& = ::Ice::Current()) const;

    virtual ::golem::tinyice::RGBA getColor(const ::Ice::Current& = ::Ice::Current()) const;

    virtual ::golem::tinyice::Mat34 getLocalPose(const ::Ice::Current& = ::Ice::Current()) const;

    virtual ::Ice::Int getGroup(const ::Ice::Current& = ::Ice::Current()) const;

    virtual void setGroup(::Ice::Int, const ::Ice::Current& = ::Ice::Current());

    virtual ::golem::tinyice::ShapeDescPtr getDesc(const ::Ice::Current& = ::Ice::Current()) const;
};

class PlaneShapeI : public ShapeI, virtual public PlaneShape {
protected:
	friend class RigidBodyI;
	bool create(const PlaneShapeDesc& desc);
	PlaneShapeI(RigidBodyI &rigidBodyI);
public:
};

class SphereShapeI : public ShapeI, virtual public SphereShape {
protected:
	friend class RigidBodyI;
	bool create(const SphereShapeDesc& desc);
	SphereShapeI(RigidBodyI &rigidBodyI);
public:
};

class CylinderShapeI : public ShapeI, virtual public CylinderShape {
protected:
	friend class RigidBodyI;
	bool create(const CylinderShapeDesc& desc);
	CylinderShapeI(RigidBodyI &rigidBodyI);
public:
};

class BoxShapeI : public ShapeI, virtual public BoxShape {
protected:
	friend class RigidBodyI;
	bool create(const BoxShapeDesc& desc);
	BoxShapeI(RigidBodyI &rigidBodyI);
public:
};

class ConvexMeshShapeI : public ShapeI, virtual public ConvexMeshShape {
protected:
	friend class RigidBodyI;
	bool create(const ConvexMeshShapeDesc& desc);
	ConvexMeshShapeI(RigidBodyI &rigidBodyI);
public:
};

//------------------------------------------------------------------------------

class ActorI : virtual public Actor {
protected:
	friend class TinyI;
	
	Ice::ObjectAdapterPtr& adapter;
	Ice::Identity id;

	TinyI &tiny;
	Context& context;
	golem::Scene& scene;
	XMLContext* pXMLContext;
	bool owner;

	ActorI(TinyI &tiny);

public:
	virtual ~ActorI();
	
	virtual ActorPrx activate();
	
	virtual void deactivate();
};

class RigidBodyI : public ActorI, virtual public RigidBody {
protected:
	friend class ShapeI;
	friend class TinyI;
	typedef golem::PublicList<ShapePrx, ShapeIPtr> ShapeList;
	
	ShapeList shapeList;
	golem::Actor *pActor;
	bool kinematic;

	ShapeIPtr createShape(const ShapeDesc* pDesc);
	ShapeIPtr createShape(const golem::Bounds* pBounds);
	bool create(const RigidBodyDesc& desc);
	RigidBodyI(TinyI &tiny);
	
public:
	virtual ~RigidBodyI();
    
	virtual ActorPrx activate();
	
	virtual void deactivate();

    virtual ::golem::tinyice::ShapePrx createShape(const ::golem::tinyice::ShapeDescPtr&, const ::Ice::Current& = ::Ice::Current());

    virtual void releaseShape(const ::golem::tinyice::ShapePrx&, const ::Ice::Current& = ::Ice::Current());

	virtual ::golem::tinyice::ShapeSeq getShapes(const ::Ice::Current& = ::Ice::Current()) const;

	virtual ::golem::tinyice::Mat34 getGlobalPose(const ::Ice::Current& = ::Ice::Current()) const;

    virtual void setGlobalPose(const ::golem::tinyice::Mat34&, const ::Ice::Current& = ::Ice::Current());
    
    virtual void setGroup(::Ice::Int, const ::Ice::Current& = ::Ice::Current());
};

//------------------------------------------------------------------------------

class JointI : public RigidBodyI, virtual public Joint {
protected:
	friend class ArmI;
	ArmI &armI;

	bool create(const JointDesc& desc, golem::JointActor* pJointActor);
	JointI(ArmI &armI);
public:
};

class ArmI : public ActorI, virtual public Arm {
protected:
	friend class JointI;
	friend class TinyI;
	typedef golem::PublicList<ActorPrx, JointIPtr> JointList;
	
	golem::PhysPlanner *pPhysPlanner;
	JointList jointList;

	JointIPtr createJoint(const JointDesc* pDesc, golem::JointActor* pJointActor);
	bool create(const ArmDesc& desc);
	bool create(const PhysPlannerDesc& desc);
	ArmI(TinyI &tiny);

public:
	virtual ~ArmI();

	virtual ActorPrx activate();
	
	virtual void deactivate();

    virtual ::golem::tinyice::GenConfigspaceState recvGenConfigspaceState(Ice::Double, const ::Ice::Current& = ::Ice::Current());

    virtual ::golem::tinyice::GenWorkspaceState recvGenWorkspaceState(Ice::Double, const ::Ice::Current& = ::Ice::Current());

    virtual void send(const ::golem::tinyice::GenConfigspaceStateSeq&, ::Ice::Double, const ::Ice::Current& = ::Ice::Current());

    virtual void stop(const ::Ice::Current& = ::Ice::Current());

    virtual ::golem::tinyice::GenConfigspaceState findTarget(const ::golem::tinyice::GenConfigspaceState&, const ::golem::tinyice::GenWorkspaceState&, const ::Ice::Current& = ::Ice::Current());

    virtual ::golem::tinyice::GenConfigspaceStateSeq findTrajectory(const ::golem::tinyice::GenConfigspaceState&, const ::golem::tinyice::GenConfigspaceState&, const ::Ice::Current& = ::Ice::Current());

    virtual ::golem::tinyice::GenConfigspaceStateSeq findConfigspaceTrajectory(const ::golem::tinyice::GenConfigspaceState&, const ::golem::tinyice::GenConfigspaceState&, const ::Ice::Current& = ::Ice::Current());

    virtual ::golem::tinyice::GenConfigspaceStateSeq findWorkspaceTrajectory(const ::golem::tinyice::GenConfigspaceState&, const ::golem::tinyice::GenWorkspaceState&, const ::Ice::Current& = ::Ice::Current());

    virtual ::golem::tinyice::Mat34Seq getForwardTransform(const ::golem::tinyice::ConfigspaceCoord&, const ::Ice::Current& = ::Ice::Current()) const;

    virtual ::golem::tinyice::Jacobian getJacobian(const ::golem::tinyice::ConfigspaceCoord&, const ::Ice::Current& = ::Ice::Current()) const;

    virtual ::golem::tinyice::JointSeq getJoints(const ::Ice::Current& = ::Ice::Current()) const;

    virtual ::Ice::Int getArmGroup(const ::Ice::Current& = ::Ice::Current()) const;

    virtual ::Ice::Int getCollisionGroup(const ::Ice::Current& = ::Ice::Current()) const;

    virtual void setCollisionGroup(::Ice::Int, const ::Ice::Current& = ::Ice::Current());

	virtual ::golem::tinyice::Mat34 getGlobalPose(const ::Ice::Current& = ::Ice::Current()) const;

    virtual void setGlobalPose(const ::golem::tinyice::Mat34&, const ::Ice::Current& = ::Ice::Current());
    
	virtual ::golem::tinyice::Mat34 getReferencePose(const ::Ice::Current& = ::Ice::Current()) const;

    virtual void setReferencePose(const ::golem::tinyice::Mat34&, const ::Ice::Current& = ::Ice::Current());
	
	virtual ::Ice::Double getTimeDelta(const ::Ice::Current& = ::Ice::Current()) const;

    virtual ::Ice::Double getTimeDeltaAsync(const ::Ice::Current& = ::Ice::Current()) const;
};

//------------------------------------------------------------------------------

class KatanaArmI : public ArmI, virtual public KatanaArm {
protected:
	friend class TinyI;

	golem::KatanaGripper* pKatanaGripper;
	
	bool create(const KatanaArmDesc& desc);
	KatanaArmI(TinyI &tiny);

public:
    virtual ::golem::tinyice::KatanaSensorDataSet gripperRecvSensorData(::Ice::Double, const ::Ice::Current& = ::Ice::Current());

    virtual ::golem::tinyice::KatanaGripperEncoderData gripperRecvEncoderData(::Ice::Double, const ::Ice::Current& = ::Ice::Current());

    virtual void gripperOpen(::Ice::Double, const ::Ice::Current& = ::Ice::Current());

    virtual void gripperClose(const ::golem::tinyice::KatanaSensorDataSet&, ::Ice::Double, const ::Ice::Current& = ::Ice::Current());

    virtual void gripperFreeze(::Ice::Double, const ::Ice::Current& = ::Ice::Current());
};

//------------------------------------------------------------------------------

/** Golem Tiny - a "tiny" version of Golem framework
*/
class TinyI : virtual public Tiny {
private:
	friend class ActorI;
	typedef golem::PublicList<ActorPrx, ActorIPtr> ActorList;

	Ice::CommunicatorPtr& communicator;
	Ice::ObjectAdapterPtr adapter;
	Ice::Identity id;

	shared_ptr<XMLParser> pParser;
	XMLContext* pXMLContext;
	shared_ptr<Context> pContext;
	shared_ptr<golem::Universe> pUniverse;
	golem::Scene* pScene;
	ActorList actorList;

	ActorIPtr createActor(const ActorDesc* pDesc);

public:
	TinyI(int argc, char *argv[], Ice::CommunicatorPtr communicator);

	virtual ~TinyI();

	virtual void activate();
	
	virtual void deactivate();

	virtual ::Ice::Double getTime(const ::Ice::Current& = ::Ice::Current()) const;

    virtual void sleep(::Ice::Double, const ::Ice::Current& = ::Ice::Current()) const;
    
    virtual bool interrupted(const ::Ice::Current& = ::Ice::Current());
	
	virtual ::golem::tinyice::ActorPrx createActor(const ::golem::tinyice::ActorDescPtr&, const ::Ice::Current& = ::Ice::Current());

    virtual void releaseActor(const ::golem::tinyice::ActorPrx&, const ::Ice::Current& = ::Ice::Current());

    virtual ::golem::tinyice::ActorSeq getActors(const ::Ice::Current& = ::Ice::Current()) const;

    virtual void bang(const ::Ice::Current& = ::Ice::Current());

};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

#endif /*_GOLEM_TINYICE_TINYICEI_H_*/
