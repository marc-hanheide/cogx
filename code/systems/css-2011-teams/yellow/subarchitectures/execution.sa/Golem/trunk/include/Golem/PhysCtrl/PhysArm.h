/** @file PhysArm.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_PHYSCTRL_PHYSARM_H_
#define _GOLEM_PHYSCTRL_PHYSARM_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Arm.h>
#include <Golem/Phys/Universe.h>
#include <Golem/Phys/Renderer.h>
#include <map>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class PhysArm;

//------------------------------------------------------------------------------

/** Renders the current and the destination arm pose */
class PoseRenderer : public DebugRenderer {
public:
	typedef shared_ptr<PoseRenderer> Ptr;

	PhysArm &physArm;

	/** PoseRenderer description */
	class Desc {
	public:
		/** Current and destination pose axes size */
		Vec3 axesSize;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			axesSize.set(Real(0.1));
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!axesSize.isFinite())
				return false;
			
			return true;
		}
	};

	PoseRenderer(PhysArm &physArm);
	bool create(const Desc &desc);
};

//------------------------------------------------------------------------------

/** Renders arm path */
class PathRenderer : public DebugRenderer {
public:
	typedef shared_ptr<PathRenderer> Ptr;
	typedef std::vector<GenConfigspaceState> States;

	PhysArm &physArm;
	States states;

	/** PathRenderer description */
	class Desc {
	public:
		/** Arm trajectory interval [tmCurrent + tmDelta0, tmCurrent + tmDelta1] */
		SecTmReal tmDelta0, tmDelta1;
		/** Arm trajectory segments */
		U32 trjSegments;
		/** Arm trajectory grids per time delta */
		U32 trjGrid;
		/** Trajectory colour */
		RGBA trjColour;
		/** Current and destination pose axes size */
		Vec3 axesSize;
		/** Display trajectory */
		bool trjShow;
		/** Display trajectory pose axes */
		bool axesShow;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			tmDelta0 = SecTmReal(-0.0); // 1 sec history
			tmDelta1 = SEC_TM_REAL_MAX; // all planned trajectory
			trjSegments = 1000;
			trjGrid = 0;
			trjColour = RGBA::WHITE;
			axesSize.set(Real(0.025));
			trjShow = true;
			axesShow = false;
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (tmDelta0 > tmDelta1)
				return false;
			if (trjSegments < 2 || !axesSize.isFinite())
				return false;
			
			return true;
		}
	};

	PathRenderer(PhysArm &physArm);
	bool create(const Desc &desc);
};

//------------------------------------------------------------------------------

/** Joint Actor */
class JointActor : public Actor, public Joint::Callback {
public:
	typedef std::vector<JointActor*> Seq;
	
	/** Object description */
	class Desc : public Actor::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(JointActor, Object::Ptr, Scene&)

	public:
		/** Pointer to Arm Joint */
		Joint* pJoint;
		/** Pointer to PhysArm */
		PhysArm* pPhysArm;

		/** Constructs Actor description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the objet parameters to the default values */
		virtual void setToDefault() {
			Actor::Desc::setToDefault();

			pJoint = NULL;
			pPhysArm = NULL;
		}

		/** Checks if the object description is valid. */
		virtual bool isValid() const {
			if (!Actor::Desc::isValid())
				return false;
			if (pJoint == NULL || pPhysArm == NULL)
				return false;

			return true;
		}
	};

protected:
	/** Pointer to Arm Joint */
	Joint* pJoint;
	/** Pointer to PhysArm */
	PhysArm* pPhysArm;

	/** Creates Actor from description. */
	bool create(const JointActor::Desc& desc);

	/** Objects can be constructed only within given Scene. */
	JointActor(Scene &scene);

public:
	/** Creates the bounds in the local coordinate frame */
	virtual const Bounds* createBounds(Bounds::Desc::Ptr pDesc);

	/** Removes the specified bounds */
	virtual void releaseBounds(const Bounds& bounds);

	virtual void syncJointBoundsDesc();

	/** Pointer to Arm Joint */
	inline const Joint* getJoint() const {
		return pJoint;
	}
};

//------------------------------------------------------------------------------

class PhysArm : public Object {
public:
	typedef shared_ptr<PhysArm> Ptr;
	friend class JointActor;
	
	/** Object description */
	class Desc : public Object::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(PhysArm, Object::Ptr, Scene&)

	public:
		/** Arm driver path */
		std::string driver;
		/** Arm description */
		Arm::Desc::Ptr pArmDesc;
		/** Auto Synchronization of Arm joint bounds descriptions */
		bool autoSyncArmBoundsDesc;

		/** Arm appearance */
		Actor::Appearance appearance;
		/** Pose renderer */
		PoseRenderer::Desc poseRendererDesc;
		/** Pose renderer state */
		bool poseShow;
		/** Path renderer */
		PathRenderer::Desc pathRendererDesc;
		/** Path renderer state */
		bool pathShow;
		
		/** Constructs PhysArm description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Object::Desc::setToDefault();

			driver.clear();
			pArmDesc.release();
			autoSyncArmBoundsDesc = true;

			appearance.solidColour.set(192, 192, 0, 100); // set yellow
			appearance.wireColour.set(127, 127, 127, 255); // set grey
			poseRendererDesc.setToDefault();
			poseShow = true;
			pathRendererDesc.setToDefault();
			pathShow = true;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Object::Desc::isValid())
				return false;
			if (pArmDesc != NULL ? !pArmDesc->isValid() : driver.empty())
				return false;
			if (!appearance.isValid() || !poseRendererDesc.isValid() || !pathRendererDesc.isValid())
				return false;

			return true;
		}
	};

protected:
	/** Arm interface */
	Arm::Ptr pArm;
	
	/** Arm bounds group */
	U32 armGroup;
	/** Collision bounds group */
	U32 collisionGroup;
	/** Auto Synchronization of Arm joint bounds descriptions */
	bool autoSyncArmBoundsDesc;

	/** Arm appearance */
	Actor::Appearance appearance;
	/** Pose renderer */
	PoseRenderer::Desc poseRendererDesc;
	PoseRenderer::Ptr pPoseRenderer;
	bool poseShow;
	/** Path renderer */
	PathRenderer::Desc pathRendererDesc;
	PathRenderer::Ptr pPathRenderer;
	bool pathShow;

	JointActor::Seq jointActors;
	std::vector<Mat34> jointPoses;
	GenConfigspaceState state;

	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);
	
	/** (Pre)processing function called BEFORE every physics simulation step and before randering. */
	virtual void preprocess(SecTmReal elapsedTime);
	
	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(SecTmReal elapsedTime);
	
	/** Renders the PhysArm. */
	virtual void render();

	/** Synchronise bounds of all joints with the bounds descriptions (@see Heuristic#syncArmBoundsDesc()) */
	virtual void syncArmBoundsDesc() {}
	
	/** Creates Joint Actor. */
	virtual void createJointActor(JointActor *&pJointActor, Joint* pJoint);
	
	/** Synchronise Joint Actors pose with the arm pose. */
	bool syncJointActorsPose(SecTmReal t, bool bMove = true);
	
	/** Creates PhysArm. */
	bool create(const Desc& desc);
	
	/** Releases resources */
	virtual void release();

	/** Objects can be constructed only in the Scene context. */
	PhysArm(Scene &scene);

public:
	/** Destructor is inaccesible */
	virtual ~PhysArm();
	
	/** Returns collection of Arm bounds 
	 * @return				pointer to the collection of bounds
	*/
	virtual Bounds::SeqPtr getArmBounds() const;

	/** Returns collection of Actors bounds (excluding Joint Actors).
	* @param group			bounds group
	* @return				pointer to the collection of bounds
	*/
	virtual Bounds::SeqPtr getCollisionBounds() const;
	
	/** Returns bounds group of the Arm
	 * @return				bounds group
	*/
	virtual U32 getArmBoundsGroup() const;

	/** Returns bounds group of the Actors which can collide with the Arm
	 * @return				bounds group
	*/
	virtual U32 getCollisionBoundsGroup() const;

	/** Sets bounds group of the Actors which can collide with the Arm
	 * @param	collisionGroup	bounds group
	*/
	virtual void setCollisionBoundsGroup(U32 collisionGroup);

	/** Access to Joint Actors
	 * @return		reference to PhysJoint collection
	 */
	inline const JointActor::Seq& getJointActors() const {
		return jointActors;
	}

	/** Returns the arm controller */
	inline const Arm &getArm() const {
		return *pArm;
	}
	inline Arm &getArm() {
		return *pArm;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYSCTRL_PHYSARM_H_*/
