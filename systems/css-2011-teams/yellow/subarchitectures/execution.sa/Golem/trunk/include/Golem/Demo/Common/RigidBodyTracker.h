/** @file RigidBodyTracker.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_RIGIDBODYTRACKER_H_
#define _GOLEM_DEMO_COMMON_RIGIDBODYTRACKER_H_

//------------------------------------------------------------------------------

#include <Golem/Phys/Renderer.h>
#include <Golem/Demo/Common/Embodiment.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Rigid body data */
struct RigidBodyData {
	/** Data sequence */
	typedef std::vector<RigidBodyData> Seq;

	/** Set of bounds of the object. */
	Bounds::SeqPtr pBoundsSeq;
	/** Reference pose of the object. */
	//Mat34 pose;
	/** Object colour */
	RGBA colour;
};

//------------------------------------------------------------------------------

class RigidBodyTracker;

/** Renders bounds */
class RigidBodyRenderer : public DebugRenderer {
public:
	typedef shared_ptr<RigidBodyRenderer> Ptr;

	/** Renderer description */
	class Desc {
	public:
		/** Rigid body data to render */
		const RigidBodyData *pRigidBodyData;

		/** Body colour */
		RGBA bodyColour;
		/** Display position point */
		bool bodyShow;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			pRigidBodyData = NULL;
			bodyColour = RGBA::YELLOW;
			bodyShow = true;
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			//if (pRigidBodyData == NULL)
			//	return false;
			
			return true;
		}
	};

	bool create(const Desc &desc);
};

//------------------------------------------------------------------------------

/** RigidBodyTracker
*/
class RigidBodyTracker : public Sensor<RigidBodyData> {
public:
	typedef shared_ptr<RigidBodyTracker> Ptr;
	typedef Sensor<RigidBodyData> Base;

	/** Object description */
	class Desc : public Base::Desc {
	public:
		/** Rigid body renderer description */
		RigidBodyRenderer::Desc rigidBodyRendererDesc;
		/** Rigid body renderer state */
		bool rigidBodyShow;
		
		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Base::Desc::setToDefault();
			
			rigidBodyRendererDesc.setToDefault();
			rigidBodyShow = false;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Base::Desc::isValid())
				return false;

			if (!rigidBodyRendererDesc.isValid())
				return false;
			
			return true;
		}
	};

protected:
	/** Rigid body renderer */
	RigidBodyRenderer::Ptr pRigidBodyRenderer;
	RigidBodyRenderer::Desc rigidBodyRendererDesc;
	bool rigidBodyShow;

	/** Current bounds collection */
	RigidBodyData rigidBodyData;
	SecTmReal timeStamp;
	Event evWait;
	/** Critical section */
	CriticalSection cs;

	/** Reads the current sensory data */
	virtual bool get(RigidBodyData &rigidBodyData, SecTmReal &timeStamp) = 0;
	
	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(SecTmReal elapsedTime);
	
	/** Renders the object. */
	virtual void render();
	
	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);
	
	/** Creates object from description. */
	bool create(const RigidBodyTracker::Desc& desc);
	
	/** Constructor */
	RigidBodyTracker(Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~RigidBodyTracker();

	/** Reads the current sensory data */
	virtual bool read(RigidBodyData &rigidBodyData, SecTmReal &timeStamp, MSecTmU32 timeOut = MSEC_TM_U32_INF);
};

//------------------------------------------------------------------------------

/** VirtualRigidBodyTracker
*/
class VirtualRigidBodyTracker : public RigidBodyTracker {
public:
	typedef shared_ptr<VirtualRigidBodyTracker> Ptr;

	/** Object description */
	class Desc : public RigidBodyTracker::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(VirtualRigidBodyTracker, Channel::Ptr, Embodiment&)

	public:
		/** Target Actor to be tracked */
		Actor *actor;
		/** Use Actor colour */
		bool actorColour;
		
		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			RigidBodyTracker::Desc::setToDefault();
			
			name = "VirtualRigidBodyTracker";
			actor = NULL;
			actorColour = false;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!RigidBodyTracker::Desc::isValid())
				return false;

			//if (actor == NULL)
			//	return false;
			
			return true;
		}
	};

protected:
	/** Target Actor to be tracked */
	Actor *actor;
	/** Use Actor colour */
	bool actorColour;
	
	/** Reads the current sensory data */
	virtual bool get(RigidBodyData &rigidBodyData, SecTmReal &timeStamp);
	
	/** Creates object from description. */
	bool create(const VirtualRigidBodyTracker::Desc& desc);
	
	/** Constructor */
	VirtualRigidBodyTracker(Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~VirtualRigidBodyTracker();

	/** Target Actor to be tracked */
	virtual Actor *getActor() const {
		return actor;
	}

	/** Target Actor to be tracked */
	virtual void setActor(Actor *actor);
};

//------------------------------------------------------------------------------

/** VirtualRigidBodyTracker
*/
class VisionRigidBodyTracker : public RigidBodyTracker {
	typedef shared_ptr<VisionRigidBodyTracker> Ptr;

public:
	/** Object description */
	class Desc : public RigidBodyTracker::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(VisionRigidBodyTracker, Channel::Ptr, Embodiment&)

	public:

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			RigidBodyTracker::Desc::setToDefault();
			
			name = "VisionRigidBodyTracker";
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!RigidBodyTracker::Desc::isValid())
				return false;

			return true;
		}
	};

protected:
	/** Reads the current sensory data */
	virtual bool get(RigidBodyData &rigidBodyData, SecTmReal &timeStamp);
	
	/** Creates object from description. */
	bool create(const VisionRigidBodyTracker::Desc& desc);
	
	/** Constructor */
	VisionRigidBodyTracker(Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~VisionRigidBodyTracker();
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_RIGIDBODYTRACKER_H_*/
