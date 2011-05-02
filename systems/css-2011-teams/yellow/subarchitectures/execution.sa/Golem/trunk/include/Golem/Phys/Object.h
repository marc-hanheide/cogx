/** @file Object.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_PHYS_OBJECT_H_
#define _GOLEM_PHYS_OBJECT_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Bounds.h>
#include <Golem/Tools/Context.h>
#include <Golem/Phys/Renderer.h>
#include <NxPhysics.h>
#include <vector>
#include <map>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class Universe;
class Scene;

//------------------------------------------------------------------------------

/** Object is a base class of all objects in the simulated world. */
class Object {
public:
	friend class Scene;
	friend class Desc;
	typedef shared_ptr<Object> Ptr;
	typedef std::vector<Object*> Seq;
	
	/** Object description */
	class Desc {
	public:
		friend class Scene;
		typedef shared_ptr<Desc> Ptr;
		typedef std::vector<Ptr> Seq;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}
		/** Virtual descructor */
		virtual ~Desc() {
		}
		/** Sets the objet parameters to the default values */
		virtual void setToDefault() {
		}
		/** Checks if the object description is valid. */
		virtual bool isValid() const {
			return true;
		}

	protected:
		/** Creates/initialises the object. */
		virtual Object::Ptr create(Scene &scene) const = 0;
	};

protected:
	/** Universe is the container of Scenes */
	Universe &universe;
	/** Scene is the container of objects */
	Scene &scene;
	/** golem::Context object */
	golem::Context &context;
	
	/** Bounds renderer */
#ifdef _COLLISION_DEBUG
	BoundsRenderer boundsRenderer;
#endif // _COLLISION_DEBUG
	
	NxPhysicsSDK* pNxPhysicsSDK;
	NxScene* pNxScene;

	/** Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y) {}
	
	/** Mouse motion handler. */
	virtual void motionHandler(int x, int y) {}
	
	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y) {}

	/** (Pre)processing function called BEFORE every physics simulation step and before randering. */
	virtual inline void preprocess(SecTmReal elapsedTime) {}

	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual inline void postprocess(SecTmReal elapsedTime) {}

	/** Custom renderer. */
	virtual void customRender() {}

	/** Renders the object. */
	virtual void render() {}

	/** Creates object from description. */
	bool create(const Object::Desc& desc);

	/** Releases resources */
	virtual void release();
	
	/** Objects can be constructed only in the Scene context. */
	Object(Scene &scene);

public:
	/** Destructor is inaccesible */
	virtual ~Object() {}

	inline golem::Context& getContext() {
		return context;
	}
	
	inline const golem::Context& getContext() const {
		return context;
	}

	inline Scene& getScene() {
		return scene;
	}
	
	inline const Scene& getScene() const {
		return scene;
	}

	inline NxScene* getNxScene() {
		return pNxScene;
	}
	
	inline const NxScene* getNxScene() const {
		return pNxScene;
	}
};

//------------------------------------------------------------------------------

/** Actor is a base class of all physically interacting objects/bodies. */
class Actor : public Object {
public:
	friend class Desc;
	typedef shared_ptr<Object> Ptr;
	typedef std::vector<Actor*> Seq;
	typedef Bounds::ConstSeq BoundsConstSeq;	
	
	/** Object appearance */
	class Appearance {
	public:
		/** Surface colour */
		RGBA solidColour;
		/** Edge colour */
		RGBA wireColour;
		/** Shadow colour */
		RGBA shadowColour;
		/** Line width */
		Real lineWidth;

		/** Appearance construction */
		Appearance() {
			Appearance::setToDefault();
		}
		/** Sets the appearance to the default state */
		void setToDefault() {
			solidColour = BoundsRenderer::glSolidColourDflt;
			wireColour = BoundsRenderer::glWireColourDflt;
			shadowColour = BoundsRenderer::glShadowColourDflt;
			lineWidth = BoundsRenderer::glLineWidthDflt;
		}
		/** Checks if the appearance is valid. */
		bool isValid() const {
			if (lineWidth < Real(1.0))
				return false;

			return true;
		}
	};

	/** Object description */
	class Desc : virtual public Object::Desc {
	public:
		/** Novodex Actor description */
		NxActorDesc nxActorDesc;
		/** Shape appearance */
		Appearance appearance;
		/** Kinematic object */
		bool kinematic;

		/** Constructs Actor description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the objet parameters to the default values */
		virtual void setToDefault() {
			Object::Desc::setToDefault();

			nxActorDesc.setToDefault();
			appearance.setToDefault();
			kinematic = false;
		}
		/** Checks if the object description is valid. */
		virtual bool isValid() const {
			if (!Object::Desc::isValid())
				return false;
			if (!nxActorDesc.isValid() || !appearance.isValid())
				return false;

			return true;
		}

	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Actor, Object::Ptr, Scene&)
	};

protected:
	struct BoundsData {
		typedef std::map<const Bounds*, BoundsData> Seq;
		
		Bounds::Ptr pBounds;
		Bounds::Desc::Ptr pBoundsDesc;
		const NxShape* pConstNxShape;
		NxShape* pNxShape;
	};
	
	/** Pointer to Novodex Actor. */
	NxActor *pNxActor;
	
	/** Bounds renderer */
#ifndef _COLLISION_DEBUG
	BoundsRenderer boundsRenderer;
#endif // _COLLISION_DEBUG
	/** Shape appearance */
	Appearance appearance;
	
	/** bounds of the Actor */
	BoundsConstSeq boundsConstSeq;
	BoundsData::Seq boundsDataSeq;
	mutable CriticalSection csBoundsSeq;

	/** Renders the object. */
	virtual void render();

	/** Creates Actor from description. */
	bool create(const Actor::Desc& desc);

	/** Releases resources */
	virtual void release();
	
	/** Objects can be constructed only within given Scene. */
	Actor(Scene &scene);

public:
	/** Destructor is inaccesible */
	virtual ~Actor();

	/** Returns Actor pose
	 * @return				current Actor pose
	*/
	virtual Mat34 getPose() const;
	
	/** Sets a new Actor pose (only kinematic Actors)
	 * @param pose			target Actor pose
	*/
	virtual void setPose(const Mat34& pose);
	
	/** Creates the bounds in the local coordinate frame
	 * @param pDesc			description of the bounds
	 * @return				pointer to the bounds; <code>NULL</code> in case of failure
	*/
	virtual const Bounds* createBounds(Bounds::Desc::Ptr pDesc);

	/** Removes the specified bounds
	 * @param bounds		the bounds to be removed
	*/
	virtual void releaseBounds(const Bounds& bounds);
	
	/** Returns bounds collection of the Actor
	 * @param group			bounds group
	 * @return				bounds collection
	*/
	virtual const BoundsConstSeq& getBoundsSeq() const;
	
	/** Returns collection of bounds in local Actor coordinates
	 * @param group			bounds group
	 * @return				bounds collection
	*/
	virtual Bounds::SeqPtr getLocalBoundsSeq(U32 group = Bounds::GROUP_ALL) const;
	
	/** Returns collection of bounds in global Actor coordinates
	 * @param group			bounds group
	 * @return				bounds collection
	*/
	virtual Bounds::SeqPtr getGlobalBoundsSeq(U32 group = Bounds::GROUP_ALL) const;
	
	/** Returns bounds description for given bounds 
	 * @param bounds		bounds
	 * @return				bounds description
	*/
	virtual const Bounds::Desc* getBoundsDesc(const Bounds& bounds) const;
	
	/** Sets the bounds group for given bounds
	 * @param bounds		bounds to be set
	 * @param group			bounds group
	*/
	virtual void setBoundsGroup(const Bounds& bounds, U32 group);

	/** Sets the bounds group for all bounds
	 * @param group			bounds group
	*/
	virtual void setBoundsGroup(U32 group);

	/** Returns shape appearance */
	virtual Actor::Appearance getAppearance() const;

	/** Sets shape appearance */
	virtual void setAppearance(const Appearance &appearance);
	
	/** Synchronisation with Bounds set */
	inline CriticalSection &getCSBoundsSeq() {
		return csBoundsSeq;
	}
	
	/** Returns Novodex Actor 
	 * @return				pointer to the actor
	*/
	inline NxActor *getNxActor() {
		return pNxActor;
	}

	/** Returns Novodex Actor 
	 * @return				constant pointer to the actor
	*/
	inline const NxActor *getNxActor() const {
		return pNxActor;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYS_OBJECT_H_*/
