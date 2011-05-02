/** @file Scene.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_PHYS_SCENE_H_
#define _GOLEM_PHYS_SCENE_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Bounds.h>
#include <Golem/Math/Collection.h>
#include <Golem/Phys/Object.h>
#include <Golem/Phys/NxDebugRenderer.h>
#include <vector>
#include <map>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class Universe;

class Scene {
	friend class Universe;

public:
	typedef shared_ptr<Scene> Ptr;
	typedef std::vector<Scene*> Seq;	
	typedef PrivateList<Object*, Object::Ptr> ObjectList;

	/** Draw mode */
	class Draw {
	public:
		/** Draw solid */
		bool solid;
		/** Draw wireframe */
		bool wire;
		/** Draw shadows */
		bool shadow;
		/** Draw debug data */
		bool debug;

		/** Construction */
		Draw() {
			Draw::setToDefault();
		}

		/** Sets mode to the default state */
		void setToDefault() {
			solid = true;
			wire = false;
			shadow = true;
			debug = false;
		}
		
		/** Sets mode to blank state */
		void setBlank() {
			solid = false;
			wire = false;
			shadow = false;
			debug = false;
		}
		
		/** Checks if mode is valid. */
		bool isValid() const {
			return true;
		}
	};

	class Desc {
		friend class Universe;

	protected:
		/** Creates/initialises the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Scene, Scene::Ptr, Universe&)

	public:
		/** Scene name */
		std::string name;
		
		/** Asynchronous scene - no background processing: pre/post process, rendering and physx */
		bool asyncScene;

		/** Novodex PhysX scene description */
		NxSceneDesc nxSceneDesc;

		/** Default scene view setup */
		Vec3 viewPointDflt;
		Vec3 viewDirDflt;

		/** Alternative scene view setup using OpenGL camera matrices */
		GLfloat glMatIntrinsic[16];
		GLfloat glMatExtrinsic[16];
		bool glMat;

		/** Default colors */
		RGBA clearColor;
		RGBA ambientColor;
		RGBA diffuseColor;
		RGBA specularColor;

		/** Default restitution coefficient */
		Real restitution;
		/** Default static friction coefficient */
		Real staticFriction;
		/** Default dynamic friction coefficient */
		Real dynamicFriction;

		/** view mouse button */
		int mouseButton;

		/** Draw mode */
		Draw draw;

		Desc() {
			setToDefault();
		}

		virtual ~Desc() {}

		virtual void setToDefault() {
			name = "";

			asyncScene = false;

			nxSceneDesc.setToDefault();
			nxSceneDesc.gravity.set(0.0f, 0.0f, -9.81f); // Z-axis

			viewPointDflt.set((Real)6.0, (Real)6.0, (Real)6.0);
			viewDirDflt.set((Real)-1.0, (Real)-1.0, (Real)-0.7);

			Mat34 id;
			id.setId();
			id.getColumn44(glMatIntrinsic);
			id.getColumn44(glMatExtrinsic);
			glMat = false;

			clearColor.set(51, 76, 76, 255);
			ambientColor.set(0, 26, 51, 0);
			diffuseColor.set(255, 255, 255, 0);
			specularColor.set(0, 0, 0, 0);

			restitution = (Real)0.1;
			staticFriction = (Real)0.2;
			dynamicFriction = (Real)0.1;

			mouseButton = 0;

			draw.setToDefault();
		}

		virtual bool isValid() const {
			if (!nxSceneDesc.isValid())
				return false;

			if (!viewPointDflt.isFinite() || Math::equals(viewPointDflt.magnitude(), REAL_ZERO, REAL_EPS))
				return false;
			if (!viewDirDflt.isFinite() || Math::equals(viewDirDflt.magnitude(), REAL_ZERO, REAL_EPS))
				return false;

			if (restitution < REAL_ZERO || staticFriction < REAL_ZERO || dynamicFriction < REAL_ZERO)
				return false;
			
			if (!draw.isValid())
				return false;
			
			return true;
		}
	};

protected:
	typedef shared_ptr<NxShapeDesc> NxShapeDescPtr;
	typedef std::map<const Bounds::Desc*, NxShapeDescPtr> NxShapeDescMap;
	typedef std::map<const NxShapeDesc*, Bounds::Desc::Ptr> BoundsDescMap;

	Desc desc;

	Universe &universe;
	golem::Context &context;
	NxPhysicsSDK* pNxPhysicsSDK;
	NxScene* pNxScene;
	
	/** Collection owns pointers to all objects */
	ObjectList objectList;
	mutable CriticalSection csObjects;

	/** Collection owns pointers to all shape and bounds descriptions */
	NxShapeDescMap nxShapeDescMap;	
	BoundsDescMap boundsDescMap;
	
	/** bounds group mask */
	U32 boundsGroupMask;

	Vec3 viewPoint;
	Vec3 viewDir;
	Vec3 viewUp;
	Vec3 viewNormal;
	
	RGBA clearColor;
	RGBA ambientColor;
	RGBA diffuseColor;
	RGBA specularColor;
	::NxDebugRenderer nxDebugRenderer;

	int mouseButton;
	int mouseX;
	int mouseY;

	/** Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	
	/** Mouse motion handler. */
	virtual void motionHandler(int x, int y);

	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);

	/** (Pre)processing function called BEFORE every physics simulation step and before randering. */
	virtual void preprocess(SecTmReal elapsedTime);

	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(SecTmReal elapsedTime);
	
	/** Renders the scene. */
	virtual void render();
	
	
	/** Init OpenGL */
	virtual void initGL();

	/** Init OpenGL camera. */
	virtual void initCamera();
	
	/** Init debug mode */
	virtual bool initDebug();

	NxShapeDescPtr createNxShapeDesc(const BoundingPlane::Desc* pDesc) const;
	Bounds::Desc::Ptr createBoundsDesc(const NxPlaneShapeDesc *pDesc) const;
	
	NxShapeDescPtr createNxShapeDesc(const BoundingSphere::Desc* pDesc) const;
	Bounds::Desc::Ptr createBoundsDesc(const NxSphereShapeDesc *pDesc) const;
	
	NxShapeDescPtr createNxShapeDesc(const BoundingBox::Desc* pDesc) const;
	Bounds::Desc::Ptr createBoundsDesc(const NxBoxShapeDesc *pDesc) const;
	
	NxShapeDescPtr createNxShapeDesc(const BoundingCylinder::Desc* pDesc);
	
	void copyShapeDesc(const ::NxConvexMeshDesc& src, BoundingConvexMesh::Desc& dst) const;
	NxShapeDescPtr createNxShapeDesc(BoundingConvexMesh::Desc* pDesc);
	Bounds::Desc::Ptr createBoundsDesc(const NxConvexShapeDesc *pDesc);
	
	void releaseNxShapeDescResources(NxShapeDescPtr& nxShapeDescPtr);

	
	/** Creates the Scene from the Scene description. */
	bool create(const Desc& desc);
	
	/** Releases resources */
	virtual void release();
	
	/** Scenes can be constructed only in the Universe context. */
	Scene(Universe &universe);

public:
	/** Destructor is inaccesible */
	virtual ~Scene();

	/** Creates Object from the Object description. */
	virtual Object* createObject(const Object::Desc& desc);

	/** Releases the Object. */
	virtual void releaseObject(Object& object);

	/** Returns collection of Objects */
	virtual const ObjectList& getObjectList() const;


	/** Creates Novodex shape description from bounds description. */
	virtual NxShapeDesc* createNxShapeDesc(Bounds::Desc::Ptr pBoundsDesc);
	
	/** Releases the Novodex shape description and related bounds description. */
	virtual void releaseNxShapeDesc(NxShapeDesc &nxShapeDesc);
	
	/** Creates bounds description from Novodex shape description. */
	virtual Bounds::Desc::Ptr createBoundsDesc(const NxShapeDesc &nxShapeDesc);

	/** Releases the bounds description and related Novodex shape description. */
	virtual void releaseBoundsDesc(Bounds::Desc::Ptr pBoundsDesc);
	

	/** Generates new bounds group mask. */
	virtual U32 createBoundsGroup();

	/** Generates new bounds group mask. */
	virtual void removeBoundsGroup(U32 group);
	
	/** Generates new bounds group mask. */
	virtual U32 getBoundsGroupMask() const;

	
	void getDraw(Draw& draw) const;
	
	void setDraw(const Draw& draw);

	/** Reset camera */
	void resetCamera();

	/** Setup camera from given view point and view direction */
	void setCamera(const Vec3& viewPoint, const Vec3& viewDir);

	/** Setup camera from given intrinsic and intrinsic matrices */
	void setCamera(const GLfloat glMatIntrinsic[], const GLfloat glMatExtrinsic[]);

	/** Asynchronous scene - no background processing: pre/post process, rendering and physx */
	bool isAsync() const {
		return desc.asyncScene;
	}
	
	/** Synchronisation with Object set */
	inline CriticalSection &getCSObjectList() {
		return csObjects;
	}
	
	inline golem::Context& getContext() {
		return context;
	}
	
	inline const golem::Context& getContext() const {
		return context;
	}

	inline Universe& getUniverse() {
		return universe;
	}
	
	inline const Universe& getUniverse() const {
		return universe;
	}

	inline NxScene* getNxScene() {
		return pNxScene;
	}
	
	inline const NxScene* getNxScene() const {
		return pNxScene;
	}

	inline NxPhysicsSDK* getNxPhysicsSDK() {
		return pNxPhysicsSDK;
	}

	inline const NxPhysicsSDK* getNxPhysicsSDK() const {
		return pNxPhysicsSDK;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYS_SCENE_H_*/
