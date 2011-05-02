/** @file Bounds.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_MATH_BOUNDS_H_
#define _GOLEM_MATH_BOUNDS_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Pointers.h>
#include <Golem/Defs/Desc.h>
#include <Golem/Math/Mat34.h>
#include <Golem/Math/TriangleMesh.h>
#include <vector>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Performance monitor */
//#define _BOUNDS_PERFMON
/** Collision detection debugging */
//#define _COLLISION_DEBUG

//------------------------------------------------------------------------------

class BoundingPlane;
class BoundingSphere;
class BoundingCylinder;
class BoundingBox;
class BoundingConvexMesh;
#ifdef _BOUNDS_PERFMON
class Context;
#endif

//------------------------------------------------------------------------------

/** Bounds interface. */
class Bounds {
public:
	typedef shared_ptr<Bounds> Ptr;
	typedef std::vector<const Bounds*> ConstSeq;
	typedef std::vector<Ptr> Seq;
	typedef shared_ptr<Seq> SeqPtr;

	/** Names of bounds. */
	static const char* Name [];
	
	/** Representations of bounds. */
	enum Type {
		/** plane */
		TYPE_PLANE,
		/** sphere */
		TYPE_SPHERE,
		/** cylinder */
		TYPE_CYLINDER,
		/** capsule */
		TYPE_CAPSULE,
		/** parallelepiped */
		TYPE_BOX,
		/** generic triangle mesh */
		TYPE_TRIANGLE_MESH,
		/** convex triangle mesh */
		TYPE_CONVEX_MESH,
	};

	/** Bounds group. */
	enum Group {
		/** Undefined */
		GROUP_UNDEF = 0,
		/** All groups */
		GROUP_ALL = -1,
		/** Default */
		GROUP_DEFAULT = (1<<0),
	};

	/** Bounds description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		typedef std::vector<const Desc*> ConstSeq;
		typedef std::vector<Ptr> Seq;
		typedef shared_ptr<Seq> SeqPtr;

	private:
		/** Type of the bounds. */
		const Type type;

	protected:
		/** Construct bounds description of given type */
		Desc(Type type) : type(type) {
			setToDefault();
		}

		/**	Assignment operator. */
		inline Desc& operator = (const Desc& desc) {
			pose = desc.pose;
			group = desc.group;
			return *this;
		}
		
	public:
		/** Destructor is inaccesible */
		virtual ~Desc() {}

		/** Pose of the bounds. */
		Mat34 pose;
		/** Bounds group. */
		U32 group;

		/** Creates bounds from the description. */
		virtual Bounds::Ptr create() const = 0;
		
		/** Clones the bounds description. */
		virtual Bounds::Desc::Ptr clone() const = 0;

		/** Returns type of bounds. */
		inline Type getType() const {
			return type;
		}

		/** Returns name of bounds. */
		inline const char* getName() const {
			return Bounds::Name[type];
		}
		
		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			pose.setId();
			group = GROUP_DEFAULT;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!pose.isFinite())
				return false;
			
			return true;
		}

		/** Clones the bounds descritpion sequence. */
		template <typename const_iterator> static Bounds::Desc::SeqPtr clone(const_iterator begin, const_iterator end, U32 group = Bounds::GROUP_ALL) {
			Bounds::Desc::SeqPtr pBoundsDescSeq(new Bounds::Desc::Seq());

			for (const_iterator i = begin; i != end; i++)
				if ((**i).group & group) {
					Bounds::Desc::Ptr pBoundsDesc = (*i)->clone();
					pBoundsDescSeq->push_back(pBoundsDesc);
				}

			return pBoundsDescSeq;
		}
	};

private:
	/** Type of the bounds. */
	const Type type;

protected:
	/** Pose of the bounds. */
	Mat34 pose;
	/** Bounds group. */
	U32 group;
	
	/** Tests intersection between two convex meshes represented by clouds of vertices */
	inline bool intersect(const Vec3 *vertices0, U32 numOfVertices0, const Vec3 &centre0, const Vec3 *vertices1, U32 numOfVertices1, const Vec3 &centre1) const {
		return	intersect2(vertices0, numOfVertices0, centre0, vertices1, numOfVertices1, centre1) &&
				intersect2(vertices1, numOfVertices1, centre1, vertices0, numOfVertices0, centre0);
	}
	bool intersect2(const Vec3 *vertices0, U32 numOfVertices0, const Vec3 &centre0, const Vec3 *vertices1, U32 numOfVertices1, const Vec3 &centre1) const;
	
	/** Tests intersection between specified planes. */
	bool intersect(const BoundingPlane& plane0, const BoundingPlane& plane1) const;

	/** Tests intersection between specified plane and sphere. */
	bool intersect(const BoundingPlane& plane, const BoundingSphere& sphere) const;

	/** Tests intersection between specified plane and cylinder. */
	bool intersect(const BoundingPlane& plane, const BoundingCylinder& cylinder) const;

	/** Tests intersection between specified plane and box. */
	bool intersect(const BoundingPlane& plane, const BoundingBox& box) const;

	/** Tests intersection between specified plane and convex mesh. */
	bool intersect(const BoundingPlane& plane, const BoundingConvexMesh& mesh) const;

	/** Tests intersection between specified spheres. */
	bool intersect(const BoundingSphere& sphere0, const BoundingSphere& sphere1) const;

	/** Tests intersection between specified sphere and cylinder. */
	bool intersect(const BoundingSphere& sphere, const BoundingCylinder& cylinder) const;

	/** Tests intersection between specified sphere and box. */
	bool intersect(const BoundingSphere& sphere, const BoundingBox& box) const;

	/** Tests intersection between specified sphere and triangle mesh. */
	bool intersect(const BoundingSphere& sphere, const BoundingConvexMesh& mesh) const;

	/** Tests intersection between specified cylinders. */
	bool intersect(const BoundingCylinder& cylinder0, const BoundingCylinder& cylinder1) const;

	/** Tests intersection between specified cylinder and box. */
	bool intersect(const BoundingCylinder& cylinder, const BoundingBox& box) const;

	/** Tests intersection between specified cylinder and triangle mesh. */
	bool intersect(const BoundingCylinder& cylinder, const BoundingConvexMesh& mesh) const;

	/** Tests intersection of the specified boxs. */
	bool intersect(const BoundingBox& box0, const BoundingBox& box1) const;

	/** Tests intersection between specified cylinder and triangle mesh. */
	bool intersect(const BoundingBox& box, const BoundingConvexMesh& mesh) const;

	/** Tests intersection between specified convex meshes. */
	bool intersect(const BoundingConvexMesh& mesh0, const BoundingConvexMesh& mesh1) const;

	/**	Assignment operator. */
	inline Bounds& operator = (const Bounds &bounds) {
		pose = bounds.pose;
		group = bounds.group;
		return *this;
	}

	Bounds(Type type) : type(type), group(GROUP_DEFAULT) {
	}

	Bounds(Type type, U32 group) : type(type), group(group) {
	}
	
public:
	/** Destructor is inaccesible */
	virtual ~Bounds() {}

	/** Returns type of bounds. */
	inline Type getType() const {
		return type;
	}

	/** Returns name of bounds. */
	inline const char* getName() const {
		return Bounds::Name[type];
	}

	/** Returns bounds group. */
	virtual inline U32 getGroup() const {
		return group;
	}

	/** Set bounds group. */
	virtual inline void setGroup(U32 group) {
		this->group = group;
	}

	/** Clones the bounds. */
	virtual Bounds::Ptr clone() const = 0;

	/** Converts bounds to description. */
	virtual Bounds::Desc::Ptr toDesc() const = 0;

	/** Checks for intersection between the specified bounds. */
	virtual bool intersect(const Bounds &bounds) const = 0;

	/** Tests intersection with specified plane. */
	virtual bool intersect(const BoundingPlane& plane) const = 0;

	/** Tests intersection with specified sphere. */
	virtual bool intersect(const BoundingSphere& sphere) const = 0;

	/** Tests intersection with specified cylinder. */
	virtual bool intersect(const BoundingCylinder& cylinder) const = 0;

	/** Tests intersection with specified box. */
	virtual bool intersect(const BoundingBox& box) const = 0;

	/** Tests intersection with specified convex mesh. */
	virtual bool intersect(const BoundingConvexMesh& mesh) const = 0;

	/** Checks if the specified point intersects the boundaries. */
	virtual bool intersect(const Vec3& p) const = 0;

	/** Checks if the specified ray intersects the boundaries. */
	virtual bool intersect(const Vec3& p0, const Vec3& p1) const = 0;

	/** Checks if the specified triangle intersects the boundaries. */
	virtual bool intersect(const Vec3& p0, const Vec3& p1, const Vec3& p2) const = 0;

	
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const Bounds &bounds) = 0;

	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingPlane& plane) = 0;

	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingSphere& sphere) = 0;

	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingCylinder& cylinder) = 0;

	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingBox& box) = 0;

	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingConvexMesh& mesh) = 0;

	/** Combines this bounding object with a point so that the resulting 
	*	bounding object encloses the original bounding object and the given point.
	*/
	virtual void combine(const Vec3& p) = 0;

	/** Combines this bounding object with an array of points so that the resulting 
	*	bounding object encloses the original bounding object and the given array of points.
	*/
	virtual void combine(const Vec3 *p, U32 numOfPoints) = 0;

	
	/** Sets the value of this Bounds object. */
	virtual void set(const Bounds &bounds) = 0;

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingPlane& plane) = 0;

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingSphere& sphere) = 0;

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingCylinder& cylinder) = 0;

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingBox& box) = 0;

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingConvexMesh& mesh) = 0;
	

	/** Returns current bounds pose. */
	virtual inline const Mat34 &getPose() const {
		return pose;
	}
	
	/** Sets new bounds pose. */
	virtual inline void setPose(const Mat34 &pose) {
		this->pose = pose;
	}

	/** Multiplies bounds pose by a given transformation. */
	virtual inline void multiplyPose(const Mat34& a, const Mat34& b) {
		this->pose.multiply(a, b);
	}

	/** Computes a distance to the bounds surface.
	*	@return		a distance to the bounds surface,
	*				negative if intersects bounds, positive otherwise
	*/
	virtual Real getSurfaceDistance(const Vec3& p) const = 0;


	/** Clones the bounds sequence. */
	template <typename const_iterator> static Bounds::SeqPtr clone(const_iterator begin, const_iterator end, U32 group = Bounds::GROUP_ALL) {
		Bounds::SeqPtr pBoundsSeq(new Bounds::Seq());

		for (const_iterator i = begin; i != end; i++)
			if ((**i).getGroup() & group) {
				Bounds::Ptr pBounds = (*i)->clone();
				pBoundsSeq->push_back(pBounds);
			}

		return pBoundsSeq;
	}
	
	/** Multiplies bounds poses by a given pose. */
	template <typename iterator> static void multiplyPose(const Mat34& pose, iterator begin, iterator end) {
		for (iterator i = begin; i != end; i++) {
			Bounds& bounds = **i;
			bounds.multiplyPose(pose, bounds.getPose());
		}
	}
	
	/** Multiplies bounds poses by a given pose. */
	template <typename iterator> static void multiplyPose(iterator begin, iterator end, const Mat34& pose) {
		for (iterator i = begin; i != end; i++) {
			Bounds& bounds = **i;
			bounds.multiplyPose(bounds.getPose(), pose);
		}
	}
	
	/** Sets the bounds group for given bounds sequence */
	template <typename const_iterator> static void setBoundsGroup(const_iterator begin, const_iterator end, U32 group) {
		for (const_iterator i = begin; i != end; i++)
			(**i).setGroup(group);
	}
	
	/** Tests intersection between specified bounds sequences. */
	template <typename const_iterator> static bool intersect(const_iterator begin0, const_iterator end0, const_iterator begin1, const_iterator end1) {
		for (const_iterator i = begin0; i != end0; i++)
			for (const_iterator j = begin1; j != end1; j++)
				if ((**i).intersect(**j))
					return true;

		return false;
	}
	
	/** Checks for intersection between the specified bounds. */
	template <typename const_iterator> static bool intersect(const_iterator begin, const_iterator end, const Bounds &bounds) {
		for (const_iterator i = begin; i != end; i++)
			if ((**i).intersect(bounds)) {
	#ifdef _COLLISION_DEBUG
				if (bDebug1) {
					pBounds1 = (**i).clone();
					pBounds2 = bounds.clone();
				}
	#endif // _COLLISION_DEBUG
				return true;
			}

		return false;
	}

	/** Checks if the specified point intersects the boundaries. */
	template <typename const_iterator> static bool intersect(const_iterator begin, const_iterator end, const Vec3& p) {
		for (const_iterator i = begin; i != end; i++)
			if ((**i).intersect(p))
				return true;

		return false;
	}

	/** Checks if the specified line intersects the boundaries. */
	template <typename const_iterator> static bool intersect(const_iterator begin, const_iterator end, const Vec3& p0, const Vec3& p1) {
		for (const_iterator i = begin; i != end; i++)
			if ((**i).intersect(p0, p1))
				return true;

		return false;
	}
	
	/** Checks if the specified triangle intersects the boundaries. */
	template <typename const_iterator> static bool intersect(const_iterator begin, const_iterator end, const Vec3& p0, const Vec3& p1, const Vec3& p2) {
		for (const_iterator i = begin; i != end; i++)
			if ((**i).intersect(p0, p1, p2))
				return true;

		return false;
	}
	
	/** Computes a distance to the bounds surface.
	*	@return		a distance to the bounds surface,
	*				negative if intersects bounds, positive otherwise
	*/
	template <typename const_iterator> static Real getSurfaceDistance(const_iterator begin, const_iterator end, const Vec3& p) {
		Real dNeg = numeric_const<Real>::MIN, dPos = numeric_const<Real>::MAX;
		
		for (const_iterator i = begin; i != end; i++) {
			const Real d = (**i).getSurfaceDistance(p);

			if (d < REAL_ZERO) {
				if (dNeg < d)
					dNeg = d;
			}
			else {
				if (dPos > d)
					dPos = d;
			}
		}

		return dNeg > numeric_const<Real>::MIN ? dNeg : dPos;
	}

#ifdef _BOUNDS_PERFMON
	static U32 collisionConvex;
	static U32 collisionConvexIter;
	static U32 collisionConvexFound;
	
	static U32 collisionPlanePlane;
	static U32 collisionPlaneSphere;
	static U32 collisionPlaneCylinder;
	static U32 collisionPlaneBox;
	static U32 collisionPlaneConvex;
	static U32 collisionSphereSphere;
	static U32 collisionSphereCylinder;
	static U32 collisionSphereBox;
	static U32 collisionSphereConvex;
	static U32 collisionCylinderCylinder;
	static U32 collisionCylinderBox;
	static U32 collisionCylinderConvex;
	static U32 collisionBoxBox;
	static U32 collisionBoxConvex;
	static U32 collisionConvexConvex;

	static void resetLog();
	static void writeLog(Context &context, const char *str);
#endif
};

//------------------------------------------------------------------------------

#ifdef _COLLISION_DEBUG
extern bool bDebug1;
extern bool bDebug2;
extern golem::Bounds::Ptr pBounds1;
extern golem::Bounds::Ptr pBounds2;
#endif // _COLLISION_DEBUG

//------------------------------------------------------------------------------

/** Bounds represented by a plane. */
class BoundingPlane : public Bounds {
public:
	/** Bounds description */
	class Desc : public Bounds::Desc {
	public:
		// Pose of the plane is ignored during construction.
		// To change the pose use BoundingPlane::setPose()
		
		/** The plane normal vector. */
		Vec3 normal;
		/** The distance from the origin. */
		Real distance;

		Desc() : Bounds::Desc(TYPE_PLANE) {
			Desc::setToDefault();
		}
		
		Desc(const Desc& desc) : Bounds::Desc(TYPE_PLANE) {
			*this = desc;
		}

		/** Creates bounds from the description. */
		CREATE_FROM_OBJECT_DESC0(BoundingPlane, Bounds::Ptr)
		
		/** Clones the bounds description. */
		virtual Bounds::Desc::Ptr clone() const {
			return Bounds::Desc::Ptr(new Desc(*this));
		}

		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			Bounds::Desc::setToDefault();

			normal.set((Real)0.0, (Real)0.0, (Real)1.0);
			distance = (Real)0.0;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Bounds::Desc::isValid())
				return false;
			
			if (!normal.isFinite() || normal.isZero())
				return false;
			if (!Math::isFinite(distance))
				return false;
			
			return true;
		}

		/**	Assignment operator. */
		inline Desc& operator = (const Desc& desc) {
			Bounds::Desc::operator = (desc);
			normal = desc.normal;
			distance = desc.distance;
			return *this;
		}
	};

protected:
	/** The plane normal vector. */
	Vec3 normal, normalBase;
	/** The distance from the origin. */
	Real distance, distanceBase;

	/** Transforms data using current bounds pose */
	void transform();

	/** Constructs plane from the description structure. */
	bool create(const Desc& desc);
	
	/** Constructs plane from triangle. */
	bool create(const Vec3& p0, const Vec3& p1, const Vec3& p2);
	
	/** Default constructor does not do any initialisation. */
	BoundingPlane() : Bounds(TYPE_PLANE) {
	}

public:
	/**	Copy constructor. */
	BoundingPlane(const BoundingPlane &plane) : Bounds(TYPE_PLANE) {
		*this = plane;
	}

	/** Clones the bounds. */
	virtual inline Bounds::Ptr clone() const {
		return Bounds::Ptr(new BoundingPlane(*this));
	}

	/** Converts bounds to description. */
	virtual Bounds::Desc::Ptr toDesc() const;

	/** Checks for intersection between the specified bounds. */
	virtual inline bool intersect(const Bounds &bounds) const {
		return bounds.intersect(*this);
	}

	/** Tests intersection with specified plane. */
	virtual inline bool intersect(const BoundingPlane& plane) const {
		return Bounds::intersect(*this, plane);
	}

	/** Tests intersection with specified sphere. */
	virtual inline bool intersect(const BoundingSphere& sphere) const {
		return Bounds::intersect(*this, sphere);
	}

	/** Tests intersection with specified cylinder. */
	virtual inline bool intersect(const BoundingCylinder& cylinder) const {
		return Bounds::intersect(*this, cylinder);
	}

	/** Tests intersection with specified box. */
	virtual inline bool intersect(const BoundingBox& box) const {
		return Bounds::intersect(*this, box);
	}

	/** Tests intersection with specified convex mesh. */
	virtual inline bool intersect(const BoundingConvexMesh& mesh) const {
		return Bounds::intersect(*this, mesh);
	}
	
	/** Tests intersection with the specified point. */
	virtual bool intersect(const Vec3& p) const;

	/** Tests intersection with the specified ray. */
	virtual bool intersect(const Vec3& p0, const Vec3& p1) const;

	/** Tests intersection with the specified triangle. */
	virtual bool intersect(const Vec3& p0, const Vec3& p1, const Vec3& p2) const;

	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const Bounds &bounds);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingPlane& plane);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingSphere& sphere);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingCylinder& cylinder);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingBox& box);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingConvexMesh& mesh);
	/** Combines this bounding object with a point so that the resulting 
	*	bounding object encloses the original bounding object and the given point.
	*/
	virtual void combine(const Vec3& p);
	/** Combines this bounding object with an array of points so that the resulting 
	*	bounding object encloses the original bounding object and the given array of points.
	*/
	virtual void combine(const Vec3 *p, U32 numOfPoints);


	/** Sets the value of this Bounds object. */
	virtual void set(const Bounds &bounds);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingPlane& plane);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingSphere& sphere);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingCylinder& cylinder);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingBox& box);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingConvexMesh& mesh);
	

	/** Sets new bounds pose. */
	virtual inline void setPose(const Mat34 &pose) {
		Bounds::setPose(pose);
		transform();
	}
	
	/** Multiplies bounds pose by a given transformation. */
	virtual inline void multiplyPose(const Mat34& a, const Mat34& b) {
		Bounds::multiplyPose(a, b);
		transform();
	}

	/** Computes a distance to the bounds surface.
	*	@return		a distance to the bounds surface,
	*				negative if intersects bounds, positive otherwise
	*/
	virtual Real getSurfaceDistance(const Vec3& p) const;
	
	/**	Assignment operator. */
	inline BoundingPlane& operator = (const BoundingPlane &bounds) {
		Bounds::operator = (bounds);

		normalBase = bounds.normalBase;
		normal = bounds.normal;
		distanceBase = bounds.distanceBase;
		distance = bounds.distance;
		return *this;
	}

	inline const Vec3& getNormal() const {
		return normal;
	}

	inline Real getDistance() const {
		return distance;
	}
};

//------------------------------------------------------------------------------

/** Bounds represented by a sphere. */
class BoundingSphere : public Bounds {
public:
	/** Bounds description */
	class Desc : public Bounds::Desc {
	public:
		/** The radius of the sphere. */
		Real radius;

		Desc() : Bounds::Desc(TYPE_SPHERE) {
			Desc::setToDefault();
		}
		
		Desc(const Desc& desc) : Bounds::Desc(TYPE_SPHERE) {
			*this = desc;
		}
		
		/** Creates bounds from the description. */
		CREATE_FROM_OBJECT_DESC0(BoundingSphere, Bounds::Ptr)
				
		/** Clones the bounds description. */
		virtual Bounds::Desc::Ptr clone() const {
			return Bounds::Desc::Ptr(new Desc(*this));
		}

		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			Bounds::Desc::setToDefault();

			radius = (Real)1.0;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Bounds::Desc::isValid())
				return false;

			if (!Math::isFinite(radius) || radius <= REAL_ZERO)
				return false;

			return true;
		}

		/**	Assignment operator. */
		inline Desc& operator = (const Desc& desc) {
			Bounds::Desc::operator = (desc);
			radius = desc.radius;
			return *this;
		}
	};

protected:
	/** The radius of the sphere. */
	Real radius;

	/** Constructs sphere from the description structure. */
	bool create(const Desc &desc);
	
	/** Default constructor does not do any initialisation. */
	BoundingSphere() : Bounds(TYPE_SPHERE) {
	}

public:
	/**	Copy constructor. */
	BoundingSphere(const BoundingSphere &sphere) : Bounds(TYPE_SPHERE) {
		*this = sphere;
	}

	/** Clones the bounds. */
	virtual inline Bounds::Ptr clone() const {
		return Bounds::Ptr(new BoundingSphere(*this));
	}

	/** Converts bounds to description. */
	virtual Bounds::Desc::Ptr toDesc() const;

	/** Checks for intersection between the specified bounds. */
	virtual inline bool intersect(const Bounds &bounds) const {
		return bounds.intersect(*this);
	}

	/** Tests intersection with specified plane. */
	virtual inline bool intersect(const BoundingPlane& plane) const {
		return Bounds::intersect(plane, *this);
	}

	/** Tests intersection with specified sphere. */
	virtual inline bool intersect(const BoundingSphere& sphere) const {
		return Bounds::intersect(*this, sphere);
	}

	/** Tests intersection with specified cylinder. */
	virtual inline bool intersect(const BoundingCylinder& cylinder) const {
		return Bounds::intersect(*this, cylinder);
	}

	/** Tests intersection with specified box. */
	virtual inline bool intersect(const BoundingBox& box) const {
		return Bounds::intersect(*this, box);
	}

	/** Tests intersection with specified convex mesh. */
	virtual inline bool intersect(const BoundingConvexMesh& mesh) const {
		return Bounds::intersect(*this, mesh);
	}
	
	/** Tests intersection with the specified point. */
	virtual bool intersect(const Vec3& p) const;

	/** Tests intersection with the specified ray. */
	virtual bool intersect(const Vec3& p0, const Vec3& p1) const;

	/** Tests intersection with the specified triangle. */
	virtual bool intersect(const Vec3& p0, const Vec3& p1, const Vec3& p2) const;


	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const Bounds &bounds);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingPlane& plane);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingSphere& sphere);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingCylinder& cylinder);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingBox& box);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingConvexMesh& mesh);
	/** Combines this bounding object with a point so that the resulting 
	*	bounding object encloses the original bounding object and the given point.
	*/
	virtual void combine(const Vec3& p);
	/** Combines this bounding object with an array of points so that the resulting 
	*	bounding object encloses the original bounding object and the given array of points.
	*/
	virtual void combine(const Vec3 *p, U32 numOfPoints);
	
	
	/** Sets the value of this Bounds object. */
	virtual void set(const Bounds &bounds);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingPlane& plane);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingSphere& sphere);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingCylinder& cylinder);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingBox& box);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingConvexMesh& mesh);
	
	/** Computes a distance to the bounds surface.
	*	@return		a distance to the bounds surface,
	*				negative if intersects bounds, positive otherwise
	*/
	virtual Real getSurfaceDistance(const Vec3& p) const;


	/**	Assignment operator. */
	inline BoundingSphere& operator = (const BoundingSphere &bounds) {
		Bounds::operator = (bounds);
		radius = bounds.radius;
		return *this;
	}

	inline const Vec3& getPosition() const {
		return pose.p;
	}

	inline Real getRadius() const {
		return radius;
	}
};

//------------------------------------------------------------------------------

/** Bounds represented by a cylinder. */
class BoundingCylinder : public Bounds, public ITriangleMesh {
public:
	/** Bounds description */
	class Desc : public Bounds::Desc {
	public:
		/** The radius of the cylinder. */
		Real radius;
		/** The length of the cylinder. */
		Real length;
		/** The number of slices of the cylinder. */
		U32 numOfSlices;

		Desc() : Bounds::Desc(TYPE_CYLINDER) {
			Desc::setToDefault();
		}
		
		Desc(const Desc& desc) : Bounds::Desc(TYPE_CYLINDER) {
			*this = desc;
		}
		
		/** Creates bounds from the description. */
		CREATE_FROM_OBJECT_DESC0(BoundingCylinder, Bounds::Ptr)
		
		/** Clones the bounds description. */
		virtual Bounds::Desc::Ptr clone() const {
			return Bounds::Desc::Ptr(new Desc(*this));
		}

		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			Bounds::Desc::setToDefault();

			radius = (Real)1.0;
			length = (Real)1.0;
			numOfSlices = 20;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Bounds::Desc::isValid())
				return false;
			
			if (!Math::isFinite(radius) || radius <= REAL_ZERO)
				return false;
			if (!Math::isFinite(length) || length <= REAL_ZERO)
				return false;
			if (numOfSlices < 3)
				return false;
			
			return true;
		}

		/**	Assignment operator. */
		inline Desc& operator = (const Desc& desc) {
			Bounds::Desc::operator = (desc);
			radius = desc.radius;
			length = desc.length;
			numOfSlices = desc.numOfSlices;
			return *this;
		}

		/** Creates a convex mesh from description. */
		bool createTriangleMesh(TriangleMesh& mesh) const;
	};

protected:
	/** The radius of the cylinder. */
	Real radius;
	/** The length of the cylinder. */
	Real length;
	/** The number of slices of the cylinder. */
	U32 numOfSlices;

	/** The position of the first face of the cylinder. */
	Vec3 position;
	/** The direction of the cylinder . */
	Vec3 normal;
	/** The distance between the first face and the centre of the coordinate frame. */
	Real distance;
	
	/** Triangle mesh of the cylinder. */
	TriangleMesh data[2];
	
	/** Transforms data using current bounds pose */
	void transform();

	/** Constructs cylinder from the description structure. */
	bool create(const Desc &desc);

	/** Default constructor does not do any initialisation. */
	BoundingCylinder() : Bounds(TYPE_CYLINDER) {
	}

public:
	/**	Copy constructor. */
	BoundingCylinder(const BoundingCylinder &cylinder) : Bounds(TYPE_CYLINDER) {
		*this = cylinder;
	}

	/** Clones the bounds. */
	virtual inline Bounds::Ptr clone() const {
		return Bounds::Ptr(new BoundingCylinder(*this));
	}

	/** Converts bounds to description. */
	virtual Bounds::Desc::Ptr toDesc() const;

	/** Checks for intersection between the specified bounds. */
	virtual inline bool intersect(const Bounds &bounds) const {
		return bounds.intersect(*this);
	}

	/** Tests intersection with specified plane. */
	virtual inline bool intersect(const BoundingPlane& plane) const {
		return Bounds::intersect(plane, *this);
	}

	/** Tests intersection with specified sphere. */
	virtual inline bool intersect(const BoundingSphere& sphere) const {
		return Bounds::intersect(sphere, *this);
	}

	/** Tests intersection with specified cylinder. */
	virtual inline bool intersect(const BoundingCylinder& cylinder) const {
		return Bounds::intersect(*this, cylinder);
	}

	/** Tests intersection with specified box. */
	virtual inline bool intersect(const BoundingBox& box) const {
		return Bounds::intersect(*this, box);
	}

	/** Tests intersection with specified convex mesh. */
	virtual inline bool intersect(const BoundingConvexMesh& mesh) const {
		return Bounds::intersect(*this, mesh);
	}
	
	/** Tests intersection with the specified point. */
	virtual bool intersect(const Vec3& p) const;

	/** Tests intersection with the specified ray. */
	virtual bool intersect(const Vec3& p0, const Vec3& p1) const;

	/** Tests intersection with the specified triangle. */
	virtual bool intersect(const Vec3& p0, const Vec3& p1, const Vec3& p2) const;


	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const Bounds &bounds);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingPlane& plane);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingSphere& sphere);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingCylinder& cylinder);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingBox& box);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingConvexMesh& mesh);
	/** Combines this bounding object with a point so that the resulting 
	*	bounding object encloses the original bounding object and the given point.
	*/
	virtual void combine(const Vec3& p);
	/** Combines this bounding object with an array of points so that the resulting 
	*	bounding object encloses the original bounding object and the given array of points.
	*/
	virtual void combine(const Vec3 *p, U32 numOfPoints);


	/** Sets the value of this Bounds object. */
	virtual void set(const Bounds &bounds);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingPlane& plane);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingSphere& sphere);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingCylinder& cylinder);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingBox& box);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingConvexMesh& mesh);
	

	/** Sets new bounds pose. */
	virtual inline void setPose(const Mat34 &pose) {
		Bounds::setPose(pose);
		transform();
	}
	
	/** Multiplies bounds pose by a given transformation. */
	virtual inline void multiplyPose(const Mat34& a, const Mat34& b) {
		Bounds::multiplyPose(a, b);
		transform();
	}

	/** Computes a distance to the bounds surface.
	*	@return		a distance to the bounds surface,
	*				negative if intersects bounds, positive otherwise
	*/
	virtual Real getSurfaceDistance(const Vec3& p) const;
	
	
	/**	Assignment operator. */
	inline BoundingCylinder& operator = (const BoundingCylinder &bounds) {		
		Bounds::operator = (bounds);

		radius = bounds.radius;
		length = bounds.length;
		numOfSlices = bounds.numOfSlices;

		position = bounds.position;
		normal = bounds.normal;
		distance = bounds.distance;

		data[0] = bounds.data[0];
		data[1] = bounds.data[1];

		return *this;
	}

	
	/** The radius of the cylinder. */
	inline Real getRadius() const {
		return radius;
	}
	
	/** The length of the cylinder. */
	inline Real getLength() const {
		return length;
	}
	
	/** The number of slices of the cylinder. */
	inline U32 getNumOfSlices() const {
		return numOfSlices;
	}
	
	/** The position of the first face of the cylinder. */
	inline const Vec3& getPosition() const {
		return position;
	}

	/** The direction of the cylinder . */
	inline const Vec3& getNormal() const {
		return normal;
	}
	
	/** The distance of the centre of the first face. */
	inline Real getDistance() const {
		return distance;
	}
	
	/** Vertices of the mesh. */
	virtual const std::vector<Vec3>& getVertices() const {
		return data[1].vertices;
	}

	/** Indices of triangles of the mesh. */
	virtual const std::vector<Triangle>& getTriangles() const {
		return data[1].triangles;
	}

	/** Normals of the mesh faces. */
	virtual const std::vector<Vec3>& getNormals() const {
		return data[1].normals;
	}

	/** Distances to the origin of the mesh faces. */
	virtual const std::vector<Real>& getDistances() const {
		return data[1].distances;
	}
};

//------------------------------------------------------------------------------

/** Bounds represented by parallelepiped. */
class BoundingBox : public Bounds {
public:
	/** Bounds description */
	class Desc : public Bounds::Desc {
	public:
		/** The dimensions are the radius of the bounds, meaning 1/2 extents in each dimension. */
		Vec3 dimensions;

		Desc() : Bounds::Desc(TYPE_BOX) {
			Desc::setToDefault();
		}
		
		Desc(const Desc& desc) : Bounds::Desc(TYPE_BOX) {
			*this = desc;
		}
		
		/** Creates bounds from the description. */
		CREATE_FROM_OBJECT_DESC0(BoundingBox, Bounds::Ptr)
				
		/** Clones the bounds description. */
		virtual Bounds::Desc::Ptr clone() const {
			return Bounds::Desc::Ptr(new Desc(*this));
		}

		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			Bounds::Desc::setToDefault();

			dimensions.set((Real)1.0, (Real)1.0, (Real)1.0);
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Bounds::Desc::isValid())
				return false;

			if (!dimensions.isFinite() || !dimensions.isPositive())
				return false;

			return true;
		}

		/**	Assignment operator. */
		inline Desc& operator = (const Desc& desc) {
			Bounds::Desc::operator = (desc);
			dimensions = desc.dimensions;
			return *this;
		}
	};

protected:
	/** The dimensions are the radius of the bounds, meaning 1/2 extents in each dimension. */
	Vec3 dimensions;
	
	/** Normals of the box. */
	Vec3 normals[3];
	/** Distances between origin of the coordinate system and faces of the box. */
	Real distances[6];
	/** Edges. */
	Vec3 edges[8];
	
	void create();

	void createFrame(Vec3& xb, Vec3& yb, const Vec3& zb) const;
	
	Real getSignedDistance(const Vec3& p) const;

	/**	Constructs box from the description structure. */
	bool create(const Desc &desc);
	
	/** Default constructor does not do any initialisation. */
	BoundingBox() : Bounds(TYPE_BOX) {
	}

public:
	/**	Copy constructor. */
	BoundingBox(const BoundingBox &box) : Bounds(TYPE_BOX) {
		*this = box;
	}

	/** Clones the bounds. */
	virtual inline Bounds::Ptr clone() const {
		return Bounds::Ptr(new BoundingBox(*this));
	}

	/** Converts bounds to description. */
	virtual Bounds::Desc::Ptr toDesc() const;

	/** Checks for intersection between the specified bounds. */
	virtual inline bool intersect(const Bounds &bounds) const {
		return bounds.intersect(*this);
	}

	/** Tests intersection with specified plane. */
	virtual inline bool intersect(const BoundingPlane& plane) const {
		return Bounds::intersect(plane, *this);
	}

	/** Tests intersection with specified sphere. */
	virtual inline bool intersect(const BoundingSphere& sphere) const {
		return Bounds::intersect(sphere, *this);
	}

	/** Tests intersection with specified cylinder. */
	virtual inline bool intersect(const BoundingCylinder& cylinder) const {
		return Bounds::intersect(cylinder, *this);
	}

	/** Tests intersection with specified box. */
	virtual inline bool intersect(const BoundingBox& box) const {
		return Bounds::intersect(*this, box);
	}

	/** Tests intersection with specified convex mesh. */
	virtual inline bool intersect(const BoundingConvexMesh& mesh) const {
		return Bounds::intersect(*this, mesh);
	}
	
	/** Tests intersection with the specified point. */
	virtual bool intersect(const Vec3& p) const;

	/** Tests intersection with the specified ray. */
	virtual bool intersect(const Vec3& p0, const Vec3& p1) const;

	/** Tests intersection with the specified triangle. */
	virtual bool intersect(const Vec3& p0, const Vec3& p1, const Vec3& p2) const;


	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const Bounds &bounds);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingPlane& plane);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingSphere& sphere);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingCylinder& cylinder);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingBox& box);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingConvexMesh& mesh);
	/** Combines this bounding object with a point so that the resulting 
	*	bounding object encloses the original bounding object and the given point.
	*/
	virtual void combine(const Vec3& p);
	/** Combines this bounding object with an array of points so that the resulting 
	*	bounding object encloses the original bounding object and the given array of points.
	*/
	virtual void combine(const Vec3 p[], U32 numOfPoints);


	/** Sets the value of this Bounds object. */
	virtual void set(const Bounds &bounds);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingPlane& plane);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingSphere& sphere);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingCylinder& cylinder);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingBox& box);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingConvexMesh& mesh);
	

	/** Sets new bounds pose. */
	virtual inline void setPose(const Mat34 &pose) {
		Bounds::setPose(pose);
		create();
	}
	
	/** Multiplies bounds pose by a given transformation. */
	virtual inline void multiplyPose(const Mat34& a, const Mat34& b) {
		Bounds::multiplyPose(a, b);
		create();
	}

	/** Computes a distance to the bounds surface.
	*	@return		a distance to the bounds surface,
	*				negative if intersects bounds, positive otherwise
	*/
	virtual Real getSurfaceDistance(const Vec3& p) const;


	/**	Assignment operator. */
	inline BoundingBox& operator = (const BoundingBox &bounds) {		
		Bounds::operator = (bounds);
		
		dimensions = bounds.dimensions;
		for (U32 i = 0; i < 3; i++)
			normals[i] = bounds.normals[i];
		for (U32 i = 0; i < 6; i++)
			distances[i] = bounds.distances[i];
		for (U32 i = 0; i < 8; i++)
			edges[i] = bounds.edges[i];

		return *this;
	}


	/** Returns the box dimensions. */
	inline const Vec3& getDimensions() const {
		return dimensions;
	}

	/** Returns normals of the box. */
	inline const Vec3 *getNormals() const {
		return normals;
	}
	
	/** Returns distances between origin of the coordinate system and faces of the box. */
	inline const Real *getDistances() const {
		return distances;
	}

	/** Returns edges. */
	inline const Vec3 *getEdges() const {
		return edges;
	}
};

//------------------------------------------------------------------------------

/** Bounds represented by a convex triangle mesh. */
class BoundingConvexMesh : public Bounds, public ITriangleMesh {
public:
	/** Bounds description */
	class Desc : public Bounds::Desc, public TriangleMesh::Desc {
	public:
		// Pose of the convex mesh is ignored during construction.
		// To change the pose use BoundingConvexMesh::setPose()

		Desc() : Bounds::Desc(TYPE_CONVEX_MESH) {
			Desc::setToDefault();
		}
		
		Desc(const Desc& desc) : Bounds::Desc(TYPE_CONVEX_MESH), TriangleMesh::Desc(desc) {
		}
				
		/** Creates bounds from the description. */
		CREATE_FROM_OBJECT_DESC0(BoundingConvexMesh, Bounds::Ptr)
				
		/** Clones the bounds description. */
		virtual Bounds::Desc::Ptr clone() const {
			return Bounds::Desc::Ptr(new Desc(*this));
		}

		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			Bounds::Desc::setToDefault();
			TriangleMesh::Desc::setToDefault();
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Bounds::Desc::isValid())
				return false;

			if (!TriangleMesh::Desc::isValid())
				return false;

			return true;
		}

		/**	Assignment operator. */
		inline Desc& operator = (const Desc& desc) {
			Bounds::Desc::operator = (desc);
			TriangleMesh::Desc::operator = (desc);
			return *this;
		}
	};

protected:
	TriangleMesh data[2];

	/** Constructs convex triangle mesh from the description structure. */
	bool create(const Desc& desc);
		
	/** Default constructor does not do any initialisation. */
	BoundingConvexMesh() : Bounds(TYPE_CONVEX_MESH) {
	}

public:
	/**	Copy constructor. */
	BoundingConvexMesh(const BoundingConvexMesh &mesh) : Bounds(TYPE_CONVEX_MESH) {
		*this = mesh;
	}

	/** Clones the bounds. */
	virtual inline Bounds::Ptr clone() const {
		return Bounds::Ptr(new BoundingConvexMesh(*this));
	}

	/** Converts bounds to description. */
	virtual Bounds::Desc::Ptr toDesc() const;

	/** Checks for intersection between the specified bounds. */
	virtual inline bool intersect(const Bounds &bounds) const {
		return bounds.intersect(*this);
	}

	/** Tests intersection with specified plane. */
	virtual inline bool intersect(const BoundingPlane& plane) const {
		return Bounds::intersect(plane, *this);
	}

	/** Tests intersection with specified sphere. */
	virtual inline bool intersect(const BoundingSphere& sphere) const {
		return Bounds::intersect(sphere, *this);
	}

	/** Tests intersection with specified cylinder. */
	virtual inline bool intersect(const BoundingCylinder& cylinder) const {
		return Bounds::intersect(cylinder, *this);
	}

	/** Tests intersection with specified box. */
	virtual inline bool intersect(const BoundingBox& box) const {
		return Bounds::intersect(box, *this);
	}

	/** Tests intersection with specified convex mesh. */
	virtual inline bool intersect(const BoundingConvexMesh& mesh) const {
		return Bounds::intersect(*this, mesh);
	}
	
	/** Tests intersection with the specified point. */
	virtual bool intersect(const Vec3& p) const;

	/** Tests intersection with the specified ray. */
	virtual bool intersect(const Vec3& p0, const Vec3& p1) const;

	/** Tests intersection with the specified triangle. */
	virtual bool intersect(const Vec3& p0, const Vec3& p1, const Vec3& p2) const;


	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const Bounds &bounds);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingPlane& plane);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingSphere& sphere);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingCylinder& cylinder);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingBox& box);
	/** Combines this bounding object with a bounding object so that the resulting 
	*	bounding object encloses the original bounding object and the given bounds object.
	*/
	virtual void combine(const BoundingConvexMesh& mesh);
	/** Combines this bounding object with a point so that the resulting 
	*	bounding object encloses the original bounding object and the given point.
	*/
	virtual void combine(const Vec3& p);
	/** Combines this bounding object with an array of points so that the resulting 
	*	bounding object encloses the original bounding object and the given array of points.
	*/
	virtual void combine(const Vec3 *p, U32 numOfPoints);


	/** Sets the value of this Bounds object. */
	virtual void set(const Bounds &bounds);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingPlane& plane);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingSphere& sphere);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingCylinder& cylinder);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingBox& box);

	/** Sets the value of this Bounds object. */
	virtual void set(const BoundingConvexMesh& mesh);
	
	
	/** Sets new bounds pose. */
	virtual inline void setPose(const Mat34 &pose) {
		Bounds::setPose(pose);
		data[1].transform(this->pose, data[0]);
	}
	
	/** Multiplies bounds pose by a given transformation. */
	virtual inline void multiplyPose(const Mat34& a, const Mat34& b) {
		Bounds::multiplyPose(a, b);
		data[1].transform(this->pose, data[0]);
	}

	/** Computes a distance to the bounds surface.
	*	@return		a distance to the bounds surface,
	*				negative if intersects bounds, positive otherwise
	*/
	virtual Real getSurfaceDistance(const Vec3& p) const;
	
	/**	Assignment operator. */
	inline BoundingConvexMesh& operator = (const BoundingConvexMesh &bounds) {		
		Bounds::operator = (bounds);
		data[0] = bounds.data[0];
		data[1] = bounds.data[1];
		return *this;
	}

	/** Vertices of the mesh. */
	virtual const std::vector<Vec3>& getVertices() const {
		return data[1].vertices;
	}

	/** Indices of triangles of the mesh. */
	virtual const std::vector<Triangle>& getTriangles() const {
		return data[1].triangles;
	}

	/** Normals of the mesh faces. */
	virtual const std::vector<Vec3>& getNormals() const {
		return data[1].normals;
	}

	/** Distances to the origin of the mesh faces. */
	virtual const std::vector<Real>& getDistances() const {
		return data[1].distances;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_BOUNDS_H_*/
