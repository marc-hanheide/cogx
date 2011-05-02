/** @file Heuristic.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_HEURISTIC_H_
#define _GOLEM_CTRL_HEURISTIC_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Waypoint.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Performance monitor */
//#define _HEURISTIC_PERFMON

//------------------------------------------------------------------------------

/** Waypoint path optimisation atomic routines */
class Heuristic {
public:
	typedef shared_ptr<Heuristic> Ptr;
	friend class Desc;
	
	/** Heuristic description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;

		/** angular distance metric: quaternion dot product  */
		bool distAngularQuatDot;

		/** characteristic workspace distance scale */
		Real distLinearFac;
		/** workspace distance factor */
		Real distWorkspaceFac;
		/** configuration space distance factor */
		Real distJointspaceFac;

		/** Linear distance maximum */
		Real distLinearMax;
		/** angular distance maximum */
		Real distAngularMax;
		/** maximum distance between two configuration space coordinates */
		ConfigspaceCoord distJointcoordMax;
		/** Rest position distance factor */
		ConfigspaceCoord distRestJointcoordFac;
		
		/** Colision detection */
		bool collisionDetection;		
		/** collision distance delta */
		Real colissionDistDelta;
		/** Objects skin thickness */
		Real skinThickness;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** virtual destructor */
		virtual ~Desc() {}
		
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Heuristic, Heuristic::Ptr, Arm&)
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			distAngularQuatDot = true;

			distLinearFac = Real(10.0);
			distWorkspaceFac = Real(0.2);//Real(0.7); // [0, 1]
			distJointspaceFac = Real(0.8); // [0, 1]
			
			distLinearMax = Real(0.25);//OLD: 0.35
			distAngularMax = Real(0.25);//OLD: 0.35
			distJointcoordMax.set(Real(0.75)*REAL_PI);
			distRestJointcoordFac.set(Real(0.01)); //0.01

			collisionDetection = true;
			colissionDistDelta = Real(0.025);
			skinThickness = Real(0.01);
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (distLinearFac < REAL_ZERO)
				return false;
			if (distWorkspaceFac < REAL_ZERO || distWorkspaceFac > REAL_ONE)
				return false;
			if (distJointspaceFac < REAL_ZERO || distJointspaceFac > REAL_ONE)
				return false;
			if (distLinearMax <= REAL_ZERO || distAngularMax <= REAL_ZERO)
				return false;
			if (!distJointcoordMax.isPositive() || !distRestJointcoordFac.isPositive())
				return false;
			if (colissionDistDelta <= REAL_ZERO || skinThickness < REAL_ZERO)
				return false;

			return true;
		}
	};

protected:
	typedef std::vector<golem::Mat34> Mat34Seq;
	/** Arm controller */
	golem::Arm &arm;
	/** Context object */
	golem::Context &context;
	
	/** angular distance metric: quaternion dot product  */
	bool distAngularQuatDot;

	/** characteristic workspace distance scale */
	Real distLinearFac;
	/** configuration space distance factor */
	Real distJointspaceFac;
	/** workspace distance factor */
	Real distWorkspaceFac;
	
	/** Linear distance maximum */
	Real distLinearMax;
	/** angular distance maximum */
	Real distAngularMax;
	/** maximum distance between two configuration space coordinates */
	ConfigspaceCoord distJointcoordMax;
	/** Rest position cost factor */
	ConfigspaceCoord distRestJointcoordFac;
	
	/** Colision detection */
	bool collisionDetection;
	/** collision distance delta */
	Real colissionDistDelta;
	/** Objects skin thickness */
	Real skinThickness;
	
	/** number of joints */
	const U32 numOfJoints;
	/** Bounds of arm joints */
	mutable std::vector<golem::Bounds::Seq> armBounds;	
	/** Poses of bounds of arm joints */
	std::vector<Mat34Seq> armBoundsPoses;
	/** The collection of bounds of objects */
	golem::Bounds::Seq collisionBounds;	
	
	/** Joint position limits */
	ConfigspaceCoord min, max, delta;
	
	mutable CriticalSection cs;

	void setPose(golem::Bounds::Seq& boundsSeq, const Mat34Seq& boundsPoses, const Mat34& pose) const;

	bool intersect(const golem::Bounds::Seq& boundsSeq0, const golem::Bounds::Seq& boundsSeq1) const;

	/** Creates Heuristic from the description. 
	* @param desc		Heuristic description
	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	bool create(const Desc& desc);

	/** Heuristic constructor */
	Heuristic(golem::Arm &arm);

public:
	/** virtual destructor */
	virtual ~Heuristic();
	
#ifdef _HEURISTIC_PERFMON
	static U32 waypointCollisionCounter, pathCollisionCounter;

	static void resetLog();
	static void writeLog(Context &context, const char *str);
#endif
	
	/** Collision detection test function for the single waypoint.
	 * @param w			waypoint
	 * @return			<code>true</code> if a collision has been detected;
	 *					<code>false</code> otherwise
	 */
	virtual bool collides(const Waypoint &w) const;

	/** Collision detection test function between specified waypoints.
	 * @param w0		waypoint
	 * @param w1		waypoint
	 * @param seq		waypoint graph
	 * @return			<code>true</code> if a collision has been detected;
	 *					<code>false</code> otherwise
	 */
	virtual bool collides(const Waypoint &w0, const Waypoint &w1) const;

	/** Synchronise bounds of all joints with the bounds descriptions
	*/
	virtual void syncArmBoundsDesc();
	
	/** Sets the collection of collision bounds
	 * @param collisionBoundsSeq	collection of collision bounds
	*/
	virtual void setCollisionBounds(const Bounds::SeqPtr &pCollisionBounds);

	/** Distance to the rest configuration in configuration space
	*/
	virtual Real getRestJointspaceDist(const ConfigspaceCoord& cc) const;

	/** Distance between two waypoints in configuration space
	*/
	virtual Real getJointspaceDist(const ConfigspaceCoord& j0, const ConfigspaceCoord& j1) const;

	/** Distance between two waypoints in configuration space
	*/
	virtual Real getJointspaceMagnitude(const ConfigspaceCoord& cc) const;

	/** Distance between two waypoints in configuration space
	*/
	virtual Real getJointspaceBoundedDist(const ConfigspaceCoord& j0, const ConfigspaceCoord& j1) const;

	/** Linear distance between two waypoints in workspace
	*/
	virtual Real getLinearDist(const Vec3& l0, const Vec3& l1) const;

	/** Angular distance between two waypoints in workspace
	*/
	virtual Real getAngularDist(const Quat& o0, const Quat& o1) const;

	/** Distance between two waypoints in workspace
	*/
	virtual Real getWorkspaceDist(const Waypoint& w0, const Waypoint& w1) const;

	/** Distance between two waypoints in workspace
	*/
	virtual Real getWorkspaceBoundedDist(const Waypoint& w0, const Waypoint& w1) const;

	/** Weighted distance between two waypoints
	*/
	virtual Real getDist(const Waypoint& w0, const Waypoint& w1) const;
	
	/** Weighted distance between two waypoints
	*/
	virtual Real getBoundedDist(const Waypoint& w0, const Waypoint& w1) const;

	/** Returns characteristic workspace distance scale */
	virtual Real getDistLinearFac() const {
		return distLinearFac;
	}
	/** Sets characteristic workspace distance scale */
	virtual void setDistLinearFac(Real distLinearFac) {
		this->distLinearFac = distLinearFac;
	}
	
	/** Returns workspace distance factor */
	virtual Real getDistWorkspaceFac() const {
		return distWorkspaceFac;
	}
	/** Sets workspace distance factor */
	virtual void setDistWorkspaceFac(Real distWorkspaceFac) {
		this->distWorkspaceFac = distWorkspaceFac;
	}
	
	/** Returns configuration space distance factor */
	virtual Real getDistJointspaceFac() const {
		return distJointspaceFac;
	}
	/** Sets configuration space distance factor */
	virtual void setDistJointspaceFac(Real distJointspaceFac) {
		this->distJointspaceFac = distJointspaceFac;
	}

	/** Returns Linear distance maximum */
	virtual Real getDistLinearMax() const {
		return distLinearMax;
	}
	/** Sets Linear distance maximum */
	virtual void setDistLinearMax(Real distLinearMax) {
		this->distLinearMax = distLinearMax;
	}

	/** Returns angular distance maximum */
	virtual Real getDistAngularMax() const {
		return distAngularMax;
	}
	/** Sets angular distance maximum */
	virtual void setDistAngularMax(Real distAngularMax) {
		this->distAngularMax = distAngularMax;
	}

	/** Returns maximum distance between two configuration space coordinates */
	virtual const ConfigspaceCoord &getDistJointcoordMax() const {
		return distJointcoordMax;
	}
	/** Sets maximum distance between two configuration space coordinates */
	virtual void setDistJointcoordMax(const ConfigspaceCoord &distJointcoordMax) {
		this->distJointcoordMax = distJointcoordMax;
	}
	
	/** Returns rest configuration cost factor */
	virtual const ConfigspaceCoord &getDistRestJointcoordFac() const {
		return distRestJointcoordFac;
	}
	/** Sets rest configuration cost factor */
	virtual void setDistRestJointcoordFac(const ConfigspaceCoord &distRestJointcoordFac) {
		this->distRestJointcoordFac = distRestJointcoordFac;
	}

	/** Collision detection */
	virtual bool hasCollisionDetection() const {
		return collisionDetection;
	}
	/** Sets/clears collision detection */
	virtual void setCollisionDetection(bool collisionDetection) {
		this->collisionDetection = collisionDetection;
	}

	/** Returns collision distance delta */
	virtual Real getColissionDistDelta() const {
		return colissionDistDelta;
	}
	/** Sets collision distance delta */
	virtual void setColissionDistDelta(Real colissionDistDelta) {
		this->colissionDistDelta = colissionDistDelta;
	}
	
	/** Returns objects skin thickness */
	virtual Real getSkinThickness() const {
		return skinThickness;
	}
	/** Sets objects skin thickness */
	virtual void setSkinThickness(Real skinThickness) {
		this->skinThickness = skinThickness;
	}

	/** Access to Arm controller
	 * @return			reference to the Arm controller
	 */
	const Arm &getArm() const {
		return arm;
	}
	Arm &getArm() {
		return arm;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_HEURISTIC_H_*/
