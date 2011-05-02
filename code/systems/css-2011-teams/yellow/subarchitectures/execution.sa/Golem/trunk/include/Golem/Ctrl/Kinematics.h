/** @file Kinematics.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_KINEMATICS_H_
#define _GOLEM_CTRL_KINEMATICS_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Heuristic.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Inverse kinematic problem solver. */
class Kinematics {
public:
	typedef shared_ptr<Kinematics> Ptr;

	/** Kinematics description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
	
		/** search range min and max */
		GenConfigspaceCoord min, max;
		/** Colision detection */
		bool collisionDetection;		
		
		/** Creates Kinematics given the description object. 
		* @return		pointer to the Kinematics, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		virtual Kinematics::Ptr create(golem::Heuristic &heuristic) const = 0;
		
		/** Constructs the description object. */
		Desc() {
			setToDefault();
		}
		
		/** virtual destructor */
		virtual ~Desc() {}
		
		/** Sets the parameters to the default values */
		void setToDefault() {
			min.pos.set(-Real(2.0)*REAL_PI);
			min.vel.set(-Real(10.0)*REAL_PI);
			min.acc.set(-Real(10.0)*REAL_PI);
			
			max.pos.set(+Real(2.0)*REAL_PI);
			max.vel.set(+Real(10.0)*REAL_PI);
			max.acc.set(+Real(10.0)*REAL_PI);

			collisionDetection = true;
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!min.pos.isFinite() || !min.vel.isFinite() || !min.acc.isFinite())
				return false;
			if (!max.pos.isFinite() || !max.vel.isFinite() || !max.acc.isFinite())
				return false;

			return true;
		}
	};

protected:	
	/** Heuristic */
	golem::Heuristic &heuristic;
	/** Arm */
	golem::Arm &arm;
	/** Context object */
	golem::Context &context;
		
	/** Generator of pseudo random numbers */
	Rand rand;
	
	/** search range min and max */
	GenConfigspaceCoord min, max;
	/** Colision detection */
	bool collisionDetection;		
	
	/** Creates Kinematics from the description. 
	* @param desc		Kinematics description
	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	bool create(const Desc& desc);

	/** Kinematics constructor */
	Kinematics(golem::Heuristic &heuristic);

public:
	/** Descructor */
	virtual ~Kinematics();

	/** Generalised inverse kinematics solver */
	virtual bool find(GenConfigspaceCoord &gj, const GenWorkspaceCoord &gw, SecTmReal runningTime = SEC_TM_REAL_MAX) = 0;
	
	/** Inverse kinematics solver */
	virtual bool find(ConfigspaceCoord &j, const WorkspaceCoord &w, SecTmReal runningTime = SEC_TM_REAL_MAX) = 0;

	/** search range min */
	virtual void setMin(const GenConfigspaceCoord &min);

	/** search range min */
	virtual const GenConfigspaceCoord &getMin() const {
		return min;
	}

	/** search range max */
	virtual void setMax(const GenConfigspaceCoord &max);

	/** search range max */
	virtual const GenConfigspaceCoord &getMax() const {
		return max;
	}
	
	/** Collision detection */
	virtual bool hasCollisionDetection() const {
		return collisionDetection;
	}
	/** Sets/clears collision detection */
	virtual void setCollisionDetection(bool collisionDetection) {
		this->collisionDetection = collisionDetection;
	}

	/** Access to Heuristic
	 * @return				reference to the Heuristic
	 */
	const Heuristic &getHeuristic() const {
		return heuristic;
	}
	Heuristic &getHeuristic() {
		return heuristic;
	}

	/** Access to Arm controller
	 * @return				reference to the Arm controller
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

#endif /*_GOLEM_CTRL_KINEMATICS_H_*/
