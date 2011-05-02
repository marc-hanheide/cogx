/** @file Planner.h
 * 
 * Arm movement control library.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_PLANNER_H_
#define _GOLEM_CTRL_PLANNER_H_

#include <Golem/Ctrl/Arm.h>
#include <Golem/Ctrl/Heuristic.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Base class for Arm movement control. */
class Planner {
public:
	typedef shared_ptr<Planner> Ptr;

	/** Path of generalised joint states constitutes trajectory in the joint space */
	typedef std::list<GenConfigspaceState> Trajectory;
	
	/** Callback interface for data synchronization */
	class Callback {
	public:
		virtual ~Callback() {}
		/** synchronization of collision bounds */
		virtual void syncCollisionBounds() {}//= 0;
		/** synchronization of find data */
		virtual void syncFindData(Trajectory::const_iterator begin, Trajectory::const_iterator end, const GenWorkspaceState* wend = NULL) {}//= 0;
		/** synchronization of send data */
		virtual void syncSendData(Trajectory::const_iterator begin, Trajectory::const_iterator end) {}//= 0;
	};

	/** Planner description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		
		/** Maximal joint velocity (absolute value) normlised to joint velocity limits */
		Real velocity;
		/** Maximal joint acceleration (absolute value) normlised to joint acceleration limits */
		Real acceleration;
		/** Arm trajectory profile description */
		Profile::Desc::Ptr pProfileDesc;
		/** Heuristic description */
		Heuristic::Desc::Ptr pHeuristicDesc;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		virtual ~Desc() {}
		
		/** Creates Planner given the description object. 
		* @return		pointer to the Planner, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		virtual Planner::Ptr create(golem::Arm &arm) const = 0;
		
		/** Sets the planner parameters to the default values */
		virtual void setToDefault() {
			velocity = Real(1.0);
			acceleration = Real(1.0);
			pProfileDesc.reset(new Polynomial4::Desc);
			pHeuristicDesc.reset(new Heuristic::Desc);
		}

		/** Checks if the planner description is valid. */
		virtual bool isValid() const {
			if (velocity <= REAL_ZERO || acceleration <= REAL_ZERO)
				return false;
			if (pProfileDesc == NULL || !pProfileDesc->isValid())
				return false;
			if (pHeuristicDesc == NULL || !pHeuristicDesc->isValid())
				return false;
			
			return true;
		}
	};

protected:
	/** Arm controller interface */
	golem::Arm &arm;
	/** Context object */
	golem::Context &context;

	/** Maximal joint velocity (absolute value) normlised to joint velocity limits */
	Real velocity;
	/** Maximal joint acceleration (absolute value) normlised to joint acceleration limits */
	Real acceleration;
	/** Arm trajectory profile */
	Profile::Ptr pProfile;
	/** Heuristic object */
	Heuristic::Ptr pHeuristic;
	
	/** Generator of pseudo random numbers */
	Rand rand;
	
	/** Callback interface for auto synchronization of collision bounds */
	Callback* pCallback;

	/** Creates Planner from the description. 
	* @param desc		Planner description
	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	bool create(const Desc& desc);

	/** Planner constructor */
	Planner(golem::Arm &arm);

public:
	/** Descructor */
	virtual ~Planner();

	/** Finds (optimal) trajectory target in the obstacle-free configuration space.
	 * @param cend		computed trajectory end (target) in the configuration space
	 * @param begin		trajectory begin in the configuration space
	 * @param wend		query trajectory end (target) in the workspace
	 * @return			<code>true</code> success; <code>false</code> otherwise 
	 */
	virtual bool findTarget(GenConfigspaceState &cend, const GenConfigspaceState &begin, const GenWorkspaceState& wend) = 0;

	/** Finds obstacle-free (optimal) trajectory in the configuration space from begin to end.
	 * @param path		trajectory
	 * @param iter		instertion point
	 * @param begin		trajectory begin in the configuration space
	 * @param end		trajectory end in the configuration space
	 * @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	 */
	virtual bool findTrajectory(Trajectory &trajectory, Trajectory::iterator iter, const GenConfigspaceState &begin, const GenConfigspaceState &end, const GenWorkspaceState* wend = NULL) = 0;

	/** Finds obstacle-free straight line trajectory in the configuration space from begin towards end.
	 * @param path		trajectory
	 * @param iter		instertion point
	 * @param begin		trajectory begin in the configuration space
	 * @param end		trajectory end in the configuration space
	 * @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	 */
	virtual bool findConfigspaceTrajectory(Trajectory &trajectory, Trajectory::iterator iter, const GenConfigspaceState &begin, const GenConfigspaceState &end, const GenWorkspaceState* wend = NULL) = 0;

	/** Finds obstacle-free straight line trajectory in the workspace from begin towards end.
	 * @param path		trajectory
	 * @param iter		instertion point
	 * @param begin		trajectory begin in the configuration space
	 * @param end		trajectory end in the workspace
	 * @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	 */
	virtual bool findWorkspaceTrajectory(Trajectory &trajectory, Trajectory::iterator iter, const GenConfigspaceState &begin, const GenWorkspaceState &end) = 0;

	/** Sends a sequence of motor commands, shift time stamps if they refer to the past.
	 * @param begin		begin of the sequence
	 * @param end		end of the sequence
	 * @param timeWait	total waiting time in milliseconds
	 * @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	template <typename _Type, typename _Ptr> bool send(_Ptr begin, _Ptr end, MSecTmU32 timeWait = MSEC_TM_U32_INF) {
		const SecTmReal shift = std::max(SEC_TM_REAL_ZERO, context.getTimer().elapsed() + arm.getTimeDeltaAsync() - begin->t);
		SysTimer timer;

		for (_Ptr i = begin; i != end; ++i) {
			const MSecTmU32 elapsed = timeWait > MSEC_TM_U32_ZERO ? timer.elapsed() : MSEC_TM_U32_ZERO;
			_Type command = *i;
			command.t += shift;
			if (!arm.send(command, timeWait < elapsed ? MSEC_TM_U32_ZERO : timeWait - elapsed))
				return false;
		}

		// TODO this should be atomic call
		if (pCallback != NULL)
			pCallback->syncSendData(begin, end);

		return true;
	}
	
	/** Stops the arm.
	 * @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	 */
	bool stop();

	/** Maximal joint velocity (absolute value) normlised to joint velocity limits
	 * @return				maximal joint velocity
	 */
	virtual Real getVelocity() const {
		return velocity;
	}
	/** Sets maximal joint velocity (absolute value) normlised to joint velocity limits
	 * @param velocity		maximal joint velocity, range: (0..inf], default: 1.0
	 */
	virtual void setVelocity(Real velocity) {
		this->velocity = velocity;
	}
	
	/** Maximal joint acceleration (absolute value) normlised to joint acceleration limits
	 * @return				maximal joint acceleration
	 */
	virtual Real getAcceleration() const {
		return acceleration;
	}
	/** Sets maximal joint acceleration (absolute value) normlised to joint acceleration limits
	 * @param acceleration		maximal joint acceleration, range: (0..inf], default: 1.0
	 */
	virtual void setAcceleration(Real acceleration) {
		this->acceleration = acceleration;
	}

	/** Returns Trajectory profile
	 * @return				Trajectory profile
	 */
	virtual Profile::Ptr getTrajectory() const {
		return pProfile;
	}
	/** Sets Trajectory profile
	 * @param pProfile	Trajectory profile
	 */
	virtual void setTrajectory(const Profile::Ptr &pProfile) {
		this->pProfile = pProfile;
	}

	/** Returns heuristic object
	 * @return				heuristic
	 */
	virtual Heuristic::Ptr getHeuristic() const {
		return pHeuristic;
	}
	/** Sets heuristic object
	 * @param pHeuristic	heuristic
	 */
	virtual void setHeuristic(const Heuristic::Ptr &pHeuristic) {
		this->pHeuristic = pHeuristic;
	}
	
	/** Callback interface for data synchronization
	 * @param pCallback	pointer to the interface
	*/
	void setCallback(Callback* pCallback) {
		this->pCallback = pCallback;
	}
	/** Callback interface for data synchronization
	 * @return				pointer to the interface
	*/
	Callback* getCallback() {
		return pCallback;
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

#endif /*_GOLEM_CTRL_PLANNER_H_*/
