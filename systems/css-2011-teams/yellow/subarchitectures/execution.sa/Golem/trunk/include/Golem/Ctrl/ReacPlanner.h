/** @file ReacPlanner.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_REACTIVEPLANNER_H_
#define _GOLEM_CTRL_REACTIVEPLANNER_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Bounds.h>
#include <Golem/Tools/Stream.h>
#include <Golem/Ctrl/Planner.h>
#include <Golem/Ctrl/DEKinematics.h>
#include <Golem/Ctrl/Transmitter.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Class realising reactive arm movement control. */
class ReacPlanner : protected Runnable {
public:
	typedef shared_ptr<ReacPlanner> Ptr;
	friend class Desc;
	
	/** Action type */
	enum Action {
		/** stop */
		ACTION_STOP,
		/** auto */
		ACTION_AUTO,
		/** reactive action */
		ACTION_MOVE,
		/** local action */
		ACTION_LOCAL,
		/** global action */
		ACTION_GLOBAL,
	};

	/** Signal type */
	typedef Transmitter::Signal Signal;

	/** ReacPlanner description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		
		/** Transmitter description */
		Transmitter::Desc::Ptr pTransmitterDesc;
		/** Signal synchronization */
		bool signalSync;
		/** Working thread priority */
		Thread::Priority threadPriority;
		/** Inter-thread signalling time out */
		MSecTmU32 threadTimeOut;
		
		/** Kinematics solver description */
		DEKinematics::Desc::Ptr pKinematicsDesc;
		/** Kinematic incremental search range */
		GenConfigspaceCoord globalRange, localRange;
		/** Kinematics solver latency/max running time */
		SecTmReal runningTime;
		/** Kinematics solver latency/max sleep time */
		SecTmReal sleepTime;
		
		/** Linear distance maximum */
		Real distLinearMax;
		/** angular distance maximum */
		Real distAngularMax;
		/** collision distance delta */
		Real colissionDistDelta;
		/** Rest position distance factor */
		ConfigspaceCoord distRestJointcoordFac;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** virtual destructor */
		virtual ~Desc() {}
		
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(ReacPlanner, ReacPlanner::Ptr, Planner&)
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			pTransmitterDesc.reset(new Transmitter::Desc);
			signalSync = true;
			threadPriority = Thread::ABOVE_NORMAL;
			threadTimeOut = 30000; //[msec]
			
			pKinematicsDesc.reset(new DEKinematics::Desc);
			pKinematicsDesc->collisionDetection = false;
			pKinematicsDesc->generator.reset(new DEKinematics::GaussianGenerator(Real(0.5)));
	

			globalRange.pos.set(Real(2.0)*REAL_PI);
			globalRange.vel.set(Real(2.0)*REAL_PI);
			globalRange.acc.set(Real(2.0)*REAL_PI);
			
			localRange.pos.set(Real(0.1)*REAL_PI);
			localRange.vel.set(Real(0.05)*REAL_PI);
			localRange.acc.set(Real(0.05)*REAL_PI);
			
			runningTime = SecTmReal(0.1);
			sleepTime = SecTmReal(0.02);

			distLinearMax = Real(0.2);
			distAngularMax = Real(0.5);
			colissionDistDelta = Real(0.005);
			distRestJointcoordFac.set(Real(10.0));
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (pTransmitterDesc == NULL || !pTransmitterDesc->isValid())
				return false;
			if (pKinematicsDesc == NULL || !pKinematicsDesc->isValid())
				return false;
			if (!globalRange.pos.isPositive() || !globalRange.vel.isPositive() || !globalRange.acc.isPositive())
				return false;
			if (!localRange.pos.isPositive() || !localRange.vel.isPositive() || !localRange.acc.isPositive())
				return false;
			if (runningTime <= SEC_TM_REAL_ZERO)
				return false;
			if (sleepTime <= SEC_TM_REAL_ZERO)
				return false;
			
			if (distLinearMax <= REAL_ZERO)
				return false;
			if (distAngularMax <= REAL_ZERO)
				return false;
			if (colissionDistDelta <= REAL_ZERO)
				return false;
			if (!distRestJointcoordFac.isPositive())
				return false;
			
			return true;
		}
	};

protected:
	/** Parameter guard */
	class ParameterGuard {
	private:
		Heuristic::Ptr pHeuristic;
		ConfigspaceCoord distRestJointcoordFac;

	public:
		ParameterGuard(const Heuristic::Ptr &pHeuristic) : pHeuristic(pHeuristic) {
			distRestJointcoordFac = pHeuristic->getDistRestJointcoordFac();
		}

		~ParameterGuard() {
			pHeuristic->setDistRestJointcoordFac(distRestJointcoordFac);
		}
	};
	
	/** Planner interface */
	golem::Planner &planner;
	/** Arm controller interface */
	golem::Arm &arm;
	/** Context object */
	golem::Context &context;
	
	/** Kinematics solver */
	DEKinematics::Ptr pKinematics;
	/** Kinematic incremental search range */
	GenConfigspaceCoord globalRange, localRange;
	/** Kinematics solver latency/max running time */
	SecTmReal runningTime;
	/** Kinematics solver latency/max sleep time */
	SecTmReal sleepTime;
	/** Arm controller time delta */
	SecTmReal tmDelta;
	/** Arm controller async time delta */
	SecTmReal tmDeltaAsync;

	/** Linear distance maximum */
	Real distLinearMax;
	/** angular distance maximum */
	Real distAngularMax;
	/** collision distance delta */
	Real colissionDistDelta;
	/** Rest position cost factor */
	ConfigspaceCoord distRestJointcoordFac;
	
	/** Transmitter */
	Transmitter::Ptr pTransmitter;
	/** Signal init */
	bool signalInit;
	/** Signal synchronization */
	bool signalSync;
	/** Action type */
	Action action;
	
	Thread thread;
	Thread::Priority threadPriority;
	MSecTmU32 threadTimeOut;
	Event evSend, evWaitBegin, evWaitEnd, evTerminate;

	virtual void run();

	virtual bool send(Action action);

	virtual bool trimPath(GenConfigspaceCoord &next, const GenConfigspaceCoord &prev) const;
	
	virtual void generateTrajectory(GenCoordTrj *trajectories, SecTmReal &duration, const GenConfigspaceState &prev, const GenConfigspaceState &next) const;
	
	virtual Action getAction(const GenConfigspaceState &pose, const Signal &target) const;
	
	virtual bool move(GenConfigspaceState &pose, GenConfigspaceState &next, const GenConfigspaceState &prev, const Signal &target, SecTmReal runningTime = SEC_TM_REAL_MAX);
	virtual bool plan(GenConfigspaceState &pose, GenConfigspaceState &next, const GenConfigspaceState &prev, const Signal &target, Action action, SecTmReal runningTime = SEC_TM_REAL_MAX);
	virtual void stop(GenConfigspaceState &pose);

	/** Creates ReacPlanner from the description. 
	* @param desc		ReacPlanner description
	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	bool create(const Desc& desc);

	/** Releases resources */
	void release();
	
	/** ReacPlanner constructor */
	ReacPlanner(golem::Planner &planner);

public:
	/** Descructor */
	virtual ~ReacPlanner();

	/** Sends control signal vector (non-blocking call). */
	virtual bool send(const Transmitter::Signal &signal, Action action = ACTION_AUTO) {
		return signal.type == Transmitter::Signal::TYPE_JOINTSPACE ? send(signal.gjs, action) : send(signal.gws, action);
	}
	
	/** Sends control signal vector in configuration space coordinates (non-blocking call). */
	virtual bool send(const GenConfigspaceState &data, Action action = ACTION_AUTO) {
		return pTransmitter->set(data) && send(action);
	}
	
	/** Sends control signal vector in workspace coordinates (non-blocking call). */
	virtual bool send(const GenWorkspaceState &data, Action action = ACTION_AUTO) {
		return pTransmitter->set(data) && send(action);
	}
	
	/** Sends control signal vector in user-defined format (non-blocking call). */
	virtual bool send(const void *data, Action action = ACTION_AUTO) {
		return pTransmitter->set(data) && send(action);
	}

	/** Sends the initial control signal vector */
	virtual bool reset(Action action = ACTION_AUTO) {
		return send(pTransmitter->getInitSignal(), action);
	}
	
	/** Stops the arm */
	virtual bool stop() {
		return send(ACTION_STOP);
	}
	
	/** Wait for event completion */
	virtual bool waitForBegin(MSecTmU32 timeOut = MSEC_TM_U32_INF);
	
	/** Wait for event completion */
	virtual bool waitForEnd(MSecTmU32 timeOut = MSEC_TM_U32_INF);
	
	
	/** Access to Planner */
	inline const Planner &getPlanner() const {
		return planner;
	}
	inline Planner &getPlanner() {
		return planner;
	}
	
	/** Access to Arm controller */
	inline const Arm &getArm() const {
		return arm;
	}
	inline Arm &getArm() {
		return arm;
	}
	
	/** Transmitter */
	virtual const Transmitter &getTransmitter() const {
		return *pTransmitter;
	}
	virtual Transmitter &getTransmitter() {
		return *pTransmitter;
	}

	/** Kinematics solver */
	virtual const DEKinematics &getKinematics() const {
		return *pKinematics;
	}
	virtual DEKinematics &getKinematics() {
		return *pKinematics;
	}

	/** Kinematic global search range */
	virtual void setGlobalRange(const GenConfigspaceCoord &globalRange) {
		this->globalRange = globalRange;
	}

	/** Kinematic global search range */
	virtual const GenConfigspaceCoord &getGlobalRange() const {
		return globalRange;
	}
	
	/** Kinematic incremental local search range */
	virtual void setLocalRange(const GenConfigspaceCoord &localRange) {
		this->localRange = localRange;
	}

	/** Kinematic incremental local search range */
	virtual const GenConfigspaceCoord &getLocalRange() const {
		return localRange;
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

	/** Returns collision distance delta */
	virtual Real getColissionDistDelta() const {
		return colissionDistDelta;
	}
	/** Sets collision distance delta */
	virtual void setColissionDistDelta(Real colissionDistDelta) {
		this->colissionDistDelta = colissionDistDelta;
	}
	
	/** Arm controller time delta */
	virtual inline SecTmReal getTimeDelta() const {
		return tmDelta;
	}

	/** Arm controller asynchronous time delta */
	virtual inline SecTmReal getTimeDeltaAsync() const {
		return tmDeltaAsync;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_REACTIVEPLANNER_H_*/
