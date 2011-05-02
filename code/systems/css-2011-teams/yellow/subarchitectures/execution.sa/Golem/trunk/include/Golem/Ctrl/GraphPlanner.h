/** @file GraphPlanner.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_GRAPHPLANNER_H_
#define _GOLEM_CTRL_GRAPHPLANNER_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Sample.h>
#include <Golem/Ctrl/Planner.h>
#include <Golem/Ctrl/PathFinder.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Performance monitor */
//#define _GRAPHPLANNER_PERFMON

//------------------------------------------------------------------------------

/** ConfigspaceCoord sample */
class ConfigspaceCoordSample : public ConfigspaceCoord, public Sample<Real> {
public:
	typedef std::vector<ConfigspaceCoordSample> Seq;

	/** Sets the parameters to the default values */
	void setToDefault() {
		ConfigspaceCoord::set(REAL_ZERO);
		Sample<Real>::setToDefault();
	}

	/** Checks if sample is valid. */
	inline bool isValid() const {
		if (!ConfigspaceCoord::isFinite() || !Sample<Real>::isValid())
			return false;
		
		return true;
	}
};

/** Extended waypoint object */
class OptWaypoint : public Waypoint {
public:
	typedef std::vector<OptWaypoint> Seq;

	/** Time */
	SecTmReal t;
	/** Time normalised */
	SecTmReal tNorm;
	/** Distance on a path */
	Real dist;
	
	/** velocity in configuration space coordinates */
	ConfigspaceCoord cvel;
	/** acceleration in configuration space coordinates */
	ConfigspaceCoord jacc;

	/** ConfigspaceCoord population */
	ConfigspaceCoordSample::Seq population;
	/** Range */
	ConfigspaceCoord range;

	/** Sets the parameters to the default values */
	void setToDefault() {
		t = tNorm = dist = REAL_ZERO;
		population.clear();
		range.setZero();
	}

	/** Sets the parameters to the default values */
	static void setToDefault(OptWaypoint& w) {
		w.setToDefault();
	}

	/** Sets the parameters from a waypoint */
	void setTo(const Waypoint &w) {
		Waypoint::operator = (w);
		setToDefault();
	}

	/** Sets the parameters from a waypoint */
	static void setTo(OptWaypoint& w1, const Waypoint &w2) {
		w1.setTo(w2);
	}

	/** Assignment operator */
	OptWaypoint &operator = (const Waypoint &w) {
		Waypoint::operator = (w);
		return *this;
	}
};

//------------------------------------------------------------------------------

/** Abstract class for Arm movement planinng using Probabilistic Road Map approach. */
class GraphPlanner : public Planner {
public:
	typedef shared_ptr<GraphPlanner> Ptr;
	friend class Desc;

	/** Path finder description */
	class PathFinderDesc {
	public:
		/** Global path finder description */
		PathFinder::Desc::Ptr pGlobalPathFinderDesc;
		
		/** Local path finder description */
		PathFinder::Desc::Ptr pLocalPathFinderDesc;

		/** Bounded distance scale factor */
		Real distScaleFac;
		/** Waypoint generator range factor */
		Real rangeFac;
		/** Local path finder number of iterations */
		U32 numOfIterations;
		
		/** Constructs the description object. */
		PathFinderDesc() {
			setToDefault();
		}
		
		/** Sets the parameters to the default values */
		void setToDefault() {
			pGlobalPathFinderDesc.reset(new PathFinder::Desc);
			pGlobalPathFinderDesc->graphSearchDesc.size = 2000;
			
			pLocalPathFinderDesc.reset(new PathFinder::Desc);
			pLocalPathFinderDesc->graphSearchDesc.size = 1000;

			distScaleFac = Real(0.75);
			rangeFac = Real(0.5);
			numOfIterations = 4;
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (pGlobalPathFinderDesc == NULL || !pGlobalPathFinderDesc->isValid())
				return false;
			
			if (pLocalPathFinderDesc != NULL && !pLocalPathFinderDesc->isValid())
				return false;
			
			if (distScaleFac <= REAL_ZERO || distScaleFac > REAL_ONE)
				return false;
			if (rangeFac <= REAL_ZERO)
				return false;
			if (numOfIterations <= REAL_ZERO)
				return false;

			return true;
		}
	};

	/** Path optimiser description */
	class PathOptimiserDesc {
	public:
		/** number of iterations per path waypoint: (0, inf) */
		U32 numOfIterations;
		/** population size: [0, inf) */
		U32 populationSize;
		/** temperature settings: inf > Tinit > Tfinal > 0 */
		Real Tinit, Tfinal;
		/** energy normalization */
		Real Enorm;
		/** crossover probability: [0, 1] */
		Real crossProb;
		/** search probability: [0, 1] */
		Real searchProb;
		/** vector combination factor: [0, inf) */
		Real combFac;
		/** vector memory factor: [0, 1] */
		Real memFac;
		
		/** path distance factor */
		Real distPathFac;
		/** path (minimum) distance threshold */
		Real distPathThr;
		
		/** Constructs the description object. */
		PathOptimiserDesc() {
			setToDefault();
		}
		
		/** Sets the parameters to the default values */
		void setToDefault() {
			numOfIterations = 500;
			populationSize = 1;
			Tinit = Real(0.1);
			Tfinal = Real(0.01);
			Enorm = Real(3.0e4);
			crossProb = Real(0.3);
			searchProb = Real(0.4);
			combFac = Real(0.2);
			memFac = Real(0.2);
			distPathFac = Real(0.9); // [0, 1]
			distPathThr = Real(0.25); // (0, 1)
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (numOfIterations <= 0)
				return false;
			if (Tinit <= REAL_ZERO || Tfinal <= REAL_ZERO)
				return false;
			if (Enorm <= REAL_ZERO)
				return false;
			if (crossProb < REAL_ZERO || crossProb > REAL_ONE)
				return false;
			if (searchProb < REAL_ZERO || searchProb > REAL_ONE)
				return false;
			if (combFac < REAL_ZERO)
				return false;
			if (memFac < REAL_ZERO || memFac > REAL_ONE)
				return false;

			if (distPathFac < REAL_ZERO || distPathFac > REAL_ONE)
				return false;
			if (distPathThr <= REAL_ZERO || distPathThr >= REAL_ONE)
				return false;

			return true;
		}
	};

	/** Local finder description */
	class LocalFinderDesc {
	public:
		/** Kinematics solver description */
		DEKinematics::Desc::Ptr pKinematicsDesc;
		/** Kinematic incremental search range */
		GenConfigspaceCoord range;
		/** Interpolation distance delta */
		Real interpolDistDelta;
		/** Collision distance delta */
		Real colissionDistDelta;
		
		/** Constructs the description object. */
		LocalFinderDesc() {
			setToDefault();
		}
		
		/** Sets the parameters to the default values */
		void setToDefault() {
			pKinematicsDesc.reset(new DEKinematics::Desc);
			pKinematicsDesc->populationSize = 100;
			pKinematicsDesc->numOfGenerations = 10000;
			pKinematicsDesc->collisionDetection = false;
			pKinematicsDesc->generator.reset(new DEKinematics::GaussianGenerator(Real(0.5)));

			range.pos.set(Real(0.1)*REAL_PI);
			range.vel.set(Real(0.05)*REAL_PI);
			range.acc.set(Real(0.05)*REAL_PI);
			
			interpolDistDelta =  Real(0.1);
			colissionDistDelta = Real(0.005);
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (pKinematicsDesc == NULL || !pKinematicsDesc->isValid())
				return false;
			if (!range.pos.isPositive() || !range.vel.isPositive() || !range.acc.isPositive())
				return false;
			if (interpolDistDelta <= REAL_ZERO)
				return false;
			if (colissionDistDelta <= REAL_ZERO)
				return false;

			return true;
		}
	};

	/** Planner description */
	class Desc : public Planner::Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		
		/** Path finder description */
		PathFinderDesc pathFinderDesc;
		/** Path optimiser description */
		PathOptimiserDesc pathOptimiserDesc;
		/** Local finder description */
		LocalFinderDesc localFinderDesc;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(GraphPlanner, Planner::Ptr, Arm&)
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Planner::Desc::setToDefault();

			pathFinderDesc.setToDefault();
			pathOptimiserDesc.setToDefault();
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Planner::Desc::isValid())
				return false;

			if (!pathFinderDesc.isValid() || !pathOptimiserDesc.isValid() || !localFinderDesc.isValid())
				return false;

			return true;
		}
	};

protected:
	/** Parameter guard */
	class ParameterGuard {
	private:
		Heuristic::Ptr pHeuristic;
		Real distLinearMax;
		Real distAngularMax;
		ConfigspaceCoord distJointcoordMax;

	public:
		ParameterGuard(Heuristic::Ptr &pHeuristic) : pHeuristic(pHeuristic) {
			distLinearMax = pHeuristic->getDistLinearMax();
			distAngularMax = pHeuristic->getDistAngularMax();
			distJointcoordMax = pHeuristic->getDistJointcoordMax();
		}

		~ParameterGuard() {
			pHeuristic->setDistLinearMax(distLinearMax);
			pHeuristic->setDistAngularMax(distAngularMax);
			pHeuristic->setDistJointcoordMax(distJointcoordMax);
		}
		
		void setBaundedDistScale(Real scale) {
			pHeuristic->setDistLinearMax(scale*distLinearMax);
			pHeuristic->setDistAngularMax(scale*distAngularMax);
			ConfigspaceCoord cc;
			for (U32 i = 0; i < CONFIG_SPACE_DIM; i++)
				cc[i] = scale*distJointcoordMax[i];
			pHeuristic->setDistJointcoordMax(cc);
		}
	};

	/** Global path finder */
	PathFinder::Ptr pGlobalPathFinder;
	/** Local path finder */
	PathFinder::Ptr pLocalPathFinder;
	
	/** Path finder description */
	PathFinderDesc pathFinderDesc;
	/** Path optimiser description */
	PathOptimiserDesc pathOptimiserDesc;
	
	/** Local finder description */
	LocalFinderDesc localFinderDesc;
	/** Kinematics solvers */
	DEKinematics::Ptr pKinematics;
	
	/** Global waypoint path */
	Waypoint::Seq globalPath;
	/** Local waypoint path */
	Waypoint::Seq localPath;	

	/** Optimised waypoint path */
	OptWaypoint::Seq optimisedPath;

	virtual Real cost(U32 n, const OptWaypoint::Seq &seq) const;
	
	virtual bool collides(U32 n, const OptWaypoint::Seq &seq) const;

	virtual bool purge(OptWaypoint::Seq &seq) const;

	virtual bool trimPath(ConfigspaceCoord &next, const ConfigspaceCoord &prev) const;
	
	/** Performs local search on waypoint path */
	virtual bool localFind(Waypoint::Seq &localPath, const ConfigspaceCoord &begin, const ConfigspaceCoord &end, ParameterGuard& parameterGuard);
	
	/** Optimises waypoint path */
	virtual bool optimize(OptWaypoint::Seq &optimisedPath, const GenConfigspaceState &begin, const GenConfigspaceState &end) const;
	
	/** Profiles waypoint path, i.e. generates trajectory */
	virtual bool profile(Trajectory& trajectory, Trajectory::iterator iter, OptWaypoint::Seq& optimisedPath, const GenConfigspaceState &begin, const GenConfigspaceState &end) const;
	
	/** Creates Planner from the description. 
	* @param desc		Planner description
	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	bool create(const Desc& desc);

	/** Planner constructor */
	GraphPlanner(golem::Arm &arm);

public:
	/** Finds (optimal) trajectory target in the obstacle-free configuration space.
	 */
	virtual bool findTarget(GenConfigspaceState &cend, const GenConfigspaceState &begin, const GenWorkspaceState& wend);

	/** Finds obstacle-free (optimal) trajectory in the configuration space from begin to end.
	 */
	virtual bool findTrajectory(Trajectory &trajectory, Trajectory::iterator iter, const GenConfigspaceState &begin, const GenConfigspaceState &end, const GenWorkspaceState* wend = NULL);

	/** Finds obstacle-free straight line trajectory in the configuration space from begin towards end.
	 */
	virtual bool findConfigspaceTrajectory(Trajectory &trajectory, Trajectory::iterator iter, const GenConfigspaceState &begin, const GenConfigspaceState &end, const GenWorkspaceState* wend = NULL);

	/** Finds obstacle-free straight line trajectory in the workspace from begin towards end.
	 */
	virtual bool findWorkspaceTrajectory(Trajectory &trajectory, Trajectory::iterator iter, const GenConfigspaceState &begin, const GenWorkspaceState &end);

	/** Returns global path finder
	 * @return				path finder
	 */
	virtual PathFinder::Ptr getGlobalPathFinder() const {
		return pGlobalPathFinder;
	}
	/** Sets global path finder
	 * @param pPathFinder	path finder
	 */
	virtual void setGlobalPathFinder(const PathFinder::Ptr &pPathFinder) {
		this->pGlobalPathFinder = pPathFinder;
	}

	/** Returns local path finder
	 * @return				path finder
	 */
	virtual PathFinder::Ptr getLocalPathFinder() const {
		return pLocalPathFinder;
	}
	/** Sets local path finder
	 * @param pPathFinder	path finder
	 */
	virtual void setLocalPathFinder(const PathFinder::Ptr &pPathFinder) {
		this->pLocalPathFinder = pPathFinder;
	}

	/** Returns path optimiser description
	 * @return					path optimiser description
	 */
	virtual const PathOptimiserDesc &getPathOptimiserDesc() const {
		return pathOptimiserDesc;
	}
	/** Sets path optimiser description
	 * @param pathOptimiserDesc	path optimiser description
	 */
	virtual void setPathOptimiserDesc(const PathOptimiserDesc &pathOptimiserDesc) {
		this->pathOptimiserDesc = pathOptimiserDesc;
	}
	
	/** Returns global waypoint path */
	const Waypoint::Seq &getGlobalPath() const {
		return globalPath;
	}
	
	/** Returns local waypoint path */
	const Waypoint::Seq &getLocalPath() const {
		return localPath;
	}
	
	/** Returns optimised waypoint path */
	const OptWaypoint::Seq &getOptimisedPath() const {
		return optimisedPath;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_GRAPHPLANNER_H_*/
