/** @file PathFinder.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_PATHFINDER_H_
#define _GOLEM_CTRL_PATHFINDER_H_

//------------------------------------------------------------------------------

#include <Golem/Math/GraphSearch.h>
#include <Golem/Ctrl/Waypoint.h>
#include <Golem/Ctrl/DEKinematics.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Performance monitor */
//#define _PATHFINDER_PERFMON

//------------------------------------------------------------------------------

class Planner;

/** Abstract class for Arm movement planinng using Probabilistic Road Map approach. */
class PathFinder : protected golem::GraphSearch {
public:
	typedef shared_ptr<PathFinder> Ptr;
	friend class Desc;

	/** Graph search description */
	class GraphSearchDesc {
	public:
		/** Waypoints generators */
		ConfigspaceCoord::Seq generators;
		/** Waypoint generator range */
		ConfigspaceCoord range;
		/** number of waypoints of the search graph */
		U32 size;
		/** maximum number of initialisation trials of a random waypoint */
		U32 waypointInitTrials;
		/** goal distance factor */
		Real distGoalFac;
		/** path distance factor */
		Real distPathFac;

		/** Constructs the description object. */
		GraphSearchDesc() {
			setToDefault();
		}
		
		/** Sets the parameters to the default values */
		void setToDefault() {
			generators.clear();
			range.set(REAL_2_PI);
			size = 2000;
			waypointInitTrials = 500;
			distGoalFac = Real(0.99); // [0, 1]
			distPathFac = Real(0.98);
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!range.isPositive())
				return false;		
			if (size < 2)
				return false;
			if (waypointInitTrials < 1)
				return false;
			if (distGoalFac < REAL_ZERO || distGoalFac > REAL_ONE)
				return false;
			if (distPathFac < REAL_ZERO || distPathFac > REAL_ONE)
				return false;

			return true;
		}
	};

	/** PathFinder description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;

		/** Kinematics solver description */
		DEKinematics::Desc::Ptr pKinematicsDesc;		
		/** Graph search description */
		GraphSearchDesc graphSearchDesc;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** virtual destructor */
		virtual ~Desc() {}
		
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(PathFinder, PathFinder::Ptr, Planner&)
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			pKinematicsDesc.reset(new DEKinematics::Desc);
			graphSearchDesc.setToDefault();
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (pKinematicsDesc == NULL || !pKinematicsDesc->isValid())
				return false;
			if (!graphSearchDesc.isValid())
				return false;

			return true;
		}
	};

	/** Waypoint pointer wrapper */
	class WaypointPtr {
	private:
		Waypoint *pWaypoint;
		static Node::cost_less less;// local classes cannot have static members

	public:
		typedef std::vector<WaypointPtr> Seq;
		
		Waypoint &operator * () const {
			return *pWaypoint;
		}
		Waypoint *operator -> () const {
			return pWaypoint;
		}
		bool operator < (const WaypointPtr &right) const {
			return less.operator () (*pWaypoint, *right.pWaypoint);
		}
		WaypointPtr(Waypoint *pWaypoint) : pWaypoint(pWaypoint) {
		}
	};
	
protected:
	/** Planner */
	golem::Planner &planner;
	/** Arm controller interface */
	golem::Arm &arm;
	/** Context object */
	golem::Context &context;

	/** Kinematics solver */
	DEKinematics::Ptr pKinematics;
	/** Graph search description */
	GraphSearchDesc graphSearchDesc;
	
	/** Waypoint graph generators */
	ConfigspaceCoord::Seq generators;
	/** Waypoint graph */
	Waypoint::Seq graph;

	/** Generator of pseudo random numbers */
	Rand rand;
	
	virtual Real cost(const Waypoint &w) const;
	
	virtual bool collides(const Waypoint &w) const;
	
	virtual Real cost(U32 i, U32 j) const;

	virtual bool collides(U32 i, U32 j) const;

	/** Generates random waypoint */
	virtual bool generateWaypoint(Waypoint& w, const ConfigspaceCoord &min, const ConfigspaceCoord &max, U32 trials) const;
	
	/** Generates random waypoint graph */
	virtual bool generateGraph(Waypoint::Seq& graph, const ConfigspaceCoord::Seq &generators, const Waypoint &target) const;
	
	/** Finds the optimal target state in the configuration space. */
	virtual bool findTarget(Waypoint::Seq& graph, const ConfigspaceCoord::Seq &generators) const;
	
	/** Finds shortest path in the configuration space. */
	bool findPath(Waypoint::Seq &path, Waypoint::Seq::iterator iter);
	
	/** Creates PathFinder from the description. 
	* @param desc		PathFinder description
	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	bool create(const Desc& desc);

	/** PathFinder constructor */
	PathFinder(golem::Planner &planner);

public:
	/** Generates graph.
	 * @param	begin		path begin in the configuration space
	 * @param	end			path end (target) in the configuration space
	 * @return				<code>true</code> success; <code>false</code> otherwise 
	 */
	virtual bool generateGraph(const ConfigspaceCoord &begin, const ConfigspaceCoord& end);

	/** Generates graph.
	 * @param	begin		path begin in the configuration space
	 * @param	end			path end (target) in the workspace
	 * @return				<code>true</code> success; <code>false</code> otherwise 
	 */
	virtual bool generateGraph(const ConfigspaceCoord &begin, const WorkspaceCoord& end);

	/** Finds target in the configuration space.
	 * @param	end			path end (target) in the configuration space
	 * @return				<code>true</code> success; <code>false</code> otherwise 
	 */
	virtual bool findTarget(ConfigspaceCoord &end);

	/** Finds path (a sequence of waypoints) in the configuration space from begin to end.
	 * @param	path		a sequence of waypoints
	 * @param	iter		insertion pointer
	 * @param	begin		path begin in the configuration space
	 * @param	end			path end in the configuration space
	 * @return				<code>true</code> success; <code>false</code> otherwise 
	 */
	virtual bool findPath(Waypoint::Seq &path, Waypoint::Seq::iterator iter, const ConfigspaceCoord &begin, const ConfigspaceCoord &end);

	/** Returns Kinematics object
	 * @return				Kinematics object
	 */
	virtual DEKinematics::Ptr getKinematics() const {
		return pKinematics;
	}

	/** Sets Kinematics object
	 * @param pKinematics	Kinematics object
	 */
	virtual void setKinematics(const DEKinematics::Ptr &pKinematics) {
		this->pKinematics = pKinematics;
	}
	
	/** Returns graph description
	 * @return				graph description
	 */
	virtual const GraphSearchDesc &getGraphSearchDesc() const {
		return graphSearchDesc;
	}
	/** Sets graph description
	 * @param graphSearchDesc		graph description
	 */
	virtual void setGraphSearchDesc(const GraphSearchDesc &graphSearchDesc) {
		this->graphSearchDesc = graphSearchDesc;
	}
	
	/** Returns waypoint graph */
	const Waypoint::Seq &getGraph() const {
		return graph;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_PATHFINDER_H_*/
