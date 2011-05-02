/** @file Waypoint.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_WAYPOINT_H_
#define _GOLEM_CTRL_WAYPOINT_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Node.h>
#include <Golem/Ctrl/Arm.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Waypoint */
class Waypoint : public Node {
public:
	/** Pointer to waypoint. */
	typedef shared_ptr<Waypoint> Ptr;
	/** Sequence of waypoints. */
	typedef std::vector<Waypoint> Seq;

	/** Arm pose in configuration space coordinates. */
	ConfigspaceCoord cpos;
	/** Arm pose in workspace coordinates. */
	WorkspaceCoord wpos;
	Quat wposq;
	/** Extended forward transformation for all joints. */
	std::vector<Mat34> trn;

	/** Default constructor initialises only GraphSearch::Node */
	Waypoint(U32 index = Node::IDX_UINI, Real cost = Node::COST_ZERO) :
		Node(index, cost)
	{}

	/** Constructs Waypoint from node */
	Waypoint(const Node& node) :
		Node(node)
	{}

	/** Copy constructor */
	Waypoint(const Waypoint& waypoint) :
		Node(waypoint.index, waypoint.cost),
		cpos(waypoint.cpos),
		wpos(waypoint.wpos),
		wposq(waypoint.wposq),
		trn(waypoint.trn)
	{}

	void setup(const Arm &arm, const ConfigspaceCoord &cpos, bool bEx = true) {
		this->cpos = cpos;
		setup(arm, bEx);
	}

	void setup(const Arm &arm, bool bEx = true) {
		if (bEx) {
			trn.resize((U32)arm.getJoints().size());
			arm.forwardTransformEx(&trn.front(), cpos);
		}
		else {
			trn.resize((U32)1);
			arm.forwardTransform(trn.back(), cpos);
		}

		wpos.multiply(trn.back(), arm.getReferencePose()); // reference pose
		wposq.fromMat33(wpos.R);
	}

	Waypoint &operator = (const Waypoint &waypoint) {
		index = waypoint.index;
		cost = waypoint.cost;
		cpos = waypoint.cpos;
		wpos = waypoint.wpos;
		wposq = waypoint.wposq;
		trn = waypoint.trn;
		return *this;
	}
};

//------------------------------------------------------------------------------

/** Waypoint in the generalised coordinates */
class GenWaypoint : public Waypoint {
public:
	/** Pointer to waypoint. */
	typedef shared_ptr<GenWaypoint> Ptr;
	/** Sequence of waypoints. */
	typedef std::vector<GenWaypoint> Seq;

	/** Arm velocity in configuration space coordinates. */
	ConfigspaceCoord cvel;
	/** Arm velocity in workspace coordinates. */
	WorkspaceVel wvel;
	/** Jacobian. */
	Jacobian jacobian;

	/** Default constructor initialises only GraphSearch::Node */
	GenWaypoint(U32 index = Node::IDX_UINI, Real cost = Node::COST_ZERO) :
		Waypoint(index, cost)
	{}

	/** Constructs GenWaypoint from node */
	GenWaypoint(const Node& node) :
		Waypoint(node)
	{}

	/** Constructs GenWaypoint from Waypoint */
	GenWaypoint(const Waypoint& waypoint) :
		Waypoint(waypoint)
	{}

	/** Copy constructor */
	GenWaypoint(const GenWaypoint& genWaypoint) :
		Waypoint(genWaypoint),
		cvel(genWaypoint.cvel),
		wvel(genWaypoint.wvel),
		jacobian(genWaypoint.jacobian)
	{}

	inline void setup(const Arm &arm, const ConfigspaceCoord &cpos, const ConfigspaceCoord &cvel, bool bEx = true) {
		this->cpos = cpos;
		this->cvel = cvel;
		setup(arm, bEx);
	}

	inline void setup(const Arm &arm, bool bEx = true) {
		Waypoint::setup(arm, bEx);

		//arm.jacobian(jacobian, cpos);
		arm.jacobianSpatial(jacobian, cpos);
		arm.jacobianFromSpatial(jacobian, jacobian, trn.back());
	}

	GenWaypoint &operator = (const GenWaypoint &genWaypoint) {
		Waypoint::operator = (genWaypoint);
		cvel = genWaypoint.cvel;
		wvel = genWaypoint.wvel;
		jacobian = genWaypoint.jacobian;
		return *this;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_WAYPOINT_H_*/
