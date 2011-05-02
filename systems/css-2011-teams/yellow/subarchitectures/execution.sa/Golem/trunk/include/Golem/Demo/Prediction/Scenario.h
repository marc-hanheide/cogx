/** @file Scenario.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_SCENARIO_H_
#define _GOLEM_DEMO_SCENARIO_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Arm.h>
#include <Golem/PhysCtrl/Creator.h>
#include <Golem/Demo/Common/PointTracker.h>
#include <Golem/Demo/Common/FTSensor.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class Generator {
public:
	/** Generator array */
	typedef std::vector<Generator> Seq;
	
	/** Generator anchor */
	Vec3 anchor;
	/** Generator axis */
	Vec3 axis;
	/** Generator direction */
	Vec3 dir;

	/** Direction linear transformation variance */
	Real dirLinVar;
	/** Direction angular transformation variance */
	Real dirAngVar;

	/** Generator weight */
	Real weight;
	/** Generator cdf */
	Real cdf;

	/** Sample CDF comparator */
	bool operator < (const Generator &right) const {
		return cdf < right.cdf;
	}
};

//------------------------------------------------------------------------------

/** Scenario
*/
class Scenario {
public:
	/** Scenario mode */
	enum Mode {
		/** Offline mode */
		MODE_OFFLINE = 0x1,
		/** Filterning */
		MODE_FILTERING = 0x2,
		/** Simulation */
		MODE_SIMULATION = 0x4,
	};

	/** Name */
	std::string name;

	/** Action */
	golem::shared_ptr<golem::GenWorkspaceState::Seq> pAction;
	/** Polyflap */
	golem::Actor::Desc *pPolyflap;
	/** Generator array */
	Generator::Seq generators;
	
	/** restitution coefficient */
	Real restitution;
	/** static friction coefficient */
	Real staticFriction;
	/** dynamic friction coefficient */
	Real dynamicFriction;
	
	/** Scenario mode */
	U32 mode;
	/** Running time */
	SecTmReal tmRunTime;
	/** Real time processing */
	bool realTime;
	/** Manual start */
	bool manualStart;
	/** Time offsets */
	SecTmReal tmOffsetBegin, tmOffsetEnd;

	/** Constructs description object. */
	Scenario() {
		Scenario::setToDefault();
	}

	virtual ~Scenario() {}
	
	/** Sets the parameters to the default values */
	virtual void setToDefault() {
		name = "Default";
		
		pAction.reset(); // reset action
		pPolyflap = NULL; // reset polyflap -> real scenario
		generators.clear();
		
		restitution = (Real)0.1;
		staticFriction = (Real)0.3;
		dynamicFriction = (Real)0.03;
		
		mode = MODE_OFFLINE | MODE_FILTERING | MODE_SIMULATION;
		tmRunTime = SecTmReal(20.0);
		realTime = false;
		manualStart = false;
		tmOffsetBegin = SecTmReal(1.0);
		tmOffsetEnd = SecTmReal(2.0);
	}

	/** Checks if the description is valid. */
	virtual bool isValid() const {
		if (pAction == NULL || pAction->size() <= 0)
			return false;
		//if (pPolyflap == NULL)
		//	return false;
		if (generators.empty())
			return false;
		if (restitution <= REAL_ZERO || staticFriction <= REAL_ZERO || dynamicFriction <= REAL_ZERO)
			return false;
		
		if (!(mode & (MODE_FILTERING | MODE_SIMULATION)))
			return false;
		if (tmRunTime < SEC_TM_REAL_ZERO || tmOffsetBegin < SEC_TM_REAL_ZERO || tmOffsetEnd < SEC_TM_REAL_ZERO)
			return false;
		
		return true;
	}
};

//------------------------------------------------------------------------------

class Performer {
public:
	/** To satisfy compiler */
	virtual ~Performer() {}
	
	/** Runs the specified Scenario */
	virtual void run(const Scenario &scenario) = 0;
};

class Experiment : public Creator {
public:
	/** Scenarios */
	std::vector<Scenario> scenarios;
	/** Number of runs of the scenario sequence */
	U32 numOfRuns;
	/** Random runs of scenarios */
	bool randomRuns;
	
	/** Constructs the Experiment object on a given Scene. */
	Experiment(golem::Scene &scene);

	/** Sets the parameters to the default values */
	virtual void setToDefault() {
		Creator::setToDefault();
		
		scenarios.clear();
		numOfRuns = 1;
		randomRuns = false;
	}

	/** Checks if the description is valid. */
	virtual bool isValid() const {
		if (scenarios.empty())
			return false;
		if (numOfRuns <= 0)
			return false;
		
		return true;
	}

	/** Run experiment */
	void run(Performer &performer);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_SCENARIO_H_*/
