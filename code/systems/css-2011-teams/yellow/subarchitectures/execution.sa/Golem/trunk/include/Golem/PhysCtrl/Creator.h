/** @file Creator.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_PHYSCTRL_CREATOR_H_
#define _GOLEM_PHYSCTRL_CREATOR_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Stream.h>
#include <Golem/Ctrl/Arm.h>
#include <Golem/Phys/Universe.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Creator
*/
class Creator {
public:
	/** Creation mode. */
	enum Mode {
		/** Create objects in the frame centre */
		MODE_CENTRE = 0,
		/** Create objects on the ground plane */
		MODE_GROUNDPLANE = 1,
	};
	
protected:
	golem::Scene &scene;
	Context &context;

	/** Memory buffer */
	MemoryWriteStream buffer;

public:
	/** Generator of pseudo random numbers */
	Rand rand;
	
	/** Creation mode. */
	Mode mode;
	/** Actor density */
	Real density;
	/** Solver iteration count */
	U32 solverIterationCount;
	/** Sleep energy threshold */
	Real sleepEnergyThreshold;
	
	/** Constructs the Creator object on a given Scene. */
	Creator(golem::Scene &scene);

	/** Virtual descructor */
	virtual ~Creator();

	/** Sets the objet parameters to the default values */
	virtual void setToDefault() {
		buffer.resetWrite();
		
		mode = MODE_GROUNDPLANE;
		density = Real(1.0);
		solverIterationCount = 255;//10;
		sleepEnergyThreshold = Real(0.0);
	}

	/** Checks if the object description is valid. */
	virtual bool isValid() const {
		if (density <= REAL_ZERO)
			return false;
		if (solverIterationCount > 255)
			return false;
		if (sleepEnergyThreshold < REAL_ZERO)
			return false;

		return true;
	}
	
	inline golem::Context& getContext() {
		return context;
	}
	
	inline const golem::Context& getContext() const {
		return context;
	}
	
	inline golem::Scene& getScene() {
		return scene;
	}
	
	inline const golem::Scene& getScene() const {
		return scene;
	}
	
	/** Actor description setup */
	golem::Actor::Desc *setupActorDesc();
	
	/** Generic bounds setup */
	void setupBoundsDesc(golem::Actor::Desc *pActorDesc, const Bounds::Desc &desc);

	/** Bounding plane setup */
	void setupBoundsDesc(golem::Actor::Desc *pActorDesc, const BoundingPlane::Desc &desc);

	/** Creates ground plane description */
	golem::Actor::Desc *createGroundPlaneDesc();
	
	/** Creates box description */
	golem::Actor::Desc *createBoxDesc(Real x = Real(1.0), Real y = Real(1.0), Real z = Real(1.0));
	
	/** Creates tree description */
	golem::Actor::Desc *createTreeDesc(Real radius = Real(1.0), Real thickness = Real(0.25));
	
	/** Creates box-based 2-flap description  */
	golem::Actor::Desc *createSimple2FlapDesc(Real width = Real(1.0), Real height = Real(1.0), Real length = Real(1.0), Real thickness = Real(0.002), Real angle = REAL_PI_2);

	/** Creates box-based 2-flap description  */
	golem::Actor::Desc *create2FlapDesc(Real width1 = Real(1.0), Real width2 = Real(1.0), Real shift1 = Real(0.0), Real shift2 = Real(0.0), Real height = Real(1.0), Real length = Real(1.0), Real thickness = Real(0.002), Real angle = REAL_PI_2, bool moveFrame = false);

	/** Generate minimum torque straight line trajectory between wc0 and wc0  */
	golem::shared_ptr<golem::GenWorkspaceState::Seq> createP2PTrajectory(const golem::WorkspaceCoord &begin, const golem::WorkspaceCoord &end, const golem::Profile &trajectory, SecTmReal duration, SecTmReal delta);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYSCTRL_CREATOR_H_*/
