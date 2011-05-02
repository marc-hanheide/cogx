/** @file PhysReacPlanner.h
 * 
 * 
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.csPlanned.bham.ac.uk">The University Of Birmingham</A>
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_PHYSCTRL_PHYSREACPLANNER_H_
#define _GOLEM_PHYSCTRL_PHYSREACPLANNER_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/ReacPlanner.h>
#include <Golem/Phys/Renderer.h>
#include <Golem/PhysCtrl/PhysPlanner.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class PhysReacPlanner;

//------------------------------------------------------------------------------

/** Reactive planner renderer */
class ReacPlannerRenderer : public DebugRenderer {
public:
	typedef shared_ptr<ReacPlannerRenderer> Ptr;

	PhysReacPlanner &physReacPlanner;

	/** Renderer description */
	class Desc {
	public:
		/** Desired pose axes size */
		Vec3 desiredPoseSize;
		/** Desired linear and angular velocity delta time */
		SecTmReal desiredVelDelta;
		/** Desired state show */
		bool desiredStateShow;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			desiredPoseSize.set(Real(0.1));
			desiredVelDelta = Real(0.2);
			desiredStateShow = true;
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!desiredPoseSize.isFinite())
				return false;
			
			return true;
		}
	};

	ReacPlannerRenderer(PhysReacPlanner &physReacPlanner);
	bool create(const Desc &desc);
};

//------------------------------------------------------------------------------

class PhysReacPlanner : public PhysPlanner {
public:
	typedef shared_ptr<PhysReacPlanner> Ptr;
	
	/** Object description */
	class Desc : public golem::PhysPlanner::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(PhysReacPlanner, Object::Ptr, Scene&)

	public:
		/** ReacPlanner description */
		ReacPlanner::Desc::Ptr pReacPlannerDesc;
				
		/** Reactive planner renderer */
		ReacPlannerRenderer::Desc reacPlannerRendererDesc;
		/** Reactive planner renderer state */
		bool reacPlannerShow;
		
		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			PhysPlanner::Desc::setToDefault();

			pReacPlannerDesc.reset(new ReacPlanner::Desc());
			reacPlannerRendererDesc.setToDefault();
			reacPlannerShow = true;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!PhysPlanner::Desc::isValid())
				return false;

			if (pReacPlannerDesc == NULL || !pReacPlannerDesc->isValid())
				return false;
			if (!reacPlannerRendererDesc.isValid())
				return false;

			return true;
		}
	};

protected:
	/** ReacPlanner interface */
	ReacPlanner::Ptr pReacPlanner;
	
	/** Reactive planner renderer */
	ReacPlannerRenderer::Desc reacPlannerRendererDesc;
	ReacPlannerRenderer::Ptr pReacPlannerRenderer;
	bool reacPlannerShow;
	
	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);
	
	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(SecTmReal elapsedTime);
	
	/** Renders the PhysReacPlanner. */
	virtual void render();

	/** Creates the PhysReacPlanner from the PhysReacPlanner description. */
	bool create(const Desc& desc);

	/** Releases resources */
	virtual void release();

	/** Objects can be constructed only in the Scene context. */
	PhysReacPlanner(Scene &scene);

public:
	/** Destructor is inaccesible */
	virtual ~PhysReacPlanner();

	/** Returns the ReacPlanner */
	inline const ReacPlanner &getReacPlanner() const {
		return *pReacPlanner;
	}
	inline ReacPlanner &getReacPlanner() {
		return *pReacPlanner;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYSCTRL_PHYSREACPLANNER_H_*/
