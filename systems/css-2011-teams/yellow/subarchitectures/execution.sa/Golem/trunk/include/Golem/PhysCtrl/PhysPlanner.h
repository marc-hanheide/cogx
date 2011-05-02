/** @file PhysPlanner.h
 * 
 * 
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.csPlanned.bham.ac.uk">The University Of Birmingham</A>
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_PHYSCTRL_PHYSPLANNER_H_
#define _GOLEM_PHYSCTRL_PHYSPLANNER_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/GraphPlanner.h>
#include <Golem/Phys/Renderer.h>
#include <Golem/PhysCtrl/PhysArm.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class PhysPlanner;

//------------------------------------------------------------------------------

/** Renders waypoint graph */
class WaypointGraphRenderer : public DebugRenderer {
public:
	typedef shared_ptr<WaypointGraphRenderer> Ptr;

	PhysPlanner &physPlanner;
	const Waypoint::Seq &graph;

	/** GraphRenderer description */
	class Desc {
	public:
		/** Graph colour */
		RGBA graphColour;
		/** Nodes pose axes size */
		Vec3 axesSize;
		/** Display graph */
		bool graphShow;
		/** Display nodes pose axes */
		bool axesShow;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			graphColour = RGBA::WHITE;
			axesSize.set(Real(0.025));
			graphShow = true;
			axesShow = false;
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!axesSize.isFinite())
				return false;
			
			return true;
		}
	};

	WaypointGraphRenderer(PhysPlanner &physPlanner, const Waypoint::Seq &graph);
	bool create(const Desc &desc);
};

//------------------------------------------------------------------------------

/** Renders goal finder data */
class GoalRenderer : public WaypointGraphRenderer {
public:
	typedef shared_ptr<GoalRenderer> Ptr;

	//PhysPlanner &physPlanner;
	Mat34 pose;

	/** GoalRenderer description */
	class Desc : public WaypointGraphRenderer::Desc {
	public:
		/** Current and destination pose axes size */
		Vec3 goalAxesSize;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			WaypointGraphRenderer::Desc::setToDefault();
			goalAxesSize.set(Real(0.1));
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!WaypointGraphRenderer::Desc::isValid())
				return false;
			if (!goalAxesSize.isFinite())
				return false;
			
			return true;
		}
	};

	GoalRenderer(PhysPlanner &physPlanner, const Waypoint::Seq &graph, const Mat34& pose);
	bool create(const Desc &desc);
};

//------------------------------------------------------------------------------

/** Renders waypoint sequence */
template <typename WaypointSeq>
class WaypointSeqRenderer : public DebugRenderer {
public:
	typedef shared_ptr<WaypointSeqRenderer> Ptr;

	PhysPlanner &physPlanner;
	const WaypointSeq &path;
	typedef typename WaypointSeq::reference Waypoint;

	/** PathRenderer description */
	class Desc {
	public:
		/** Path colour */
		RGBA pathColour;
		/** Nodes pose axes size */
		Vec3 axesSize;
		/** Display path */
		bool pathShow;
		/** Display nodes pose axes */
		bool axesShow;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			pathColour = RGBA::WHITE;
			axesSize.set(Real(0.025));
			pathShow = true;
			axesShow = false;
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!axesSize.isFinite())
				return false;
			
			return true;
		}
	};

	WaypointSeqRenderer(PhysPlanner &physPlanner, const WaypointSeq &path);
	
	bool create(const Desc &desc);
};

typedef WaypointSeqRenderer<Waypoint::Seq> WaypointPathRenderer;
typedef WaypointSeqRenderer<OptWaypoint::Seq> WaypointPathRendererEx;

//------------------------------------------------------------------------------

class PhysPlanner : public PhysArm, public Planner::Callback {
public:
	typedef shared_ptr<PhysPlanner> Ptr;
	
	/** Object description */
	class Desc : public PhysArm::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(PhysPlanner, Object::Ptr, Scene&)

	public:
		/** Planner description */
		GraphPlanner::Desc::Ptr pPlannerDesc;
		
		/** Auto set collision bounds */
		bool autoSetCollisionBounds;
		
		/** Goal renderer */
		GoalRenderer::Desc goalRendererDesc;
		/** Goal renderer state */
		bool goalShow;

		/** Global waypoint graph renderer */
		WaypointGraphRenderer::Desc globalGraphRendererDesc;
		/** Global waypoint graph renderer state */
		bool globalGraphShow;
		/** Global waypoint path renderer */
		WaypointPathRenderer::Desc globalPathRendererDesc;
		/** Global waypoint path renderer state */
		bool globalPathShow;

		/** Local waypoint graph renderer */
		WaypointGraphRenderer::Desc localGraphRendererDesc;
		/** Local waypoint graph renderer state */
		bool localGraphShow;
		/** Local waypoint path renderer */
		WaypointPathRenderer::Desc localPathRendererDesc;
		/** Local waypoint path renderer state */
		bool localPathShow;
		
		/** Optimised waypoint path renderer */
		WaypointPathRendererEx::Desc optimisedPathRendererExDesc;
		/** Optimised waypoint path renderer state */
		bool optimisedPathExShow;
		
		/** Maximum show time */
		SecTmReal tmShowDuration;
		
		/** Constructs PhysArm description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			PhysArm::Desc::setToDefault();
			autoSetCollisionBounds = true;

			pPlannerDesc.reset(new GraphPlanner::Desc());

			goalRendererDesc.setToDefault();
			goalShow = false;

			globalGraphRendererDesc.setToDefault();
			globalGraphRendererDesc.graphColour = RGBA::YELLOW;
			globalGraphShow = false;
			
			globalPathRendererDesc.setToDefault();
			globalPathRendererDesc.pathColour = RGBA::CYAN;
			globalPathShow = false;

			localGraphRendererDesc.setToDefault();
			localGraphRendererDesc.graphColour = RGBA::YELLOW;
			localGraphShow = false;
			
			localPathRendererDesc.setToDefault();
			localPathRendererDesc.pathColour = RGBA::MAGENTA;
			localPathShow = false;

			optimisedPathRendererExDesc.setToDefault();
			optimisedPathRendererExDesc.pathColour = RGBA::BLACK;
			optimisedPathRendererExDesc.axesShow = true;
			optimisedPathExShow = true;

			tmShowDuration = SecTmReal(60.0); // 60 sec
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!PhysArm::Desc::isValid())
				return false;

			if (pPlannerDesc == NULL || !pPlannerDesc->isValid())
				return false;
			if (!goalRendererDesc.isValid())
				return false;
			if (!globalGraphRendererDesc.isValid() || !globalPathRendererDesc.isValid())
				return false;
			if (!localGraphRendererDesc.isValid() || !localPathRendererDesc.isValid())
				return false;
			if (!optimisedPathRendererExDesc.isValid())
				return false;
			if (tmShowDuration < SEC_TM_REAL_ZERO)
				return false;

			return true;
		}
	};

protected:
	struct Renderable {
		SecTmReal tmEnd;
		GoalRenderer::Ptr pGoalRenderer;
		WaypointGraphRenderer::Ptr pGlobalGraphRenderer;
		WaypointPathRenderer::Ptr pGlobalPathRenderer;
		WaypointGraphRenderer::Ptr pLocalGraphRenderer;
		WaypointPathRenderer::Ptr pLocalPathRenderer;
		WaypointPathRendererEx::Ptr pOptimisedPathRendererEx;
	};
	
	typedef std::map<SecTmReal, Renderable> RenderableMap;
	typedef std::pair<SecTmReal, Renderable> RenderablePair;
	
	/** Planner interface */
	GraphPlanner::Ptr pPlanner;

	/** Auto set collision bounds */
	bool autoSetCollisionBounds;
	
	/** Goal renderer */
	GoalRenderer::Desc goalRendererDesc;
	bool goalShow;
	
	/** Global waypoint graph renderer */
	WaypointGraphRenderer::Desc globalGraphRendererDesc;
	bool globalGraphShow;
	/** Global waypoint path renderer */
	WaypointPathRenderer::Desc globalPathRendererDesc;
	bool globalPathShow;
	
	/** Local waypoint graph renderer */
	WaypointGraphRenderer::Desc localGraphRendererDesc;
	bool localGraphShow;
	/** Local waypoint path renderer */
	WaypointPathRenderer::Desc localPathRendererDesc;
	bool localPathShow;
	
	/** Optimised waypoint path renderer */
	WaypointPathRendererEx::Desc optimisedPathRendererExDesc;
	bool optimisedPathExShow;
	
	/** Maximum show time */
	SecTmReal tmShowDuration;

	RenderableMap renderablesPlanned, renderablesSent;
	PhysPlanner::Renderable renderable;
	CriticalSection csPlanned, csSent;

	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);
	
	/** Renders the PhysPlanner. */
	virtual void render();

	/** Synchronise bounds of all joints with the bounds descriptions */
	virtual void syncArmBoundsDesc();
	
	/** Sets the collection of collision bounds */
	virtual void syncCollisionBounds();

	/** synchronization of find data */
	virtual void syncFindData(Planner::Trajectory::const_iterator begin, Planner::Trajectory::const_iterator end, const GenWorkspaceState* wend = NULL);

	/** synchronization of send data */
	virtual void syncSendData(Planner::Trajectory::const_iterator begin, Planner::Trajectory::const_iterator end);
	
	/** Creates PhysPlanner. */
	bool create(const Desc& desc);
	
	/** Releases resources */
	virtual void release();

	/** Objects can be constructed only in the Scene context. */
	PhysPlanner(Scene &scene);

public:
	/** Destructor is inaccesible */
	virtual ~PhysPlanner();

	/** Returns the Planner */
	inline const GraphPlanner &getPlanner() const {
		return *pPlanner;
	}
	inline GraphPlanner &getPlanner() {
		return *pPlanner;
	}
};

//------------------------------------------------------------------------------

template <typename WaypointSeq>
WaypointSeqRenderer<WaypointSeq>::WaypointSeqRenderer(PhysPlanner &physPlanner, const WaypointSeq &path) :
	physPlanner(physPlanner), path(path)
{}

template <typename WaypointSeq>
bool WaypointSeqRenderer<WaypointSeq>::create(const Desc &desc) {
	if (!desc.isValid())
		return false;

	const U32 size = (U32)path.size();
	if (size <= 1)
		return true;
	
	// reserve memory buffer
	reset();
	if (desc.pathShow)
		reserveLines(size - 1);
	if (desc.axesShow)
		reserveAxesInc(size);

	const Mat34 referencePose = physPlanner.getArm().getReferencePose();
	Mat34 trn0, trn1;
	
	typename WaypointSeq::const_iterator i = path.begin();
	trn0.multiply(i->trn.back(), referencePose);	
	
	if (desc.axesShow)
		addAxes(trn0, desc.axesSize);
	
	while (++i != path.end()) {
		trn1.multiply(i->trn.back(), referencePose);	
		
		if (desc.pathShow)
			addLine(trn0.p, trn1.p, desc.pathColour);
		if (desc.axesShow)
			addAxes(trn1, desc.axesSize);
		
		trn0 = trn1;
	}

	return true;
}

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYSCTRL_PHYSPLANNER_H_*/
