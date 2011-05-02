/** @file PhysPlanner.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/PhysCtrl/PhysPlanner.h>
#include <Golem/PhysCtrl/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

WaypointGraphRenderer::WaypointGraphRenderer(PhysPlanner &physPlanner, const golem::Waypoint::Seq &graph) :
	physPlanner(physPlanner), graph(graph)
{}

bool WaypointGraphRenderer::create(const Desc &desc) {
	if (!desc.isValid())
		return false;

	const U32 size = (U32)graph.size();
	if (size <= 0)
		return true;

	// reserve memory buffer
	reset();
	if (desc.graphShow)
		reservePoints(size);
	if (desc.axesShow)
		reserveAxesInc(size);

	const Mat34 referencePose = physPlanner.getArm().getReferencePose();
	Mat34 trn;
	for (golem::Waypoint::Seq::const_iterator i = graph.begin(); i != graph.end(); i++) {
		trn.multiply(i->trn.back(), referencePose);	
		
		if (desc.graphShow)
			addPoint(trn.p, desc.graphColour);
		if (desc.axesShow)
			addAxes(trn, desc.axesSize);
	}

	return true;
}

//------------------------------------------------------------------------------

GoalRenderer::GoalRenderer(PhysPlanner &physPlanner, const golem::Waypoint::Seq &graph, const Mat34& pose) :
	WaypointGraphRenderer(physPlanner, graph), pose(pose)
{
}

bool GoalRenderer::create(const Desc &desc) {
	if (!WaypointGraphRenderer::create(desc))
		return false;

	// reset memory buffer
	//reset();

	addAxes(pose, desc.goalAxesSize);

	return true;
}

//------------------------------------------------------------------------------

PhysPlanner::PhysPlanner(Scene &scene) : PhysArm(scene) {
}

PhysPlanner::~PhysPlanner() {
}

//------------------------------------------------------------------------------

bool PhysPlanner::create(const Desc& desc) {
	PhysArm::create(desc); // throws
	
	autoSetCollisionBounds = desc.autoSetCollisionBounds;

	goalRendererDesc = desc.goalRendererDesc;
	goalShow = desc.goalShow;
	globalGraphRendererDesc = desc.globalGraphRendererDesc;
	globalGraphShow = desc.globalGraphShow;
	globalPathRendererDesc = desc.globalPathRendererDesc;
	globalPathShow = desc.globalPathShow;
	localGraphRendererDesc = desc.localGraphRendererDesc;
	localGraphShow = desc.localGraphShow;
	localPathRendererDesc = desc.localPathRendererDesc;
	localPathShow = desc.localPathShow;
	optimisedPathRendererExDesc = desc.optimisedPathRendererExDesc;
	optimisedPathExShow = desc.optimisedPathExShow;
	tmShowDuration = desc.tmShowDuration;
	
	pPlanner = dynamic_pointer_cast<GraphPlanner::Ptr>(desc.pPlannerDesc->create(*pArm));
	if (pPlanner == NULL)
		throw MsgPhysPlannerUnknownPlanner(Message::LEVEL_CRIT, "PhysPlanner::create(): Unknown planner");

	// install callback interface
	pPlanner->setCallback(this);

	return true;
}

void PhysPlanner::release() {
	pPlanner.release();
	PhysArm::release();
}

//------------------------------------------------------------------------------

void PhysPlanner::syncArmBoundsDesc() {
	if (autoSyncArmBoundsDesc)
		pPlanner->getHeuristic()->syncArmBoundsDesc();
}

void PhysPlanner::syncCollisionBounds() {
	if (autoSetCollisionBounds && pPlanner->getHeuristic()->hasCollisionDetection())
		pPlanner->getHeuristic()->setCollisionBounds(getCollisionBounds());
}

void PhysPlanner::syncFindData(Planner::Trajectory::const_iterator begin, Planner::Trajectory::const_iterator end, const GenWorkspaceState* wend) {
	const Mat34 referencePose = getArm().getReferencePose();
	Mat34 target;

	if (wend == NULL) {
		getArm().forwardTransform(target, (--end)->pos);
		target.multiply(target, referencePose); // reference pose
	}
	else {
		target = wend->pos;
	}
	
	// prepare renderable data
	PhysPlanner::Renderable renderable;

	// goal
	renderable.pGoalRenderer.reset(new GoalRenderer(*this, pPlanner->getGlobalPathFinder()->getKinematics()->getPosPopulation(), target));
	renderable.pGoalRenderer->create(goalRendererDesc);
	
	// global waypoint graph
	renderable.pGlobalGraphRenderer.reset(new WaypointGraphRenderer(*this, pPlanner->getGlobalPathFinder()->getGraph()));
	renderable.pGlobalGraphRenderer->create(globalGraphRendererDesc);
	// global waypoint path
	renderable.pGlobalPathRenderer.reset(new WaypointPathRenderer(*this, pPlanner->getGlobalPath()));
	renderable.pGlobalPathRenderer->create(globalPathRendererDesc);
	
	if (pPlanner->getLocalPathFinder() != NULL) {
		// local waypoint graph
		renderable.pLocalGraphRenderer.reset(new WaypointGraphRenderer(*this, pPlanner->getLocalPathFinder()->getGraph()));
		renderable.pLocalGraphRenderer->create(localGraphRendererDesc);
		// local waypoint path
		renderable.pLocalPathRenderer.reset(new WaypointPathRenderer(*this, pPlanner->getLocalPath()));
		renderable.pLocalPathRenderer->create(localPathRendererDesc);
	}
	
	// optimised waypoint path
	renderable.pOptimisedPathRendererEx.reset(new WaypointPathRendererEx(*this, pPlanner->getOptimisedPath()));
	renderable.pOptimisedPathRendererEx->create(optimisedPathRendererExDesc);

	renderable.tmEnd = (--end)->t + tmShowDuration;

	CriticalSectionWrapper csw(csPlanned);
	renderablesPlanned.insert(RenderablePair( - begin->t, renderable));
}

void PhysPlanner::syncSendData(Planner::Trajectory::const_iterator begin, Planner::Trajectory::const_iterator end) {
	PhysPlanner::RenderablePair pair;
	{
		CriticalSectionWrapper csw(csPlanned);
		PhysPlanner::RenderableMap::iterator i = renderablesPlanned.lower_bound( - begin->t);
		if (i == renderablesPlanned.end())
			return; // no renderables

		pair = *i;
		// erase the sent renderable and all renderebles which begin earlier the sent one
		renderablesPlanned.erase(i, renderablesPlanned.end());
	}
	{
		CriticalSectionWrapper csw(csSent);
		renderablesSent.insert(pair);
	}
}
	
//------------------------------------------------------------------------------

void PhysPlanner::keyboardHandler(unsigned char key, int x, int y) {
	PhysArm::keyboardHandler(key, x, y);
	
	switch (key) {
	case 3:// F3
		goalShow = !goalShow;
		break;
	case 4:// F4
		globalGraphShow = !globalGraphShow;
		break;
	case 5:// F5
		globalPathShow = !globalPathShow;
		break;
	case 6:// F6
		localGraphShow = !localGraphShow;
		break;
	case 7:// F7
		localPathShow = !localPathShow;
		break;
	case 8:// F8
		optimisedPathExShow = !optimisedPathExShow;
		break;
	}

#ifdef _COLLISION_DEBUG
	if (pBounds1 != NULL && pBounds2 != NULL) {
		Mat34 m1 = pBounds2->getPose();
		Mat34 m2;
		m2.setId();
		
		switch (key) {
		case 'a':
			bDebug2 = !bDebug2;
			break;
		case 'u':
			m2.R.rotX(0.05);
			break;
		case 'i':
			m2.R.rotX(-0.05);
			break;
		case 'o':
			m2.R.rotY(0.05);
			break;
		case 'p':
			m2.R.rotY(-0.05);
			break;
		case 'k':
			m2.R.rotZ(0.05);
			break;
		case 'l':
			m2.R.rotZ(-0.05);
			break;
		case '1':
			m2.p.v1 += 0.01;
			break;
		case '2':
			m2.p.v1 -= 0.01;
			break;
		case '3':
			m2.p.v2 += 0.01;
			break;
		case '4':
			m2.p.v2 -= 0.01;
			break;
		case '5':
			m2.p.v3 += 0.01;
			break;
		case '6':
			m2.p.v3 -= 0.01;
			break;
		}
		
		m1.multiply(m2, m1);
		pBounds2->setPose(m1);
		context.getMessageStream()->write(Message::LEVEL_DEBUG, "COLLISION: %s", pBounds1->intersect(*pBounds2) ? "YES" : "NO");
	}
#endif // _COLLISION_DEBUG
}

void PhysPlanner::render() {
	PhysArm::render();

#ifdef _COLLISION_DEBUG
	if (bDebug1) {
		if (pBounds1 != NULL)
			boundsRenderer.renderWire(*pBounds1);
		if (pBounds2 != NULL)
			boundsRenderer.renderWire(*pBounds2);
	}
#endif // _COLLISION_DEBUG

	if (!scene.getUniverse().suspended()) {
		CriticalSectionWrapper csw(csSent);
		
		const SecTmReal tmCurrent = context.getTimer().elapsed();
		
		RenderableMap::iterator i = renderablesSent.lower_bound( - tmCurrent);
		if (i == renderablesSent.end())
			return; // no renderables

		if (tmCurrent < i->second.tmEnd) {
			renderable = i->second;
			// erase all renderebles beginning earlier the sent one
			renderablesSent.erase(++i, renderablesSent.end());
		}
		else {
			// erase all renderebles beginning from the sent one on
			renderablesSent.erase(i, renderablesSent.end());
			return;
		}
	}

	if (goalShow && renderable.pGoalRenderer != NULL)
		renderable.pGoalRenderer->render();
	if (globalGraphShow && renderable.pGlobalGraphRenderer != NULL)
		renderable.pGlobalGraphRenderer->render();
	if (globalPathShow && renderable.pGlobalPathRenderer != NULL)
		renderable.pGlobalPathRenderer->render();
	if (localGraphShow && renderable.pLocalGraphRenderer != NULL)
		renderable.pLocalGraphRenderer->render();
	if (localPathShow && renderable.pLocalPathRenderer != NULL)
		renderable.pLocalPathRenderer->render();
	if (optimisedPathExShow && renderable.pOptimisedPathRendererEx != NULL)
		renderable.pOptimisedPathRendererEx->render();
}

//------------------------------------------------------------------------------
