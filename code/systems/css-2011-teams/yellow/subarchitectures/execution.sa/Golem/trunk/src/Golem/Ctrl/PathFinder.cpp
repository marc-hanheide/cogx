/** @file PathFinder.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Ctrl/PathFinder.h>
#include <Golem/Ctrl/Planner.h>
#include <Golem/Ctrl/Msg.h>
#include <algorithm>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Node::cost_less PathFinder::WaypointPtr::less;

PathFinder::PathFinder(golem::Planner &planner) :
	planner(planner),
	arm(planner.getArm()),
	context(arm.getContext()),
	rand(context.getRandSeed())
{
}

bool PathFinder::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgPathFinderInvalidDesc(Message::LEVEL_CRIT, "PathFinder::create(): Invalid description");

	pKinematics = dynamic_pointer_cast<DEKinematics::Ptr>(desc.pKinematicsDesc->create(*planner.getHeuristic()));
	if (pKinematics == NULL)
		throw MsgPathFinderUnknownKinematics(Message::LEVEL_CRIT, "PathFinder::create(): Unknown kinematics solver");

	graphSearchDesc = desc.graphSearchDesc;

	return true;
}

//------------------------------------------------------------------------------

Real PathFinder::cost(const Waypoint &w) const {
	const Real d0 = planner.getHeuristic()->getRestJointspaceDist(w.cpos);	
	const Real d1 = planner.getHeuristic()->getWorkspaceDist(w, graph[Node::IDX_GOAL]);
	
	return graphSearchDesc.distGoalFac*d1 + (REAL_ONE - graphSearchDesc.distGoalFac)*d0;
}

bool PathFinder::collides(const Waypoint &w) const {
	return planner.getHeuristic()->collides(w);
}

Real PathFinder::cost(U32 i, U32 j) const {
	const Heuristic &heuristic = *planner.getHeuristic();
	const Waypoint &w0 = graph[i];
	const Waypoint &w1 = graph[j];

	const Real d0 = heuristic.getBoundedDist(w0, w1);
	if (d0 >= Node::COST_INF)
		return Node::COST_INF;
	const Real d1 = heuristic.getRestJointspaceDist(w1.cpos);
	const Real d2 = heuristic.getDist(w1, graph[Node::IDX_GOAL]);

	return graphSearchDesc.distGoalFac*d2 + (REAL_ONE - graphSearchDesc.distGoalFac)*
		(graphSearchDesc.distPathFac*d0 + (REAL_ONE - graphSearchDesc.distPathFac)*d1);
}

bool PathFinder::collides(U32 i, U32 j) const {
#ifdef _COLLISION_DEBUG
	bDebug1 = true;
#endif // _COLLISION_DEBUG
	return planner.getHeuristic()->collides(graph[i], graph[j]);
}

//------------------------------------------------------------------------------

bool PathFinder::generateWaypoint(Waypoint& w, const ConfigspaceCoord &min, const ConfigspaceCoord &max, U32 trials) const {
	// number of joints
	const U32 numOfJoints = (U32)arm.getJoints().size();
	
	for (;;) {
		for (U32 i = 0; i < numOfJoints; i++)
			w.cpos[i] = rand.nextUniform(min[i], max[i]);
		w.setup(arm);
		w.cost = cost(w);

		if (w.cost < Node::COST_INF && !collides(w))
			break;
		if (trials-- > 0)
			continue;
		
		return false;
	}

	return true;
}

bool PathFinder::generateGraph(Waypoint::Seq& graph, const ConfigspaceCoord::Seq &generators, const Waypoint &target) const {
	const U32 numOfGenerators = (U32)generators.size();
	if (numOfGenerators <= 0) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "PathFinder::generateGraph(): no specified reference generators");
		return false;
	}
	
	// first waypoint index and init size
	ASSERT(Node::IDX_GOAL == 0)
	ASSERT(Node::IDX_ROOT == 1)
	const U32 initIndex = Node::IDX_ROOT + numOfGenerators;
	//const U32 oldSize = std::max(initIndex, size());
	const U32 newSize = std::max(initIndex, graphSearchDesc.size);

	// resize graph
	graph.resize(newSize);

	// setup GOAL graph waypoint
	graph[Node::IDX_GOAL] = target;

	// setup graph generators
	for (U32 i = 0; i < generators.size(); i++) {
		Waypoint &w = graph[Node::IDX_ROOT + i];
		w.cpos = generators[i];
		w.setup(arm);
		w.cost = cost(w);

		// do not check collisions at the root/init pose since the arm can be in contact with objects
		if (w.cost >= Node::COST_INF || i != 0 && collides(w)) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "PathFinder::generateGraph(): reference waypoint #%d is not attainable", i	);
			return false;
		}
	}

	// number of joints
	const U32 numOfJoints = (U32)arm.getJoints().size();
	
	// joint position limits for each generator
	ConfigspaceCoord::Seq min(numOfGenerators), max(numOfGenerators);
	for (U32 i = 0; i < numOfGenerators; i++)
		for (U32 j = 0; j < numOfJoints; j++) {
			const Joint &joint = *arm.getJoints()[j];
			min[i][j] = std::max(joint.getMin().pos, generators[i][j] - graphSearchDesc.range[j]);
			max[i][j] = std::min(joint.getMax().pos, generators[i][j] + graphSearchDesc.range[j]);
		}

	for (U32 i = initIndex; i < newSize; i++) {
		const U32 generator = rand.next()%numOfGenerators;
		
		if (!generateWaypoint(graph[i], min[generator], max[generator], graphSearchDesc.waypointInitTrials)) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "PathFinder::generateGraph(): unable to generate node #%d using generator #%d", i, generator);
			return false;
		}
	}
	
	return true;
}

//------------------------------------------------------------------------------

bool PathFinder::findTarget(Waypoint::Seq& graph, const ConfigspaceCoord::Seq &generators) const {
	///** Waypoint cost comparator */
	//static Node::cost_less less;
	
	///** Waypoint pointer wrapper */
	//class WaypointPtr {
	//private:
	//	Waypoint *pWaypoint;
	//	//static Node::cost_less less;// local classes cannot have static members

	//public:
	//	typedef std::vector<WaypointPtr> Seq;
	//	
	//	Waypoint &operator * () const {
	//		return *pWaypoint;
	//	}
	//	Waypoint *operator -> () const {
	//		return pWaypoint;
	//	}
	//	bool operator < (const WaypointPtr &right) {
	//		return less.operator () (*pWaypoint, *right.pWaypoint);
	//	}
	//	WaypointPtr(Waypoint *pWaypoint) : pWaypoint(pWaypoint) {
	//	}
	//};
	
	WaypointPtr::Seq waypointPtrGraph;
	waypointPtrGraph.reserve(graph.size());
	for (U32 i = 0; i < graph.size(); i++)
		waypointPtrGraph.push_back(WaypointPtr(&graph[i]));
	
	// sort waypoints (pointers) from the lowest to the highest cost
	std::sort(waypointPtrGraph.begin(), waypointPtrGraph.end());
	
	// Create waypoint population
	const U32 populationSize = std::min(pKinematics->getPopulationSize(), (U32)waypointPtrGraph.size());
	Waypoint::Seq population;
	population.reserve(populationSize);
	for (U32 i = 0; i < populationSize; i++)
		population.push_back(*waypointPtrGraph[i]);

	pKinematics->setPosPopulation(population);

	// number of joints
	const U32 numOfJoints = U32(arm.getJoints().size());
	
	// joint position limits are determined by generators
	GenConfigspaceCoord min = pKinematics->getMin();
	GenConfigspaceCoord max = pKinematics->getMax();
	
	for (U32 j = 0; j < numOfJoints; j++) {
		Real &jmin = min.pos[j];
		Real &jmax = max.pos[j];
		jmin = numeric_const<Real>::MAX;
		jmax = numeric_const<Real>::MIN;

		for (ConfigspaceCoord::Seq::const_iterator i = generators.begin(); i != generators.end(); i++) {
			if (jmin > (*i)[j])
				jmin = (*i)[j];
			if (jmax < (*i)[j])
				jmax = (*i)[j];
		}
		
		jmin = jmin - graphSearchDesc.range[j];
		jmax = jmax + graphSearchDesc.range[j];
	}

	pKinematics->setMin(min);
	pKinematics->setMax(max);

	if (!pKinematics->find(graph[Node::IDX_GOAL].cpos, graph[Node::IDX_GOAL].wpos))
		return false;

	return true;
}

bool PathFinder::findPath(Waypoint::Seq &path, Waypoint::Seq::iterator iter) {
	// initialise graph search data
	(void)GraphSearch::resize((U32)graph.size());

	typedef std::vector<Node> NodePath;
	NodePath nodePath;
	
	(void)GraphSearch::find(nodePath, nodePath.end());
	if (nodePath.size() < 2) {
		// path not found
		context.getMessageStream()->write(Message::LEVEL_ERROR, "PathFinder::findPath(): cannot find path"	);
		return false;
	}

	for (NodePath::const_iterator i = nodePath.begin(); i != nodePath.end(); i++)
		iter = ++path.insert(iter, graph[i->index]);

	return true;
}

//------------------------------------------------------------------------------

bool PathFinder::generateGraph(const ConfigspaceCoord &begin, const ConfigspaceCoord &end) {
	WorkspaceCoord wend;
	arm.forwardTransform(wend, end);
	return PathFinder::generateGraph(begin, wend);
}

bool PathFinder::generateGraph(const ConfigspaceCoord &begin, const WorkspaceCoord &end) {
	// setup waypoints generators
	generators.clear();
	generators.reserve((U32)graphSearchDesc.generators.size() + 1);
	// Root/begin state is always at the front of the generators sequence
	generators.insert(generators.begin(), begin);
	generators.insert(generators.end(), graphSearchDesc.generators.begin(), graphSearchDesc.generators.end());

	// setup target node
	Waypoint target;
	target.cost = Node::COST_INF; // required by both: PathFinder::findTarget() and PathFinder::findPath()
	target.cpos = begin; // should not be random
	target.wpos = end; // required by PathFinder::cost(const Waypoint&) in PathFinder::generateWaypoint()
	target.wposq.fromMat33(end.R); // required by PathFinder::cost(const Waypoint&)

#ifdef _PATHFINDER_PERFMON
	PerfTimer t;
#endif
	if (!generateGraph(graph, generators, target))
		return false;
#ifdef _PATHFINDER_PERFMON
	context.getMessageStream()->write(Message::LEVEL_DEBUG,
		"PathFinder::generateGraph(): time elapsed = %f [sec]", t.elapsed()
	);
#endif

	return true;
}

bool PathFinder::findTarget(ConfigspaceCoord &end) {
	if (graph.empty()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "PathFinder::findTarget(): empty graph!");
		return false;
	}

#ifdef _PATHFINDER_PERFMON
	PerfTimer t;
#endif
	// find the target state
	if (!findTarget(graph, generators))
		return false;
#ifdef _PATHFINDER_PERFMON
	context.getMessageStream()->write(Message::LEVEL_DEBUG,
		"PathFinder::findTarget(): time elapsed = %f [sec]",
		t.elapsed()
	);
#endif
	end = graph[Node::IDX_GOAL].cpos;

	return true;
}

bool PathFinder::findPath(Waypoint::Seq &path, Waypoint::Seq::iterator iter, const ConfigspaceCoord &begin, const ConfigspaceCoord &end) {
	if (graph.empty()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "PathFinder::findPath(): empty graph!"	);
		return false;
	}

	// setup target node
	graph[Node::IDX_GOAL].setup(arm, end);
	
#ifdef _PATHFINDER_PERFMON
	PerfTimer t;
#endif
	// find node path
	if (!findPath(path, iter))
		return false;
#ifdef _PATHFINDER_PERFMON
	context.getMessageStream()->write(Message::LEVEL_DEBUG,
		"PathFinder::findPath(): time elapsed = %f [sec], len = %d",
		t.elapsed(), path.size()
	);
#endif

	return true;
}

//------------------------------------------------------------------------------
