/** @file GraphPlanner.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Ctrl/GraphPlanner.h>
#include <Golem/Ctrl/Msg.h>
#include <algorithm>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GraphPlanner::GraphPlanner(golem::Arm &arm) :
	Planner(arm)
{
}

bool GraphPlanner::create(const Desc& desc) {
	Planner::create(desc); // throws

	pathFinderDesc = desc.pathFinderDesc;

	pGlobalPathFinder = pathFinderDesc.pGlobalPathFinderDesc->create(*this); // throws
	if (pathFinderDesc.pLocalPathFinderDesc != NULL) {
		pLocalPathFinder = pathFinderDesc.pLocalPathFinderDesc->create(*this); // throws
	}
	
	pathOptimiserDesc = desc.pathOptimiserDesc;
	localFinderDesc = desc.localFinderDesc;

	pKinematics = dynamic_pointer_cast<DEKinematics::Ptr>(localFinderDesc.pKinematicsDesc->create(*pHeuristic));
	if (pKinematics == NULL)
		throw MsgGraphPlannerUnknownKinematics(Message::LEVEL_CRIT, "GraphPlanner::create(): Unknown kinematics solver");

	return true;
}

//------------------------------------------------------------------------------

Real GraphPlanner::cost(U32 n, const OptWaypoint::Seq &seq) const {
	ASSERT(seq.size() > 1)

	Real d0 = REAL_ZERO;
	if (n > 0) {
		d0 = pHeuristic->getBoundedDist(seq[n - 1], seq[n]);
		if (d0 >= Node::COST_INF)
			return Node::COST_INF;
	}
	
	Real d1 = REAL_ZERO;
	if (n < seq.size() - 1) {
		d1 = pHeuristic->getBoundedDist(seq[n], seq[n + 1]);
		if (d1 >= Node::COST_INF)
			return Node::COST_INF;
	}
	
	Real d2 = pHeuristic->getRestJointspaceDist(seq[n].cpos);

	return pathOptimiserDesc.distPathFac*(d0 + d1) + (REAL_ONE - pathOptimiserDesc.distPathFac)*d2;
}

bool GraphPlanner::collides(U32 n, const OptWaypoint::Seq &seq) const {
	ASSERT(seq.size() > 1)

	if (n > 0 && pHeuristic->collides(seq[n - 1], seq[n]))
		return true;
	if (n < seq.size() - 1 && pHeuristic->collides(seq[n], seq[n + 1]))
		return true;

	return false;
}

//------------------------------------------------------------------------------

bool GraphPlanner::purge(OptWaypoint::Seq &seq) const {
	if (seq.size() < 2) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "GraphPlanner::purge(): no waypoints to purge"	);
		return false;
	}

	Real distMax = REAL_ZERO;
	for (U32 k = 1; k < U32(seq.size()); k++)
		distMax = std::max(distMax, pHeuristic->getBoundedDist(seq[k - 1], seq[k]));
	
	if (distMax >= Node::COST_INF) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "GraphPlanner::purge(): infinite path distance found");
		return false;
	}

	Real d0, d1, d2;
	for (U32 k = 1; k < U32(seq.size()) - 1;) {
		d0 = pHeuristic->getBoundedDist(seq[k - 1], seq[k]);
		d1 = pHeuristic->getBoundedDist(seq[k], seq[k + 1]);
		if (d0 > pathOptimiserDesc.distPathThr*distMax && d1 > pathOptimiserDesc.distPathThr*distMax)
			goto NEXT;
		
		d2 = pHeuristic->getBoundedDist(seq[k - 1], seq[k + 1]);
		if (d2 >= Node::COST_INF)
			goto NEXT;

		if (pHeuristic->collides(seq[k - 1], seq[k + 1]))
			goto NEXT;

		// remove the waypoint k
		seq.erase(seq.begin() + k);
		//context.getMessageStream()->write(Message::LEVEL_DEBUG, "GraphPlanner::purge(): waypoint %d removed", k);
		continue;

		NEXT:
		k++;
	}

	// compute total cost
	for (U32 k = 0; k < U32(seq.size()); k++) {
		const Real c = cost(k, seq);
		if (c >= Node::COST_INF) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "GraphPlanner::purge(): infinite path cost at waypoint #%d", k	);
			return false;
		}
		if (collides(k, seq)) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "GraphPlanner::purge(): collision detected at waypoint #%d", k	);
			return false;
		}
		
		seq[k].cost = c;
	}

	return true;
}

bool GraphPlanner::trimPath(ConfigspaceCoord &next, const ConfigspaceCoord &prev) const {
	// number of joints
	const U32 numOfJoints = U32(arm.getJoints().size());
	
	Waypoint w, w0, w1;
	w.trn.resize(numOfJoints);
	w0.setup(arm, prev, false);
	w1.setup(arm, next, false);
	
	const Real dist = pHeuristic->getDist(w0, w1);
	const U32 steps = (U32)Math::ceil(dist/localFinderDesc.colissionDistDelta);
	
	// the first waypoint does not collide by assumption
	next = prev;
	for (U32 n = 0; n < steps; n++) {
		// lineary interpolate arm pose
		for (U32 j = 0; j < numOfJoints; j++)
			w.cpos[j] = w0.cpos[j] + (w1.cpos[j] - w0.cpos[j])*Real(n + 1)/Real(steps);
		
		arm.forwardTransformEx(&w.trn.front(), w.cpos);
		
		if (pHeuristic->collides(w))
			return false;

		next = w.cpos;
	}
	
	return true;
}

//------------------------------------------------------------------------------

bool GraphPlanner::localFind(Waypoint::Seq &localPath, const ConfigspaceCoord &begin, const ConfigspaceCoord &end, ParameterGuard& parameterGuard) {
	Real scale = REAL_ONE;
	for (U32 i = 0; i < pathFinderDesc.numOfIterations; i++) {
		// scale maximum distance between waypoints
		scale *= pathFinderDesc.distScaleFac;
		parameterGuard.setBaundedDistScale(scale);

		// get graph description
		PathFinder::GraphSearchDesc graphSearchDesc = pLocalPathFinder->getGraphSearchDesc();
		
		// setup graph generators, do not double root node
		graphSearchDesc.generators.clear();
		for (Waypoint::Seq::const_iterator i = ++localPath.begin(); i != localPath.end(); i++)
			graphSearchDesc.generators.push_back(i->cpos);
		
		// set graph generators range
		for (U32 i = 0; i < CONFIG_SPACE_DIM; i++)
			graphSearchDesc.range[i] = Real(pathFinderDesc.rangeFac*pHeuristic->getDistJointcoordMax()[i]);
				
		// update graph description
		pLocalPathFinder->setGraphSearchDesc(graphSearchDesc);

		// and generate local graph
		if (!pLocalPathFinder->generateGraph(begin, end)) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "GraphPlanner::localFind(): unable to generate graph");
			return false;
		}

		// find node path on local graph
		localPath.clear();
		if (!pLocalPathFinder->findPath(localPath, localPath.begin(), begin, end)) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "GraphPlanner::localFind(): unable to find path");
			return false;
		}
	}

	return true;
}

//#define SA_PERFMON

bool GraphPlanner::optimize(OptWaypoint::Seq &optimisedPath, const GenConfigspaceState &begin, const GenConfigspaceState &end) const {
	// number of joints
	const U32 numOfJoints = U32(arm.getJoints().size());
	
	// joint position limits
	ConfigspaceCoord min, max;
	for (U32 i = 0; i < numOfJoints; i++) {
		const Joint &joint = *arm.getJoints()[i];
		min[i] = joint.getMin().pos;
		max[i] = joint.getMax().pos;
	}
	
	// purge the path
	if (!purge(optimisedPath))
		return false;
	
#ifdef SA_PERFMON
	U32 n = 0, SA[2] = {0};
	static U32 nn = 0, id = 0;
	PerfTimer timer;
	SecTmReal t;
	static SecTmReal tt = SEC_TM_REAL_ZERO;
#endif //SA_PERFMON

	const U32 numOfIterations = optimisedPath.size() <= 2 ? 0 : (U32)optimisedPath.size()*pathOptimiserDesc.numOfIterations;
	for (U32 s = 0; s < numOfIterations; s++) {
		// select random waypoint
		const U32 index = 1 + rand.next()%((U32)optimisedPath.size() - 2);
		OptWaypoint &wCurr = optimisedPath[index];
		OptWaypoint &wPrev = optimisedPath[index - 1];
		OptWaypoint &wNext = optimisedPath[index + 1];
		Waypoint wTmp = wCurr;

		// temperature cooling schedule
		const Real T = pathOptimiserDesc.Tfinal +
			(pathOptimiserDesc.Tinit - pathOptimiserDesc.Tfinal)*(numOfIterations - s)/numOfIterations;
		
		// initialization
		const bool bInit = wCurr.population.empty();
		// search
		const bool bSearch = bInit || pathOptimiserDesc.searchProb > rand.nextUniform<Real>();
		
		// crossover
		const U32 crossPoint = rand.next()%numOfJoints;
		bool bCross = true;
		
		// random population vector
		ConfigspaceCoordSample* pPopulationVector = bInit ? NULL : &wCurr.population[0];
		// trial vector
		ConfigspaceCoordSample trialVector;

		// range
		for (U32 i = 0; i < numOfJoints; i++)
			wCurr.range[i] = Math::abs(wNext.cpos[i] - wPrev.cpos[i]);
		
		for (U32 i = (crossPoint + 1)%numOfJoints; i != crossPoint; i = (i + 1)%numOfJoints) {
			const Real range = wCurr.range[i] + wNext.range[i] + wPrev.range[i];
			
			const Real val = bCross ? range*rand.nextUniform(-T, +T) : REAL_ZERO;
			//const Real val = bCross ? range*rand.nextGaussian(REAL_ZERO, T) : REAL_ZERO;
			trialVector[i] = bSearch ? val : (*pPopulationVector)[i] + pathOptimiserDesc.combFac*val;
			
			if (bCross)
				wCurr.cpos[i] = Math::clamp(wCurr.cpos[i] + trialVector[i], min[i], max[i]);
			if (!bInit && pathOptimiserDesc.crossProb < rand.nextUniform<Real>())
				bCross = false;
		}

		// setup waypoint
		wCurr.setup(arm);

		// modification of a single waypoint on path affects neighbours as well
		Real cCurr, cPrev, cNext, cNew, cOld;
		
		cCurr = cost(index, optimisedPath);
		if (cCurr >= Node::COST_INF)
			goto REDO;
		cPrev = cost(index - 1, optimisedPath);
		if (cPrev >= Node::COST_INF)
			goto REDO;
		cNext = cost(index + 1, optimisedPath);
		if (cNext >= Node::COST_INF)
			goto REDO;

		// new cost
		cNew = cPrev + cCurr + cNext;
		// old cost
		cOld = wPrev.cost + wCurr.cost + wNext.cost;
		
		{
			// SA parameters
			const Real dE = pathOptimiserDesc.Enorm*(cNew - cOld);
			const bool b1 = dE < REAL_ZERO;
			const bool b2 = !b1 && Math::exp(-dE) > rand.nextUniform<Real>();

#ifdef SA_PERFMON
			if (b1) SA[0]++;
			if (b2) SA[1]++;
#endif //SA_PERFMON
			
			// update old cost if the new cost is lower and there are no collisions
			if ((b1 || b2) && !collides(index, optimisedPath)) {
				wCurr.cost = cCurr;
				wPrev.cost = cPrev;
				wNext.cost = cNext;

				if (bInit) {
					wCurr.population.push_back(trialVector);
				}
				else {
					for (U32 i = 0; i < numOfJoints; i++)
						(*pPopulationVector)[i] = pathOptimiserDesc.memFac*trialVector[i] +
							(REAL_ONE - pathOptimiserDesc.memFac)*(*pPopulationVector)[i];
				}
				
				// purge the path
				if (!purge(optimisedPath))
					return false;

#ifdef SA_PERFMON
				n++;
#endif //SA_PERFMON

				continue;
			}
		}

		REDO:
		wCurr = wTmp;
	}

#ifdef SA_PERFMON
	nn += n;
	t = timer.elapsed();
	tt+=t;

	Real c = REAL_ZERO;
	for (OptWaypoint::Seq::const_iterator i = optimisedPath.begin(); i != optimisedPath.end(); i++) c += i->cost;
	static Real cc = REAL_ZERO;
	cc+=c;
	context.getMessageStream()->write(Message::LEVEL_DEBUG, "#%d (ni=%d, ps=%d, ti=%.3f, tf=%.3f, cp=%.3f, sp=%.3f, cf=%.3f, mf=%.3f): l=%d, t=%.3f(%.3f), n=%d(%d), c=%.4f(%.4f), SA=(%d, %d)",
		++id, pathOptimiserDesc.numOfIterations, pathOptimiserDesc.populationSize, pathOptimiserDesc.Tinit, pathOptimiserDesc.Tfinal, pathOptimiserDesc.crossProb, pathOptimiserDesc.searchProb, pathOptimiserDesc.combFac, pathOptimiserDesc.memFac,
		optimisedPath.size(), t, tt, n, nn, c, cc, SA[0], SA[1]
	);
#endif //SA_PERFMON

	return true;
}

//------------------------------------------------------------------------------

bool GraphPlanner::profile(Trajectory& trajectory, Trajectory::iterator iter, OptWaypoint::Seq& optimisedPath, const GenConfigspaceState &begin, const GenConfigspaceState &end) const {
	// number of joints
	const U32 numOfJoints = U32(arm.getJoints().size());
	
	OptWaypoint::Seq::iterator i, j, k, l, first = optimisedPath.begin(), last = --optimisedPath.end();

	// propose trajectory duration 
	first->t = begin.t;
	first->cvel = begin.vel;
	first->jacc = begin.acc;
	last->t = end.t;
	last->cvel = end.vel;
	last->jacc = end.acc;

	// calculate waypoint distances
	k = first;
	k->dist = REAL_ZERO;
	while (k != last) {
		j = k++;
		k->dist = j->dist + pHeuristic->getDist(*j, *k);
		//k->dist = j->dist + pHeuristic->getWorkspaceDist(j->wpos, k->wpos);
		//k->dist = j->dist + pHeuristic->getJointspaceDist(j->cpos, k->cpos);
	}

	// total trajectory length
	const Real length = last->dist;

	// calculate inverse waypoint distances
	k = first;
	k->tNorm = SEC_TM_REAL_ZERO;
	while (k++ != last) {
		Real t;
		if (!pProfile->getDistanceInverse(t, k->dist/length)) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Planner::profile(): cannot compute inverse trajectory profile");
			return false;
		}
		k->tNorm = SecTmReal(t);
	}

	// distance between waypoints cannot be smaller than:
	const SecTmReal dt = arm.getTimeDelta();

	// interpolate waypoints
	Real gain = REAL_ONE;
	bool bLimits = true;
	INTERPOLATE:

	GenCoordTrj interpolTrj[CONFIG_SPACE_DIM];
	const I32 numOfTrajectories = (I32)Math::floor((last->t - first->t)/dt + SecTmReal(0.5));
	last->t = first->t + dt*numOfTrajectories;
	k = first;
	for (I32 s = 0; s <= numOfTrajectories; s++) {
		const SecTmReal t = first->t + SecTmReal(s)*dt;
		const Real dist = length * pProfile->getDistance(SecTmReal(s)/numOfTrajectories);

		while (k->dist <= dist && k != last) {
			j = k++;
			i = j; l = k;
			if (i != first)
				i--;
			if (l != last)
				l++;
			
			k->t = first->t + SecTmReal(numOfTrajectories)*dt*k->tNorm;
			l->t = first->t + SecTmReal(numOfTrajectories)*dt*l->tNorm;
			
			// 4 cases
			bool case4dddd = i != j && j != k && k != l;
			bool case3dvdd = i == j && j != k && k != l;
			bool case3dddv = i != j && j != k && k == l;
			bool case2dvdv = i == j && j != k && k == l;
			
			for (U32 n = 0; n < numOfJoints; n++) {
				GenCoordTrj &interpol = interpolTrj[n];

				if (case4dddd)
					interpol.set4dddd(i->t, j->t, k->t, l->t, i->cpos[n], j->cpos[n], k->cpos[n], l->cpos[n]);
				if (case3dvdd)
					interpol.set3dvdd(j->t, k->t, l->t, j->cpos[n], j->cvel[n], k->cpos[n], l->cpos[n]);
				if (case3dddv)
					interpol.set3dddv(i->t, j->t, k->t, i->cpos[n], j->cpos[n], k->cpos[n], k->cvel[n]);
				if (case2dvdv)
					interpol.set2dvdv(j->t, k->t, j->cpos[n], j->cvel[n], k->cpos[n], k->cvel[n]);
				
				if (bLimits) {
					const Joint &joint = *arm.getJoints()[n];
					Real tmin, tmax, min, max, g;
					
					interpol.getVelocityExtrema(tmin, tmax, min, max);
					g = Math::abs(min/(velocity*joint.getMin().vel));
					if (gain < g)
						gain = g;
					g = Math::abs(max/(velocity*joint.getMax().vel));
					if (gain < g)
						gain = g;

					interpol.getAccelerationExtrema(tmin, tmax, min, max);
					g = Math::sqrt(Math::abs(min/(acceleration*joint.getMin().acc)));
					if (gain < g)
						gain = g;
					g = Math::sqrt(Math::abs(max/(acceleration*joint.getMax().acc)));
					if (gain < g)
						gain = g;
				}
			}
		}

		if (bLimits)
			continue;

		GenConfigspaceState state;

		for (U32 n = 0; n < numOfJoints; n++)
			state.set(n, interpolTrj[n].get(t));
		state.t = t;

		iter = ++trajectory.insert(iter, state);
	}

	if (bLimits) {
		// change the trajectory duration only if the trajectory velocity or acceleration
		// are above limits
		if (gain > REAL_ONE)
			last->t = first->t + (last->t - first->t)*gain;
		
		bLimits = false;
		goto INTERPOLATE;
	}

	return true;
}

//------------------------------------------------------------------------------

bool GraphPlanner::findTarget(GenConfigspaceState &cend, const GenConfigspaceState &begin, const GenWorkspaceState& wend) {
#ifdef _GRAPHPLANNER_PERFMON
	PerfTimer t;
#endif

	// TODO this should be atomic call
	if (pCallback != NULL)
		pCallback->syncCollisionBounds();

#ifdef _GRAPHPLANNER_PERFMON
	t.reset();
#endif
	// generate graph
	if (!pGlobalPathFinder->generateGraph(begin.pos, wend.pos)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "GraphPlanner::findTarget(): unable to generate global graph");
		return false;
	}

	// find target
	if (!pGlobalPathFinder->findTarget(cend.pos)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "GraphPlanner::findTarget(): unable to find global path");
		return false;
	}
	cend.t = wend.t;
	cend.vel.set(REAL_ZERO);
	cend.acc.set(REAL_ZERO);

#ifdef _GRAPHPLANNER_PERFMON
	context.getMessageStream()->write(Message::LEVEL_DEBUG,
		"GlobalPathFinder::findTarget(): time elapsed = %f [sec], len = %d",
		t.elapsed(), globalPath.size()
	);
#endif

	return true;
}

//------------------------------------------------------------------------------

bool GraphPlanner::findTrajectory(Trajectory &trajectory, Trajectory::iterator iter, const golem::GenConfigspaceState &begin, const golem::GenConfigspaceState &end, const GenWorkspaceState* wend) {
#ifdef _GRAPHPLANNER_PERFMON
	PerfTimer t;
#endif
	
#ifdef _GRAPHPLANNER_PERFMON
#ifdef _HEURISTIC_PERFMON
	Heuristic::resetLog();
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::resetLog();
#endif
#endif

	// TODO this should be atomic call
	if (pCallback != NULL)
		pCallback->syncCollisionBounds();

#ifdef _GRAPHPLANNER_PERFMON
	t.reset();
#endif
	// generate global graph
	if (!pGlobalPathFinder->generateGraph(begin.pos, end.pos)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "GraphPlanner::findTrajectory(): unable to generate global graph");
		return false;
	}

	// find node path on global graph
	globalPath.clear();
	if (!pGlobalPathFinder->findPath(globalPath, globalPath.begin(), begin.pos, end.pos)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "GraphPlanner::findTrajectory(): unable to find global path");
		return false;
	}
#ifdef _GRAPHPLANNER_PERFMON
	context.getMessageStream()->write(Message::LEVEL_DEBUG,
		"GlobalPathFinder::findPath(): time elapsed = %f [sec], len = %d",
		t.elapsed(), globalPath.size()
	);
#endif

	ParameterGuard parameterGuard(pHeuristic);
	
	if (pLocalPathFinder != NULL) {
#ifdef _GRAPHPLANNER_PERFMON
		t.reset();
#endif
		localPath = globalPath;
		if (!localFind(localPath, begin.pos, end.pos, parameterGuard))
			return false;
#ifdef _GRAPHPLANNER_PERFMON
		context.getMessageStream()->write(Message::LEVEL_DEBUG,
			"GraphPlanner::localFind(): time elapsed = %f [sec], len = %d",
			t.elapsed(), localPath.size()
		);
#endif
		
		// setup extended waypoints using localPath
		optimisedPath.resize(localPath.size());
		Math::for_each(optimisedPath.begin(), optimisedPath.end(), localPath.begin(), (void (*)(OptWaypoint&, const Waypoint&))&OptWaypoint::setTo);
	}
	else {
		// setup extended waypoints using globalPath
		optimisedPath.resize(globalPath.size());
		Math::for_each(optimisedPath.begin(), optimisedPath.end(), globalPath.begin(), (void (*)(OptWaypoint&, const Waypoint&))&OptWaypoint::setTo);
	}

#ifdef _GRAPHPLANNER_PERFMON
#ifdef _HEURISTIC_PERFMON
	Heuristic::writeLog(context, "PathFinder::find()");
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::writeLog(context, "PathFinder::find()");
#endif
#endif

#ifdef _GRAPHPLANNER_PERFMON
#ifdef _HEURISTIC_PERFMON
	Heuristic::resetLog();
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::resetLog();
#endif
	t.reset();
#endif
	if (!optimize(optimisedPath, begin, end))
		return false;
#ifdef _GRAPHPLANNER_PERFMON
	context.getMessageStream()->write(Message::LEVEL_DEBUG,
		"GraphPlanner::optimize(): time elapsed = %f [sec]", t.elapsed()
	);
#ifdef _HEURISTIC_PERFMON
	Heuristic::writeLog(context, "GraphPlanner::optimize()");
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::writeLog(context, "GraphPlanner::optimize()");
#endif
#endif

#ifdef _GRAPHPLANNER_PERFMON
	t.reset();
#endif
	if (!profile(trajectory, iter, optimisedPath, begin, end))
		return false;
#ifdef _GRAPHPLANNER_PERFMON
	context.getMessageStream()->write(Message::LEVEL_DEBUG,
		"GraphPlanner::profile(): time elapsed = %f [sec], len = %d",
		t.elapsed(), trajectory.size()
	);
#endif

	// TODO this should be atomic call
	if (pCallback != NULL)
		pCallback->syncFindData(iter != trajectory.end() ? iter : trajectory.begin(), trajectory.end(), wend);
	
	return true;
}

//------------------------------------------------------------------------------

bool GraphPlanner::findConfigspaceTrajectory(Trajectory &trajectory, Trajectory::iterator iter, const golem::GenConfigspaceState &begin, const golem::GenConfigspaceState &end, const GenWorkspaceState* wend) {
#ifdef _GRAPHPLANNER_PERFMON
	PerfTimer t;
#endif
	
	// TODO this should be atomic call
	if (pCallback != NULL)
		pCallback->syncCollisionBounds();
	
	globalPath.clear();
	localPath.clear();
	optimisedPath.clear();

	OptWaypoint ow;
	ow.setup(arm, begin.pos);
	optimisedPath.push_back(ow);
	ow.setup(arm, end.pos);
	optimisedPath.push_back(ow);
	
	if (!profile(trajectory, iter, optimisedPath, begin, end))
		return false;

	// TODO this should be atomic call
	if (pCallback != NULL)
		pCallback->syncFindData(iter != trajectory.end() ? iter : trajectory.begin(), trajectory.end(), wend);

#ifdef _GRAPHPLANNER_PERFMON
	context.getMessageStream()->write(Message::LEVEL_DEBUG,
		"GraphPlanner::findConfigspaceTrajectory(): time elapsed = %f [sec], len = %d",
		t.elapsed(), trajectory.size()
	);
#endif
	
	return true;
}

//------------------------------------------------------------------------------

bool GraphPlanner::findWorkspaceTrajectory(Trajectory &trajectory, Trajectory::iterator iter, const golem::GenConfigspaceState &begin, const golem::GenWorkspaceState &wend) {
#ifdef _GRAPHPLANNER_PERFMON
	PerfTimer t;
#endif
	
	// TODO this should be atomic call
	if (pCallback != NULL)
		pCallback->syncCollisionBounds();
	
	globalPath.clear();
	localPath.clear();
	optimisedPath.clear();

	OptWaypoint curr;
	curr.setup(arm, begin.pos);
	optimisedPath.push_back(curr);
	
	Waypoint w0, w1;
	w0.wpos = curr.wpos;
	w0.wposq = curr.wposq;
	w1.wpos = wend.pos;
	w1.wposq.fromMat33(wend.pos.R);

	// number of joints
	const U32 numOfJoints = U32(arm.getJoints().size());
	
	const Real dist = pHeuristic->getWorkspaceDist(w0, w1); // always >= 0
	if (dist < REAL_EPS) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "GraphPlanner::findWorkspaceTrajectory(): target overlaps with the current pose");
		return false;
	}

	const U32 steps = (U32)Math::ceil(dist/localFinderDesc.interpolDistDelta); // always > 0

	for (U32 n = 0; n < steps; n++) {
		// initialization
		const bool bInit = n == 0;

		// normalized distance
		const Real distance = Real(n + 1)/Real(steps);
		
		// previous targets
		const OptWaypoint::Seq::const_iterator prev0 = --optimisedPath.end();
		const OptWaypoint::Seq::const_iterator prev1 = prev0 - 1;

		// current target (linear interpolation)
		curr.wpos.p.interpolate(w0.wpos.p, w1.wpos.p, distance);
		curr.wposq.slerp(w0.wposq, w1.wposq, distance);
		curr.wpos.R.fromQuat(curr.wposq);

		// initial search range
		GenConfigspaceCoord min, max;
		
		// generate min/max around initial waypoint
		for (U32 i = 0; i < numOfJoints; i++) {
			const Real pos = bInit ? prev0->cpos[i] : Real(2.0)*prev0->cpos[i] - prev1->cpos[i]; // linear extrapolation
			const Real range = localFinderDesc.range.pos[i];
			min.pos[i] = pos - range;
			max.pos[i] = pos + range;
		}

		pKinematics->setMin(min);
		pKinematics->setMax(max);

		if (!pKinematics->find(curr.cpos, curr.wpos)) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "GraphPlanner::findWorkspaceTrajectory(): unable to solve inverse kinematics");
			return false;
		}

		curr.setup(arm);

		if (!trimPath(curr.cpos, prev0->cpos)) {
			optimisedPath.push_back(curr);
			break;
		}

		optimisedPath.push_back(curr);
	}

	GenConfigspaceState cend;
	cend.t = wend.t;
	cend.pos = optimisedPath.back().cpos;
	cend.vel.set(REAL_ZERO);
	cend.acc.set(REAL_ZERO);

	if (!profile(trajectory, iter, optimisedPath, begin, cend))
		return false;

	// TODO this should be atomic call
	if (pCallback != NULL)
		pCallback->syncFindData(iter != trajectory.end() ? iter : trajectory.begin(), trajectory.end(), &wend);

#ifdef _GRAPHPLANNER_PERFMON
	context.getMessageStream()->write(Message::LEVEL_DEBUG,
		"GraphPlanner::findWorkspaceTrajectory(): time elapsed = %f [sec], len = %d",
		t.elapsed(), trajectory.size()
	);
#endif
	
	return true;
}

//------------------------------------------------------------------------------
