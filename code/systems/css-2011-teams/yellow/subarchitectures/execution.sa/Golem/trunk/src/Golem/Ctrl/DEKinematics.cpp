/** @file DEKinematics.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

//#include <Golem/Tools/Debug.h>
#include <Golem/Ctrl/DEKinematics.h>
#include <Golem/Ctrl/Planner.h>
#include <Golem/Ctrl/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

DEKinematics::DEKinematics(golem::Heuristic &heuristic) :
	Kinematics(heuristic)
{
	zero.set(REAL_ZERO);
}

bool DEKinematics::create(const Desc& desc) {
	Kinematics::create(desc); // throws
	
	populationSize = desc.populationSize;
	numOfGenerations = desc.numOfGenerations;
	diffFac = desc.diffFac;
	crossProb = desc.crossProb;
	distGoalFac = desc.distGoalFac;
	distPrevFac = desc.distPrevFac;
	posVelFac = desc.posVelFac;
	generator = desc.generator;
	generator->setRandSeed(context.getRandSeed());
	
	bInitVelPopulation = bInitPosPopulation = true;
	
	return true;
}

//------------------------------------------------------------------------------

void DEKinematics::setPosPopulation(const Waypoint::Seq &population) {
	this->posPopulation = population;
	bInitPosPopulation = false;
}

const Waypoint::Seq &DEKinematics::getPosPopulation() const {
	return posPopulation;
}

void DEKinematics::setVelPopulation(const GenWaypoint::Seq &population) {
	this->velPopulation = population;
	bInitVelPopulation = false;
}

const GenWaypoint::Seq &DEKinematics::getVelPopulation() const {
	return velPopulation;
}

Real DEKinematics::costPos(const ConfigspaceCoord &jCurr, const ConfigspaceCoord &cPrev, const Waypoint &wCurr, const Waypoint &wGoal) const {
	const Real d0 = heuristic.getWorkspaceDist(wCurr, wGoal);
	const Real d1 = heuristic.getJointspaceDist(jCurr, cPrev);
	const Real d2 = heuristic.getRestJointspaceDist(jCurr);
	
	return distGoalFac*d0 + (REAL_ONE - distGoalFac)*(distPrevFac*d1 + (REAL_ONE - distPrevFac)*d2);
}

Real DEKinematics::costVel(const ConfigspaceCoord &jVelCurr, const ConfigspaceCoord &jVelPrev, const Twist &wVelCurr, const Twist &wVelGoal) const {
//	const Real d0 = heuristic.getVelocityDist(wCurr, wGoal);
//	const Real d1 = heuristic.getJointspaceDist(jCurr, cPrev);
//	const Real d2 = heuristic.getJointspaceDist(jCurr, zero);
	
//	return distGoalFac*d0 + (REAL_ONE - distGoalFac)*(distPrevFac*d1 + (REAL_ONE - distPrevFac)*d2);

	const Real distGoalFac = Real(0.99);
	const Real d0 = heuristic.getLinearDist(wVelCurr.v, wVelGoal.v);
	const Real d1 = heuristic.getJointspaceMagnitude(jVelCurr);

	return heuristic.getDistLinearFac()*distGoalFac*d0 + (REAL_ONE - distGoalFac)*d1;
}

bool DEKinematics::collides(const Waypoint &w) const {
	return heuristic.collides(w);
}

//------------------------------------------------------------------------------

bool DEKinematics::find(U32 &solution, Waypoint::Seq &population, const ConfigspaceCoord &jc, const WorkspaceCoord &wc, const Generator &generator, SecTmReal runningTime) const {
	SecTmReal tmStart = context.getTimer().elapsed();
	
	// number of joints
	const U32 numOfJoints = U32(arm.getJoints().size());
	
	// collision detection
	bool hasCollisionDetection = this->hasCollisionDetection() && heuristic.hasCollisionDetection();

	// goal waypoint setup
	Waypoint wgoal;
	wgoal.wpos = wc;
	wgoal.wposq.fromMat33(wc.R);

	// population size
	const U32 initialPopulationSize = U32(population.size());
	population.resize(populationSize);
	
	U32 numOfSolutions = initialPopulationSize;

	while (numOfSolutions < populationSize) {
		if (runningTime < context.getTimer().elapsed() - tmStart)
			goto SELECT;
		
		Waypoint &w = population[numOfSolutions];

		// generate configuration
		if (numOfSolutions == initialPopulationSize)
			w.cpos = jc;
		else
			generator.next(w.cpos, min.pos, max.pos, numOfJoints);

		// setup the waypoint
		w.setup(arm, hasCollisionDetection);
		w.cost = costPos(w.cpos, jc, w, wgoal);
		// check if the position has infinite cost or collides
		if (w.cost >= Node::COST_INF || hasCollisionDetection && collides(w))
			continue;

		numOfSolutions++;
	}

	for (U32 n = 0; n < numOfGenerations; n++) {
		if (runningTime < context.getTimer().elapsed() - tmStart)
			goto SELECT;
		
		// we assume here that 4 << populationSize; otherwise is better to use random permutation
		U32 randIndex[4];
		for (U32 i = 0; i < 4;) {
			REPEAT:
			U32 index = rand.next()%populationSize;
			for (U32 j = 0; j < i; j++)
				if (randIndex[j] == index)
					goto REPEAT;
			randIndex[i++] = index;
		}

		Waypoint &w0 = population[randIndex[0]];
		Waypoint &w1 = population[randIndex[1]];
		Waypoint &w2 = population[randIndex[2]];
		Waypoint &w3 = population[randIndex[3]];

		GenWaypoint w;
		w.cpos = w0.cpos;
		
		const U32 crossPoint = rand.next()%numOfJoints;
		U32 i = crossPoint;
		
		do {
			w.cpos[i] = Math::clamp(w1.cpos[i] + diffFac*(w2.cpos[i] - w3.cpos[i]), min.pos[i], max.pos[i]);
			i = (i + 1)%numOfJoints;
		} while (i != crossPoint && crossProb > rand.nextUniform<Real>());
		
		// setup the waypoint
		w.setup(arm, hasCollisionDetection);
		w.cost = costPos(w.cpos, jc, w, wgoal);
		// check if the position is no better than before or collides
		if (w.cost >= w0.cost || hasCollisionDetection && collides(w))
			continue;

		w0 = w;
	}

	// find the best
	SELECT:
	if (numOfSolutions <= 0)
		return false;

	solution = 0;
	for (U32 i = 1; i < numOfSolutions; i++)
		if (population[solution].cost > population[i].cost)
			solution = i;

	return true;
}

//------------------------------------------------------------------------------

bool DEKinematics::find(U32 &solution, GenWaypoint::Seq &population, const GenConfigspaceCoord &gj, const GenWorkspaceCoord &gw, const Generator &generator, SecTmReal runningTime) const {
	SecTmReal tmStart = context.getTimer().elapsed();
	
	// number of joints
	const U32 numOfJoints = U32(arm.getJoints().size());
	
	const Mat34 referencePose = arm.getReferencePose();
	
	// population size
	const U32 initialPopulationSize = U32(population.size());
	population.resize(populationSize);
	
	U32 numOfSolutions = initialPopulationSize;

	// setup initial waypoint
	GenWaypoint winit;
	winit.cpos = gj.pos;
	winit.setup(arm, false);

	WorkspaceVel wvel;
	referencePose.adjointTransform(wvel, gw.vel);
	
	while (numOfSolutions < populationSize) {
		if (runningTime < context.getTimer().elapsed() - tmStart)
			goto SELECT;
		
		GenWaypoint &w = population[numOfSolutions];

		// generate configuration
		w = winit;
		if (numOfSolutions == initialPopulationSize)
			w.cvel = gj.vel;
		else
			generator.next(w.cvel, min.vel, max.vel, numOfJoints);

		// velocity
		arm.velocityFromJacobian(w.wvel, w.jacobian, w.cvel);
		
		// calculate cost
		w.cost = costVel(w.cvel, gj.vel, w.wvel, wvel);
		
		// check if the position has infinite cost
		if (w.cost >= Node::COST_INF)
			continue;

		numOfSolutions++;
	}

	for (U32 n = 0; n < numOfGenerations; n++) {
		if (runningTime < context.getTimer().elapsed() - tmStart)
			goto SELECT;

		// we assume here that 4 << populationSize; otherwise is better to use random permutation
		U32 randIndex[4];
		for (U32 i = 0; i < 4;) {
			REPEAT:
			U32 index = rand.next()%populationSize;
			for (U32 j = 0; j < i; j++)
				if (randIndex[j] == index)
					goto REPEAT;
			randIndex[i++] = index;
		}

		GenWaypoint &w0 = population[randIndex[0]];
		GenWaypoint &w1 = population[randIndex[1]];
		GenWaypoint &w2 = population[randIndex[2]];
		GenWaypoint &w3 = population[randIndex[3]];

		ConfigspaceCoord tjvel = w0.cvel;
		WorkspaceVel twvel = w0.wvel;
		Real tcost = w0.cost;
		
		const U32 crossPoint = rand.next()%numOfJoints;
		U32 i = crossPoint;
		
		do {
			tjvel[i] = Math::clamp(w1.cvel[i] + diffFac*(w2.cvel[i] - w3.cvel[i]), min.vel[i], max.vel[i]);
			i = (i + 1)%numOfJoints;
		} while (i != crossPoint && crossProb > rand.nextUniform<Real>());
		
		// velocity
		arm.velocityFromJacobian(twvel, w0.jacobian, tjvel);

		// cost
		tcost = costVel(tjvel, gj.vel, twvel, wvel);

		// check if the position is no better than before or collides
		if (tcost >= w0.cost)
			continue;

		w0.cvel = tjvel;
		w0.wvel = twvel;
		w0.cost = tcost;
	}

	// find the best
	SELECT:
	if (numOfSolutions <= 0)
		return false;

	solution = 0;
	for (U32 i = 1; i < numOfSolutions; i++)
		if (population[solution].cost > population[i].cost)
			solution = i;

	GenWaypoint w = population[solution];
	
//	const Real d0 = heuristic.getLinearDist(w.wvel.v, wvel.v);
//	const Real d1 = heuristic.getJointspaceMagnitude(w.cvel);
//	context.getLogger()->post(new Message(Message::LEVEL_DEBUG, "[%f, %f, %f, %f, %f], tar=%f, curr=%f, cost=(%f, %f)",
//		w.cvel[0], w.cvel[1], w.cvel[2], w.cvel[3], w.cvel[4],
//		wvel.v.magnitude(), w.wvel.v.magnitude(), d0, d1
//	);


	return true;
}

//------------------------------------------------------------------------------

bool DEKinematics::find(GenConfigspaceCoord &gj, const GenWorkspaceCoord &gw, SecTmReal runningTime) {	
	const SecTmReal tmPos = SecTmReal(posVelFac)*runningTime;
	const SecTmReal tmVel = SecTmReal(REAL_ONE - posVelFac)*runningTime;

	//gj.pos.set(REAL_ZERO);
	//gj.vel.set(REAL_ZERO);
	gj.acc.set(REAL_ZERO);
	
	// clear population
	if (bInitPosPopulation)
		posPopulation.clear();

	U32 solution;
	if (!DEKinematics::find(solution, posPopulation, gj.pos, gw.pos, *generator, tmPos))
		return false;
	gj.pos = posPopulation[solution].cpos;
	bInitPosPopulation = true;	

	// clear population
	if (bInitVelPopulation)
		velPopulation.clear();

	if (!DEKinematics::find(solution, velPopulation, gj, gw, *generator, tmVel))
		return false;
	gj.vel = velPopulation[solution].cvel;
//	gj.vel.set(REAL_ZERO);
	bInitVelPopulation = true;	

	return true;
}

bool DEKinematics::find(ConfigspaceCoord &jc, const WorkspaceCoord &wc, SecTmReal runningTime) {
	// clear population
	if (bInitPosPopulation)
		posPopulation.clear();

	U32 solution;
	if (!DEKinematics::find(solution, posPopulation, jc, wc, *generator, runningTime))
		return false;
	jc = posPopulation[solution].cpos;
	bInitPosPopulation = true;

	return true;
}

//------------------------------------------------------------------------------
