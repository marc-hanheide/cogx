/** @file ReacPlanner.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

//#include <Golem/Tools/Debug.h>
#include <Golem/Ctrl/ReacPlanner.h>
#include <Golem/Ctrl/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

ReacPlanner::ReacPlanner(golem::Planner &planner) :
	planner(planner), arm(planner.getArm()), context(arm.getContext())
{
}

ReacPlanner::~ReacPlanner() {
	release();
}

bool ReacPlanner::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgReacPlannerInvalidDesc(Message::LEVEL_CRIT, "ReacPlanner::create(): Invalid description");

	pTransmitter = desc.pTransmitterDesc->create(arm); // throws

	signalSync = desc.signalSync;
	
	pKinematics = dynamic_pointer_cast<DEKinematics::Ptr>(desc.pKinematicsDesc->create(*planner.getHeuristic()));
	if (pKinematics == NULL)
		throw MsgReacPlannerUnknownKinematics(Message::LEVEL_CRIT, "ReacPlanner::create(): Unknown kinematics solver");

	globalRange = desc.globalRange;
	localRange = desc.localRange;
	threadPriority = desc.threadPriority;
	threadTimeOut = desc.threadTimeOut;

	evTerminate.set(false);
	evSend.set(false);
	evWaitBegin.set(false);
	evWaitEnd.set(false);
	signalInit = true;

	if (!thread.start(this))
		throw MsgReacPlannerThreadLaunch(Message::LEVEL_CRIT, "ReacPlanner::launch(): Unable to launch ReacPlanner thread");

	if (!thread.setPriority(threadPriority))
		context.getMessageStream()->write(Message::LEVEL_WARNING, "ReacPlanner::launch(): Unable to change ReacPlanner thread priority");
	
	runningTime = desc.runningTime;
	sleepTime = desc.sleepTime;
	tmDelta = Math::ceil(std::max(runningTime + sleepTime, arm.getTimeDelta()), arm.getTimeQuant());
	tmDeltaAsync = tmDelta*Real(3.0); // TODO get rid of the magic constant
	
	distLinearMax = desc.distLinearMax;
	distAngularMax = desc.distAngularMax;
	colissionDistDelta = desc.colissionDistDelta;
	distRestJointcoordFac = desc.distRestJointcoordFac;

	return true;
}

void ReacPlanner::release() {
	evTerminate.set(true);
	evWaitBegin.set(true);
	evWaitEnd.set(true);
	evSend.set(true);

	if (!thread.join(threadTimeOut))
		context.getMessageStream()->write(Message::LEVEL_ERROR, "ReacPlanner::release(): Cannot stop control thread");
}

//------------------------------------------------------------------------------

void ReacPlanner::run() {
	Action actionPrev = ACTION_STOP;
	GenConfigspaceState cPrev, cNext, cPose;

	if (!arm.lookupCommand(cPose, SEC_TM_REAL_MAX)) {
		context.getMessageStream()->write(Message::LEVEL_CRIT, "ReacPlanner::run(): arm state query error");
		return;
	}

	for (;;) {
		if (!evSend.wait()) {
			context.getMessageStream()->write(Message::LEVEL_WARNING, "ReacPlanner::run(): Event evSend timeout"	);
			continue;
		}
		
		if (evTerminate.wait(0))
			break;
		
		evSend.set(!signalSync);

		// get the target state
		Signal target;
		if (!pTransmitter->get(target)) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "ReacPlanner::run(): Transmitter get error");
			continue;
		}

		signalInit = false;
		
		Action action = this->action;
		if (action == ACTION_AUTO)
			action = getAction(cPose, target);

		switch (action) {
		case ACTION_STOP:
			stop(cPose);// stop the arm
			evSend.set(false); // always wait for the next command
			break;
		case ACTION_MOVE:
			if (!move(cPose, cNext, cPrev, target, runningTime)) {
				//stop(cPose);// stop the arm
				continue;
			}
			break;
		case ACTION_LOCAL:
			if (actionPrev == ACTION_MOVE) {
				stop(cPose);// stop the arm
			}
			if (!plan(cPose, cNext, cPrev, target, ACTION_LOCAL, runningTime)) {
				evSend.set(true);
				continue;// try again
			}
			break;
		case ACTION_GLOBAL:
			if (actionPrev == ACTION_MOVE) {
				stop(cPose);// stop the arm
			}
			if (!plan(cPose, cNext, cPrev, target, ACTION_GLOBAL, runningTime)) {
				evSend.set(true);
				continue;// try again
			}
			break;
		}

		// Take the last arm state
		if (!arm.lookupCommand(cPose, SEC_TM_REAL_MAX)) {
			context.getMessageStream()->write(Message::LEVEL_CRIT, "ReacPlanner::run(): arm state query error");
			return;
		}

		evWaitBegin.set(true);
		
		context.getTimer().sleep(
			std::max(SEC_TM_REAL_ZERO, cPose.t - context.getTimer().elapsed() - getTimeDeltaAsync() + getTimeDelta())
		);
//		if (evTerminate.wait(MSecTmU32(1000.0*std::max(SEC_TM_REAL_ZERO, cPose.t - context.getTimer().elapsed() - getTimeDeltaAsync() + getTimeDelta()))))
//			break;
		
		evWaitEnd.set(true);
		
		cPrev = cNext;
		actionPrev = action;
	}
}

bool ReacPlanner::send(Action action) {
	this->action = action;
	evWaitBegin.set(false);
	evWaitEnd.set(false);
	evSend.set(signalInit || signalSync || action != ACTION_STOP);

	return true;
}

bool ReacPlanner::waitForBegin(MSecTmU32 timeOut) {
	if (!evWaitBegin.wait(timeOut)) {
		//context.getMessageStream()->write(Message::LEVEL_WARNING, "ReacPlanner::waitForBegin(): Event timeout");
		return false;
	}
	
	return true;
}

bool ReacPlanner::waitForEnd(MSecTmU32 timeOut) {
	if (!evWaitEnd.wait(timeOut)) {
		//context.getMessageStream()->write(Message::LEVEL_WARNING, "ReacPlanner::waitForEnd(): Event timeout");
		return false;
	}
	
	return true;
}

//------------------------------------------------------------------------------

bool ReacPlanner::trimPath(GenConfigspaceCoord &next, const GenConfigspaceCoord &prev) const {
	// number of joints
	const U32 numOfJoints = U32(arm.getJoints().size());
	
	Waypoint w, w0, w1;
	w.trn.resize(numOfJoints);
	w0.setup(arm, prev.pos, false);
	w1.setup(arm, next.pos, false);
	
	const Real dist = planner.getHeuristic()->getDist(w0, w1);
	const U32 steps = (U32)Math::ceil(dist/colissionDistDelta);
	
	// the first waypoint does not collide by assumption
	next.pos = prev.pos;
	for (U32 n = 0; n < steps; n++) {
		// lineary interpolate arm pose
		for (U32 j = 0; j < numOfJoints; j++)
			w.cpos[j] = w0.cpos[j] + (w1.cpos[j] - w0.cpos[j])*Real(n + 1)/Real(steps);
		
		arm.forwardTransformEx(&w.trn.front(), w.cpos);
		
		if (planner.getHeuristic()->collides(w)) {
			next.vel.set(Real(0.0));
			next.acc.set(Real(0.0));
			return false;
		}

		next.pos = w.cpos;
	}
	
	return true;
}

void ReacPlanner::generateTrajectory(GenCoordTrj *trajectories, SecTmReal &duration, const GenConfigspaceState &prev, const GenConfigspaceState &next) const {	
	// number of joints
	const U32 numOfJoints = U32(arm.getJoints().size());
	
	duration = getTimeDelta();
	Real gain = REAL_ONE;
	bool bLimits = true;
	
	REPEAT:
	for (U32 i = 0; i < numOfJoints; i++) {
		const Joint &joint = *arm.getJoints()[i];
		const GenCoord p = prev.get(i);
		const GenCoord n = next.get(i);
		
		// Trajectories are simple 3-rd degree polynomials
		GenCoordTrj &t = trajectories[i];
		
		// create a trajectory
		t.set(REAL_ZERO, Real(duration), p, n);
		
		// check the target trajectory against velocity and acceleration limits
		if (bLimits) {
			Real tmin, tmax, min, max, g;
			
			t.getVelocityExtrema(tmin, tmax, min, max);
			g = Math::abs(min/(planner.getVelocity()*joint.getMin().vel));
			if (gain < g)
				gain = g;
			g = Math::abs(max/(planner.getVelocity()*joint.getMax().vel));
			if (gain < g)
				gain = g;

			t.getAccelerationExtrema(tmin, tmax, min, max);
			g = Math::sqrt(Math::abs(min/(planner.getAcceleration()*joint.getMin().acc)));
			if (gain < g)
				gain = g;
			g = Math::sqrt(Math::abs(max/(planner.getAcceleration()*joint.getMax().acc)));
			if (gain < g)
				gain = g;
		}
	}

	if (bLimits) {
		if (gain > REAL_ONE)
			duration *= gain;
		bLimits = false;
		goto REPEAT;
	}
}

ReacPlanner::Action ReacPlanner::getAction(const GenConfigspaceState &cPose, const Signal &target) const {
	WorkspaceCoord w0, w1;

	const Mat34 referencePose = arm.getReferencePose();
	
	arm.forwardTransform(w0, cPose.pos);
	w0.multiply(w0, referencePose); // reference pose
	if (target.type == Signal::TYPE_JOINTSPACE) {
		arm.forwardTransform(w1, target.gjs.pos);
		w1.multiply(w1, referencePose); // reference pose
	}
	else {
		w1 = target.gws.pos;
	}

	// linear distance
	const Real p = planner.getHeuristic()->getLinearDist(w0.p, w1.p);
	if (p > distLinearMax)
		return ACTION_GLOBAL;
	
	// angular distance
	const Quat q0(w0.R), q1(w1.R); 
	const Real q = planner.getHeuristic()->getAngularDist(q0, q1);
	if (q > distAngularMax)
		return ACTION_GLOBAL;

	return ACTION_MOVE;
}

//------------------------------------------------------------------------------

bool ReacPlanner::move(GenConfigspaceState &pose, GenConfigspaceState &next, const GenConfigspaceState &prev, const Signal &target, SecTmReal runningTime) {
	// number of joints
	const U32 numOfJoints = U32(arm.getJoints().size());
	const SecTmReal timeDelta = getTimeDelta();
	
	// TODO this should be atomic call
	if (planner.getCallback() != NULL)
		planner.getCallback()->syncCollisionBounds();

	if (target.type == Signal::TYPE_JOINTSPACE) {
		next = target.gjs;
	}
	else {
		// initial search range
		GenConfigspaceCoord min, max;
		
		// TODO use better predictive filter
		// generate min/max
		for (U32 i = 0; i < numOfJoints; i++) {
			const Joint &joint = *arm.getJoints()[i];
			
			const Real pos = Math::clamp(
				prev.pos[i] + prev.vel[i]*timeDelta,// + Real(0.5)*(prev.acc[i]*timeDelta*timeDelta),
				joint.getMin().pos, joint.getMax().pos
			);
			//const Real vel = Math::clamp(
			//	prev.vel[i],// + prev.acc[i]*timeDelta,
			//	joint.getMin().vel, joint.getMax().vel
			//);
			//const Real acc = Math::clamp(
			//	prev.acc[i],
			//	joint.getMin().acc, joint.getMax().acc
			//);
			
			min.pos[i] = pos - localRange.pos[i];
			max.pos[i] = pos + localRange.pos[i];
			min.vel[i] = - localRange.vel[i];//vel - localRange.vel[i];
			max.vel[i] = + localRange.vel[i];//vel + localRange.vel[i];
			min.acc[i] = - localRange.acc[i];//acc - localRange.acc[i];
			max.acc[i] = + localRange.acc[i];//acc + localRange.acc[i];
		}

		pKinematics->setMin(min);
		pKinematics->setMax(max);

		ParameterGuard parameterGuard(planner.getHeuristic());
		planner.getHeuristic()->setDistRestJointcoordFac(distRestJointcoordFac);

		if (!pKinematics->find(next, target.gws, runningTime)) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "ReacPlanner::moveLocal(): unable to find solution");
			return false;
		}
	}

	if (planner.getHeuristic()->hasCollisionDetection())
		if (!trimPath(next, prev)) {// collision detected
//			context.getMessageStream()->write(Message::LEVEL_NOTICE, "ReacPlanner::move(): collision");
		}

	SecTmReal duration;
	GenCoordTrj trajectories[CONFIG_SPACE_DIM];
	generateTrajectory(trajectories, duration, pose, next);
	//pose = next; // ignore velocity and acceleration constraints
	
	for (U32 i = 0; i < numOfJoints; i++)
		pose.set(i, trajectories[i].get(Real(getTimeDelta())));
	pose.t += getTimeDelta();
	
	// Send data (wait until buffer is empty)
	if (!arm.send(pose)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "ReacPlanner::moveLocal(): send error"	);
		return false;
	}

	return true;
}

bool ReacPlanner::plan(GenConfigspaceState &pose, GenConfigspaceState &next, const GenConfigspaceState &prev, const Signal &target, Action action, SecTmReal runningTime) {
	// number of joints
	const U32 numOfJoints = U32(arm.getJoints().size());

	GenConfigspaceState cend;
	GenWorkspaceState wend;
	Planner::Trajectory trajectory;
	
	// extract target
	if (target.type == Signal::TYPE_JOINTSPACE) {
		(GenConfigspaceCoord&)cend = target.gjs;
		cend.t = pose.t + std::max(getTimeDelta(), target.gjs.t - context.getTimer().elapsed() - getTimeDeltaAsync());

		if (action == ReacPlanner::ACTION_LOCAL ?
			!planner.findConfigspaceTrajectory(trajectory, trajectory.begin(), pose, cend) :
			!planner.findTrajectory(trajectory, trajectory.begin(), pose, cend)
			)
			return false;
	}
	else {
		(GenWorkspaceCoord&)wend = target.gws;
		wend.t = pose.t + std::max(getTimeDelta(), target.gws.t - context.getTimer().elapsed() - getTimeDeltaAsync());

		if (action == ReacPlanner::ACTION_LOCAL ?
			!planner.findWorkspaceTrajectory(trajectory, trajectory.begin(), pose, wend) :
			!planner.findTarget(cend, pose, wend) || !planner.findTrajectory(trajectory, trajectory.begin(), pose, cend)
			)
			return false;
	}

	// and send it
	if (!planner.send<GenConfigspaceState>(trajectory.begin(), trajectory.end())) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "ReacPlanner::plan(): send error");
		return false;
	}

	next = pose = trajectory.back();
	
	return true;
}

void ReacPlanner::stop(GenConfigspaceState &pose) {
	// number of joints
	const U32 numOfJoints = U32(arm.getJoints().size());
	
	// compute trajectory for each joint
	for (U32 j = 0; j < numOfJoints; j++) {
		golem::GenCoordTrj trj;
		// boundary conditions are: at t0 {pos, vel, acc}; at t1 {pos=?, vel=0, acc=?}
		trj.set2dvav(SEC_TM_REAL_ZERO, getTimeDelta(), pose.pos[j], pose.vel[j], pose.acc[j], Real(0.0));
		pose.set(j, trj.get(getTimeDelta()));
	}
	//pose.t += getTimeDelta();
	pose.t = context.getTimer().elapsed() + getTimeDeltaAsync();
	
	if (!arm.send(pose)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "ReacPlanner::stop(): send error");
		return;
	}
}

//------------------------------------------------------------------------------
