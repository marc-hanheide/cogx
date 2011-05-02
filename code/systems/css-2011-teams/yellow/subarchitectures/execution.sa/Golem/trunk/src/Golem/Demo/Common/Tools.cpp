/** @file Tools.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/PhysCtrl/Creator.h>
#include <Golem/Demo/Common/Tools.h>
#include <Golem/Demo/Common/Msg.h>
#include <sstream>
#include <iomanip>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

char* golem::strdup(const char* src) {
	if (src == NULL)
		return NULL;
	
	size_t len = strlen(src) + 1;
	char* dst = new char [len];

	if (dst != NULL) {
		memcpy(dst, src, len);
	}

	return dst;
}

char* golem::strndup(const char* src, size_t n) {
	if (src == NULL)
		return NULL;
	
	size_t len = strlen(src) + 1;
	if (len > n + 1)
		len = n + 1;

	char* dst = new char [len];

	if (dst != NULL) {
		memcpy(dst, src, len);
		dst[n] = 0;
	}

	return dst;
}

char* golem::istreamstrdup(std::istream& istr, char delim) {
	std::streamoff pos = istr.tellg(), shift = 0;
	size_t len = 1;
	
	while (shift < 2) {
		char c;
		istr.get(c);
		
		if (!istr)
			break;
		else if (c == delim) {
			if (len > 1)
				break;
			shift++;
		}
		else
			len++;
	}

	char* dst = new char [len];
	dst[0] = 0;
	
	istr.seekg(pos + shift);
	if (shift < 2)
		istr >> dst;
	
	return dst;
}

//------------------------------------------------------------------------------

void golem::armInfo(Arm& arm) {
	MessageStream *stream = arm.getContext().getMessageStream();
	const U32 numOfJoints = (U32)arm.getJoints().size();
	
	// Arm info
	stream->write(Message::LEVEL_INFO, "%s", arm.getName().c_str());
	stream->write(Message::LEVEL_INFO, "Minimal trajectory duration  = %.4f [sec]", arm.getTimeDelta());
	stream->write(Message::LEVEL_INFO, "Maximal reaction time        = %.4f [sec]", arm.getTimeDeltaAsync());
	stream->write(Message::LEVEL_INFO, "Trajectory granularity/quant = %.4f [sec]", arm.getTimeQuant());
	
	// Read joints
	for (U32 i = 0; i < numOfJoints; i++) {
		const Joint &joint = *arm.getJoints()[i];

		// Joint info
		stream->write(Message::LEVEL_INFO, "%s", joint.getName().c_str());
		stream->write(Message::LEVEL_INFO, "pos <min, max> = <%.4f, %.4f> [rad]", joint.getMin().pos,  joint.getMax().pos);
		stream->write(Message::LEVEL_INFO, "vel <min, max> = <%.4f, %.4f> [rad/sec]", joint.getMin().vel,  joint.getMax().vel);
		stream->write(Message::LEVEL_INFO, "acc <min, max> = <%.4f, %.4f> [rad/sec^2]", joint.getMin().acc,  joint.getMax().acc);
	}
}

void golem::armState(Arm &arm, const GenConfigspaceState &state) {
	MessageStream *stream = arm.getContext().getMessageStream();
	const U32 numOfJoints = (U32)arm.getJoints().size();

	stream->write(Message::LEVEL_INFO, "%s: {time} = {%.4f [sec]}", arm.getName().c_str(), state.t);
	
	// Print the arm state
	for (U32 i = 0; i < numOfJoints; i++) {
		const Joint &joint = *arm.getJoints()[i];
		
		stream->write(Message::LEVEL_INFO, "%s: {pos, vel, acc} = {%.4f [rad], %.4f [rad/sec], %.4f [rad/sec^2]}",
			joint.getName().c_str(), state.pos[i], state.vel[i], state.acc[i]);
	}
}

void golem::armPose(Arm &arm, const GenConfigspaceState &state) {
	MessageStream *stream = arm.getContext().getMessageStream();
	const U32 numOfJoints = (U32)arm.getJoints().size();
	//std::stringstream names;
	std::stringstream values;

	values << std::setw(7) << std::setprecision(4) << std::fixed;
	//names << "time";
	values << state.t << " [sec]";

	for (U32 i = 0; i < numOfJoints; i++) {
		//const Joint &joint = *arm.getJoints()[i];
		//names << ", " << joint.getName().c_str();
		values << ", " << state.pos[i] << " [rad]";
	}
	
	//stream->write(Message::LEVEL_INFO, "%s: {%s} = {%s}", arm.getName().c_str(), names.str().c_str(), values.str().c_str());
	stream->write(Message::LEVEL_INFO, "%s: {%s}", arm.getName().c_str(), values.str().c_str());
}

void golem::armWatch(Arm &arm) {
	MessageStream *stream = arm.getContext().getMessageStream();
	GenConfigspaceState last, state;

	// Get the last trajectory point
	if (!arm.lookupCommand(last, SEC_TM_REAL_MAX))
		throw Message(Message::LEVEL_CRIT, "armWatch(): arm pose query error");

	// Watch the movement
	do {
		// Get the current position of joints
		if (!arm.recv(state))
			throw Message(Message::LEVEL_CRIT, "armWatch(): receive error");
		
		golem::armPose(arm, state);
	} while (state.t < last.t);
}

void golem::armMove(Arm &arm, const GenConfigspaceCoord &end, Real velocity, Real acceleration) {
	MessageStream *stream = arm.getContext().getMessageStream();
	const U32 numOfJoints = (U32)arm.getJoints().size();

	// The soonest possible trajectory beginning
	const SecTmReal tmAsync = arm.getContext().getTimer().elapsed() + arm.getTimeDeltaAsync();

	// Get the last sent trajectory point
	GenConfigspaceState s[2];
	if (arm.lookupCommand(s, s + 2, tmAsync, tmAsync) != s + 2)
		throw Message(Message::LEVEL_CRIT, "armMove(): arm pose query error");

	GenConfigspaceState begin = s[1].t < tmAsync ? s[1] : s[0];//s[1];
	begin.t = tmAsync;
	
	// Print the arm state
	//golem::armState(arm, begin);

	// Trajectories are simple 3-rd degree polynomials
	GenCoordTrj trajectories[CONFIG_SPACE_DIM];
	SecTmReal duration = arm.getTimeDelta();
	Real gain = REAL_ONE;
	bool bLimits = true;
	
	REPEAT:
	for (U32 i = 0; i < numOfJoints; i++) {
		const Joint &joint = *arm.getJoints()[i];
		const GenCoord b = begin.get(i);
		const GenCoord e = end.get(i);
		GenCoordTrj &t = trajectories[i];
		
		// create a trajectory
		t.set(Real(begin.t), Real(begin.t + duration), b, e);
		
		// check the target trajectory against velocity and acceleration limits
		if (bLimits) {
			Real tmin, tmax, min, max, g;
			
			t.getVelocityExtrema(tmin, tmax, min, max);
			g = Math::abs(min/(velocity*joint.getMin().vel));
			if (gain < g)
				gain = g;
			g = Math::abs(max/(velocity*joint.getMax().vel));
			if (gain < g)
				gain = g;

			t.getAccelerationExtrema(tmin, tmax, min, max);
			g = Math::sqrt(Math::abs(min/(acceleration*joint.getMin().acc)));
			if (gain < g)
				gain = g;
			g = Math::sqrt(Math::abs(max/(acceleration*joint.getMax().acc)));
			if (gain < g)
				gain = g;
		}
	}

	if (bLimits) {
		duration *= gain;
		bLimits = false;
		goto REPEAT;
	}
	
	// Split the whole trajectory into TimeDelta-length trajectories
	const U32 numOfTrj = (U32)Math::ceil(duration/arm.getTimeDelta());

	// Trajectory point
	GenConfigspaceState next;
	// Send the trajectory to the arm
	for (U32 j = 0; j <= numOfTrj; j++) {
		// Sample trajectory point in relative time window
		next.t = begin.t + SecTmReal(j)*arm.getTimeDelta();
		for (U32 i = 0; i < numOfJoints; i++)
			next.set(i, trajectories[i].get(Real(next.t)));

		// Send data (wait until buffer is empty)
		if (!arm.send(next))
			throw Message(Message::LEVEL_CRIT, "armMove(): send error");
	}
}

void golem::armRandMove(Arm& arm, Real velocity, Real acceleration) {
	static Rand rand(arm.getContext().getRandSeed());
	const U32 numOfJoints = (U32)arm.getJoints().size();

	// Generate trajectory end-point
	GenConfigspaceCoord end;
	for (U32 i = 0; i < numOfJoints; i++) {
		const Joint &joint = *arm.getJoints()[i];
		// Random joint position, zero final velocity and acceleration
		end.set(i, GenCoord(rand.nextUniform(joint.getMin().pos, joint.getMax().pos), REAL_ZERO, REAL_ZERO));
	}

	// Choose movement velocity and acceleration to 100%
	armMove(arm, end, velocity, acceleration);
}

void golem::armZeroMove(Arm& arm, Real velocity, Real acceleration, SecTmReal delay) {
	MessageStream *stream = arm.getContext().getMessageStream();

	// Get the current position of joints
	GenConfigspaceState begin;
	if (!arm.recv(begin))
		throw Message(Message::LEVEL_CRIT, "armZeroMove(): receive error");
	
	// Generate trajectory end-point
	GenConfigspaceCoord end;
	// Zero joint position, velocity and acceleration
	end.set(GenCoord(REAL_ZERO, REAL_ZERO, REAL_ZERO));

	// Choose movement velocity and acceleration to 100%
	stream->write(Message::LEVEL_INFO, "Moving to the ZERO pose");
	armMove(arm, end, velocity, acceleration); // zero pose
	armWatch(arm);

	arm.getContext().getTimer().sleep(delay);

	// Choose movement velocity and acceleration to 100%
	stream->write(Message::LEVEL_INFO, "Moving to the INITIAL pose");
	armMove(arm, begin, velocity, acceleration); // and back
	armWatch(arm);
}

void golem::armStop(Arm& arm) {
	// The soonest possible trajectory end
	const SecTmReal tmAsync = arm.getContext().getTimer().elapsed() + arm.getTimeDeltaAsync();
	
	GenConfigspaceState s[2];
	arm.lookupCommand(s, s + 2, tmAsync, tmAsync);
	
	// find the destination pose
	GenConfigspaceState end;
	// as soon as possible i.e.:
	end.t = std::min(s[0].t + arm.getTimeDelta(), tmAsync);
	// compute trajectory for each joint
	for (U32 j = 0; j < arm.getJoints().size(); j++) {
		golem::GenCoordTrj trj;
		// boundary conditions are: at t0 {pos, vel, acc}; at t1 {pos=?, vel=0, acc=?}
		trj.set2dvav(s[0].t, end.t, s[0].pos[j], s[0].vel[j], s[0].acc[j], Real(0.0));
		end.set(j, trj.get(end.t));
	}
	
	arm.send(end);
}

//------------------------------------------------------------------------------

void golem::fromCartesianPose(golem::WorkspaceCoord& pose, const golem::Vec3 &p, const golem::Vec3 &q) {
	pose.p = p;
	pose.R.setId();

	golem::Mat33 tmp;
	
	tmp.rotX(q.v1);
	pose.R.multiply(tmp, pose.R);
	
	tmp.rotY(q.v2);
	pose.R.multiply(tmp, pose.R);
	
	tmp.rotZ(q.v3);
	pose.R.multiply(tmp, pose.R);
}

//------------------------------------------------------------------------------

bool golem::waypointFromLineTrajectory(
	golem::GenWorkspaceState& waypoint,
	const golem::Profile& profile,
	const golem::WorkspaceCoord& begin,
	const golem::WorkspaceCoord& end,
	SecTmReal duration,
	SecTmReal t
)
{
	if (duration < numeric_const<SecTmReal>::EPS)
		return false;

	const Quat qbegin(begin.R), qend(end.R);
	
	Vec3 dir;
	dir.subtract(end.p, begin.p);
	const Real length = dir.normalise();
	if (length < REAL_EPS)
		return false;

	const Real durationNorm = profile.getDuration()/Real(duration);
	const Real lengthNorm = REAL_ONE/profile.getLength();

	// time
	waypoint.t = Math::clamp(t, SEC_TM_REAL_ZERO, duration);

	// pose
	const Real distance = profile.getDistance(waypoint.t*durationNorm)*lengthNorm;
	waypoint.pos.p.multiplyAdd(length*distance, dir, begin.p);
	Quat q;
	q.slerp(qbegin, qend, distance);
	waypoint.pos.R.fromQuat(q);

	// velocity
	const Real velocity = profile.getVelocity(waypoint.t*durationNorm)*lengthNorm;
	waypoint.vel.v.multiply(length*velocity/duration, dir);
	waypoint.vel.w.setZero(); // TODO
	// e.g. find angular difference dR between two poses at time t-1 and t+1 (R - rotation matrix):
	// dR*R(t-1) = R(t+1); dR = R(t+1)*R(t-1)^-1; vel(t) = {axis(dR), angle(dR)/(2.0*delta)}
	
	return true;
}

//------------------------------------------------------------------------------
