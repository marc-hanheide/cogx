/** @file Tools.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_TOOLS_H_
#define _GOLEM_DEMO_COMMON_TOOLS_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Arm.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

char* strdup(const char* src);

char* strndup(const char* src, size_t n);

char* istreamstrdup(std::istream& istr, char delim = '\n');

//------------------------------------------------------------------------------

/* Displays most important arm parameters. 
* @param arm			Arm interface
*/
void armInfo(Arm &arm);

/* Displays arm state (time, pose, pose velocity, pose acceleration).
* @param arm			Arm interface
* @param state			state to be displayed
*/
void armState(Arm &arm, const GenConfigspaceState &state);

/* Displays arm pose + time.
* @param arm			Arm interface
* @param state			state to be displayed
*/
void armPose(Arm &arm, const GenConfigspaceState &state);

/* Displays a sequence of moving arm poses until given time stamp.
* @param arm			Arm interface
* @param until			time stamp
*/
void armWatch(Arm &arm);

/* Moves the arm to the specified target pose given maximum velocity and acceleration.
* The function does not perform collison detection.
* @param arm			Arm interface
* @param end			end/target arm pose
* @param velocity		desired normalised maximal velocity related to the joints limits [0, 1]
* @param acceleration	desired normalised maximal acceleration related to the joints limits [0, 1]
*/
void armMove(Arm &arm, const GenConfigspaceCoord &end, Real velocity = Real(1.0), Real acceleration = Real(1.0));

/* Moves the arm to a randomly generated target pose.
* The function does not perform collison detection - DO NOT use it with real arms!.
* @param velocity		desired normalised maximal velocity related to the joints limits [0, 1]
* @param acceleration	desired normalised maximal acceleration related to the joints limits [0, 1]
* @param arm			Arm interface
*/
void armRandMove(Arm &arm, Real velocity = Real(1.0), Real acceleration = Real(1.0));

/* Moves the arm to the zero pose and back.
* The function does not perform collison detection.
* @param arm			Arm interface
* @param velocity		desired normalised maximal velocity related to the joints limits [0, 1]
* @param acceleration	desired normalised maximal acceleration related to the joints limits [0, 1]
* @param delay			time gap between movements
*/
void armZeroMove(Arm &arm, Real velocity = Real(1.0), Real acceleration = Real(1.0), SecTmReal delay = SEC_TM_REAL_ZERO);

/* Stops the arm.
* The function does not perform collison detection.
* @param arm			Arm interface
*/
void armStop(Arm& arm);

//------------------------------------------------------------------------------

/* Computes pose from given position and Euler angles.
* @param pose			computed pose
* @param p				XYZ position
* @param q				XYZ Euler angles
*/
void fromCartesianPose(golem::WorkspaceCoord& pose, const golem::Vec3 &p, const golem::Vec3 &q);

//------------------------------------------------------------------------------

/* Computes a waypoint at a given time on a straight line trajectory of a given profile and duration.
* @param waypoint		computed waypoint
* @param profile		trajectory profile
* @param begin			trajectory begin: begin != end
* @param end			trajectory end: begin != end
* @param duration		trajectory duration: duration > 0
* @param t				trajectory interpolated waypoint: t = <0, duration>
* @return				TRUE if succeded, FALSE otherwise
*/
bool waypointFromLineTrajectory(
	golem::GenWorkspaceState& waypoint,
	const golem::Profile& profile,
	const golem::WorkspaceCoord& begin,
	const golem::WorkspaceCoord& end,
	SecTmReal duration,
	SecTmReal t
);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_TOOLS_H_*/
