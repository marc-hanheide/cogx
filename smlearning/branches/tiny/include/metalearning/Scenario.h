/** @file Scenario.h
 * 
 * Learning scenario where the arm moves along a straight line
 * using reactive trajectory planner and collision detection
 * simulating a pushing action on a polyflap
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the arm simulators
 *
 * offline and active modes of learning are available
 *
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 * Copyright 2009      Sergio Roa
 *
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk/~msk">The University Of Birmingham</A>
 * @author      Jan Hanzelka - DFKI

   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.

 
 */

#ifndef SMLEARNING_SCENARIO_H_
#define SMLEARNING_SCENARIO_H_


#include <XMLParser.h>
#include <PhysReacPlanner.h>
#include <Katana.h>
#include <Simulator.h>
#include <Message.h>
#include <Tools/MsgTools.h>
#include <Tools/Tools.h>
#include <Creator.h>
#include <Math.h>
#include <XMLDataPhys.h>
#include <iostream>
#include <tools/data_handling.h>
#include <tools/math_helpers.h>

using namespace std;
using namespace golem;
using namespace golem::tools;


namespace smlearning {

#define MAX_PLANNER_TRIALS 50

///
///This class encapsulates objects, agents and general configuration
///of the learning scenario for the robot
///
class Scenario : protected Runnable
{
	Thread thread;
	int numSequences;
	int startingPosition;
	int argc;
	char **argv;
public:
	///
	///constructor
	///
	Scenario () {}

	///
	///The experiment performed in this method behaves as follows:
	///The arm randomly selects any of the possible actions.
	///Data are gathered and stored in a binary file for future use
	///with learning machines running offline learning experiments.
	///
	bool runSimulatedOfflineExperiment (int argc, char *argv[], int numSequences = 100, int startingPosition = 0);

	virtual void run ();

	///
	///creates objects (groundplane, polyflap)
	///
	void setupSimulatedObjects(Scene &scene, golem::Context &context);

	///
	///creates polyflap and puts it in the scene
	///
	Actor* setupPolyflap(Scene &scene, Vec3 position, Vec3 rotation, Vec3 dimensions, golem::Context &context);

	///
	///creates a finger actor and sets bounds
	///
	void createFinger(std::vector<Bounds::Desc::Ptr> &bounds, Mat34 &referencePose, const Mat34 &pose, MemoryStream &buffer);
	///
	///Adding bounds to an actor
	///
	void addBounds(Actor* pActor, std::vector<const Bounds*> &boundsSeq, const std::vector<Bounds::Desc::Ptr> &boundsDescSeq);

	///
	///Hack to solve a collision problem (don't know if it is still there):
	///Function that checks if arm hitted the polyflap while approaching it
	///
	bool checkPfPosition(const Actor* polyFlapActor, const Mat34& refPos);

	///
	///calculate final pose according to the given direction angle
	///
	void setMovementAngle(const int angle, golem::WorkspaceCoord& pose,const Real& distance,const Vec3& normVec,const Vec3& orthVec);

	///
	///calculate position to direct the arm given parameters set in the learning scenario
	///
	void setPointCoordinates(Vec3& position, const Vec3& normalVec, const Vec3& orthogonalVec, const Real& spacing, const Real& horizontal, const Real& vertical);

	///
	///calls setPointCoordinates for a discrete number of different actions
	///
	void setCoordinatesIntoTarget(const int startPosition, Vec3& positionT,const Vec3& polyflapNormalVec, const Vec3& polyflapOrthogonalVec,const Real& dist, const Real& side, const Real& center, const Real& top, const Real& over);

	
};


}; /* namespace smlearning */




#endif /* SMLEARNING_SCENARIO_H_ */
