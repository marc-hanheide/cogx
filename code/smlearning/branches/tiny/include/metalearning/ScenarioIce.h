/** @file ScenarioIce.h
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
 * Ice Interface
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

#ifndef SMLEARNING_SCENARIOICE_H_
#define SMLEARNING_SCENARIOICE_H_
#define USING_ICE

// #include <controller/PhysReacPlanner.h>
// #include <controller/Katana.h>
// #include <controller/Simulator.h>
// #include <tools/Msg.h>
#include <system/Bounds.h>
#include <tools/Tools.h>
// #include <tools/Creator.h>
#include <tools/data_handling.h>
#include <tools/math_helpers.h>
#include <math.h>
#include <tiny/ice/Types.h>
#include <Ice/Ice.h>
#include <TinyIce/TinyIce.hh>
#include <tiny/ice/Desc.h>
using namespace golem::tinyice;


#include <iostream>

using namespace std;
// using namespace golem;
// using namespace golem::ctrl;
// using namespace golem::phys;
// using namespace golem::tools;


namespace smlearning {

#define MAX_PLANNER_TRIALS 50

typedef double Real;

///
///This class encapsulates objects, agents and general configuration
///of the learning scenario for the robot
///
class ScenarioIce : virtual public Ice::Application
{
public:
	///
	///constructor
	///
	ScenarioIce () {}

	///
	///The experiment performed in this method behaves as follows:
	///The arm randomly selects any of the possible actions.
	///Data are gathered and stored in a binary file for future use
	///with learning machines running offline learning experiments.
	///
	int run (int argc, char *argv[]);

	///
	///creates polyflap and puts it in the scene
	///
	void setupPolyflap(TinyPrx& pTiny, RigidBodyPrx& pObject, Vec3 position, Real rotationZ, Vec3 dimensions);

	///
	///creates a finger actor and sets bounds
	///
	void createFinger(JointPrx& pEffector, ArmPrx& pArm);

	///
	///Hack to solve a collision problem (don't know if it is still there):
	///Function that checks if arm hitted the polyflap while approaching it
	///
	bool checkPfPosition(RigidBodyPrx& pObject, const Mat34& refPos);

	///
	///calculate final pose according to the given direction angle
	///
	void setMovementAngle(const int angle, golem::ctrl::WorkspaceCoord& pose,const Real& distance,const Vec3& normVec,const Vec3& orthVec);

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
