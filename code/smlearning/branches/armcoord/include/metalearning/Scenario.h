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

#pragma once
#ifndef SMLEARNING_SCENARIO_H_
#define SMLEARNING_SCENARIO_H_

#include <Golem/Application.h>
#include <XMLParser.h>
#include <PhysReacPlanner.h>
#include <Katana.h>
#include <Simulator.h>
#include <Message.h>
#include <Tools/MsgTools.h>
#include <Tools/Tools.h>
#include <Creator.h>
#include <Math.h>
#include <XMLDataCtrl.h>
#include <XMLDataPhys.h>
#include <iostream>
#include <tools/data_handling.h>
#include <tools/math_helpers.h>

//#include <tr1/tuple>


using namespace std;
using namespace golem;
using namespace golem::tools;
//using namespace std::tr1;


namespace smlearning {

#define MAX_PLANNER_TRIALS 50


//------------------------------------------------------------------------------

///
///This class encapsulates objects, agents and general configuration
///of the learning scenario for the robot
///
class Scenario : public golem::Object
{
public:


	/** Just Interrupted */
	class Interrupted {};
	
	/** Object description */
	class Desc : public golem::Object::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC(Scenario, golem::Object::Ptr, golem::Scene)

	public:
		/** Arm */
		golem::PhysReacPlanner::Desc armDesc;
		/** Effector bounds group */
		golem::U32 effectorGroup;
		/** Finger */
		golem::Bounds::Desc::Seq fingerDesc;
		/** Local end-effector reference pose */
		golem::WorkspaceCoord referencePose;
		/** Global end-effector home pose */
		golem::WorkspaceCoord homePose;
		/** Motion effects stabilization time period */
		golem::SecTmReal speriod;
		/** 2D motion constraint */
		bool motion2D;

		/** assumed maximum range of polyflap X-coordinate location during the experiment */
		Real maxRange;
		
		//minimal duration of a movement (by normal speed)
		SecTmReal minDuration;
		//Polyflap Position and orientation
		Vec3 startPolyflapPosition; 
		Vec3 startPolyflapRotation; 
		//Polyflap dimensions		
		Vec3 polyflapDimensions; 

//	 	//vertical distance from the ground
// 		const Real over = 0.01;
		//vertical distance from the ground considering fingertip radius
		Real over;
		//distance from the front/back of the polyflap
		Real dist; 
		//distance from the side of the polyflap
		Real side; 
		//center of the polyflap
		Real center; 
		//distance from the top of the polyflap
		//const Real top = polyflapDimensions.v2* 1.2;
		Real top; 
		//lenght of the movement		
		Real distance; 

		
		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			golem::Object::Desc::setToDefault();
			
			// default arm description
			armDesc.setToDefault();
			armDesc.pArmDesc.reset(new golem::KatSimArm::Desc);
			armDesc.pPlannerDesc->pHeuristicDesc->distJointcoordMax[4] = golem::Real(1.0)*golem::REAL_PI; // last joint of Katana
			// Effector group
			effectorGroup = 0x4;
			// finger setup
			fingerDesc.clear();
			// end-effector reference pose
			referencePose.setId();
			// end-effector home pose
			homePose.R.rotX(-0.5*golem::REAL_PI); // end-effector pointing downwards
			homePose.p.set(golem::Real(0.0), golem::Real(0.1), golem::Real(0.1));
			// Other stuff
			motion2D = false;
			speriod = 1.0;
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!golem::Object::Desc::isValid())
				return false;
			if (!armDesc.isValid() || !referencePose.isFinite() || !homePose.isFinite())
				return false;
			for (golem::Bounds::Desc::Seq::const_iterator i = fingerDesc.begin(); i != fingerDesc.end(); i++)
				if (!(*i)->isValid())
					return false;
			
			return true;
		}
	};

	/** Run experiment */
	///
	///The experiment performed in this method behaves as follows:
	///The arm randomly selects any of the possible actions.
	///Data are gathered and stored in a binary file for future use
	///with learning machines running offline learning experiments.
	///
	void run(int argc, char* argv[]);

	friend map<Vec3, int, compare_Vec3> get_canonical_positions ();
protected:
	/** Description */
	Desc desc;
	/** Arm */
	golem::PhysReacPlanner* arm;
	/** End-effector */
	golem::JointActor* effector;
	/** End-effector bounds */
	golem::Bounds::Seq effectorBounds;
	/** Object */
	golem::Actor* object;
	/** Obstacles */
	golem::Actor* obstacles;
	/** default polyflap bounds */
	golem::Bounds::SeqPtr objectLocalBounds;
	Actor::Appearance appearance;
	/** Trial data */
	LearningData learningData;
	/** Dataset */
 	DataSet data;
	/** Time */
	golem::SecTmReal trialTime;
	/** Random number generator */
	golem::Rand randomG;
	/** Y (roll) angle of polyflap */
	Real currentPfRoll;
	/** X (pitch) angle of polyflap */
	Real currentPfPitch;
	/** Z (yaw) angle of polyflap */
	Real currentPfYaw;
	/** Y position of polyflap */
	Real currentPfY;
///
	Vec3 polyflapPosition;
	Vec3 polyflapNormalVec;
	Vec3 polyflapOrthogonalVec;
	Vec3 positionT;
	golem::GenWorkspaceState target;
	Vec3 orientationT;
	int startPosition;
	int speed;
	SecTmReal duration;
	WorkspaceCoord end;
	int horizontalAngle;
	int numSequences;

	SecTmReal tmDeltaAsync;
	golem::GenConfigspaceState initial;
	GenWorkspaceState home;
	int startingPosition;
	golem::Bounds::SeqPtr curPol;

	// Real reachedAngle;
	
	/** iteration counter */
	int iteration;
	/** const number of SM regions */
	static const int smregionsCount = 18;
	/** threshold angle when polyflap is flipping over */

	/** Creator */
	golem::Creator creator;
	/** Synchronization objects */
	golem::CriticalSection cs;
	golem::Event ev;
	volatile bool bStart, bStop, bRec;



	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);
	/** Creates Scenario from description. */
	bool create(const Scenario::Desc& desc);
	/** Releases resources */
	virtual void release();
	/** Objects can be constructed only in the Scene context. */
	Scenario(golem::Scene &scene);

	///
	///creates polyflap and puts it in the scene (from position and rotation)
	///
	Actor* setupPolyflap(/*Scene &scene, Vec3 position, Vec3 rotation, Vec3 dimensions, golem::Context &context*/);
	
	///
	///creates polyflap and puts it in the scene (from pose)
	///
	Actor* setupPolyflap(Scene &scene, Mat34& globalPose, Vec3 dimensions);
	
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
	static void setPointCoordinates(Vec3& position, const Vec3& normalVec, const Vec3& orthogonalVec, const Real& spacing, const Real& horizontal, const Real& vertical);

	///
	///calls setPointCoordinates for a discrete canonical number of different actions
	///
	static void setCoordinatesIntoTarget(const int startPosition, Vec3& positionT,const Vec3& polyflapNormalVec, const Vec3& polyflapOrthogonalVec,const Real& dist, const Real& side, const Real& center, const Real& top, const Real& over);





	void initializePolyflap();

	void  initializeMovement();

	
	void setUpMovement();

	void firstInit();

	void setupHome();

	void setupLoop(int argc, char* argv[]);

	void sendPosition(golem::GenWorkspaceState position, golem::ReacPlanner::Action action);

	void initWriting();

	void writePosAndOr();

	void writeSpeedAndAngle();

	void writeVectorIntoSequence();

	void initData();

	void moveFinger();

	void writeSequenceIntoDataset();

	void setCollisionDetection(bool b);

	void printSequenceInfo();

	void moveFingerUp();

	void removePolyflap();

	void pepareHomeMovement();

	void iterationEndInfo();

	void finishIteration();

	void moveToInitial();

	void writeDatasetIntoBinary();

	void createPolyflapObject();

	void computeVectors();

	void setPositionT();

	void defineStartPosition();

	void prepareTarget();

};

//------------------------------------------------------------------------------

/** Reads/writes Scenario description from/to a given context */
bool XMLData(Scenario::Desc &val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

/** Application */
class PushingApplication : public golem::Application {
	
protected:
	/** Runs Application */
	virtual void run(int argc, char *argv[]);
};


}; /* namespace smlearning */


//------------------------------------------------------------------------------



#endif /* SMLEARNING_SCENARIO_H_ */
