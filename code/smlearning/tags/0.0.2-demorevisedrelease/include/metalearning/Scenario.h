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
		/** assumed minimum value for polyflap Z-coordinate location during experiment (in xml file should have value of -0.01... bug in PhysX?) */
		Real minZ;
		
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
	/** polyflap position (used to compute starting point coordinates) */
	Vec3 polyflapPosition;
	/** normal vector of the polyflap  |_ --> */
	Vec3 polyflapNormalVec;
	/** orthogonal vector of the polyflap */
	Vec3 polyflapOrthogonalVec;
	/** coordinates vector used for target description */
	Vec3 positionT;
	/** rotations vector used for target description */
	Vec3 orientationT;
	/** workspace state used to describe the starting point and also the end point of trajectory */
	golem::GenWorkspaceState target;
	/** position chosen for the start of experiment trajectory */
	int startPosition;
	/** speed of movement along the experiment trajectory */
	int speed;
	/** minimal duration of the movement along the experiment trajectory */
	SecTmReal duration;
	/** pose describing the endpoint of the experiment trajectory */
	WorkspaceCoord end;
	/** chosen horizontal angle of the experiment trajectory */
	int horizontalAngle;
	/** number of collected sequences in one experiment session */
	int numSequences;
	/** asynchron time delta*/
	SecTmReal tmDeltaAsync;
	/** spacestate describing the initial position of the arm (first and last position of the arm in every experiment session) */
	golem::GenConfigspaceState initial;
	/** workspace state describing the home position (first and last position of every experiment iteration) */
	GenWorkspaceState home;
	/** help variable used to determine the startPosition variable */
	int startingPosition;
	/** pointer to bounds describing polyflap, used for some initial polyflap computations */
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
	Actor* setup_polyflap(/*Scene &scene, Vec3 position, Vec3 rotation, Vec3 dimensions, golem::Context &context*/);
	
	///
	///creates polyflap and puts it in the scene (from pose)
	///
	Actor* setup_polyflap(Scene &scene, Mat34& globalPose, Vec3 dimensions);
	
	///
	///Hack to solve a collision problem (don't know if it is still there):
	///Function that checks if arm hitted the polyflap while approaching it
	///
	bool check_pf_position(const Actor* polyFlapActor, const Mat34& refPos);

	///
	///calculate final pose according to the given direction angle
	///
	void set_movement_angle(const int angle, golem::WorkspaceCoord& pose,const Real& distance,const Vec3& normVec,const Vec3& orthVec);

	///
	///calculate position to direct the arm given parameters set in the learning scenario
	///
	static void set_point_coordinates(Vec3& position, const Vec3& normalVec, const Vec3& orthogonalVec, const Real& spacing, const Real& horizontal, const Real& vertical);

	///
	///calls setPointCoordinates for a discrete canonical number of different actions
	///
	static void set_coordinates_into_target(const int startPosition, Vec3& positionT,const Vec3& polyflapNormalVec, const Vec3& polyflapOrthogonalVec,const Real& dist, const Real& side, const Real& center, const Real& top, const Real& over);




	///
	///prepares the polyflap to use
	///
	virtual void initialize_polyflap();

	///
	///choose and describe the start point of the experiment trajectory
	///
	virtual void  initialize_movement();

	///
	///describe the experiment trajectory
	///
	void set_up_movement();

	///
	///initialize the experiment
	///
	void first_init();

	///
	///describe the home position (position, where the finger starts and ends every iteration)
	///
	void setup_home();

	///
	///describe the lenght of experiment (number of sequences) and if given, the starting position
	///
	void setup_loop(int argc, char* argv[]);

	///
	///try to find a path to given position, if found, move the finegr along it and wait for it to stop
	///
	void send_position(golem::GenWorkspaceState position, golem::ReacPlanner::Action action);

	///
	///create feature vector and sequence
	///
	void init_writing();

	///
	///write finger features to the vector
	///
	void write_finger_pos_and_or();

	///
	///write finger features to the vector
	///
	void write_finger_speed_and_angle();

	///
	///add the vector to the current sequence
	///
	void write_motor_vector_into_sequence();

	///
	///initialize learning data
	///
	void init_data();

	///
	///set the end of the experiment trajectory, initialize learning data and let the finger move along the experiment trajectory
	///
	void move_finger();

	///
	///write vector sequence into current dataset
	///
	void write_sequence_into_dataset();

	///
	///turn the finger collision detection on (true) or off (false)
	///
	void set_collision_detection(bool b);

	///
	///print out desired sequenc information
	///
	void print_sequence_info();

	///
	///move finger up in order to increase the chances of finding a suitable path to home position
	///
	void move_finger_up();

	///
	///remove the polyflap object from the scene
	///
	void remove_polyflap();

	///
	///describe the movement to home position
	///
	void prepare_home_movement();

	///
	///print out desired information at the end of a sequence
	///
	void iteration_end_info();
	
	///
	///finish current iteration
	///
	void check_interrupted();

	///
	///move the arm to its initial position
	///
	void move_to_initial();

	///
	///write obtained dataset into a binary file
	///
	virtual void write_dataset_into_binary();

	
	///
	///creates the polyflap object, obtains bounds and determines current rotation of the polyflap
	///
	void create_polyflap_object();

	///
	///computes normal and orthogonal vector of the polyflap and determines the position of the polyflap
	///
	void compute_vectors();

	///
	///set current position of the polyflap as default position for computing of the starting position
	///
	void set_positionT();

	///
	///choose the starting position
	///
	virtual void define_start_position();

	///
	///set the variable target so that it obtains the coordinates of the start point of the experiment trajectory
	///
	void prepare_target();

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