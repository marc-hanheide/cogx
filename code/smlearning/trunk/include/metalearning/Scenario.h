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
#include <tools/data_structs.h>
#include <tools/math_helpers.h>

#include <boost/program_options.hpp>


using namespace std;
using namespace golem;
using namespace golem::tools;
namespace po = boost::program_options;


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
		//Real maxRange;
		/** assumed minimum value for polyflap Z-coordinate location during experiment (in xml file should have value of -0.01... bug in PhysX?) */
		Real minZ;
		/** assumed minimum value for polyflap X-coordinate location during experiment */
		Real minX;
		/** assumed minimum value for polyflap Y-coordinate location during experiment */
		Real minY;	
		/** assumed maximum value for polyflap Z-coordinate location during experiment */
		Real maxZ;
		/** assumed maximum value for polyflap X-coordinate location during experiment */
		Real maxX;
		/** assumed maximum value for polyflap Y-coordinate location during experiment */
		Real maxY;	
		
		/** minimal duration of a movement (by normal speed) */
		SecTmReal minDuration;
		/** Polyflap Position */
		Vec3 startPolyflapPosition;
		/** Polyflap orientation */
		Vec3 startPolyflapRotation; 
		/** Polyflap dimensions */
		Vec3 polyflapDimensions; 

//	 	//vertical distance from the ground
// 		const Real over = 0.01;
		/** vertical distance from the ground considering fingertip radius */
		Real over;
		/** distance from the front/back of the polyflap */
		Real dist; 
		/** distance from the side of the polyflap */
		Real side; 
		/** center of the polyflap */
		Real center; 
		/** distance from the top of the polyflap */
		//const Real top = polyflapDimensions.v2* 1.2;
		Real top; 
		/** lenght of the movement */
		Real distance;
		/** string containing the list of available starting positions */
		string startingPositionsConfig;
		
		
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


			maxX = 0.4;
			maxY = 0.4;
			maxZ = 0.4;
			minX = 0.0;
			minY = 0.0;
			minZ = -0.01;

			minDuration = 5.0;

			startPolyflapPosition.set(0.2, 0.2, 0.2);
			startPolyflapRotation.set(0.0, 0.0, 0.0);
			polyflapDimensions.set(0.1, 0.1, 0.1);

			over = 0.017;
			dist = 0.05;
			side = polyflapDimensions.v1*0.6;
			center = polyflapDimensions.v2*0.6;
			top = polyflapDimensions.v2 - 0.02;

			distance = 0.2;

			startingPositionsConfig = "1-18";



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

	/** const number of starting positions */
	static const int startingPositionsCount = 24;
	/** constant used for assertions (motorCommandVector size should be predefined) */
	static const int motorVectorSize = 5;
	/** constant used for assertions (featureVector size should be predefined) */
	static const int featureVectorSize = 12;
	/** constant used for assertions (polyflap feature vector size should be predefined) */
	static const int pfVectorSize = 6;
	/** constant used for assertions (effector feature vector size should be predefined) */
	static const int efVectorSize = 6;
	/** polyflap width */
	static const Real pfWidth = 0.002;

	/** Run experiment */
	///
	///The experiment performed in this method behaves as follows:
	///The arm randomly selects any of the possible actions.
	///Data are gathered and stored in a binary file for future use
	///with learning machines running offline learning experiments.
	///
	void run(int argc, char* argv[]);

	///
	///set experiment default values
	///
	//virtual void init(map<string, string> m);
	virtual void init(boost::program_options::variables_map vm);

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
 	DataSetStruct data;
	/** base file name for dataset */
	string dataFileName;
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
	Real horizontalAngle;
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
	/** list of starting positions obtained from config xml file */
	vector<int> availableStartingPositions;
	/** flag to decide storing labels (for pattern recognition) */
	bool storeLabels;
	/* vector logging used starting positions throughout the experiment */
	vector<double> usedStartingPositions;

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
	void set_movement_angle(const Real angle, golem::WorkspaceCoord& pose,const Real& distance,const Vec3& normVec,const Vec3& orthVec);

	///
	///select a random action
	///
	virtual void choose_action ();

	///
	///prepares the polyflap to use
	///
	virtual void initialize_polyflap();

	///
	///choose and describe the start point of the experiment trajectory
	///
	virtual void calculate_starting_position_coord ();

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
	void write_finger_pos_and_or(FeatureVector& featureVector, const Vec3& pos);

	///
	///write finger features to the vector
	///
	void write_finger_speed_and_angle(FeatureVector& featureVector, const int speed, const Real horizontalAngle);

	///
	///add the vector to the current sequence
	///
	void write_motor_vector_into_current_sequence();

	///
	///add the feature vector to the current sequence
	///
	void write_feature_vector_into_current_sequence(FeatureVector& featureVector);

	///
	///initialize learning data
	///
	void init_data();

	///
	///set the end of the experiment trajectory, initialize learning data and let the finger move along the experiment trajectory
	///
	void move_finger();

	///
	///storing a feature vector
	///
	void add_feature_vector (FeatureVector& currentFeatureVector, LearningData::Chunk& chunk);

	///
	///storing a label (in this case polyflap status)
	///
	void add_label (FeatureVector& currentFeatureVector, LearningData::Chunk& chunk);

	///
	///write vector sequence into current dataset
	///
	void write_current_sequence_into_dataset(DataSet& data);

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
	virtual void write_data ();

	
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
	void init_positionT(Vec3& pos);

	///
	///choose the starting position
	///
	virtual void define_start_position ();

	///
	///set the variable target so that it obtains the coordinates of the start point of the experiment trajectory
	///
	void prepare_target();

	///
	///select random angle (discrete or continouos)
	///
	Real choose_angle(Real min, Real max, string form);


	

};

//------------------------------------------------------------------------------

/** Reads/writes Scenario description from/to a given context */
bool XMLData(Scenario::Desc &val, golem::XMLContext* context);

//------------------------------------------------------------------------------

/** Application */
class PushingApplication : public golem::Application {
	


public:
	/** Main function */
	virtual int main(int argc, char *argv[]);

	virtual void define_program_options_desc();
	virtual int read_program_options(int argc, char *argv[]);

	boost::program_options::variables_map vm;

	
protected:

	boost::program_options::options_description prgOptDesc;
	boost::program_options::positional_options_description p;	

	virtual int start_experiment(char *argv[]);

	/** Runs Application */
	virtual void run(int argc, char *argv[]);



};


}; /* namespace smlearning */


//------------------------------------------------------------------------------



#endif /* SMLEARNING_SCENARIO_H_ */
