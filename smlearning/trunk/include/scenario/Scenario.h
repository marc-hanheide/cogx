/** @file Scenario.h
 * 
 * Learning scenario where the arm moves along a straight line
 * using reactive trajectory planner and collision detection
 * simulating a pushing action on an arbitrary given object.
 * That object can either be a simulated or a real one.
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the arm simulators
 *
 * offline and active modes of learning are available
 *
 * @author	Sergio Roa (DFKI)
 *
 * @version 2.0 beta
 *
 * Copyright 2009      Sergio Roa
 *
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk/~msk">The University Of Birmingham</A>
 * @author      Jan Hanzelka - DFKI
 * @author	Manuel Noll (DFKI)

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

#include <boost/iostreams/tee.hpp>
#include <boost/iostreams/stream.hpp>
#include <Golem/Tools/XMLParser.h>
#include <Golem/Device/Katana/Katana.h>
#include <Golem/Tools/Message.h>
#include <Golem/Tools/Msg.h>
#include <Golem/Demo/Common/Tools.h>
#include <Golem/PhysCtrl/Creator.h>
#include <Golem/Math/Math.h>
#include <Golem/Tools/XMLData.h>

#include <iostream>
#include <metalearning/data_structs.h>
#include <smltools/math_helpers.h>
#include <boost/program_options.hpp>
#include <scenario/ActorObject.h>
#include <scenario/ArmActor.h>

// using namespace std;
// using namespace golem;
namespace po = boost::program_options;


namespace smlearning {

#define MAX_PLANNER_TRIALS 50

class Scenario : public golem::Object
{
public:


	/** Just Interrupted */
	class Interrupted {};
	
	/** Object description */
	class Desc : public golem::Object::Desc {

	public:		
		/** Constructs description object */
		Desc() 
		{
			Desc::setToDefault();
		}
		/** description(configuration) of the golem arm */
		ArmActor::Desc descArmActor;
		/** description of an arbitrary actor that will be manipulated by the golem arm */
		ActorObject::Desc descActorObject;
		/** Checks if the description is valid. */
		virtual bool isValid() const 
		{
			return (golem::Object::Desc::isValid() && descArmActor.isValid() && 								descActorObject.isValid() );
		}
		void setToDefault();

		LearningData::FeaturesLimits featLimits;
					
		/** string containing the list of available starting positions */
		string startingPositionsConfig;
		/** lenght of the movement */
		Real distance;
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Scenario, golem::Object::Ptr, golem::Scene&)
		
	};

	/** cto, such that Scenario can only be created with(in) a scene (context) [requirement of golem]*/
	Scenario(golem::Scene&);
	/** destructor */
	~Scenario ();
	/** set experiment default values */
	virtual void init(boost::program_options::variables_map vm);
	/** remove the actor object from the scene */
	void removeObject();
	/** Run experiment */
	void run(int argc, char* argv[]);
	/** sets the actor to be used */
	void setActorObject(ConcreteActor*);
	/** get class name */
	static string getName () { return "Scenario"; }
			
protected:
	/** const number of starting positions */
	static const int startingPositionsCount = 24;
	/** Releases resources */
	virtual void release();
	/** calculate the start coordinates of the arm */
	virtual void calculateStartCoordinates();
	/** select a random action */
	virtual void chooseAction ();
	/** select random angle (discrete or continouos) */
	Real chooseAngle(const Real&, const Real&, const bool& =true) const;
	/** Creates Scenario from description. */
	bool create(const Scenario::Desc& desc);
	/** Describe the experiment trajectory */
	virtual void initMovement();
	/** write data chunk (used in postprocess function) */
	void writeChunk (LearningData::Chunk& chunk);
	/** (Post)processing function called AFTER every physics simulation step and before rendering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);
	/** calculate final pose according to the given direction angle */
	void setMovementAngle(const Real, golem::WorkspaceCoord&,const Real&,const Vec3&,const Vec3&);
	/** write obtained dataset into a binary file */
	void writeData ();

	//NOTE: variables are alphabetically ordered
	/** the main actor : the Golem arm himself */
	ArmActor* 		arm;
	/** list of starting positions obtained from config xml file */
	std::vector<int>	availableStartingPositions;
	/** Creator */
	golem::Creator 		creator;
	/** Dataset */
	LearningData::DataSet 	data;
	/** base file name for dataset */
	std::string		dataFileName;
	/** description instance of the scenario */
	Scenario::Desc 		desc;
	/** minimal duration of the movement along the experiment trajectory */
	SecTmReal 		duration;
	/** pose describing the endpoint of the experiment trajectory */
	WorkspaceCoord 		end;
	/** Ground plane */
	golem::Actor* 		groundPlane;	
	/** Trial data */
	LearningData 		learningData;		
	/** number of collected sequences in one experiment session */
	int 			numSequences;
	/** and the guest star : the object better known as the thing */
	ActorObject* 		object;
	/** orientation of the object */
	Vec3			orientationTarget;
	/** coordinates vector used for target description */
	Vec3 			positionTarget;
	/** Random number generator */
	golem::Rand 		randomG;
	/** position chosen for the start of experiment trajectory */
	int 			startPosition;
	/** help varibale used to determine the startPosition variable */
	int 			startingPosition;
	// /** flag to decide storing labels (for pattern recognition) */
	// bool 			storeLabels;
	/** workspace state used to describe the starting point and also the end point of trajectory */
	golem::GenWorkspaceState target;
	/** vector logging used starting positions throughout the experiment */
	vector<double> 		usedStartingPositions;
	/** initialize the experiment */
	void _init();
	/** object (e.g. Polyflap)*/
	ConcreteActor* _concreteActor;

	/** Synchronization objects */
	golem::CriticalSection cs;
	volatile bool bStart;
	volatile bool bStop;
	volatile bool bRec;			

			

};

}; // namespace smlearning 


#endif // SMLEARNING_SCENARIO_H_ 
