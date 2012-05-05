/** @file Scenario.cpp
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

#include <scenario/Scenario.h>

namespace smlearning {

///////// Public //////////
/** \brief cto, such that Scenario can only be created with(in) a scene (context) [requirement of golem]
*
* \param scene is the context within the Scenario is created
*/
Scenario::Scenario(golem::Scene &scene) : Object(scene), creator(scene) {
	arm = NULL;
	object = NULL;
	groundPlane = NULL;
	bStart = bStop = bRec = false;
}

/** \brief Destructor
 */
Scenario::~Scenario ()
{
	if (arm != NULL)
		delete arm;
	if (_concreteActor != NULL)
		delete _concreteActor;
	
}


void Scenario::release() {
	if (arm->getArm() != NULL)
		scene.releaseObject(*arm->getArm());
	if (groundPlane != NULL)
		scene.releaseObject(*groundPlane);
}

///////// Public //////////
/**  \brief set experiment default values such as the number of sequences, the starting pos and whether to store labels or not
*/
void Scenario::init(boost::program_options::variables_map vm)
{
	numSequences = 10000;
	startingPosition = 0;
	// storeLabels = false;
	
	if (vm.count("numSequences")) {
		numSequences = atoi(vm["numSequences"].as<string>().c_str());
	}

	if (vm.count("startingPosition")) {
		startingPosition = vm["startingPosition"].as<int>();
	}

	// if (vm.count("storeLabels")) {
	// 	storeLabels = true;
	// }

	dataFileName = get_base_filename_from_time ();

	availableStartingPositions = parse_startingPositions(desc.startingPositionsConfig, startingPositionsCount);
	learningData.setToDefault(desc.featLimits);

}

///////// Public //////////
/** \brief remove the object from the scene
*/
void Scenario::removeObject(){
	// remove object
	scene.releaseObject(*object);
	object = NULL;
}

///
///write data chunk (used in postprocess function)
///
void Scenario::writeChunk (LearningData::Chunk& chunk) {
	//arm->getArm().lookupInp(chunk.action.armState, context.getTimer().elapsed()); //not present anymore
	arm->getArm()->getArm().lookupState(chunk.action.armState, context.getTimer().elapsed()); //other possibility: lookupCommand
	chunk.action.effectorPose = arm->getEffector()->getPose();
	chunk.action.effectorPose.multiply (chunk.action.effectorPose, arm->getEffectorBounds().at(1)->getPose());
	chunk.action.effectorPose.R.toEuler (chunk.action.efRoll, chunk.action.efPitch, chunk.action.efYaw);
	chunk.action.horizontalAngle = arm->getHorizontalAngle ();
	chunk.action.pushDuration = arm->getPushDuration();
	chunk.action.endEffectorPose = end;
	chunk.action.endEffectorPose.R.toEuler (chunk.action.endEfRoll, chunk.action.endEfPitch, chunk.action.endEfYaw);

	chunk.object.objectPose = object->getPose();
	chunk.object.objectPose.R.toEuler (chunk.object.obRoll, chunk.object.obPitch, chunk.object.obYaw);

	// golem::Mat34 p = chunk.object.objectPose; 
	// golem::Mat34 p = chunk.action.effectorPose; 
	// golem::Mat34 p = chunk.action.endEffectorPose;
	// cout << "pose: ";

	// cout << p.p.v1 << " " << p.p.v2 << " " << p.p.v3 << " " << chunk.action.endEfRoll << " " << chunk.action.endEfPitch << " " << chunk.action.endEfYaw << endl;

	// if (storeLabels) add_label (chunk);
	// trialTime += SecTmReal(1.0)/universe.getRenderFrameRate();

}


void Scenario::postprocess(SecTmReal elapsedTime) {
	if (bStart) {
		CriticalSectionWrapper csw(cs);
		if (object == NULL) {
			return;
		}

		LearningData::Chunk chunk;
		writeChunk (chunk);

		//LearningData::write_chunk_to_featvector (chunk.featureVector, chunk, normalize<Real>, learningData.featLimits);
		
		// 		learningData.data.push_back(chunk);
			
		learningData.currentChunkSeq.push_back (chunk);

		// currentPfRoll = chunk.object.obRoll;
		// currentPfY = chunk.object.objectPose.p.v2;
	}
}

///////// Public //////////
/** \brief starts the run of the experiment
*
* The experiment performed in this method behaves as follows:
* The arm randomly selects any of the possible actions.
* Data are gathered and stored in a binary file for future use
* with learning machines running offline learning experiments.
*
*/
void Scenario::run(int argc, char* argv[]) {

	//set: random seed;init arm; get initial config
	std::cout <<"_init "<<std::endl;
	_init();
	std::cout <<"starting loop "<<std::endl;
	//start of the experiment loop
	for (int iteration = 0; iteration<numSequences; iteration++) {
		
		// experiment main loops
		creator.setToDefault();
		//create and setup object, compute its vectors
		std::cout <<"creating object "<<std::endl;
		object = dynamic_cast<ActorObject*>(scene.createObject(desc.descActorObject));
		object->setShape(scene,_concreteActor);
		std::cout <<"choose action "<<std::endl;
		chooseAction ();						//select a random action		
		std::cout <<"calculate starting coordinates "<<std::endl;
		calculateStartCoordinates ();					//compute coordinates of start position
		std::cout << "sending position ";
		cout << Real(target.pos.p.v1) << " " << Real(target.pos.p.v2) << " " << Real(target.pos.p.v3) << endl;
		arm->sendPosition(context,target , ReacPlanner::ACTION_GLOBAL);	//move the finger to the beginnnig of experiment trajectory
		learningData.currentChunkSeq.clear();				//create sequence for this loop run
		std::cout << "init movement"<<std::endl;
		initMovement(); 						//compute direction and other features of trajectory
		std::cout << "move finger"<<std::endl;
		arm->moveFinger(context,target, bStart,duration,end);		//move the finger along described experiment trajectory
		data.push_back(learningData.currentChunkSeq);			//write sequence into dataset
		arm->setCollisionDetection(false);				//turn off collision detection
		cout << "sequence size: " << learningData.currentChunkSeq.size() << endl;//print out current seq info
		arm->moveFingerUp(context,target, object->getDescription().dimensions);	//move the finger up to avoid path finding problems 
		removeObject();							//remove polyflap object from the scene
		arm->moveFingerToStartPose(context);				//move finger to initial position	
		context.getMessageStream()->write(Message::LEVEL_INFO, "Done");
		cout << "Iteration " << iteration << " completed!" << endl; 	//print out end information of this iteration
		if (universe.interrupted()) throw Interrupted();		//finish this iteration

	}

	arm->moveArmToStartPose(context); 	//move the arm to its initial position 
	writeData ();				//write obtained data into a binary file


}

///////// Public //////////
/** \brief sets the actor to be used 
*
* \param concreteActor is a pointer to a concrete actor that is used as object shape
*/
void Scenario::setActorObject(ConcreteActor* concreteActor)
{
	_concreteActor = concreteActor;
}

///////// Protected //////////
/** \brief calculate the start coordinates of the arm
*/
void Scenario::calculateStartCoordinates()
{

	ActorObject::Desc tmpDesc 	= object->getDescription();
	Vec3 position 			= object->getPosition();
	Vec3 normal			= object->getNormalVec();
	Vec3 orthogonal			= object->getOrthogonalVec();
	Vec3 orientation		= object->getOrientation();

	positionTarget.set(Real(position.v1), Real(position.v2), Real(position.v3));

	set_coordinates_into_target(startPosition, positionTarget, normal, orthogonal, tmpDesc.dist, tmpDesc.side, tmpDesc.center, tmpDesc.top, tmpDesc.over);

	cout << "Position " << startPosition-1 << endl;
	
	// and set target waypoint
	fromCartesianPose(target.pos, positionTarget, orientationTarget);
	// cout << "target pos: " << target.pos.p.v1 << ", " << target.pos.p.v2 << ", " << target.pos.p.v3 << endl;
	target.vel.setId(); // it doesn't move
	
	target.t = context.getTimer().elapsed() + arm->getDeltaAsync() + desc.descArmActor.minDuration; // i.e. the movement will last at least 5 sec

}


///////// Protected //////////
/** \brief select a random action
*/
void Scenario::chooseAction () {
	//choose starting position

	if (startingPosition == 0)
		startPosition = availableStartingPositions[floor(randomG.nextUniform (0.0,Real(availableStartingPositions.size())))];
	else
		startPosition = startingPosition;
	

	assert (startPosition >= 1 && startPosition <= startingPositionsCount);

	usedStartingPositions.push_back(startPosition);
	
	arm->setPushDuration(3);
	
	arm->setHorizontalAngle(chooseAngle(Real(60.0), Real(120.0)));


}

///////// Protected //////////
/** \brief select random angle (discrete or continouos)
*/
Real Scenario::chooseAngle(const Real& min,const Real& max,const bool& continuous) const
{
	Real res;
	if (!continuous)
		res = floor(randomG.nextUniform (min, max));
	else 
		res = randomG.nextUniform (min, max);
	return res;
}


///////// Protected //////////
/** \brief Creates Scenario from description. 
*
* \param desc contains the description of the scenerie and the objects within, e.g. golem arm and actor
*/
bool Scenario::create(const Scenario::Desc& desc) {
	if (!desc.isValid()) {
		context.getMessageStream()->write(Message::LEVEL_CRIT, "Scenario::create(): invalid description");
		return false;
	}
	this->desc = desc;

	return ( !((groundPlane = dynamic_cast<Actor*>(scene.createObject(*creator.createGroundPlaneDesc()))) == NULL)  &&
 (arm = new ArmActor(desc.descArmActor, scene))!=NULL );
		// !((arm = dynamic_cast<ArmActor*> (scene.createObject(desc.descArmActor))) == NULL) );
		//instead of using the scene.createObject one should use an ArmActor procedure
}

///////// Protected //////////
/** \brief Describe the experiment trajectory
*
*/
void Scenario::initMovement()
{
	// Trajectory duration calculated from a speed constant.

	cout << "push duration: " << arm->getPushDuration() << endl;		
	duration 		= arm->getPushDuration();
	// Trajectory end pose equals begin + shift along Y axis
	end 			= target.pos;	

	Vec3 normal 		= object->getNormalVec();
	Vec3 orthogonal 	= object->getOrthogonalVec();
	Vec3 position		= object->getPosition();

	Vec3 centerNormalVec =
		computeNormalVector(
				    Vec3 (positionTarget.v1, positionTarget.v2, positionTarget.v3),
				    Vec3 (position.v1, position.v2, object->getDescription().dimensions.v2*0.5)
				    );
	//and it's orthogonal
	Vec3 centerOrthogonalVec = computeOrthogonalVec(centerNormalVec);

	setMovementAngle(arm->getHorizontalAngle(), end, desc.distance, centerNormalVec, centerOrthogonalVec);
	cout << "Horizontal direction angle: " << arm->getHorizontalAngle() << " degrees" << endl;

}

///////// Protected //////////
/** \brief calculate final pose according to the given direction angle
*
* \param angle
* \param pose
* \param distance
* \param orthVec 
*/
void Scenario::setMovementAngle(const Real angle, golem::WorkspaceCoord& pose,const Real& distance,const Vec3& normVec,const Vec3& orthVec) {
	pose.p.v1 += Real(sin(angle/180.0*REAL_PI)*(distance*normVec.v1)); 
	pose.p.v2 += Real(sin(angle/180.0*REAL_PI)*(distance*normVec.v2)); 
	pose.p.v1 += Real(cos(angle/180.0*REAL_PI)*(distance*orthVec.v1)); 
	pose.p.v2 += Real(cos(angle/180.0*REAL_PI)*(distance*orthVec.v2)); 
	pose.p.v3 += 0.0;	
}

///////// Protected //////////
/** \brief write obtained dataset into a binary file
*/
void Scenario::writeData (){
	
	/////////////////////////////////////////////////
	//writing the dataset into binary file
	LearningData::write_dataset (dataFileName, data, learningData.featLimits);
	/////////////////////////////////////////////////

	string stpFileName = dataFileName + ".stp";
	ofstream writeToFile (stpFileName.c_str(), ios::out | ios::binary);
	write_vector<double> (writeToFile, usedStartingPositions);
}

///////// Private //////////
/** \brief initializes the experiment 
*/
void Scenario::_init()
{

	randomG.setRandSeed (context.getRandSeed());		// initialize random seed:
	arm->init(context);		// get initial configuration (it is the current joint configuration) and set async time
	// Define the initial pose in the Cartesian workspace
	orientationTarget.set(Real(-0.5*REAL_PI), Real(0.0*REAL_PI), Real(0.0*REAL_PI));
	arm->setMaxTrials(MAX_PLANNER_TRIALS);

}

void Scenario::Desc::setToDefault()
{
	golem::Object::Desc::setToDefault();
			
	// default arm description
	descArmActor.armDesc.setToDefault();
	
	descArmActor.armDesc.pPlannerDesc->pHeuristicDesc->distJointcoordMax[4] = golem::Real(1.0)*golem::REAL_PI; // last joint of Katana
	// Effector group
	descArmActor.effectorGroup = 0x4;
	// finger setup
	descArmActor.fingerDesc.clear();
	// end-effector reference pose
	descArmActor.referencePose.setId();
	// end-effector home pose
	descArmActor.homePose.R.rotX(-0.5*golem::REAL_PI); // end-effector pointing downwards
	descArmActor.homePose.p.set(golem::Real(0.0), golem::Real(0.1), golem::Real(0.1));
	descArmActor.speriod = 1.0;

	featLimits.maxX = 0.4;
	featLimits.maxY = 0.4;
	featLimits.maxZ = 0.4;
	featLimits.minX = 0.0;
	featLimits.minY = 0.0;
	featLimits.minZ = -0.01;
	featLimits.minDuration = 3.0;
	featLimits.maxDuration = 5.0;

	descArmActor.minDuration = 5.0;

	descActorObject.startPosition.set(0.2, 0.2, 0.2);
	descActorObject.startRotation.set(0.0, 0.0, 0.0);
	descActorObject.dimensions.set(0.1, 0.1, 0.1);
	descActorObject.width = 0.002;

	descActorObject.over = 0.017;
	descActorObject.dist = 0.05;
	descActorObject.side = descActorObject.dimensions.v1*0.6;
	descActorObject.center = descActorObject.dimensions.v2*0.6;
	descActorObject.top = descActorObject.dimensions.v2 - 0.02;

	distance = 0.2;

	startingPositionsConfig = "1-18";

}

}; // namespace smlearning 
