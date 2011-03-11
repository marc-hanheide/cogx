/** @file ActiveLearnScenario.h
 * 
 * @author	Sergio Roa (DFKI)
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#ifndef _SMLEARNING_ACTIVELEARNSCENARIO_H_
#define _SMLEARNING_ACTIVELEARNSCENARIO_H_

//------------------------------------------------------------------------------

#include <Golem/Phys/Application.h>
#include <Golem/PhysCtrl/PhysReacPlanner.h>
#include <Golem/Device/Katana/Katana.h>
//#include <Simulator.h> //not present anymore
#include <Golem/Math/Rand.h>
#include <Golem/Phys/Renderer.h>

#include <algorithm>

#include <metalearning/Scenario.h>
#include <metalearning/ActiveRNN.h>
#include <metalearning/SMRegion.h>
#include <Ice/Ice.h>
#include <PlotApp.hh>



//------------------------------------------------------------------------------

namespace smlearning {


//------------------------------------------------------------------------------

/** ActiveLearnScenario class */
class ActiveLearnScenario : public smlearning::Scenario {
public:
	/** Run experiment */
	///
	///The experiment performed in this method behaves as follows:
	///The arm either selects an action which maximizes the learning progress
	///or selects any of the possible actions randomly according to a near greedy rule.
	///Active Learning is implemented using Intelligent Artificial Curiosity algorithm.
	///
	void run(int argc, char* argv[]);

	///
	///set experiment default values
	///
	virtual void init(boost::program_options::variables_map vm);


	/** Object description */
	class Desc : public Scenario::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(ActiveLearnScenario, golem::Object::Ptr, golem::Scene&)

	public:
		/** Constructs description object */
		Desc() {
			Scenario::Desc::setToDefault();
		}
	};
	



	
protected:

	/** training sequence for the LSTM */
	rnnlib::DataSequence* trainSeq;

	/** map of indexed sensorimotor regions */
	SMRegion::RegionsMap regions;

	/** type for vector of candidate actions */
	typedef vector<LearningData::Chunk::Action> ActionsVector;

	/** regions counter for defining regions indices */
	int regionsCount;

	/** current context Region */
	SMRegion* currentRegion;

	/** file name of a trained learner (empty if not provided as a program arg) */
	string netconfigFileName;

	/** method for feature selection (basis/markov/...?) */
	unsigned int featureSelectionMethod;

	/** constant number of maximum candidate actions */
	static const int maxNumberCandidateActions = 1000;

	/** near greedy action selection probability bound for choosing random actions */
	static const double neargreedyActionProb = 0.3;

	/** number of instances to decide splitting a region */
	int splittingCriterion1;

	/** size of motor Context Vector */
	static const int motorContextSize = 6;
	/** size of motor Feature Vector */
	int motorVectorSize;

	/** canonical positions */
	map<Vec3, int, compare_Vec3> positionsT;
	



	/** Objects can be constructed only in the Scene context. */
	ActiveLearnScenario(golem::Scene &scene) : Scenario (scene) { };

	/** restore predicted last polyflap and effector pose from neural activations */
	golem::Mat34 get_pfefPose_from_outputActivations (const rnnlib::SeqBuffer<double>& outputActivations, int startIndex/*, Real maxRange, Real minZ*/, golem::Mat34& predictedPfPose, golem::Mat34& predictedEfPose);

	/** restore sequence of predicted polyflap poses from neural activations */
	template<class Denormalize>
	void get_pfefSeq_from_outputActivations (const rnnlib::SeqBuffer<double>& outputActivations, int sIndex/*, Real maxRange, Real minZ*/, vector<golem::Mat34>& currentPredictedPfSeq, vector<golem::Mat34>& currentPredictedEfSeq, Denormalize denormalize) {

		currentPredictedPfSeq.clear();
		for (int i=0; i < outputActivations.shape[0]; i++) {
			golem::Mat34 currentPredictedPfPose;
			golem::Mat34 currentPredictedEfPose;
			int startIndex = sIndex;

			//extract effector Pose
			// currentPredictedEfPose.p.v1 = denormalize(outputActivations[i][startIndex++], 0.0, maxRange);
			// currentPredictedEfPose.p.v2 = denormalize(outputActivations[i][startIndex++], 0.0, maxRange);
			// currentPredictedEfPose.p.v3 = denormalize(outputActivations[i][startIndex++], 0.0, maxRange);
			// Real efRoll, efPitch, efYaw;
			// efRoll = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
			// efPitch = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
			// efYaw = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
			// currentPredictedEfPose.R.fromEuler(efRoll, efPitch, efYaw);
			// currentPredictedEfSeq.push_back (currentPredictedEfPose);
		
			//extract polyflap Pose
			currentPredictedPfPose.p.v1 = denormalize(outputActivations[i][startIndex++], desc.featLimits.minX, desc.featLimits.maxX);
			currentPredictedPfPose.p.v2 = denormalize(outputActivations[i][startIndex++], desc.featLimits.minY, desc.featLimits.maxY);
			currentPredictedPfPose.p.v3 = denormalize(outputActivations[i][startIndex++], desc.featLimits.minZ, desc.featLimits.maxZ);
			Real pfRoll, pfPitch, pfYaw;
			pfRoll = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
			pfPitch = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
			pfYaw = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
			currentPredictedPfPose.R.fromEuler(pfRoll, pfPitch, pfYaw);
			currentPredictedPfSeq.push_back (currentPredictedPfPose);
		}
	}

	
	/** Renders the object. */
        virtual void render();

	/** (Post)processing function called AFTER every physics simulation step and before rendering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);

	///
	///prepares the polyflap to use
	///
	virtual void initialize_polyflap();

	///
	///select an optimal action from a random set of actions
	///
	virtual void choose_action ();

	///
	///write obtained dataset into a binary file
	///
	virtual void write_data (bool final = false);

	///
	///get the action that maximizes learning progress
	///
	LearningData::Chunk::Action get_action_maxLearningProgress (const ActionsVector& candidateActions);
		
	///
	///Update learners according to a sensorimotor region splitting criterion
	///
	void update_learners ();
	
	///
	///Obtain current context region pointer
	///
	void update_currentRegion ();

	///
	///variance calculation of a dataset
	///
	double variance (const LearningData::DataSet& data, int sMContextSize);

	///
	///minimality criterion for splitting a sensorimotor region
	///
	double evaluate_minimality (const LearningData::DataSet& firstSplittingSet, const LearningData::DataSet& secondSplittingSet, int sMContextSize);

	///
	///partition of regions according to variance of dataset instances
	///
	void split_region (SMRegion& region);

};

//------------------------------------------------------------------------------

/** Application */
class ActivePushingApplication : public smlearning::PushingApplication {
	
public:
	/** Main function */
	virtual int main(int argc, char *argv[]);

	virtual void define_program_options_desc();
	//virtual void read_program_options(int argc, char *argv[]);


protected:




	/** Runs Application */
	virtual void run(int argc, char *argv[]);

};

//------------------------------------------------------------------------------

};	// namespace

#endif /* _SMLEARNING_ACTIVELEARNSCENARIO_H_ */
