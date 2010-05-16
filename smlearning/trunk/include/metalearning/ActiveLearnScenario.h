/** @file ActiveLearnScenario.h
 * 
 * @author	Sergio Roa (DFKI)
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _SMLEARNING_ACTIVELEARNSCENARIO_H_
#define _SMLEARNING_ACTIVELEARNSCENARIO_H_

//------------------------------------------------------------------------------

#include <Golem/Application.h>
#include <Golem/PhysReacPlanner.h>
#include <Golem/Katana.h>
#include <Golem/Simulator.h>
#include <Golem/Rand.h>
#include <Golem/Renderer.h>

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
	/** Object description */
	class Desc : public Scenario::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC(ActiveLearnScenario, golem::Object::Ptr, golem::Scene)

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
	typedef map<int, SMRegion> RegionsMap;
	RegionsMap regions;

	/** struct defining an action tuple */
	struct Action { int startPosition; int speed; Real horizontalAngle; };
	typedef vector<pair<FeatureVector, Action> > ActionsVector;

	/** regions counter for defining regions indices */
	int regionsCount;

	/** current context Region */
	SMRegion* currentRegion;

	/** file name of a trained learner (empty if not provided as a program arg) */
	string netconfigFileName;

	/** constant number of maximum candidate actions */
	static const int maxNumberCandidateActions = 1000;

	/** near greedy action selection probability bound for choosing random actions */
	static const double neargreedyActionProb = 0.3;

	/** number of instances to decide splitting a region */
	int splittingCriterion1;

	/** Objects can be constructed only in the Scene context. */
	ActiveLearnScenario(golem::Scene &scene) : Scenario (scene) { };

	/** restore predicted last polyflap and effector pose from neural activations */
	golem::Mat34 get_pfefPose_from_outputActivations (const rnnlib::SeqBuffer<double>& outputActivations, int startIndex, Real maxRange, Real minZ, golem::Mat34& predictedPfPose, golem::Mat34& predictedEfPose);

	/** restore sequence of predicted polyflap poses from neural activations */
	void get_pfefSeq_from_outputActivations (const rnnlib::SeqBuffer<double>& outputActivations, int startIndex, Real maxRange, Real minZ, vector<golem::Mat34>& currentPredictedPfSeq,  vector<golem::Mat34>& currentPredictedEfSeq);

	/** Renders the object. */
        virtual void render();

	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);

	///
	///describe the lenght of experiment (number of sequences) and if given, the starting position
	///calculate the splittingCriterion1 constant according to nr. of sequences
	virtual void setup_loop(int argc, char* argv[]);

	/**
	   load training data in RNNLIB format
	*/
	void load_current_trainSeq (int inputSize, int outputSize);

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
	virtual void write_data ();

	///
	///get the index of the actions vector that maximizes learning progress
	///
	pair<FeatureVector, Action> get_actionsIdx_maxLearningProgress (const ActionsVector& candidateActions);
		
	///
	///Update learners according to a sensorimotor region splitting criterion
	///
	void update_learners ();
	
	///
	///Find the appropriate region index according to the given sensorimotor context
	///
	int get_SMRegion (const FeatureVector& SMContext);

	///
	///Obtain current context region pointer
	///
	void update_currentRegion ();

	///
	///variance calculation of a dataset
	///
	double variance (const DataSet& data, int sMContextSize);

	///
	///minimality criterion for splitting a sensorimotor region
	///
	double evaluate_minimality (const DataSet& firstSplittingSet, const DataSet& secondSplittingSet, int sMContextSize);

	///
	///partition of regions according to variance of dataset instances
	///
	void split_region (SMRegion& region);

};

//------------------------------------------------------------------------------

/** Reads/writes Scenario description from/to a given context */
bool XMLData(ActiveLearnScenario::Desc &val, golem::XMLContext* context, bool create = false);
//------------------------------------------------------------------------------

/** Application */
class ActivePushingApplication : public golem::Application {
	
protected:
	/** Runs Application */
	virtual void run(int argc, char *argv[]);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /* _SMLEARNING_ACTIVELEARNSCENARIO_H_ */
