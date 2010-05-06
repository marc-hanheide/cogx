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
	/** LSTM active learner */
	ActiveRNN learner;
	/** to know if net has been already built */
	bool netBuilt;
	/** training sequence for the LSTM */
	rnnlib::DataSequence* trainSeq;
	/** map of indexed sensorimotor regions */
	typedef map<int, SMRegion> RegionsMap;
	RegionsMap regions;
	/** Objects can be constructed only in the Scene context. */
	ActiveLearnScenario(golem::Scene &scene) : Scenario (scene) {};

	/** restore predicted last polyflap and effector pose from neural activations */
	golem::Mat34 get_pfefPose_from_outputActivations (rnnlib::SeqBuffer<double> outputActivations, int startIndex, Real maxRange, Real minZ, golem::Mat34& predictedPfPose, golem::Mat34& predictedEfPose);
	/** restore sequence of predicted polyflap poses from neural activations */
	void get_pfefSeq_from_outputActivations (rnnlib::SeqBuffer<double> outputActivations, int startIndex, Real maxRange, Real minZ, vector<golem::Mat34>& currentPredictedPfSeq,  vector<golem::Mat34>& currentPredictedEfSeq);
	/** Renders the object. */
        virtual void render();
	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);
	/** load training data in RNNLIB format */
	void load_current_trainSeq (int inputSize, int outputSize);

	///
	///prepares the polyflap to use
	///
	virtual void initialize_polyflap();

	///
	///choose and describe the start point of the experiment trajectory
	///
	virtual void initialize_movement();

	///
	///choose the starting position
	///
	virtual void define_start_position();

	///
	///write obtained dataset into a binary file
	///
	virtual void write_dataset_into_binary();

	///
	///Update learners according to an sensorimotor region splitting criterion
	///
	void update_learners ();
	
	///
	///Find the appropriate region index according to the given sensorimotor context
	///
	int get_SMRegion (const FeatureVector SMContext);

	///
	///partition of regions according to variance of dataset instances
	///
	void split_region (int region);

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
