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
#include <Ice/Ice.h>
#include <PlotApp.hh>


//------------------------------------------------------------------------------

namespace smlearning {




//------------------------------------------------------------------------------

/** ActiveLearnScenario class */
class ActiveLearnScenario : public smlearning::Scenario {
	/** training sequence for the LSTM */
	rnnlib::DataSequence* trainSeq;
public:
	/** Run experiment */
	///
	///The experiment performed in this method behaves as follows:
	///The arm either selects an action which maximizes the learning progress
	///or selects any of the possible actions randomly according to a near greedy rule.
	///Active Learning is implemented using Intelligent Artificial Curiosity algorithm.
	///
	void run(int argc, char* argv[]);
protected:
	/** LSTM active learner */
	ActiveRNN learner;
	/** to know if net has been already built (initialized in constructor) */
	bool netBuilt;
	/** Objects can be constructed only in the Scene context. */
	ActiveLearnScenario(golem::Scene &scene) : Scenario (scene) {};

	/** restore predicted last polyflap pose from neural activations */
	golem::Mat34 getPfPoseFromOutputActivations (rnnlib::SeqBuffer<double> outputActivations, int startIndex, Real maxRange);
	/** restore sequence of predicted polyflap poses from neural activations */
	void getPfSeqFromOutputActivations (rnnlib::SeqBuffer<double> outputActivations, int startIndex, Real maxRange, vector<golem::Mat34>& currentPredictedObjSeq);
	/** Renders the object. */
        virtual void render();
	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);
	/** load training data in RNNLIB format */
	void loadCurrentTrainSeq (int inputSize, int outputSize);
 

};

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
